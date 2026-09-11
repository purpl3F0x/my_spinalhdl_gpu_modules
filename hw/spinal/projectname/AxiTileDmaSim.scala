package projectname

import spinal.core._
import spinal.core.sim._
import spinal.lib._
import spinal.lib.bus.amba4.axi._
import spinal.lib.bus.amba4.axilite._
import scala.collection.mutable

// =============================================================================
// SpinalHDL Simulation Memory Model for Axi4WriteOnly Interface
// Models DRAM write slave behavior (AW, W, B channels) with Sparse Storage
// =============================================================================
class AxiWriteOnlyMemorySim(axi: Axi4WriteOnly, clockDomain: ClockDomain) {
  val memory = mutable.Map[Long, Byte]()

  case class AwTransaction(addr: Long, len: Int, size: Int, id: Int)
  val awQueue = mutable.Queue[AwTransaction]()
  val bQueue  = mutable.Queue[(Int, Int)]() // (id, resp)

  def start(): Unit = {
    // AW Handshake Channel
    fork {
      axi.aw.ready #= true
      while (true) {
        clockDomain.waitSampling()
        if (axi.aw.valid.toBoolean && axi.aw.ready.toBoolean) {
          val addr = axi.aw.payload.addr.toLong
          val len  = axi.aw.payload.len.toInt
          val size = axi.aw.payload.size.toInt
          val id   = axi.aw.payload.id.toInt
          awQueue.enqueue(AwTransaction(addr, len, size, id))
        }
      }
    }

    // W & B Handshake Channels
    fork {
      var currentAw: AwTransaction = null
      var beatIndex = 0
      axi.b.valid #= false
      axi.w.ready #= true

      while (true) {
        clockDomain.waitSampling()

        if (currentAw == null && awQueue.nonEmpty) {
          currentAw = awQueue.dequeue()
          beatIndex = 0
        }

        val wFire = axi.w.valid.toBoolean && axi.w.ready.toBoolean && (currentAw != null)

        if (wFire) {
          val wdata = axi.w.payload.data.toBigInt
          val wstrb = axi.w.payload.strb.toBigInt
          val wlast = axi.w.payload.last.toBoolean

          val bytesPerBeat = 1 << currentAw.size
          val beatAddr = currentAw.addr + (beatIndex * bytesPerBeat)

          for (byteIdx <- 0 until bytesPerBeat) {
            if (((wstrb >> byteIdx) & 1) != 0) {
              val byteVal = ((wdata >> (byteIdx * 8)) & 0xFF).toByte
              memory(beatAddr + byteIdx) = byteVal
            }
          }

          if (wlast) {
            // Burst complete: schedule B OKAY response
            bQueue.enqueue((currentAw.id, 0)) // OKAY = 0
            if (awQueue.nonEmpty) {
              currentAw = awQueue.dequeue()
              beatIndex = 0
            } else {
              currentAw = null
              beatIndex = 0
            }
          } else {
            beatIndex += 1
          }
        }

        // Drive B Response Channel
        if (bQueue.nonEmpty) {
          val (bid, bresp) = bQueue.head
          axi.b.valid #= true
          axi.b.payload.id #= bid
          axi.b.payload.resp #= bresp
          if (axi.b.ready.toBoolean) {
            bQueue.dequeue()
          }
        } else {
          axi.b.valid #= false
        }
      }
    }
  }

  def readByte(addr: Long): Int = {
    memory.getOrElse(addr, 0.toByte).toInt & 0xFF
  }

  def readInt(addr: Long): Long = {
    val b0 = readByte(addr + 0).toLong
    val b1 = readByte(addr + 1).toLong
    val b2 = readByte(addr + 2).toLong
    val b3 = readByte(addr + 3).toLong
    (b3 << 24) | (b2 << 16) | (b1 << 8) | b0
  }
}

// =============================================================================
// SpinalHDL AXI-Lite Simulation Driver for Control Slave Interface
// =============================================================================
class SimAxiLiteDriver(axil: AxiLite4, clockDomain: ClockDomain) {
  def reset(): Unit = {
    axil.aw.valid #= false
    axil.w.valid  #= false
    axil.b.ready  #= true
    axil.ar.valid #= false
    axil.r.ready  #= true
  }

  def write(address: Long, data: Long): Unit = {
    axil.aw.valid #= true
    axil.aw.payload.addr #= address
    axil.w.valid #= true
    axil.w.payload.data #= data
    axil.w.payload.strb #= 0xF
    axil.b.ready #= true

    var awDone = false
    var wDone  = false
    while (!awDone || !wDone) {
      clockDomain.waitSampling()
      if (axil.aw.ready.toBoolean) {
        axil.aw.valid #= false
        awDone = true
      }
      if (axil.w.ready.toBoolean) {
        axil.w.valid #= false
        wDone = true
      }
    }

    while (!axil.b.valid.toBoolean) {
      clockDomain.waitSampling()
    }
    axil.b.ready #= false
    clockDomain.waitSampling()
  }

  def read(address: Long): Long = {
    axil.ar.valid #= true
    axil.ar.payload.addr #= address
    axil.r.ready #= true

    do {
      clockDomain.waitSampling()
    } while (!axil.ar.ready.toBoolean)
    axil.ar.valid #= false

    while (!axil.r.valid.toBoolean) {
      clockDomain.waitSampling()
    }
    val rdata = axil.r.payload.data.toLong
    axil.r.ready #= false
    clockDomain.waitSampling()
    rdata
  }
}

// =============================================================================
// SpinalHDL Simulation Memory Model for BRAM Read Interface
// Simulates a dual-port BRAM:
// - Direct write interface for testbench image population
// - Synchronous read port connected to dut.io.tile_ram with configurable latency (1, 2... cycles)
// =============================================================================
class SimBramMemory(
    tileRam: TileRamReadPort,
    clockDomain: ClockDomain,
    beatShift: Int,
    totalBeats: Int,
    latency: Int = 1
) {
  require(latency >= 1, "BRAM read latency must be at least 1 cycle")
  val memory = mutable.Map[Int, BigInt]()

  def writeWord(wordAddr: Int, wordData: BigInt): Unit = {
    memory(wordAddr) = wordData
  }

  def start(): Unit = {
    tileRam.dout #= 0
    fork {
      val pipeline = Array.fill(latency)(BigInt(0))
      val validPipe = Array.fill(latency)(false)

      while (true) {
        clockDomain.waitSampling()

        // Shift pipeline stages forward
        for (i <- 0 until (latency - 1)) {
          pipeline(i) = pipeline(i + 1)
          validPipe(i) = validPipe(i + 1)
        }

        // Sample read request (respecting active-high tileRam.rst)
        if (tileRam.en.toBoolean && !tileRam.rst.toBoolean) {
          val byteAddr = tileRam.addr.toLong
          val wordAddr = (byteAddr >> beatShift).toInt
          val data = memory.getOrElse(wordAddr, BigInt(0))
          pipeline(latency - 1) = data
          validPipe(latency - 1) = true
        } else {
          pipeline(latency - 1) = BigInt(0)
          validPipe(latency - 1) = false
        }

        if (validPipe(0)) {
          tileRam.dout #= pipeline(0)
        }
      }
    }
  }
}

// =============================================================================
// Main Simulation Testbench: 128-bit Bus, 4x3 Grid (12 Tiles of 128x128)
// =============================================================================
object AxiTileDmaSim extends App {
  // Test Configuration: 128x128 tile, 128-bit AXI bus (4 px/clock), 4x3 grid
  val config = AxiTileDmaConfig(
    addressWidth   = 32,
    pixelsPerClock = 4,    // 4 pixels/clock = 128-bit AXI data bus
    tileWidth      = 128,  // 128 pixels wide
    tileHeight     = 128,  // 128 lines high
    maxBurstLength = 256,
    maxOutstanding = 8,
    idWidth        = 4,
    cdcDepth       = 16,
    useAutoAdvance = true,
    bramReadLatency = 4    // 3-cycle BRAM read latency (testing 3-cycle UltraScale+ BRAM pipeline mode)
  )

  // Fast simulation with IVerilog backend
  val simConfig = SimConfig
    .withConfig(
      SpinalConfig(
        defaultConfigForClockDomains = ClockDomainConfig(resetActiveLevel = LOW)
      )
    )
    .withIVerilog

  simConfig.compile(new AxiTileDma(config)).doSim("AxiTileDma_4x3_Grid_Test") { dut =>
    // -------------------------------------------------------------------------
    // Dual Asynchronous Clock Stimulus Generation
    // AXI Bus Clock: 100 MHz (period 10 ns)
    // Fast RAM Clock: 200 MHz (period 5 ns)
    // -------------------------------------------------------------------------
    dut.clockDomain.forkStimulus(period = 10)     // clk & reset (AXI domain)
    dut.ramClock.forkStimulus(period = 5)         // ram_clk & ram_reset (RAM domain)

    val axiClock = dut.clockDomain
    val ramClock = dut.ramClock

    // Instantiate simulation helpers
    val axilDriver = new SimAxiLiteDriver(dut.io.s_axil_ctrl, axiClock)
    axilDriver.reset()

    val dramSim = new AxiWriteOnlyMemorySim(dut.io.m_axi, axiClock)
    dramSim.start()

    val bramSim = new SimBramMemory(
      tileRam = dut.io.tile_ram,
      clockDomain = ramClock,
      beatShift = config.beatShift,
      totalBeats = config.totalBeats,
      latency = config.bramReadLatency
    )
    bramSim.start()

    axiClock.waitSampling(10)
    ramClock.waitSampling(10)

    // Fast BRAM Write Helper (Populates 128-bit words in 128x128 Tile BRAM)
    def writeBramWord(wordAddr: Int, wordData128: BigInt): Unit = {
      bramSim.writeWord(wordAddr, wordData128)
    }

    // -------------------------------------------------------------------------
    // TEST PARAMETERS: 4x3 Grid of 128x128 Tiles
    // -------------------------------------------------------------------------
    val gridTilesX = 4
    val gridTilesY = 3
    val tileWidth  = 128
    val tileHeight = 128
    val frameWidthPixels = gridTilesX * tileWidth   // 512 pixels
    val frameHeightLines = gridTilesY * tileHeight  // 384 lines
    val frameRowStrideBytes = frameWidthPixels * 4  // 2048 bytes (Power of 2: 2^11)
    val dramBaseAddr = 0x00010000L

    println(s"======================================================================")
    println(s"Starting 2D Tile Frame Buffer DMA Simulation (128-bit Bus)")
    println(s"  Grid Size        : ${gridTilesX}x${gridTilesY} tiles (Total 12 tiles)")
    println(s"  Tile Resolution  : ${tileWidth}x${tileHeight} pixels (32-bit RGBA)")
    println(s"  AXI Bus Width    : 128 bits (${config.pixelsPerClock} pixels/clock)")
    println(s"  Frame Resolution : ${frameWidthPixels}x${frameHeightLines} pixels")
    println(s"  Frame Row Stride : ${frameRowStrideBytes} bytes (Power of 2: 2^11)")
    println(s"  DRAM Base Address: 0x${dramBaseAddr.toHexString}")
    println(s"======================================================================")

    // Program DMA Control Registers via AXI-Lite
    axilDriver.write(0x08, dramBaseAddr & 0xFFFFFFFFL)     // BASE_ADDR_LOW
    axilDriver.write(0x0C, (dramBaseAddr >> 32) & 0xFFFFFFFFL) // BASE_ADDR_HIGH
    axilDriver.write(0x10, frameRowStrideBytes)            // STRIDE = 2048
    axilDriver.write(0x14, gridTilesX)                     // TILES_X = 4
    axilDriver.write(0x18, gridTilesY)                     // TILES_Y = 3
    axilDriver.write(0x00, 0x04)                           // IRQ_ENABLE = 1

    println("[AXI-Lite] Initialized BaseAddr, Stride, TilesX, TilesY registers.")

    val totalTiles = gridTilesX * gridTilesY
    var tileCount = 0

    // Transfer all tiles in 2D Raster Order: (0,0) -> (1,0) -> (2,0) -> (3,0) -> (0,1) ...
    for (tileY <- 0 until gridTilesY) {
      for (tileX <- 0 until gridTilesX) {
        tileCount += 1
        println(s"\n--- [Tile $tileCount/$totalTiles] Processing Tile ($tileX, $tileY) ---")

        // 1. Fill high-speed BRAM with test pattern: (tileX, tileY, localY, localX)
        for (localY <- 0 until tileHeight) {
          for (beatX <- 0 until (tileWidth / config.pixelsPerClock)) {
            val wordAddr = localY * (tileWidth / config.pixelsPerClock) + beatX
            var word128: BigInt = 0

            for (px <- 0 until config.pixelsPerClock) {
              val localX = beatX * config.pixelsPerClock + px
              // 32-bit pixel encoding: [31..24]=tileX, [23..16]=tileY, [15..8]=localY, [7..0]=localX
              val pixelVal = ((tileX.toLong & 0xFF) << 24) |
                             ((tileY.toLong & 0xFF) << 16) |
                             ((localY.toLong & 0xFF) << 8)  |
                             (localX.toLong & 0xFF)
              word128 |= (BigInt(pixelVal) & 0xFFFFFFFFL) << (px * 32)
            }
            writeBramWord(wordAddr, word128)
          }
        }

        // 2. Trigger DMA Start via AXI-Lite
        axilDriver.write(0x00, 0x05) // START (bit 0) | IRQ_ENABLE (bit 2)

        // 3. Poll Status register until TILE_DONE (bit 1) is asserted
        var status = 0L
        var pollCycles = 0
        do {
          axiClock.waitSampling(10)
          status = axilDriver.read(0x04)
          pollCycles += 10
          if (pollCycles % 2000 == 0) {
            val perf = axilDriver.read(0x24)
            val pos  = axilDriver.read(0x1C)
            println(s"    [Poll $pollCycles] Status=0x${status.toHexString}, PerfCycles=$perf, Pos=0x${pos.toHexString}")
          }
        } while ((status & 0x02) == 0 && pollCycles < 50000)

        assert((status & 0x02) != 0, s"Timeout waiting for Tile ($tileX, $tileY) to finish!")
        val perfCycles = axilDriver.read(0x24)
        val curPos     = axilDriver.read(0x1C)
        val nextX      = curPos & 0xFFFF
        val nextY      = (curPos >> 16) & 0xFFFF

        println(f"    Completed in $perfCycles cycles. Status=0x$status%X. Next Tile Pos=($nextX, $nextY)")

        // Clear DONE status flag
        axilDriver.write(0x04, 0x02)

        val finalStatus = axilDriver.read(0x04)

        // Check FRAME_DONE on the very last tile (3, 2)
        if (tileX == gridTilesX - 1 && tileY == gridTilesY - 1) {
          assert((finalStatus & 0x04) != 0, "FRAME_DONE bit (0x04) must be asserted on final tile completion!")
          assert(nextX == 0 && nextY == 0, "Tile coordinates must wrap back to (0,0) after frame completion!")
          println("    >> FRAME_DONE successfully asserted and coordinates wrapped to (0,0)!")
        }
      }
    }

    // -------------------------------------------------------------------------
    // VERIFICATION: Check Entire 512x384 Frame in Simulated DRAM
    // -------------------------------------------------------------------------
    println("\n======================================================================")
    println("Verifying DRAM pixel buffer integrity across the full 512x384 frame...")
    println("======================================================================")

    var errorCount = 0

    for (globalY <- 0 until frameHeightLines) {
      for (globalX <- 0 until frameWidthPixels) {
        val expectedTileX  = globalX / tileWidth
        val expectedTileY  = globalY / tileHeight
        val expectedLocalX = globalX % tileWidth
        val expectedLocalY = globalY % tileHeight

        val expectedPixel = ((expectedTileX.toLong & 0xFF) << 24) |
                            ((expectedTileY.toLong & 0xFF) << 16) |
                            ((expectedLocalY.toLong & 0xFF) << 8)  |
                            (expectedLocalX.toLong & 0xFF)

        val pixelByteAddr = dramBaseAddr + (globalY * frameRowStrideBytes) + (globalX * 4)

        // Read 32-bit pixel from simulated DRAM
        val actualPixel = dramSim.readInt(pixelByteAddr)

        if (actualPixel != expectedPixel) {
          if (errorCount < 10) {
            println(f"[ERROR] Mismatch at Global ($globalX%3d, $globalY%3d) [DRAM Addr: 0x$pixelByteAddr%08X]: " +
                    f"Expected 0x$expectedPixel%08X (Tile=($expectedTileX,$expectedTileY), Local=($expectedLocalX,$expectedLocalY)), " +
                    f"Actual = 0x$actualPixel%08X")
          }
          errorCount += 1
        }
      }
    }

    if (errorCount == 0) {
      val totalPixels = frameWidthPixels * frameHeightLines
      println(s"\n[SUCCESS] ALL $totalPixels PIXELS VERIFIED PERFECTLY IN DRAM!")
      println(s"  - 4x3 Grid (12 tiles of 128x128, 128-bit AXI bus) successfully verified.")
      println(s"  - Multi-cycle 2D raster stride & auto-advance verified.")
      println(s"  - Asynchronous dual-clock CDC verified without data corruption.")
      println("======================================================================\n")
    } else {
      println(s"\n[FAILURE] Total $errorCount pixel mismatches found in DRAM!")
      assert(false, s"Test failed with $errorCount errors.")
    }
  }
}
