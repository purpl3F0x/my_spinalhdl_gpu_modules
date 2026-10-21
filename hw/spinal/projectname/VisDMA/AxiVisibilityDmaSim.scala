package projectname

import spinal.core._
import spinal.core.sim._
import spinal.lib._
import spinal.lib.bus.amba4.axi._
import spinal.lib.bus.amba3.apb._
import scala.collection.mutable

// =============================================================================
// SpinalHDL Simulation Memory Model for Axi4ReadOnly Interface
// =============================================================================
class AxiReadOnlyMemorySim(axi: Axi4ReadOnly, clockDomain: ClockDomain) {
  val memory = mutable.Map[Long, BigInt]()

  case class ArTransaction(addr: Long, len: Int, size: Int, id: Int)
  val arQueue = mutable.Queue[ArTransaction]()

  def setWord64(addr: Long, data: BigInt, axiDataWidth: Int = 128): Unit = {
    val bytesPerBeat = axiDataWidth / 8
    val beatAddr = addr & ~(bytesPerBeat - 1L)
    val offsetInBeat = (addr - beatAddr).toInt
    val sliceShift = offsetInBeat * 8
    val sliceMask = BigInt("FFFFFFFFFFFFFFFF", 16) << sliceShift
    val current = memory.getOrElse(beatAddr, BigInt(0))
    val updated = (current & ~sliceMask) | ((data & BigInt("FFFFFFFFFFFFFFFF", 16)) << sliceShift)
    memory(beatAddr) = updated
    memory(addr) = data
  }

  def start(): Unit = {
    // AR Channel
    fork {
      axi.ar.ready #= true
      while (true) {
        clockDomain.waitSampling()
        if (axi.ar.valid.toBoolean && axi.ar.ready.toBoolean) {
          val addr = axi.ar.payload.addr.toLong
          val len = axi.ar.payload.len.toInt
          val size = axi.ar.payload.size.toInt
          val id = axi.ar.payload.id.toInt
          arQueue.enqueue(ArTransaction(addr, len, size, id))
        }
      }
    }

    // R Channel
    fork {
      var currentAr: ArTransaction = null
      var beatIndex = 0
      axi.r.valid #= false

      while (true) {
        clockDomain.waitSampling()

        if (currentAr == null && arQueue.nonEmpty) {
          currentAr = arQueue.dequeue()
          beatIndex = 0
        }

        if (currentAr != null) {
          val bytesPerBeat = 1 << currentAr.size
          val beatAddr = currentAr.addr + (beatIndex * bytesPerBeat)
          val rdata = memory.getOrElse(beatAddr, BigInt(0))
          val isLast = (beatIndex == currentAr.len)

          axi.r.valid #= true
          axi.r.payload.data #= rdata
          axi.r.payload.id #= currentAr.id
          axi.r.payload.resp #= 0 // OKAY
          axi.r.payload.last #= isLast

          if (axi.r.ready.toBoolean) {
            if (isLast) {
              currentAr = null
              beatIndex = 0
            } else {
              beatIndex += 1
            }
          }
        } else {
          axi.r.valid #= false
        }
      }
    }
  }
}

// =============================================================================
// SpinalHDL Simulation Behavioral Model for AXI DataMover
// =============================================================================
class SimDataMoverResponder(
    cmdStream: Stream[Bits],
    stsStream: Stream[Bits],
    clockDomain: ClockDomain,
    addressWidth: Int = 32
) {
  case class ReceivedCmd(
      btt: Long,
      tag: Int,
      saddr: Long,
      raw: BigInt
  )

  val receivedCommands = mutable.ArrayBuffer[ReceivedCmd]()
  val stsQueue = mutable.Queue[(Int, Boolean)]()

  def start(): Unit = {
    // Consume Commands
    fork {
      cmdStream.ready #= true
      while (true) {
        clockDomain.waitSampling()
        if (cmdStream.valid.toBoolean && cmdStream.ready.toBoolean) {
          val raw = cmdStream.payload.toBigInt
          val btt = (raw & 0x7fffff).toLong
          val tag = ((raw >> 30) & 0x3).toInt
          val saddr = if (addressWidth <= 32) {
            ((raw >> 32) & 0xffffffffL).toLong
          } else {
            ((raw >> 32) & 0xffffffffffffffffL).toLong
          }

          val cmd = ReceivedCmd(btt, tag, saddr, raw)
          receivedCommands += cmd
          stsQueue.enqueue((tag, false)) // Normal OKAY response
        }
      }
    }

    // Emit Status Responses
    fork {
      stsStream.valid #= false
      while (true) {
        clockDomain.waitSampling()
        if (stsQueue.nonEmpty) {
          val (tag, isError) = stsQueue.head
          val stsByte = if (!isError) (0x80 | (tag & 0xf)) else (0x40 | (tag & 0xf))

          stsStream.valid #= true
          stsStream.payload #= stsByte

          if (stsStream.ready.toBoolean) {
            stsQueue.dequeue()
          }
        } else {
          stsStream.valid #= false
        }
      }
    }
  }
}

// =============================================================================
// Simulation Testbench for AxiVisibilityDma (using IVerilog)
// =============================================================================
object AxiVisibilityDmaSim extends App {
  val config = AxiVisibilityDmaConfig(
    addressWidth = 32,
    axiDataWidth = 128, // 128-bit AXI Read Master (configurable >= 64 bits)
    bttWidth = 23, // 23-bit configurable BTT
    gridWidth = 10,
    gridHeight = 6,
    vbWordBits = 64,
    triangleSizeBytes = 128,
    maxBurstLength = 64,
    maxOutstandingReads = 4,
    cmdFifoDepth = 64,
    stsFifoDepth = 4,
    vbFifoDepth = 64, // >= maxBurstLength
    useAutoAdvance = true
  )

  val simConfig = SimConfig
    .withConfig(
      SpinalConfig(
        defaultConfigForClockDomains = ClockDomainConfig(resetActiveLevel = LOW)
      )
    )
    .withIVerilog

  simConfig.compile(new AxiVisibilityDma(config)).doSim("AxiVisibilityDma_Test") { dut =>
    dut.clockDomain.forkStimulus(period = 10)

    val apbDriver = new SimApbDriver(dut.io.s_apb, dut.clockDomain)
    val vbMemory = new AxiReadOnlyMemorySim(dut.io.m_axi_vb, dut.clockDomain)
    val dmResponder = new SimDataMoverResponder(
      cmdStream = dut.io.m_axis_datamover_cmd,
      stsStream = dut.io.s_axis_datamover_sts,
      clockDomain = dut.clockDomain,
      addressWidth = config.addressWidth
    )

    apbDriver.reset()
    vbMemory.start()
    dmResponder.start()

    dut.clockDomain.waitSampling(5)

    println("=================================================================")
    println("Test 1: Register Read / Write and Address Alignment Checking")
    println("=================================================================")

    // Triangle Geometry Base Address = 0x8000_1000 (lower 7 bits are 0x00)
    apbDriver.write(0x10, 0x8000107fL)
    val readTriLow = apbDriver.read(0x10)
    println(f"Wrote TRI_BASE_LOW: 0x8000107F, Read Back: 0x$readTriLow%08X (Lower 7 bits must be 0)")
    assert((readTriLow & 0x7f) == 0, "Error: lower 7 bits of TRI_BASE_LOW were not ignored!")

    // Visibility Buffer Base Address = 0x4000_2000 (lower 4 bits are 0 for 128-bit)
    apbDriver.write(0x08, 0x4000200fL)
    val readVbLow = apbDriver.read(0x08)
    println(f"Wrote VB_BASE_LOW: 0x4000200F, Read Back: 0x$readVbLow%08X (Lower 4 bits must be 0 for 128-bit AXI)")
    assert((readVbLow & 0x0f) == 0, "Error: lower 4 bits of VB_BASE_LOW were not ignored!")

    // Check ready pin is high when idle
    assert(dut.io.ready.toBoolean, "Error: engine ready output must be true when idle!")

    println("=================================================================")
    println("Test 2: Visibility Filtering & Unchunked BTT Aggregation (Tile 0)")
    println("=================================================================")
    val N = 16
    val vbBaseAddr = 0x40000000L
    val triBaseAddr = 0x80000000L

    apbDriver.write(0x08, vbBaseAddr)
    apbDriver.write(0x10, triBaseAddr)
    apbDriver.write(0x18, N) // TRI_COUNT = 16
    apbDriver.write(0x1c, 0) // TILE_INDEX = 0

    // Visibility pattern for Tile 0 (bit 0):
    // Tri 0..2: Visible (3 consecutive)  -> Cmd 0: Addr = 0x8000_0000, BTT = 3 * 128 = 384 bytes
    // Tri 3:    Invisible
    // Tri 4..5: Visible (2 consecutive)  -> Cmd 1: Addr = 0x8000_0200, BTT = 2 * 128 = 256 bytes
    // Tri 6..7: Invisible
    // Tri 8..12: Visible (5 consecutive) -> Unchunked! Merged into single BTT!
    //           -> Cmd 2: Addr = 0x8000_0400, BTT = 5 * 128 = 640 bytes (Tri 8..12)
    // Tri 13..14: Invisible
    // Tri 15:   Visible (1 triangle)     -> Cmd 3: Addr = 0x8000_0780, BTT = 1 * 128 = 128 bytes

    val visibilityMaskTile0 = Array(
      true, true, true, // 0, 1, 2
      false, // 3
      true, true, // 4, 5
      false, false, // 6, 7
      true, true, true, true, true, // 8, 9, 10, 11, 12 (5 contiguous triangles!)
      false, false, // 13, 14
      true // 15
    )

    for (i <- 0 until N) {
      var wordVal: BigInt = 0
      if (visibilityMaskTile0(i)) {
        wordVal |= 1 // bit 0 set
      }
      if (i < 2) {
        wordVal |= 2 // bit 1 set for triangles 0 and 1
      }
      vbMemory.setWord64(vbBaseAddr + (i * 8), wordVal)
    }

    // Enable IRQ and Auto-Advance in CTRL (IRQ_EN=1, AUTO_ADV_EN=1): 0x0D
    println("Triggering START for Tile 0...")
    apbDriver.write(0x00, 0x0d)

    var timeout = 500
    var done = false
    while (!done && timeout > 0) {
      dut.clockDomain.waitSampling(5)
      val status = apbDriver.read(0x04)
      done = (status & 0x02) != 0 // Bit 1 = DONE
      timeout -= 1
    }

    assert(done, "Timeout waiting for DMA tile transfer completion!")
    println("Tile 0 transfer completed!")

    // Verify interrupt and ready
    assert(dut.io.irq.toBoolean, "Error: IRQ output must be high upon completion!")
    assert(dut.io.ready.toBoolean, "Error: ready output must be high when idle!")

    // Read status counters
    val visCount = apbDriver.read(0x24)
    val cmdCount = apbDriver.read(0x28)
    val stsCount = apbDriver.read(0x2c)
    println(f"STAT_VIS: $visCount, STAT_CMD: $cmdCount, STAT_STS: $stsCount")
    println("Verifying issued DataMover commands (Unchunked):")
    for ((cmd, idx) <- dmResponder.receivedCommands.zipWithIndex) {
      println(f"  Cmd $idx: Addr = 0x${cmd.saddr}%08X, BTT = ${cmd.btt} bytes (${cmd.btt / 128} triangles)")
    }
    assert(visCount == 11, f"Expected 11 visible triangles, got $visCount")
    assert(cmdCount == 4, f"Expected 4 DataMover commands, got $cmdCount")
    assert(stsCount == 4, f"Expected 4 DataMover status responses, got $stsCount")

    assert(dmResponder.receivedCommands(0).btt == 384, "Cmd 0 BTT mismatch (expected 384)")
    assert(dmResponder.receivedCommands(0).saddr == 0x80000000L, "Cmd 0 SADDR mismatch")

    assert(dmResponder.receivedCommands(1).btt == 256, "Cmd 1 BTT mismatch (expected 256)")
    assert(dmResponder.receivedCommands(1).saddr == 0x80000200L, "Cmd 1 SADDR mismatch")

    // 5 triangles * 128 bytes = 640 bytes in a SINGLE command!
    assert(dmResponder.receivedCommands(2).btt == 640, "Cmd 2 BTT mismatch (expected unchunked 640 bytes)")
    assert(dmResponder.receivedCommands(2).saddr == 0x80000400L, "Cmd 2 SADDR mismatch")

    assert(dmResponder.receivedCommands(3).btt == 128, "Cmd 3 BTT mismatch (expected 128)")
    assert(dmResponder.receivedCommands(3).saddr == 0x80000780L, "Cmd 3 SADDR mismatch")

    println("=================================================================")
    println("Test 3: Tile Auto-Advancement to Tile 1")
    println("=================================================================")
    val curTileIdx = apbDriver.read(0x1c)
    println(f"Current TILE_INDEX after auto-advance: $curTileIdx (Expected 1)")
    assert(curTileIdx == 1, f"Expected TILE_INDEX == 1, got $curTileIdx")

    // Clear DONE and IRQ (Write 1 to clear)
    apbDriver.write(0x04, 0x0a)
    apbDriver.write(0x40, 0x07) // INTR_INT_RAW W1C (done, error, frame_done) drives io.irq
    dut.clockDomain.waitSampling(2)
    assert(!dut.io.irq.toBoolean, "Error: IRQ should be deasserted after W1C clear!")

    dmResponder.receivedCommands.clear()

    println("Triggering START for Tile 1...")
    apbDriver.write(0x00, 0x0d)

    timeout = 500
    done = false
    while (!done && timeout > 0) {
      dut.clockDomain.waitSampling(5)
      val status = apbDriver.read(0x04)
      done = (status & 0x02) != 0
      timeout -= 1
    }
    assert(done, "Timeout waiting for Tile 1 transfer completion!")
    println("Tile 1 transfer completed!")

    val visCountTile1 = apbDriver.read(0x24)
    val cmdCountTile1 = apbDriver.read(0x28)
    println(f"Tile 1 STAT_VIS: $visCountTile1, STAT_CMD: $cmdCountTile1")
    assert(visCountTile1 == 2, f"Expected 2 visible triangles in Tile 1, got $visCountTile1")
    assert(cmdCountTile1 == 1, f"Expected 1 merged command in Tile 1, got $cmdCountTile1")
    assert(dmResponder.receivedCommands(0).btt == 256, "Tile 1 BTT should be 256 bytes")

    val nextTileIdx = apbDriver.read(0x1c)
    println(f"Current TILE_INDEX after second auto-advance: $nextTileIdx (Expected 2)")
    assert(nextTileIdx == 2, f"Expected TILE_INDEX == 2, got $nextTileIdx")

    println("=================================================================")
    println("Test 4: Odd Triangle Count (N = 13) with 128-bit AXI Master")
    println("=================================================================")
    // Clear DONE and IRQ
    apbDriver.write(0x04, 0x0a)
    apbDriver.write(0x40, 0x07) // INTR_INT_RAW W1C (done, error, frame_done) drives io.irq
    dut.clockDomain.waitSampling(2)

    val oddN = 13
    apbDriver.write(0x18, oddN) // TRI_COUNT = 13
    dmResponder.receivedCommands.clear()

    // Visibility pattern for Tile 2 (bit 2):
    // Tri 0..1: Visible (2 consecutive)  -> Cmd 0: BTT = 2 * 128 = 256B
    // Tri 2:    Invisible
    // Tri 3..6: Visible (4 consecutive)  -> Cmd 1: BTT = 4 * 128 = 512B
    // Tri 7..11: Invisible
    // Tri 12:   Visible (odd last tri)   -> Cmd 2: BTT = 1 * 128 = 128B
    val visibilityMaskTile2 = Array(
      true, true, // 0, 1
      false, // 2
      true, true, true, true, // 3, 4, 5, 6
      false, false, false, false, false, // 7..11
      true // 12 (last triangle)
    )

    for (i <- 0 until oddN) {
      val wordVal: BigInt = if (visibilityMaskTile2(i)) (BigInt(1) << 2) else BigInt(0)
      vbMemory.setWord64(vbBaseAddr + (i * 8), wordVal)
    }

    println("Triggering START for Tile 2 (Odd N = 13)...")
    apbDriver.write(0x00, 0x0d)

    timeout = 500
    done = false
    while (!done && timeout > 0) {
      dut.clockDomain.waitSampling(5)
      val status = apbDriver.read(0x04)
      done = (status & 0x02) != 0
      timeout -= 1
    }
    assert(done, "Timeout waiting for Tile 2 transfer completion!")
    println("Tile 2 transfer completed!")

    val visCountTile2 = apbDriver.read(0x24)
    val cmdCountTile2 = apbDriver.read(0x28)
    println(f"Tile 2 STAT_VIS: $visCountTile2, STAT_CMD: $cmdCountTile2")
    assert(visCountTile2 == 7, f"Expected 7 visible triangles in Tile 2, got $visCountTile2")
    assert(cmdCountTile2 == 3, f"Expected 3 commands in Tile 2, got $cmdCountTile2")
    assert(dmResponder.receivedCommands(0).btt == 256, "Cmd 0 BTT should be 256 bytes")
    assert(dmResponder.receivedCommands(1).btt == 512, "Cmd 1 BTT should be 512 bytes")
    assert(dmResponder.receivedCommands(2).btt == 128, "Cmd 2 BTT should be 128 bytes")

    val finalTileIdx = apbDriver.read(0x1c)
    println(f"Current TILE_INDEX after third auto-advance: $finalTileIdx (Expected 3)")
    assert(finalTileIdx == 3, f"Expected TILE_INDEX == 3, got $finalTileIdx")

    println("=================================================================")
    println("ALL TESTS PASSED SUCCESSFULLY!")
    println("=================================================================")
  }
}
