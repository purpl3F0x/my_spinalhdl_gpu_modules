package projectname

import spinal.core._
import spinal.core.sim._
import spinal.lib._
import spinal.lib.bus.amba4.axi._
import spinal.lib.bus.amba4.axilite._
import scala.collection.mutable

// =============================================================================
// SpinalHDL Simulation Memory Model for 32-bit Word Visibility Buffer
// =============================================================================
class AxiReadOnlyMemorySim4x(axi: Axi4ReadOnly, clockDomain: ClockDomain) {
  val memory = mutable.Map[Long, BigInt]()

  case class ArTransaction(addr: Long, len: Int, size: Int, id: Int)
  val arQueue = mutable.Queue[ArTransaction]()

  def setWord32(addr: Long, data: Long, axiDataWidth: Int = 128): Unit = {
    val bytesPerBeat = axiDataWidth / 8
    val beatAddr = addr & ~(bytesPerBeat - 1L)
    val offsetInBeat = (addr - beatAddr).toInt
    val sliceShift = offsetInBeat * 8
    val sliceMask = BigInt("FFFFFFFF", 16) << sliceShift
    val current = memory.getOrElse(beatAddr, BigInt(0))
    val updated = (current & ~sliceMask) | ((BigInt(data) & BigInt("FFFFFFFF", 16)) << sliceShift)
    memory(beatAddr) = updated
    memory(addr) = BigInt(data)
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
class SimDataMoverResponder4x(
    cmdStream: Stream[Bits],
    stsStream: Stream[Bits],
    clockDomain: ClockDomain,
    addressWidth: Int = 32
) {
  case class ReceivedCmd(
      btt: Long,
      tag: Int,
      saddr: Long
  )
  val receivedCommands = mutable.ArrayBuffer[ReceivedCmd]()
  val stsQueue = mutable.Queue[(Int, Boolean)]()

  def start(): Unit = {
    stsStream.valid #= false
    stsStream.payload #= 0
    cmdStream.ready #= true

    fork {
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

          receivedCommands += ReceivedCmd(btt, tag, saddr)
          stsQueue.enqueue((tag, false))
        }
      }
    }

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
// Simulation Testbench for AxiVisibilityDma4x (4 Triangles per Clock)
// =============================================================================
object AxiVisibilityDma4xSim {
  def main(args: Array[String]): Unit = {
    val simConfig = SimConfig.allOptimisation.withIVerilog
    simConfig.compile(new AxiVisibilityDma4x(AxiVisibilityDma4xConfig(
      addressWidth = 32,
      axiDataWidth = 128,
      bttWidth = 23,
      gridWidth = 6,
      gridHeight = 5,
      vbWordBits = 32,
      triangleSizeBytes = 128,
      useAutoAdvance = true,
      trianglesPerCycle = 4
    ))).doSim { dut =>
      dut.clockDomain.forkStimulus(period = 10)

      val vbMemory = new AxiReadOnlyMemorySim4x(dut.io.m_axi_vb, dut.clockDomain)
      vbMemory.start()

      val dmResponder = new SimDataMoverResponder4x(
        dut.io.m_axis_datamover_cmd,
        dut.io.s_axis_datamover_sts,
        dut.clockDomain,
        addressWidth = 32
      )
      dmResponder.start()

      val axilDriver = new SimAxiLiteDriver(dut.io.s_axil_ctrl, dut.clockDomain)
      axilDriver.reset()

      dut.clockDomain.waitSampling(5)

      println("=================================================================")
      println("Test 1: Register Read/Write Check on AxiVisibilityDma4x")
      println("=================================================================")
      val vbBaseAddr = 0x10000000L
      val triBaseAddr = 0x80000000L

      axilDriver.write(0x08, vbBaseAddr)
      axilDriver.write(0x10, triBaseAddr)
      val rbVbLow = axilDriver.read(0x08)
      val rbTriLow = axilDriver.read(0x10)
      println(f"Readback VB_BASE: 0x$rbVbLow%08X, TRI_BASE: 0x$rbTriLow%08X")

      val N = 16
      axilDriver.write(0x18, N)
      axilDriver.write(0x1c, 0) // Tile 0

      // Pattern:
      // Tris 0..2:   1, 1, 1 (Run 1: 3 tris)
      // Tri 3:       0
      // Tris 4..5:   1, 1    (Run 2: 2 tris)
      // Tri 6:       0
      // Tris 7..11:  1, 1, 1, 1, 1 (Run 3: 5 tris)
      // Tris 12..14: 0, 0, 0
      // Tri 15:      1       (Run 4: 1 tri)
      val maskTile0 = Array(
        true, true, true, false,
        true, true, false, true,
        true, true, true, true,
        false, false, false, true
      )

      for (i <- 0 until N) {
        var wordVal: Long = 0
        if (maskTile0(i)) {
          wordVal |= 1L // bit 0 for tile 0
        }
        if (i < 2) {
          wordVal |= 2L // bit 1 for tile 1 (tris 0, 1)
        }
        vbMemory.setWord32(vbBaseAddr + (i * 4), wordVal)
      }

      println("=================================================================")
      println("Test 2: 4-Triangles-per-Clock Streaming on 16 Triangles (Tile 0)")
      println("=================================================================")
      axilDriver.write(0x00, 0x0d) // START=1, IRQ_EN=1, AUTO_ADV=1

      var timeout = 500
      var done = false
      while (!done && timeout > 0) {
        dut.clockDomain.waitSampling(5)
        val status = axilDriver.read(0x04)
        done = (status & 0x02) != 0
        timeout -= 1
      }

      assert(done, "Timeout waiting for DMA tile transfer completion!")
      println("Tile 0 transfer completed!")

      val visCount = axilDriver.read(0x24)
      val cmdCount = axilDriver.read(0x28)
      val stsCount = axilDriver.read(0x2c)
      println(f"STAT_VIS: $visCount, STAT_CMD: $cmdCount, STAT_STS: $stsCount")
      for ((cmd, idx) <- dmResponder.receivedCommands.zipWithIndex) {
        println(f"  Cmd $idx: Addr = 0x${cmd.saddr}%08X, BTT = ${cmd.btt} bytes (${cmd.btt / 128} triangles)")
      }

      assert(visCount == 11, f"Expected 11 visible triangles, got $visCount")
      assert(cmdCount == 4, f"Expected 4 DataMover commands, got $cmdCount")
      assert(stsCount == 4, f"Expected 4 DataMover status responses, got $stsCount")

      // Verify command lengths and addresses
      assert(dmResponder.receivedCommands(0).btt == 384, "Cmd 0 BTT mismatch (expected 384)")
      assert(dmResponder.receivedCommands(0).saddr == 0x80000000L, "Cmd 0 SADDR mismatch")

      assert(dmResponder.receivedCommands(1).btt == 256, "Cmd 1 BTT mismatch (expected 256)")
      assert(dmResponder.receivedCommands(1).saddr == 0x80000200L, "Cmd 1 SADDR mismatch")

      assert(dmResponder.receivedCommands(2).btt == 640, "Cmd 2 BTT mismatch (expected 640)")
      assert(dmResponder.receivedCommands(2).saddr == 0x80000380L, "Cmd 2 SADDR mismatch")

      assert(dmResponder.receivedCommands(3).btt == 128, "Cmd 3 BTT mismatch (expected 128)")
      assert(dmResponder.receivedCommands(3).saddr == 0x80000780L, "Cmd 3 SADDR mismatch")

      println("=================================================================")
      println("Test 3: Alternating 1010 Pattern (Stress Test Dual-Push Queue)")
      println("=================================================================")
      // Clear status
      axilDriver.write(0x04, 0x0a)
      dut.clockDomain.waitSampling(2)
      dmResponder.receivedCommands.clear()

      // Reset tile index back to 0 for stress test
      axilDriver.write(0x1c, 0) // TILE_INDEX = 0
      axilDriver.write(0x18, 8) // 8 triangles

      // Pattern: 1, 0, 1, 0,  0, 1, 0, 1
      val altMask = Array(true, false, true, false, false, true, false, true)
      for (i <- 0 until 8) {
        val wordVal: Long = if (altMask(i)) 1L else 0L
        vbMemory.setWord32(vbBaseAddr + (i * 4), wordVal)
      }

      axilDriver.write(0x00, 0x05) // START=1, IRQ_EN=1 (no auto-adv)

      timeout = 500
      done = false
      while (!done && timeout > 0) {
        dut.clockDomain.waitSampling(5)
        val status = axilDriver.read(0x04)
        done = (status & 0x02) != 0
        timeout -= 1
      }
      assert(done, "Timeout waiting for Alternating 1010 test!")
      println("Alternating 1010 test completed successfully!")

      val altVis = axilDriver.read(0x24)
      val altCmd = axilDriver.read(0x28)
      println(f"Alt STAT_VIS: $altVis (Expected 4), STAT_CMD: $altCmd (Expected 4)")
      assert(altVis == 4, f"Expected 4 visible, got $altVis")
      assert(altCmd == 4, f"Expected 4 single-triangle commands, got $altCmd")

      println("=================================================================")
      println("Test 4: Odd Triangle Count (N = 13)")
      println("=================================================================")
      axilDriver.write(0x04, 0x0a)
      dut.clockDomain.waitSampling(2)
      dmResponder.receivedCommands.clear()

      axilDriver.write(0x1c, 0) // TILE_INDEX = 0
      val oddN = 13
      axilDriver.write(0x18, oddN)
      // All 13 visible
      for (i <- 0 until oddN) {
        vbMemory.setWord32(vbBaseAddr + (i * 4), 1L)
      }
      // Word 13, 14, 15 have garbage bits in memory that MUST be masked out by hardware
      vbMemory.setWord32(vbBaseAddr + (13 * 4), 0xFFFFFFFFL)
      vbMemory.setWord32(vbBaseAddr + (14 * 4), 0xFFFFFFFFL)
      vbMemory.setWord32(vbBaseAddr + (15 * 4), 0xFFFFFFFFL)

      axilDriver.write(0x00, 0x05)

      timeout = 500
      done = false
      while (!done && timeout > 0) {
        dut.clockDomain.waitSampling(5)
        val status = axilDriver.read(0x04)
        done = (status & 0x02) != 0
        timeout -= 1
      }
      assert(done, "Timeout waiting for Odd test!")
      val oddVis = axilDriver.read(0x24)
      val oddCmd = axilDriver.read(0x28)
      println(f"Odd N=13 STAT_VIS: $oddVis (Expected 13), STAT_CMD: $oddCmd (Expected 1)")
      assert(oddVis == 13, f"Expected 13 visible triangles, got $oddVis (padding was not masked!)")
      assert(oddCmd == 1, f"Expected 1 merged command of 13 triangles, got $oddCmd")
      assert(dmResponder.receivedCommands(0).btt == 13 * 128, f"Expected BTT ${13*128}, got ${dmResponder.receivedCommands(0).btt}")

      println("=================================================================")
      println("ALL AXI VISIBILITY DMA 4X TESTS PASSED SUCCESSFULLY!")
      println("=================================================================")
    }
  }
}
