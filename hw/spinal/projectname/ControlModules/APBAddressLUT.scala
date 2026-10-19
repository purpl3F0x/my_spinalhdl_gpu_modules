package projectname

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.misc.SizeMapping

class ApbLutRam(portCount: Int = 4) extends Component {
  val wordCount = 128
  val baseRegion = SizeMapping(0, wordCount * 4)
  val sizeRegion = SizeMapping(wordCount * 4, wordCount * 4)
  val totalWords = wordCount * 2
  val byteCount = totalWords * 4

  val s_apb = slave(Apb3(Apb3Config(addressWidth = log2Up(byteCount), dataWidth = 32)))

  val lut = Vec.fill(portCount) {
    new Bundle {
      val data_addr = in UInt (log2Up(wordCount) bits)
      val base = out Bits (32 bits)
      val size = out Bits (4 bits)
    }
  }

  val baseRams = Array.fill(portCount)(Mem(Bits(32 bits), wordCount))
  val sizeRams = Array.fill(portCount)(Mem(Bits(4 bits), wordCount))
  for (i <- 0 until portCount) {
    baseRams(i).setTechnology(distributedLut)
    sizeRams(i).setTechnology(distributedLut)
  }

  val baseHit = baseRegion.hit(s_apb.PADDR)
  val sizeHit = sizeRegion.hit(s_apb.PADDR)
  val apbWordAddr = (s_apb.PADDR >> 2).resize(log2Up(wordCount) bits)

  val apbWrite =
    s_apb.PSEL(0) &&
      s_apb.PENABLE &&
      s_apb.PWRITE &&
      !ClockDomain.current.isResetActive

  for (i <- 0 until portCount) {
    baseRams(i).write(
      enable = apbWrite && baseHit,
      address = apbWordAddr,
      data = s_apb.PWDATA
    )

    sizeRams(i).write(
      enable = apbWrite && sizeHit,
      address = apbWordAddr,
      data = s_apb.PWDATA(3 downto 0)
    )
  }

  for (i <- 0 until portCount) {
    lut(i).base := baseRams(i).readAsync(lut(i).data_addr)
    lut(i).size := sizeRams(i).readAsync(lut(i).data_addr)
  }

  // APB read: same word-index, returning base or size depending on the selected region.
  val baseReadData = baseRams(0).readAsync(apbWordAddr).resize(32 bits)
  val sizeReadData = sizeRams(0).readAsync(apbWordAddr).resize(32 bits)
  val apbReadData = Mux(baseHit, baseReadData, sizeReadData)

  s_apb.PREADY := True
  s_apb.PSLVERROR := False
  s_apb.PRDATA := apbReadData
}

// Generate Verilog
object ApbLutRamVerilog extends App {
  SpinalConfig(
    targetDirectory = "rtl/ApbLutRam",
    defaultConfigForClockDomains = ClockDomainConfig(
      resetActiveLevel = LOW,
      resetKind = SYNC
    )
  ).withPrivateNamespace.generateVerilog(new ApbLutRam())
}