package projectname

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.amba4.axi._
import spinal.lib.bus.misc.SizeMapping

case class Axi4ToApbXbar(
    addressWidth: Int = 16,
    dataWidth: Int = 32,
    idWidth: Int = 4,
    axiPipelineStages: Int = 1,
    apbPipelineStages: Int = 1
) extends Component {
  require(axiPipelineStages >= 0)
  require(apbPipelineStages >= 0)

  val apbBridge = Axi4SharedToApb3Bridge(
    addressWidth = addressWidth,
    dataWidth = dataWidth,
    idWidth = idWidth
  )

  val (axiConfig, apbConfig) = (apbBridge.axiConfig, apbBridge.apbConfig)

  // The LUT exposes 1 KiB; each thread register block exposes 128 bytes.
  val lutApbConfig = apbConfig.copy(addressWidth = 10)
  val threadApbConfig = apbConfig.copy(addressWidth = 7)

  val io = new Bundle {
    val m_axi = slave(Axi4(axiConfig))
    val lut = master(Apb3(lutApbConfig))
    val threadRegs = Vec(master(Apb3(threadApbConfig)), 4)
  }

  noIoPrefix()

  val axiPipeline = Axi4Pipeline(axiConfig, axiPipelineStages)
  axiPipeline.io.input <> io.m_axi

  val axiShared = Axi4ToAxi4Shared(axiPipeline.io.output)
  apbBridge.io.axi <> axiShared

  val lutSlice = Apb3RegSlice(lutApbConfig, apbPipelineStages)
  val threadSlices = Array.fill(4)(Apb3RegSlice(threadApbConfig, apbPipelineStages))
  lutSlice.io.m >> io.lut
  for (i <- 0 until 4) threadSlices(i).io.m >> io.threadRegs(i)

  // 1 KiB LUT followed by four 128-byte thread-register windows.
  val decoder = Apb3Decoder(
    master = apbBridge.io.apb,
    slaves = List(
      lutSlice.io.s -> SizeMapping(0x0000, 0x400),
      threadSlices(0).io.s -> SizeMapping(0x1000, 0x080),
      threadSlices(1).io.s -> SizeMapping(0x1080, 0x080),
      threadSlices(2).io.s -> SizeMapping(0x1100, 0x080),
      threadSlices(3).io.s -> SizeMapping(0x1180, 0x080)
    )
  )
}

object Axi4ToApbXbarVerilog  extends App {
  SpinalConfig(
    targetDirectory = "rtl/Axi4ToApbXbar",
    defaultConfigForClockDomains = ClockDomainConfig(
      resetActiveLevel = LOW,
      resetKind = SYNC
    )
  ).withPrivateNamespace.generateVerilog(Axi4ToApbXbar())
}
