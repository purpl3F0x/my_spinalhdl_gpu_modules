package projectname

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.amba4.axi._
import spinal.lib.bus.misc.SizeMapping

// Address map:
//   0x000 - 0x0FF  LUT (0x000 texture base, 0x080 texture size)
//   0x100 - 0x1FF  Reserved
//   0x200 - 0x20F  Frontend config
//   0x210 - 0x24F  Thread[0..3] regs, 16 B each
case class Axi4ToApbXbar(
    addressWidth: Int = 16,
    dataWidth: Int = 32,
    idWidth: Int = 4,
    inputFifoDepth: Int = 32,
    apbPipelineStages: Int = 1
) extends Component {
  require(inputFifoDepth >= 0)
  require(apbPipelineStages >= 0)

  val apbBridge = Axi4SharedToApb3Bridge(
    addressWidth = addressWidth,
    dataWidth = dataWidth,
    idWidth = idWidth
  )

  val (axiConfig, apbConfig) = (apbBridge.axiConfig, apbBridge.apbConfig)

  val lutApbConfig = apbConfig.copy(addressWidth = 8)
  val frontendApbConfig = apbConfig.copy(addressWidth = 4)
  val threadApbConfig = apbConfig.copy(addressWidth = 4)

  val io = new Bundle {
    val m_axi = slave(Axi4(axiConfig))
    val lut = master(Apb3(lutApbConfig))
    val frontend = master(Apb3(frontendApbConfig))
    val threadRegs = Vec(master(Apb3(threadApbConfig)), 4)
  }

  noIoPrefix()

  // Request-side FIFOs absorb bursts of CPU accesses; the APB side drains them one at a time.
  val axiBuffered = Axi4(axiConfig)
  axiBuffered.ar << io.m_axi.ar.queue(inputFifoDepth)
  axiBuffered.aw << io.m_axi.aw.queue(inputFifoDepth)
  axiBuffered.w << io.m_axi.w.queue(inputFifoDepth)
  io.m_axi.r << axiBuffered.r
  io.m_axi.b << axiBuffered.b

  val axiShared = Axi4ToAxi4Shared(axiBuffered)
  apbBridge.io.axi <> axiShared

  val lutSlice = Apb3RegSlice(lutApbConfig, apbPipelineStages)
  val frontendSlice = Apb3RegSlice(frontendApbConfig, apbPipelineStages)
  val threadSlices = Array.fill(4)(Apb3RegSlice(threadApbConfig, apbPipelineStages))
  lutSlice.io.m >> io.lut
  frontendSlice.io.m >> io.frontend
  for (i <- 0 until 4) threadSlices(i).io.m >> io.threadRegs(i)

  // (name, slave port, mapping); also used to emit the JSON address map
  val addressMap = List(
    ("lut", lutSlice.io.s, SizeMapping(0x000, 0x100)),
    ("frontend", frontendSlice.io.s, SizeMapping(0x200, 0x010))
  ) ++ (0 until 4).map(i => (s"thread$i", threadSlices(i).io.s, SizeMapping(0x210 + i * 0x10, 0x010)))

  val decoder = Apb3Decoder(
    master = apbBridge.io.apb,
    slaves = addressMap.map { case (_, port, mapping) => port -> mapping }
  )

  def addressMapJson: String = {
    val entries = addressMap.map { case (name, _, mapping) =>
      f"""    {"name": "$name", "base": "0x${mapping.base.toLong}%03X", "size": ${mapping.size.toLong}}"""
    }
    entries.mkString("{\n  \"regions\": [\n", ",\n", "\n  ]\n}\n")
  }
}

object Axi4ToApbXbarVerilog  extends App {
  val targetDirectory = "rtl/Axi4ToApbXbar"
  val report = SpinalConfig(
    targetDirectory = targetDirectory,
    defaultConfigForClockDomains = ClockDomainConfig(
      resetActiveLevel = LOW,
      resetKind = SYNC
    )
  ).withPrivateNamespace.generateVerilog(Axi4ToApbXbar())

  java.nio.file.Files.writeString(
    java.nio.file.Paths.get(targetDirectory, "address_map.json"),
    report.toplevel.addressMapJson
  )
}
