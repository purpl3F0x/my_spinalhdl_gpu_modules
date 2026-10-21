package projectname

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.amba4.axi._
import spinal.lib.bus.misc.SizeMapping

// Address map:
//   0x000 - 0x0FF  LUT (0x000 texture base, 0x080 texture size)
//   0x100 - 0x7FF  Reserved
//   0x800 - 0x80F  Frontend config
//   0x810 - 0x84F  Thread[0..3] regs, 16 B each
//   0xF00 - 0xF7F  Tile DMA
//   0xF80 - 0xFFF  Visibility DMA
case class Axi4ToApbXbar(
    masters: Int = 2,
    addressWidth: Int = 12,
    dataWidth: Int = 32,
    idWidth: Int = 4,
    inputFifoDepth: Int = 32,
    apbPipelineStages: Int = 1
) extends Component {
  require(masters >= 1)
  require(inputFifoDepth >= 0)
  require(apbPipelineStages >= 0)

  // The arbiter extends IDs with the master index to route responses back
  val apbBridge = Axi4SharedToApb3Bridge(
    addressWidth = addressWidth,
    dataWidth = dataWidth,
    idWidth = idWidth + log2Up(masters)
  )

  val axiConfig = apbBridge.axiConfig.copy(idWidth = idWidth)
  val apbConfig = apbBridge.apbConfig

  val lutApbConfig = apbConfig.copy(addressWidth = 8)
  val frontendApbConfig = apbConfig.copy(addressWidth = 4)
  val threadApbConfig = apbConfig.copy(addressWidth = 4)
  val dmaApbConfig = apbConfig.copy(addressWidth = 7)

  val io = new Bundle {
    val m_axi = Vec(slave(Axi4(axiConfig)), masters)
    val lut = master(Apb3(lutApbConfig))
    val frontend = master(Apb3(frontendApbConfig))
    val threadRegs = Vec(master(Apb3(threadApbConfig)), 4)
    val tileDma = master(Apb3(dmaApbConfig))
    val visDma = master(Apb3(dmaApbConfig))
  }

  noIoPrefix()

  // Vivado AXI naming (<prefix>_awaddr, ...) and aximm interface attributes on each master port
  for ((m, i) <- io.m_axi.zipWithIndex) {
    val prefix = s"m_axi_$i"
    m.flatten.foreach { signal =>
      val port = signal.getName().replace(s"${prefix}_", "").replace("_payload_", "").replace("_", "")
      signal.setName(s"${prefix}_$port")
      signal.addAttribute("X_INTERFACE_INFO", s"xilinx.com:interface:aximm:1.0 $prefix ${port.toUpperCase}")
    }
    m.ar.valid.addAttribute("X_INTERFACE_MODE", "slave")
    m.ar.valid.addAttribute(
      "X_INTERFACE_PARAMETER",
      s"PROTOCOL AXI4, READ_WRITE_MODE READ_WRITE, ADDR_WIDTH $addressWidth, DATA_WIDTH $dataWidth, ID_WIDTH $idWidth, " +
        s"NUM_READ_OUTSTANDING $inputFifoDepth, NUM_WRITE_OUTSTANDING $inputFifoDepth, MAX_BURST_LENGTH 1"
    )
  }

  // Request-side FIFOs absorb bursts of accesses per master; the APB side drains them one at a time.
  val arbiter = Axi4SharedArbiter(
    outputConfig = apbBridge.axiConfig,
    readInputsCount = 0,
    writeInputsCount = 0,
    sharedInputsCount = masters,
    routeBufferSize = 4
  )

  for ((m, i) <- io.m_axi.zipWithIndex) {
    val axiBuffered = Axi4(axiConfig)
    axiBuffered.ar << m.ar.queue(inputFifoDepth)
    axiBuffered.aw << m.aw.queue(inputFifoDepth)
    axiBuffered.w << m.w.queue(inputFifoDepth)
    m.r << axiBuffered.r
    m.b << axiBuffered.b
    arbiter.io.sharedInputs(i) <> Axi4ToAxi4Shared(axiBuffered)
  }

  // Round-robin between masters
  apbBridge.io.axi <> arbiter.io.output

  val lutSlice = Apb3RegSlice(lutApbConfig, apbPipelineStages)
  val frontendSlice = Apb3RegSlice(frontendApbConfig, apbPipelineStages)
  val threadSlices = Array.fill(4)(Apb3RegSlice(threadApbConfig, apbPipelineStages))
  lutSlice.io.m >> io.lut
  frontendSlice.io.m >> io.frontend
  for (i <- 0 until 4) threadSlices(i).io.m >> io.threadRegs(i)
  val tileDmaSlice = Apb3RegSlice(dmaApbConfig, apbPipelineStages)
  val visDmaSlice = Apb3RegSlice(dmaApbConfig, apbPipelineStages)
  tileDmaSlice.io.m >> io.tileDma
  visDmaSlice.io.m >> io.visDma

  // (name, slave port, mapping); also used to emit the JSON address map
  val addressMap = List(
    ("lut", lutSlice.io.s, SizeMapping(0x000, 0x100)),
    ("frontend", frontendSlice.io.s, SizeMapping(0x800, 0x010))
  ) ++ (0 until 4).map(i => (s"thread$i", threadSlices(i).io.s, SizeMapping(0x810 + i * 0x10, 0x010))) ++ List(
    ("tileDma", tileDmaSlice.io.s, SizeMapping(0xf00, 0x080)),
    ("visDma", visDmaSlice.io.s, SizeMapping(0xf80, 0x080))
  )

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
