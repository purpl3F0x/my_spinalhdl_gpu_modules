package projectname

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba4.axi._
import spinal.lib.bus.amba4.axilite._
import spinal.lib.bus.regif._
import spinal.lib.bus.regif.AccessType._
import spinal.lib.fsm._

// =============================================================================
// Configuration for AxiVisibilityDma4x
// Optimized for 30 bins (32-bit visibility words) and 4 triangles per clock
// =============================================================================
case class AxiVisibilityDma4xConfig(
    addressWidth: Int = 32, // Address width (32 up to 64 bits)
    axiDataWidth: Int = 128, // AXI4 Read Master data width (128 bits = 4x32-bit visibility words)
    bttWidth: Int = 23, // DataMover BTT (Bytes To Transfer) width (up to 23 bits)
    gridWidth: Int = 6, // Visibility grid width in tiles (default: 6)
    gridHeight: Int = 5, // Visibility grid height in tiles (default: 5, total: 30 tiles)
    vbWordBits: Int = 32, // Visibility buffer word width (32 bits: 30 bits grid + 2 bits padding)
    triangleSizeBytes: Int = 128, // Triangle data footprint in bytes
    maxBurstLength: Int = 256, // Max AXI4 Read burst beats (1..256)
    maxOutstandingReads: Int = 8, // Max outstanding AXI4 Read address transactions
    cmdFifoDepth: Int = 64, // Depth of DataMover command FIFO
    stsFifoDepth: Int = 4, // Depth of DataMover status FIFO
    vbFifoDepth: Int = 256, // Depth of visibility read decoupling FIFO
    useAutoAdvance: Boolean = true, // Support automatic tile index advancing (0..29)
    trianglesPerCycle: Int = 4 // Process 4 triangles per clock cycle (1.2 GTris/s @ 300MHz)
) {
  val gridTiles: Int = gridWidth * gridHeight
  require(gridTiles <= vbWordBits, s"gridTiles ($gridTiles) cannot exceed vbWordBits ($vbWordBits)")
  require(axiDataWidth >= vbWordBits, s"axiDataWidth ($axiDataWidth) must be >= vbWordBits ($vbWordBits)")
  require(isPow2(axiDataWidth), "axiDataWidth must be a power of 2 (e.g. 128, 256)")
  require(
    axiDataWidth % vbWordBits == 0,
    s"axiDataWidth ($axiDataWidth) must be a multiple of vbWordBits ($vbWordBits)"
  )
  require(addressWidth >= 32 && addressWidth <= 64, "addressWidth must be between 32 and 64 bits")
  require(isPow2(triangleSizeBytes), "triangleSizeBytes must be a power of 2 (e.g. 128)")
  require(maxBurstLength >= 1 && maxBurstLength <= 256, "maxBurstLength must be between 1 and 256 beats")
  require(cmdFifoDepth >= 4 && isPow2(cmdFifoDepth), "cmdFifoDepth must be a power of 2 >= 4")
  require(stsFifoDepth >= 4 && isPow2(stsFifoDepth), "stsFifoDepth must be a power of 2 >= 4")
  require(vbFifoDepth >= 4 && isPow2(vbFifoDepth), "vbFifoDepth must be a power of 2 >= 4")
  require(
    vbFifoDepth >= maxBurstLength,
    s"vbFifoDepth ($vbFifoDepth) must be >= maxBurstLength ($maxBurstLength) to guarantee full burst absorption"
  )
  require(
    bttWidth >= 8 && bttWidth <= 23,
    s"bttWidth ($bttWidth) must be between 8 and 23 bits (Xilinx DataMover limit)"
  )

  val wordsPerBeat: Int = axiDataWidth / vbWordBits // 128 / 32 = 4 triangles per beat
  val triangleShift: Int = log2Up(triangleSizeBytes) // 7 bits for 128 bytes
  val triWordAddrWidth: Int = addressWidth - triangleShift // Address truncated by 7 bits
  val axiBytesPerBeat: Int = axiDataWidth / 8 // 16 bytes for 128-bit
  val axiBeatShift: Int = log2Up(axiBytesPerBeat) // 4 bits for 16 bytes
  val vbBytesPerBeat: Int = axiBytesPerBeat
  val vbBeatShift: Int = axiBeatShift
  val vbWordAddrWidth: Int = addressWidth - axiBeatShift

  val bttTriCountWidth: Int = bttWidth - triangleShift // 23 - 7 = 16 bits
  val maxBttTriangles: BigInt = (BigInt(1) << bttTriCountWidth) - 1

  val cmdWidth: Int = if (addressWidth <= 32) 72 else 104
  val stsWidth: Int = 8

  def getAxi4ReadConfig: Axi4Config = Axi4Config(
    addressWidth = addressWidth,
    dataWidth = axiDataWidth,
    idWidth = 4,
    useBurst = true,
    useSize = true,
    useLen = true,
    useLast = true,
    useResp = true,
    useId = true
  )

  def getAxiLiteConfig: AxiLite4Config = AxiLite4Config(
    addressWidth = 8,
    dataWidth = 32
  )
}

// =============================================================================
// Multi-Push Register FIFO (Up to 4 concurrent writes in 1 clock cycle)
// =============================================================================
case class MultiPushFifo4x[T <: Data](dataType: HardType[T], depth: Int = 16, maxPush: Int = 4) extends Component {
  require(isPow2(depth) && depth >= maxPush, "depth must be a power of 2 >= maxPush")

  val io = new Bundle {
    val pushValids   = in Vec(Bool(), maxPush)
    val pushPayloads = in Vec(dataType(), maxPush)
    val pop          = master(Stream(dataType()))
    val freeSpace    = out UInt(log2Up(depth + 1) bits)
    val occupancy    = out UInt(log2Up(depth + 1) bits)
  }

  val mem    = Vec(Reg(dataType()), depth)
  val wrPtr  = Reg(UInt(log2Up(depth) bits)) init (0)
  val rdPtr  = Reg(UInt(log2Up(depth) bits)) init (0)
  val occReg = Reg(UInt(log2Up(depth + 1) bits)) init (0)

  val free = depth - occReg
  io.freeSpace := free
  io.occupancy := occReg

  val pushOffsets = Vec(UInt(log2Up(maxPush + 1) bits), maxPush)
  var runningCount: UInt = U(0, log2Up(maxPush + 1) bits)

  for (i <- 0 until maxPush) {
    pushOffsets(i) := runningCount
    runningCount = runningCount + io.pushValids(i).asUInt
  }
  val totalPushed = runningCount

  for (i <- 0 until maxPush) {
    when(io.pushValids(i)) {
      val targetAddr = (wrPtr + pushOffsets(i)).resize(log2Up(depth))
      mem(targetAddr) := io.pushPayloads(i)
    }
  }

  val popFire = io.pop.fire
  wrPtr := wrPtr + totalPushed.resized
  when(popFire) { rdPtr := rdPtr + 1 }

  occReg := occReg + totalPushed.resized - popFire.asUInt

  io.pop.valid   := occReg > 0
  io.pop.payload := mem(rdPtr)
}

// =============================================================================
// AxiVisibilityDma4x (4 Triangles per Clock, 4x32-bit Visibility Words)
// =============================================================================
class AxiVisibilityDma4x(
    val config: AxiVisibilityDma4xConfig = AxiVisibilityDma4xConfig(),
    val dmClockDomain: ClockDomain = null
) extends Component {

  val dmClock = if (dmClockDomain != null) dmClockDomain else ClockDomain.current
  val isDualClock = (dmClockDomain != null) && (dmClockDomain != ClockDomain.current)

  val io = new Bundle {
    // 1. AXI-Lite Slave Interface for Register Programming (aclk domain)
    val s_axil_ctrl = slave(AxiLite4(config.getAxiLiteConfig))

    // 2. AXI4 Read Master Interface for Visibility Buffer (aclk domain)
    val m_axi_vb = master(Axi4ReadOnly(config.getAxi4ReadConfig))

    // 3. AXI-Stream Master for DataMover Commands (dmClock domain)
    val m_axis_datamover_cmd = master(Stream(Bits(config.cmdWidth bits)))

    // 4. AXI-Stream Slave for DataMover Status (dmClock domain)
    val s_axis_datamover_sts = slave(Stream(Bits(config.stsWidth bits)))

    // 5. Discrete Hardware Status & Interrupt Pins
    val ready = out Bool()
    val irq = out Bool()
    val irq_done = out Bool()
    val irq_error = out Bool()
    val irq_frame_done = out Bool()
  }

  noIoPrefix()

  def renameAxiForVivado(bundle: Bundle, prefix: String): Unit = {
    bundle.flatten.foreach { signal =>
      val name = signal.getName()
      val cleanName = name
        .replaceFirst("^io_", "")
        .replace(s"${prefix}_", "")
        .replace("_payload_", "")
        .replace("_payload", "")
        .replace("_", "")

      signal.setName(s"${prefix}_${cleanName}")
    }
  }

  def renameAxisForVivado[T <: Data](stream: Stream[T], prefix: String): Unit = {
    stream.valid.setName(s"${prefix}_tvalid")
    stream.ready.setName(s"${prefix}_tready")
    stream.payload.setName(s"${prefix}_tdata")

    stream.valid.addAttribute("X_INTERFACE_INFO", s"xilinx.com:interface:axis:1.0 ${prefix} TVALID")
    stream.ready.addAttribute("X_INTERFACE_INFO", s"xilinx.com:interface:axis:1.0 ${prefix} TREADY")
    stream.payload.addAttribute("X_INTERFACE_INFO", s"xilinx.com:interface:axis:1.0 ${prefix} TDATA")
  }

  renameAxiForVivado(io.s_axil_ctrl, "s_axil_ctrl")
  renameAxiForVivado(io.m_axi_vb, "m_axi_vb")
  renameAxisForVivado(io.m_axis_datamover_cmd, "m_axis_datamover_cmd")
  renameAxisForVivado(io.s_axis_datamover_sts, "s_axis_datamover_sts")

  io.s_axil_ctrl.flatten.foreach { sig =>
    val name = sig.getName()
    val suffix = name.replace("s_axil_ctrl_", "").toUpperCase
    sig.addAttribute("X_INTERFACE_INFO", s"xilinx.com:interface:aximm:1.0 s_axil_ctrl $suffix")
  }
  io.s_axil_ctrl.ar.valid.addAttribute(
    "X_INTERFACE_PARAMETER",
    "READ_WRITE_MODE READ_WRITE, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1"
  )

  io.m_axi_vb.flatten.foreach { sig =>
    val name = sig.getName()
    val suffix = name.replace("m_axi_vb_", "").toUpperCase
    sig.addAttribute("X_INTERFACE_INFO", s"xilinx.com:interface:aximm:1.0 m_axi_vb $suffix")
  }
  io.m_axi_vb.ar.valid.addAttribute(
    "X_INTERFACE_PARAMETER",
    s"NUM_READ_OUTSTANDING ${config.maxOutstandingReads}, READ_WRITE_MODE READ_ONLY"
  )

  io.irq.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:interrupt:1.0 irq INTERRUPT")
  io.irq.addAttribute("X_INTERFACE_PARAMETER", "SENSITIVITY LEVEL_HIGH")
  io.irq_done.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:interrupt:1.0 irq_done INTERRUPT")
  io.irq_done.addAttribute("X_INTERFACE_PARAMETER", "SENSITIVITY LEVEL_HIGH")
  io.irq_error.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:interrupt:1.0 irq_error INTERRUPT")
  io.irq_error.addAttribute("X_INTERFACE_PARAMETER", "SENSITIVITY LEVEL_HIGH")
  io.irq_frame_done.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:interrupt:1.0 irq_frame_done INTERRUPT")
  io.irq_frame_done.addAttribute("X_INTERFACE_PARAMETER", "SENSITIVITY LEVEL_HIGH")

  clockDomain.clock.setName("aclk")
  clockDomain.reset.setName("aresetn")
  clockDomain.clock.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:clock:1.0 aclk CLK")
  clockDomain.clock.addAttribute(
    "X_INTERFACE_PARAMETER",
    if (isDualClock) {
      "ASSOCIATED_BUSIF s_axil_ctrl:m_axi_vb, ASSOCIATED_RESET aresetn"
    } else {
      "ASSOCIATED_BUSIF s_axil_ctrl:m_axi_vb:m_axis_datamover_cmd:s_axis_datamover_sts, ASSOCIATED_RESET aresetn"
    }
  )
  clockDomain.reset.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:reset:1.0 aresetn RST")
  clockDomain.reset.addAttribute("X_INTERFACE_PARAMETER", "POLARITY ACTIVE_LOW")

  if (isDualClock) {
    dmClock.clock.setName("dm_clk")
    dmClock.reset.setName("dm_resetn")
    dmClock.clock.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:clock:1.0 dm_clk CLK")
    dmClock.clock.addAttribute(
      "X_INTERFACE_PARAMETER",
      "ASSOCIATED_BUSIF m_axis_datamover_cmd:s_axis_datamover_sts, ASSOCIATED_RESET dm_resetn"
    )
    dmClock.reset.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:reset:1.0 dm_resetn RST")
    dmClock.reset.addAttribute("X_INTERFACE_PARAMETER", "POLARITY ACTIVE_LOW")
  }

  // ===========================================================================
  // 1. AXI-LITE REGISTER INTERFACE (via SpinalHDL regif)
  // ===========================================================================
  val busif = AxiLite4BusInterface(io.s_axil_ctrl, (0x0000, 256), "AXI_VISIBILITY_DMA4X")

  val dmaBusy      = RegInit(False)
  val axiReadError = RegInit(False)
  val dmErrorFlag  = RegInit(False)
  val dmLastStsReg = Reg(Bits(8 bits)) init (0)
  val cycleCounter = Reg(UInt(32 bits)) init (0)
  val statVisCount = Reg(UInt(32 bits)) init (0)
  val statCmdCount = Reg(UInt(32 bits)) init (0)
  val statStsCount = Reg(UInt(32 bits)) init (0)

  // 0x00: CONTROL REGISTER
  val REG_CTRL = busif.newRegAt(address = 0x00, doc = "DMA Control Register")
  val START = REG_CTRL.field(Bool(), W1P, doc = "Trigger visibility scan and command generation")
  val SOFT_RESET = REG_CTRL.field(Bool(), W1P, doc = "Software reset for engine and internal FIFOs")
  val IRQ_ENABLE = REG_CTRL.field(Bool(), RW, resetValue = 1, doc = "Global Interrupt Enable (1: enabled)")
  val AUTO_ADV_EN =
    if (config.useAutoAdvance) REG_CTRL.field(Bool(), RW, resetValue = 1, doc = "Enable auto-advancing tile index")
    else null

  // 0x04: STATUS REGISTER
  val REG_STATUS = busif.newRegAt(address = 0x04, doc = "DMA Status Register")
  val BUSY = REG_STATUS.field(Bool(), RO, doc = "Engine active busy status")
  val DONE = REG_STATUS.field(Bool(), W1C, doc = "Tile processing complete (Write 1 to clear)")
  val ERROR = REG_STATUS.field(Bool(), W1C, doc = "Error detected on AXI read or DataMover (Write 1 to clear)")
  val IRQ_STATUS = REG_STATUS.field(Bool(), W1C, doc = "Interrupt pending (Write 1 to clear)")
  val FRAME_DONE =
    if (config.useAutoAdvance)
      REG_STATUS.field(Bool(), W1C, doc = s"All ${config.gridTiles} tiles completed / wrapped (Write 1 to clear)")
    else null
  val AXI_ERR_FLAG = REG_STATUS.field(Bool(), RO, doc = "AXI Read Master Bus Error")
  val DM_ERR_FLAG = REG_STATUS.field(Bool(), RO, doc = "DataMover Status Error")

  BUSY := dmaBusy
  AXI_ERR_FLAG := axiReadError
  DM_ERR_FLAG := dmErrorFlag

  // 0x08: VB_BASE_ADDR_LOW (16-byte aligned for 128-bit)
  val REG_VB_LOW = busif.newRegAt(address = 0x08, doc = "Visibility Buffer Base Address [31:0] (16-byte aligned)")
  REG_VB_LOW.reserved(config.vbBeatShift bits)
  val VB_BASE_LOW = REG_VB_LOW.field(
    UInt((32 - config.vbBeatShift) bits),
    RW,
    resetValue = 0,
    doc = "Visibility Buffer Base Address [31:4]"
  )

  // 0x0C: VB_BASE_ADDR_HIGH
  val REG_VB_HIGH = busif.newRegAt(address = 0x0c, doc = "Visibility Buffer Base Address [63:32]")
  val VB_BASE_HIGH =
    REG_VB_HIGH.field(UInt(32 bits), RW, resetValue = 0, doc = "Upper 32 bits of Visibility Buffer Base Address")

  // 0x10: TRI_BASE_ADDR_LOW (128-byte aligned)
  val REG_TRI_LOW = busif.newRegAt(address = 0x10, doc = "Triangle Geometry Base Address [31:0] (128-byte aligned)")
  REG_TRI_LOW.reserved(config.triangleShift bits)
  val TRI_BASE_LOW = REG_TRI_LOW.field(
    UInt((32 - config.triangleShift) bits),
    RW,
    resetValue = 0,
    doc = "Triangle Geometry Base Address [31:7]"
  )

  // 0x14: TRI_BASE_ADDR_HIGH
  val REG_TRI_HIGH = busif.newRegAt(address = 0x14, doc = "Triangle Geometry Base Address [63:32]")
  val TRI_BASE_HIGH =
    REG_TRI_HIGH.field(UInt(32 bits), RW, resetValue = 0, doc = "Upper 32 bits of Triangle Geometry Base Address")

  // 0x18: TRI_COUNT (N)
  val REG_TRI_COUNT = busif.newRegAt(address = 0x18, doc = "Triangle Count (N elements)")
  val TRI_COUNT = REG_TRI_COUNT.field(
    UInt(32 bits),
    RW,
    resetValue = 0,
    doc = "Total number of triangle visibility elements to evaluate"
  )

  // 0x1C: TILE_INDEX
  val REG_TILE_IDX = busif.newRegAt(address = 0x1c, doc = s"Active Tile Index (0..${config.gridTiles - 1})")
  val TILE_INDEX = REG_TILE_IDX.field(
    UInt(log2Up(config.gridTiles) bits),
    RW,
    resetValue = 0,
    doc = s"Tile selection bit index (0..${config.gridTiles - 1}) in 32-bit visibility word"
  )

  // 0x20: BTT_LIMIT (128-byte aligned)
  val REG_BTT_LIMIT = busif.newRegAt(address = 0x20, doc = "BTT Byte Limit per Command (128-byte aligned)")
  REG_BTT_LIMIT.reserved(config.triangleShift bits)
  val BTT_LIMIT_TRI = REG_BTT_LIMIT.field(
    UInt(config.bttTriCountWidth bits),
    RW,
    resetValue = config.maxBttTriangles,
    doc = "BTT triangle count limit [bttWidth-1 : 7]"
  )

  // 0x24: STAT_VIS_COUNT
  val REG_STAT_VIS = busif.newRegAt(address = 0x24, doc = "Visible Triangle Counter")
  val STAT_VIS = REG_STAT_VIS.field(UInt(32 bits), RO, doc = "Total visible triangles encountered in current tile")
  STAT_VIS := statVisCount

  // 0x28: STAT_CMD_COUNT
  val REG_STAT_CMD = busif.newRegAt(address = 0x28, doc = "DataMover Commands Issued Counter")
  val STAT_CMD = REG_STAT_CMD.field(UInt(32 bits), RO, doc = "Total DataMover command packets sent")
  STAT_CMD := statCmdCount

  // 0x2C: STAT_STS_COUNT
  val REG_STAT_STS = busif.newRegAt(address = 0x2c, doc = "DataMover Status Received Counter")
  val STAT_STS = REG_STAT_STS.field(UInt(32 bits), RO, doc = "Total DataMover status responses received")
  STAT_STS := statStsCount

  // 0x30: DM_LAST_STS
  val REG_DM_LAST = busif.newRegAt(address = 0x30, doc = "Last DataMover Status Byte")
  val DM_LAST_STS = REG_DM_LAST.field(Bits(8 bits), RO, doc = "Last received raw status byte from DataMover")
  DM_LAST_STS := dmLastStsReg

  // 0x34: PERF_CYCLES
  val REG_PERF = busif.newRegAt(address = 0x34, doc = "Performance Cycle Counter")
  val PERF_CYCLES = REG_PERF.field(UInt(32 bits), RO, doc = "Elapsed clock cycles during active tile scan")
  PERF_CYCLES := cycleCounter

  // 0x40 - 0x4C: INTERRUPT FACTORY
  val intrDone = False
  val intrError = False
  val intrFrameDone = False

  val INTR = busif.newIntrRFMS4(addr = 0x40, doc = "DMA Interrupt Controller")
  val irqDoneStat =
    INTR.fieldAt(0, intrDone, maskRstVal = 0, doc = "Tile done interrupt (1: unmasked/open)")(SymbolName("done"))
  val irqErrorStat =
    INTR.fieldAt(1, intrError, maskRstVal = 0, doc = "Error interrupt (1: unmasked/open)")(SymbolName("error"))
  val irqFrameDoneStat = if (config.useAutoAdvance) {
    INTR.fieldAt(2, intrFrameDone, maskRstVal = 0, doc = "Frame done interrupt (1: unmasked/open)")(
      SymbolName("frame_done")
    )
  } else null

  val irqMerged = INTR.intr()

  busif.accept(HtmlGenerator("axi_visibility_dma4x_regmap", "AXI_VISIBILITY_DMA4X"))
  busif.accept(CHeaderGenerator("axi_visibility_dma4x_regmap", "AXI_VISIBILITY_DMA4X"))
  busif.accept(JsonGenerator("axi_visibility_dma4x_regmap"))

  val fullVbWordAddr = UInt(config.vbWordAddrWidth bits)
  if (config.addressWidth > 32) {
    fullVbWordAddr := (VB_BASE_HIGH @@ VB_BASE_LOW).resized
  } else {
    fullVbWordAddr := VB_BASE_LOW.resized
  }
  val fullVbBaseAddr = fullVbWordAddr @@ U(0, config.vbBeatShift bits)

  val fullTriWordAddr = UInt(config.triWordAddrWidth bits)
  if (config.addressWidth > 32) {
    fullTriWordAddr := (TRI_BASE_HIGH @@ TRI_BASE_LOW).resized
  } else {
    fullTriWordAddr := TRI_BASE_LOW.resized
  }

  val rawBits = INTR.RAW.readBits
  io.ready := !dmaBusy
  io.irq := irqMerged
  io.irq_done := rawBits(0)
  io.irq_error := rawBits(1)
  io.irq_frame_done := (if (config.useAutoAdvance) rawBits(2) else False)

  when(IRQ_ENABLE && (intrDone || intrError || intrFrameDone)) {
    IRQ_STATUS := True
  }

  when(dmaBusy) {
    cycleCounter := cycleCounter + 1
  }

  val activeTileIdx = Reg(UInt(log2Up(config.gridTiles) bits)) init (0)

  // ===========================================================================
  // 2. AXI READ BOUNDARY BUFFER & VISIBILITY FILTER (4x32-bit Words per Beat)
  // ===========================================================================
  val rBuffered = io.m_axi_vb.r.m2sPipe()

  val visBitsIn = Bits(config.wordsPerBeat bits)
  for (k <- 0 until config.wordsPerBeat) {
    visBitsIn(k) := rBuffered.payload.data(k * config.vbWordBits + activeTileIdx)
  }

  val vbFifo = StreamFifo(
    dataType = Bits(config.wordsPerBeat bits),
    depth = config.vbFifoDepth
  )

  vbFifo.io.push.valid := rBuffered.valid
  vbFifo.io.push.payload := visBitsIn
  rBuffered.ready := vbFifo.io.push.ready

  // ===========================================================================
  // 3. AXI4 READ MASTER ENGINE (aclk domain)
  // ===========================================================================
  val arAddrReg = Reg(UInt(config.addressWidth bits)) init (0)
  val arBeatsRemaining = Reg(UInt(32 bits)) init (0)
  val rBeatsRemaining = Reg(UInt(32 bits)) init (0)
  val outstandingReads = Reg(UInt(log2Up(config.maxOutstandingReads + 1) bits)) init (0)

  val bytesTo4KbBoundary = U(4096, 13 bits) - arAddrReg(11 downto 0).resize(13)
  val beatsTo4KbBoundary = (bytesTo4KbBoundary >> config.vbBeatShift).resize(16)
  val maxBurstBeats = U(config.maxBurstLength, 16 bits)

  val currentBurstLen = UInt(16 bits)
  currentBurstLen := arBeatsRemaining.min(beatsTo4KbBoundary.resized).min(maxBurstBeats.resized).resized

  val canIssueAr = (arBeatsRemaining > 0) && (outstandingReads < config.maxOutstandingReads)
  io.m_axi_vb.ar.valid := canIssueAr && dmaBusy
  io.m_axi_vb.ar.addr := arAddrReg
  io.m_axi_vb.ar.len := (currentBurstLen - 1).resized
  io.m_axi_vb.ar.size := U(config.vbBeatShift, 3 bits)
  io.m_axi_vb.ar.burst := Axi4.burst.INCR
  io.m_axi_vb.ar.id := 0
  if (config.getAxi4ReadConfig.useRegion) io.m_axi_vb.ar.region.clearAll()
  if (config.getAxi4ReadConfig.useLock) io.m_axi_vb.ar.lock.clearAll()
  if (config.getAxi4ReadConfig.useCache) io.m_axi_vb.ar.cache := B"0011"
  if (config.getAxi4ReadConfig.useProt) io.m_axi_vb.ar.prot.clearAll()
  if (config.getAxi4ReadConfig.useQos) io.m_axi_vb.ar.qos.clearAll()

  val arFire = io.m_axi_vb.ar.fire
  val rFire = io.m_axi_vb.r.fire
  val rLast = io.m_axi_vb.r.last

  when(arFire) {
    val burstBytes = currentBurstLen << config.vbBeatShift
    arAddrReg := arAddrReg + burstBytes.resized
    arBeatsRemaining := arBeatsRemaining - currentBurstLen.resized
  }

  when(arFire && !(rFire && rLast)) {
    outstandingReads := outstandingReads + 1
  }.elsewhen(!arFire && (rFire && rLast)) {
    outstandingReads := outstandingReads - 1
  }

  when(rFire) {
    rBeatsRemaining := rBeatsRemaining - 1
    when(io.m_axi_vb.r.resp === Axi4.resp.SLVERR || io.m_axi_vb.r.resp === Axi4.resp.DECERR) {
      axiReadError := True
      ERROR := True
      intrError := True
    }
  }

  // ===========================================================================
  // 4. MULTI-PUSH COMMAND QUEUE & DATA-MOVER COMMAND STREAM
  // ===========================================================================
  val cmdQueue = MultiPushFifo4x(
    dataType = DataMoverCmdCompact(config.triWordAddrWidth, config.bttTriCountWidth),
    depth = 16,
    maxPush = 4
  )

  val cmdStreamInternal = Stream(DataMoverCmdCompact(config.triWordAddrWidth, config.bttTriCountWidth))
  cmdStreamInternal << cmdQueue.io.pop

  val cmdStreamOut = if (isDualClock) {
    val cc = StreamFifoCC(
      dataType = DataMoverCmdCompact(config.triWordAddrWidth, config.bttTriCountWidth),
      depth = config.cmdFifoDepth,
      pushClock = clockDomain,
      popClock = dmClock
    )
    cc.io.push <> cmdStreamInternal
    cc.io.pop
  } else {
    val sync = StreamFifo(
      dataType = DataMoverCmdCompact(config.triWordAddrWidth, config.bttTriCountWidth),
      depth = config.cmdFifoDepth
    )
    sync.io.push <> cmdStreamInternal
    sync.io.pop
  }

  val dmArea = new ClockingArea(dmClock) {
    val cmdPayload = Bits(config.cmdWidth bits)
    cmdPayload := 0

    val bttBytes = cmdStreamOut.payload.triCount @@ U(0, config.triangleShift bits)
    cmdPayload(config.bttWidth - 1 downto 0) := bttBytes.asBits

    cmdPayload(23) := True // INCR burst
    cmdPayload(27 downto 24) := 0 // DSA
    cmdPayload(28) := False // EOF
    cmdPayload(29) := False // DRR
    cmdPayload(31 downto 30) := cmdStreamOut.payload.tag(1 downto 0)

    val startByteAddr = cmdStreamOut.payload.triWordAddr @@ U(0, config.triangleShift bits)
    if (config.addressWidth <= 32) {
      cmdPayload(63 downto 32) := startByteAddr.resize(32).asBits
    } else {
      cmdPayload(32 + config.addressWidth - 1 downto 32) := startByteAddr.asBits
    }

    io.m_axis_datamover_cmd.valid := cmdStreamOut.valid
    io.m_axis_datamover_cmd.payload := cmdPayload
    cmdStreamOut.ready := io.m_axis_datamover_cmd.ready
  }

  // ===========================================================================
  // 5. STATUS STREAM & IN-FLIGHT RETIREMENT TRACKER
  // ===========================================================================
  val stsStreamInternal = if (isDualClock) {
    val ccSts = StreamFifoCC(
      dataType = Bits(config.stsWidth bits),
      depth = config.stsFifoDepth,
      pushClock = dmClock,
      popClock = clockDomain
    )
    ccSts.io.push <> io.s_axis_datamover_sts
    ccSts.io.pop
  } else {
    val syncSts = StreamFifo(
      dataType = Bits(config.stsWidth bits),
      depth = config.stsFifoDepth
    )
    syncSts.io.push <> io.s_axis_datamover_sts
    syncSts.io.pop
  }

  stsStreamInternal.ready := True

  when(stsStreamInternal.fire) {
    val stsByte = stsStreamInternal.payload(7 downto 0)
    dmLastStsReg := stsByte
    statStsCount := statStsCount + 1

    val isOkay = stsByte(7)
    val hasError = !isOkay || stsByte(6 downto 4) =/= 0
    when(hasError) {
      dmErrorFlag := True
      ERROR := True
      intrError := True
    }
  }

  // ===========================================================================
  // 6. 4-TRIANGLES-PER-CLOCK RUN-LENGTH AGGREGATOR (aclk domain)
  // ===========================================================================
  val triCounter = Reg(UInt(32 bits)) init (0)
  val shadowTriCount = Reg(UInt(32 bits)) init (0)
  val shadowTriWordBase = Reg(UInt(config.triWordAddrWidth bits)) init (0)
  val shadowMaxTriangles = Reg(UInt(config.bttTriCountWidth bits)) init (config.maxBttTriangles)
  val tagCounter = Reg(UInt(4 bits)) init (0)

  val runActive = RegInit(False)
  val runStartWord = Reg(UInt(config.triWordAddrWidth bits)) init (0)
  val runLength = Reg(UInt(config.bttTriCountWidth bits)) init (0)

  val curTriWordAddr = Reg(UInt(config.triWordAddrWidth bits)) init (0)
  val filterBeatsRemaining = Reg(UInt(32 bits)) init (0)
  val isLastBeatReg = RegInit(False)
  val remainderReg = Reg(UInt(2 bits)) init (0)

  val rawV0 = vbFifo.io.pop.payload(0)
  val rawV1 = vbFifo.io.pop.payload(1)
  val rawV2 = vbFifo.io.pop.payload(2)
  val rawV3 = vbFifo.io.pop.payload(3)

  // Pure 1-LUT valid masking without any 32-bit comparator
  val isLastBeat = isLastBeatReg
  val isValid0 = True
  val isValid1 = !isLastBeat || (remainderReg === 0) || (remainderReg > 1)
  val isValid2 = !isLastBeat || (remainderReg === 0) || (remainderReg === 3)
  val isValid3 = !isLastBeat || (remainderReg === 0)

  val v0 = rawV0 && isValid0
  val v1 = rawV1 && isValid1
  val v2 = rawV2 && isValid2
  val v3 = rawV3 && isValid3

  val stepCount = UInt(32 bits)
  stepCount := Mux(
    isValid3,
    U(4, 32 bits),
    Mux(isValid2, U(3, 32 bits), Mux(isValid1, U(2, 32 bits), U(1, 32 bits)))
  )

  // Base triangle address for this beat directly from register
  val beatTriWordAddr = curTriWordAddr
  val triAddr0 = beatTriWordAddr
  val triAddr1 = beatTriWordAddr + 1
  val triAddr2 = beatTriWordAddr + 2
  val triAddr3 = beatTriWordAddr + 3

  // Count consecutive leading 1s starting at Tri 0 (1..4)
  val leadOnes = UInt(3 bits)
  leadOnes := Mux(!v1, U(1, 3 bits),
              Mux(!v2, U(2, 3 bits),
              Mux(!v3, U(3, 3 bits), U(4, 3 bits))))

  // Check if previous run can extend
  val canExtend = (runLength + leadOnes.resized <= shadowMaxTriangles)

  // Candidate 0: Emit previous run (if active and cannot continue into Tri 0)
  val emitPrev_valid = runActive && (!v0 || !canExtend)
  val emitPrev_start = runStartWord
  val emitPrev_len   = runLength

  // Candidate 1: Run passing through / starting at Tri 0
  val run0_start = (runActive && canExtend) ? runStartWord | triAddr0
  val run0_baseLen = (runActive && canExtend) ? runLength | U(0, config.bttTriCountWidth bits)
  val run0_len = run0_baseLen + leadOnes.resized

  val emitTri0_valid = v0 && (leadOnes < 4 || isLastBeat)
  val emitTri0_start = run0_start
  val emitTri0_len   = run0_len

  // Candidate 2: Run starting at Tri 1 (starts only if !v0 && v1)
  val tri1_starts = !v0 && v1
  val tri1_len = UInt(config.bttTriCountWidth bits)
  tri1_len := Mux(!v2, U(1, config.bttTriCountWidth bits),
              Mux(!v3, U(2, config.bttTriCountWidth bits), U(3, config.bttTriCountWidth bits)))

  val emitTri1_valid = tri1_starts && (!v3 || isLastBeat)
  val emitTri1_start = triAddr1
  val emitTri1_len   = tri1_len

  // Candidate 3: Run starting at Tri 2 (starts only if !v1 && v2)
  val tri2_starts = !v1 && v2
  val tri2_len = (!v3) ? U(1, config.bttTriCountWidth bits) | U(2, config.bttTriCountWidth bits)

  val emitTri2_valid = tri2_starts && (!v3 || isLastBeat)
  val emitTri2_start = triAddr2
  val emitTri2_len   = tri2_len

  // Candidate 4: Run starting at Tri 3 (starts only if !v2 && v3)
  val tri3_starts = !v2 && v3
  val emitTri3_valid = tri3_starts && isLastBeat
  val emitTri3_start = triAddr3
  val emitTri3_len   = U(1, config.bttTriCountWidth bits)

  // Map into 4 non-overlapping output slots:
  // emitTri0 and emitTri1 are mutually exclusive because emitTri0 requires v0=1 and emitTri1 requires v0=0
  val slot0_valid = emitPrev_valid
  val slot0_start = emitPrev_start
  val slot0_len   = emitPrev_len

  val slot1_valid = emitTri0_valid | emitTri1_valid
  val slot1_start = emitTri0_valid ? emitTri0_start | emitTri1_start
  val slot1_len   = emitTri0_valid ? emitTri0_len   | emitTri1_len

  val slot2_valid = emitTri2_valid
  val slot2_start = emitTri2_start
  val slot2_len   = emitTri2_len

  val slot3_valid = emitTri3_valid
  val slot3_start = emitTri3_start
  val slot3_len   = emitTri3_len

  val emitValids = Vec(Bool(), 4)
  val emitStarts = Vec(UInt(config.triWordAddrWidth bits), 4)
  val emitLens   = Vec(UInt(config.bttTriCountWidth bits), 4)

  emitValids(0) := slot0_valid
  emitStarts(0) := slot0_start
  emitLens(0)   := slot0_len

  emitValids(1) := slot1_valid
  emitStarts(1) := slot1_start
  emitLens(1)   := slot1_len

  emitValids(2) := slot2_valid
  emitStarts(2) := slot2_start
  emitLens(2)   := slot2_len

  emitValids(3) := slot3_valid
  emitStarts(3) := slot3_start
  emitLens(3)   := slot3_len

  // Next run carried over to next beat (if !isLastBeat and v3 is True)
  val nextRunActive = Bool()
  val nextRunStart  = UInt(config.triWordAddrWidth bits)
  val nextRunLength = UInt(config.bttTriCountWidth bits)

  when(!isLastBeat && v3) {
    nextRunActive := True
    when(v0 && v1 && v2) {
      nextRunStart  := run0_start
      nextRunLength := run0_len
    }.elsewhen(!v0 && v1 && v2) {
      nextRunStart  := triAddr1
      nextRunLength := 3
    }.elsewhen(!v1 && v2) {
      nextRunStart  := triAddr2
      nextRunLength := 2
    }.otherwise {
      nextRunStart  := triAddr3
      nextRunLength := 1
    }
  }.otherwise {
    nextRunActive := False
    nextRunStart  := 0
    nextRunLength := 0
  }

  // Calculate tag offsets and total emits
  val e0 = emitValids(0).asUInt.resize(3)
  val e1 = emitValids(1).asUInt.resize(3)
  val e2 = emitValids(2).asUInt.resize(3)
  val e3 = emitValids(3).asUInt.resize(3)

  val emitOffsets = Vec(UInt(3 bits), 4)
  emitOffsets(0) := 0
  emitOffsets(1) := e0
  emitOffsets(2) := e0 + e1
  emitOffsets(3) := e0 + e1 + e2
  val totalEmitsThisCycle = e0 + e1 + e2 + e3

  // ===========================================================================
  // 7. PIPELINE REGISTER STAGE (Breaks critical path between Evaluator and cmdQueue)
  // ===========================================================================
  val pipeValids   = Vec(RegInit(False), 4)
  val pipeStarts   = Vec(Reg(UInt(config.triWordAddrWidth bits)) init (0), 4)
  val pipeLens     = Vec(Reg(UInt(config.bttTriCountWidth bits)) init (0), 4)
  val pipeTags     = Vec(Reg(Bits(4 bits)) init (0), 4)
  val visCountReg  = Reg(UInt(3 bits)) init (0)

  // cmdQueue is fed purely from pipeline registers!
  vbFifo.io.pop.ready := False
  for (i <- 0 until 4) {
    cmdQueue.io.pushValids(i) := pipeValids(i)
    cmdQueue.io.pushPayloads(i).triWordAddr := pipeStarts(i)
    cmdQueue.io.pushPayloads(i).triCount    := pipeLens(i)
    cmdQueue.io.pushPayloads(i).tag         := pipeTags(i)
  }

  // Counters updated purely from register stage (zero combinational path from data path)
  val pipeEmitsCount =
    pipeValids(0).asUInt.resize(3) + pipeValids(1).asUInt.resize(3) +
    pipeValids(2).asUInt.resize(3) + pipeValids(3).asUInt.resize(3)
  statCmdCount := statCmdCount + pipeEmitsCount.resized
  statVisCount := statVisCount + visCountReg.resized

  object FilterState4x extends SpinalEnum {
    val IDLE, FILTER_STREAM, WAIT_PIPE, WAIT_RETIRE, ADVANCE_TILE = newElement()
  }
  val filterState = RegInit(FilterState4x.IDLE)

  when(SOFT_RESET) {
    filterState := FilterState4x.IDLE
    dmaBusy := False
    axiReadError := False
    dmErrorFlag := False
    statVisCount := 0
    statCmdCount := 0
    statStsCount := 0
    visCountReg := 0
    runActive := False
    runLength := 0
    outstandingReads := 0
    pipeValids.foreach(_ := False)
  }.otherwise {
    switch(filterState) {
      is(FilterState4x.IDLE) {
        when(START && !dmaBusy) {
          dmaBusy := True
          DONE := False
          ERROR := False
          IRQ_STATUS := False
          if (config.useAutoAdvance) FRAME_DONE := False
          axiReadError := False
          dmErrorFlag := False
          cycleCounter := 0
          statVisCount := 0
          statCmdCount := 0
          statStsCount := 0
          visCountReg := 0
          tagCounter := 0
          runActive := False
          runLength := 0
          pipeValids.foreach(_ := False)

          activeTileIdx := TILE_INDEX
          shadowTriCount := TRI_COUNT
          shadowTriWordBase := fullTriWordAddr

          shadowMaxTriangles := (BTT_LIMIT_TRI === 0) ? U(
            config.maxBttTriangles,
            config.bttTriCountWidth bits
          ) | BTT_LIMIT_TRI

          arAddrReg := fullVbBaseAddr
          val beatShift = log2Up(config.wordsPerBeat)
          val totalBeats = UInt(32 bits)
          totalBeats := ((TRI_COUNT + (config.wordsPerBeat - 1)) >> beatShift).resized
          arBeatsRemaining := totalBeats
          rBeatsRemaining := totalBeats
          filterBeatsRemaining := totalBeats
          isLastBeatReg := (totalBeats <= 1)
          remainderReg := TRI_COUNT(1 downto 0)
          curTriWordAddr := fullTriWordAddr
          triCounter := 0

          when(TRI_COUNT === 0) {
            filterState := FilterState4x.ADVANCE_TILE
          }.otherwise {
            filterState := FilterState4x.FILTER_STREAM
          }
        }
      }

      is(FilterState4x.FILTER_STREAM) {
        when(vbFifo.io.pop.valid && cmdQueue.io.freeSpace >= 8) {
          vbFifo.io.pop.ready := True
          filterBeatsRemaining := filterBeatsRemaining - 1
          isLastBeatReg := (filterBeatsRemaining === 2)
          curTriWordAddr := curTriWordAddr + 4
          triCounter := triCounter + stepCount

          val visCountThisCycle =
            (v0.asUInt.resize(3) + v1.asUInt.resize(3) + v2.asUInt.resize(3) + v3.asUInt.resize(3))
          visCountReg := visCountThisCycle
          tagCounter  := tagCounter + totalEmitsThisCycle.resized

          for (i <- 0 until 4) {
            pipeValids(i) := emitValids(i)
            pipeStarts(i) := emitStarts(i)
            pipeLens(i)   := emitLens(i)
            pipeTags(i)   := (tagCounter + emitOffsets(i)).resize(4).asBits
          }

          when(isLastBeat) {
            runActive := False
            runLength := 0
            filterState := FilterState4x.WAIT_PIPE
          }.otherwise {
            runActive    := nextRunActive
            runStartWord := nextRunStart
            runLength    := nextRunLength
          }
        }.otherwise {
          visCountReg := 0
          pipeValids.foreach(_ := False)
        }
      }

      is(FilterState4x.WAIT_PIPE) {
        visCountReg := 0
        pipeValids.foreach(_ := False)
        filterState := FilterState4x.WAIT_RETIRE
      }

      is(FilterState4x.WAIT_RETIRE) {
        visCountReg := 0
        pipeValids.foreach(_ := False)
        when(cmdQueue.io.occupancy === 0 && (statStsCount === statCmdCount)) {
          filterState := FilterState4x.ADVANCE_TILE
        }
      }

      is(FilterState4x.ADVANCE_TILE) {
        dmaBusy := False
        DONE := True
        intrDone := True

        when(IRQ_ENABLE) {
          IRQ_STATUS := True
        }

        if (config.useAutoAdvance) {
          when(AUTO_ADV_EN) {
            when(TILE_INDEX === (config.gridTiles - 1)) {
              TILE_INDEX := 0
              FRAME_DONE := True
              intrFrameDone := True
            }.otherwise {
              TILE_INDEX := TILE_INDEX + 1
            }
          }
        }

        filterState := FilterState4x.IDLE
      }
    }
  }
}

// =============================================================================
// Verilog RTL Generation Entry Point for AxiVisibilityDma4x
// =============================================================================
object AxiVisibilityDma4xVerilog {
  def main(args: Array[String]): Unit = {
    val config = AxiVisibilityDma4xConfig(
      addressWidth = 32,
      axiDataWidth = 128, // 128-bit AXI Read Master (4x32-bit visibility words)
      bttWidth = 23,
      gridWidth = 6, // 6x5 = 30 tiles
      gridHeight = 5,
      vbWordBits = 32, // 32-bit visibility word per triangle
      triangleSizeBytes = 128,
      maxBurstLength = 256,
      maxOutstandingReads = 8,
      cmdFifoDepth = 64,
      stsFifoDepth = 4,
      vbFifoDepth = 256,
      useAutoAdvance = true,
      trianglesPerCycle = 4 // 4 triangles per clock cycle
    )

    SpinalConfig(
      targetDirectory = "rtl/AxiVisibilityDma4x",
      defaultConfigForClockDomains = ClockDomainConfig(resetActiveLevel = LOW)
    ).withPrivateNamespace.generateVerilog(
      new AxiVisibilityDma4x(
        config = config,
        dmClockDomain = null
      )
    )
  }
}
