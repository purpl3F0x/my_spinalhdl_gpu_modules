package projectname

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba4.axi._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.regif._
import spinal.lib.bus.regif.AccessType._
import spinal.lib.fsm._

// =============================================================================
// Tiled RAM (BRAM) Direct Read Port Interface (Operates in ramClock domain)
// =============================================================================
case class TileRamReadPort(addrWidth: Int, dataWidth: Int) extends Bundle with IMasterSlave {
  val clk = Bool()
  val rst = Bool()
  val en = Bool()
  val addr = UInt(addrWidth bits)
  val dout = Bits(dataWidth bits)

  override def asMaster(): Unit = {
    out(clk, rst, en, addr)
    in(dout)
  }
}

// =============================================================================
// Configuration Bundle for AxiTileDma
// =============================================================================
case class AxiTileDmaConfig(
    addressWidth: Int = 32,
    pixelsPerClock: Int = 4, // Number of 32-bit pixels per clock cycle (1, 2, 4, 8, 16)
    tileWidth: Int = 128, // Tile width in pixels (compile-time constant, e.g. 128)
    tileHeight: Int = 128, // Tile height in rows (compile-time constant, e.g. 128)
    maxBurstLength: Int = 256, // Max AXI burst beats (AXI4 max = 256)
    maxOutstanding: Int = 8, // Max outstanding AXI write transactions
    idWidth: Int = 4, // AXI Master transaction ID width
    cdcDepth: Int = 16, // Depth of async dual-clock CDC FIFO between RAM clock and AXI clock
    useAutoAdvance: Boolean = true, // Compile-time option: enable 2D raster tile coordinate advancing
    bramReadLatency: Int = 1 // Read latency in clock cycles (1 for primitive output reg, 2 for optional pipeline reg)
) {
  require(pixelsPerClock >= 1 && isPow2(pixelsPerClock), "pixelsPerClock must be a power of 2 (1, 2, 4, 8, 16...)")
  require(tileWidth > 0 && tileHeight > 0, "tile dimensions must be positive")
  require(tileWidth % pixelsPerClock == 0, "tileWidth must be an integer multiple of pixelsPerClock")
  require(maxBurstLength >= 1 && maxBurstLength <= 256, "maxBurstLength must be between 1 and 256 beats")
  require(cdcDepth >= 8 && isPow2(cdcDepth), "cdcDepth must be a power of 2 >= 8")
  require(bramReadLatency >= 1, "bramReadLatency must be at least 1 clock cycle")
  require(
    cdcDepth >= (bramReadLatency + 4),
    s"cdcDepth ($cdcDepth) must be >= bramReadLatency + 4 (${bramReadLatency + 4})"
  )

  val pixelBits: Int = 32
  val pixelBytes: Int = 4
  val dataWidth: Int = pixelsPerClock * pixelBits
  val bytesPerBeat: Int = pixelsPerClock * pixelBytes
  val beatShift: Int = log2Up(bytesPerBeat)

  // Compile-time tile geometry (128x128 tile)
  val tileHeightShift: Int = log2Up(tileHeight) // 7 bits for 128 rows
  val tileWidthShift: Int = log2Up(tileWidth) // 7 bits for 128 cols
  val beatsPerRow: Int = tileWidth / pixelsPerClock
  val totalBeats: Int = beatsPerRow * tileHeight
  val bytesPerRow: Int = tileWidth * pixelBytes
  val totalBytes: Int = tileWidth * tileHeight * pixelBytes
  val ramWordAddrWidth: Int = log2Up(totalBeats) max 1
  val ramByteAddrWidth: Int = log2Up(totalBytes) max 1
  val ramAddrWidth: Int = ramByteAddrWidth // Byte-addressable BRAM address width

  def getAxi4Config: Axi4Config = Axi4Config(
    addressWidth = addressWidth,
    dataWidth = dataWidth,
    idWidth = idWidth,
    useBurst = true,
    useSize = true,
    useLen = true,
    useLast = true,
    useResp = true,
    useStrb = true,
    useId = idWidth > 0
  )

  def getApbConfig: Apb3Config = Apb3Config(
    addressWidth = 7,
    dataWidth = 32
  )
}

// =============================================================================
// 2D Tile Frame Buffer DMA Module (Dual Clock Domains with Async CDC FIFO)
// =============================================================================
class AxiTileDma(
    val config: AxiTileDmaConfig,
    val ramClockDomain: ClockDomain = null
) extends Component {

  // High-frequency BRAM clock domain
  val ramClock = if (ramClockDomain != null) ramClockDomain else ClockDomain.external("ram", withReset = true)

  val io = new Bundle {
    // 1. APB3 Slave Interface for Register Programming (AXI clock domain)
    val s_apb = slave(Apb3(config.getApbConfig))

    // 2. AXI4 Full Write Master Interface to DRAM (AXI clock domain)
    val m_axi = master(Axi4WriteOnly(config.getAxi4Config))

    // 3. Tiled RAM (BRAM) Read Port (Operates in high-frequency ramClock domain)
    val tile_ram = master(TileRamReadPort(config.ramByteAddrWidth, config.dataWidth))

    // 4. Interrupt Outputs (AXI clock domain)
    val irq = out Bool ()
    val irq_done = out Bool ()
    val irq_error = out Bool ()
    val irq_frame_done = out Bool ()
  }

  // Clear top-level "io_" prefix for clean Verilog / EDA naming
  noIoPrefix()

  // Vivado AXI standard bus port renaming helper
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

  renameAxiForVivado(io.m_axi, "m_axi")

  // Forward RAM clock and active-high reset to BRAM interface
  // Input ram_resetn is active-low, but BRAM output tile_ram_rst is ALWAYS active-high
  io.tile_ram.clk := ramClock.readClockWire
  io.tile_ram.rst := !ramClock.readResetWire

  // Vivado BRAM Interface synthesis attributes for IP Integrator block designs
  io.tile_ram.clk.addAttribute("X_INTERFACE_INFO", "xilinx.com:interface:bram:1.0 tile_ram CLK")
  io.tile_ram.clk.addAttribute(
    "X_INTERFACE_PARAMETER",
    s"MASTER_TYPE BRAM_CTRL, MEM_ECC NONE, MEM_WIDTH ${config.dataWidth}, MEM_SIZE ${config.totalBytes}, READ_WRITE_MODE READ_ONLY, READ_LATENCY ${config.bramReadLatency}"
  )
  io.tile_ram.rst.addAttribute("X_INTERFACE_INFO", "xilinx.com:interface:bram:1.0 tile_ram RST")
  io.tile_ram.rst.addAttribute("X_INTERFACE_PARAMETER", "POLARITY ACTIVE_HIGH")
  io.tile_ram.en.addAttribute("X_INTERFACE_INFO", "xilinx.com:interface:bram:1.0 tile_ram EN")
  io.tile_ram.addr.addAttribute("X_INTERFACE_INFO", "xilinx.com:interface:bram:1.0 tile_ram ADDR")
  io.tile_ram.dout.addAttribute("X_INTERFACE_INFO", "xilinx.com:interface:bram:1.0 tile_ram DOUT")

  // AXI Clock and Reset names & Vivado interface association attributes
  clockDomain.clock.setName("aclk")
  clockDomain.reset.setName("aresetn")
  clockDomain.clock.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:clock:1.0 aclk CLK")
  clockDomain.clock.addAttribute(
    "X_INTERFACE_PARAMETER",
    "ASSOCIATED_BUSIF m_axi, ASSOCIATED_RESET aresetn"
  )
  clockDomain.reset.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:reset:1.0 aresetn RST")
  clockDomain.reset.addAttribute("X_INTERFACE_PARAMETER", "POLARITY ACTIVE_LOW")

  // RAM Clock and Reset names & Vivado interface association attributes (input reset is active-low ram_resetn)
  ramClock.clock.setName("ram_clk")
  ramClock.reset.setName("ram_resetn")
  ramClock.clock.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:clock:1.0 ram_clk CLK")
  ramClock.clock.addAttribute(
    "X_INTERFACE_PARAMETER",
    "ASSOCIATED_BUSIF tile_ram, ASSOCIATED_RESET ram_resetn"
  )
  ramClock.reset.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:reset:1.0 ram_resetn RST")
  ramClock.reset.addAttribute("X_INTERFACE_PARAMETER", "POLARITY ACTIVE_LOW")

  // Vivado Interrupt interface attributes
  io.irq.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:interrupt:1.0 irq INTERRUPT")
  io.irq.addAttribute("X_INTERFACE_PARAMETER", "SENSITIVITY LEVEL_HIGH")
  io.irq_done.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:interrupt:1.0 irq_done INTERRUPT")
  io.irq_done.addAttribute("X_INTERFACE_PARAMETER", "SENSITIVITY LEVEL_HIGH")
  io.irq_error.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:interrupt:1.0 irq_error INTERRUPT")
  io.irq_error.addAttribute("X_INTERFACE_PARAMETER", "SENSITIVITY LEVEL_HIGH")
  io.irq_frame_done.addAttribute("X_INTERFACE_INFO", "xilinx.com:signal:interrupt:1.0 irq_frame_done INTERRUPT")
  io.irq_frame_done.addAttribute("X_INTERFACE_PARAMETER", "SENSITIVITY LEVEL_HIGH")

  // ===========================================================================
  // AXI CLOCK DOMAIN LOGIC
  // ===========================================================================

  // ---------------------------------------------------------------------------
  // Register Map via SpinalHDL Built-in regif
  // ---------------------------------------------------------------------------
  val busif = Apb3BusInterface(io.s_apb, (0x0000, 128), "AXI_TILE_DMA")

  // Status and Debug registers driven by hardware
  val dmaBusy = RegInit(False)
  val brespVal = Reg(Bits(2 bits)) init (0)
  val curLineIdx = Reg(UInt(log2Up(config.tileHeight + 1) bits)) init (0)
  val cycleCounter = Reg(UInt(32 bits)) init (0)

  // 0x00: CONTROL
  val REG_CTRL = busif.newRegAt(address = 0x00, doc = "DMA Control Register")
  val START = REG_CTRL.field(Bool(), W1P, doc = "Trigger 128x128 tile transfer to DRAM")
  val RESET_POS =
    if (config.useAutoAdvance) REG_CTRL.field(Bool(), W1P, doc = "Reset tile coordinates (TileX=0, TileY=0)") else null
  if (!config.useAutoAdvance) REG_CTRL.reserved(1 bits)
  val SOFT_RESET = REG_CTRL.field(Bool(), W1P, doc = "Software reset for DMA FSM and internal pointers")

  // 0x04: STATUS (Operational status flags; interrupt events managed by INTR factory at 0x40)
  val REG_STATUS = busif.newRegAt(address = 0x04, doc = "DMA Status Register")
  val BUSY = REG_STATUS.field(Bool(), RO, doc = "DMA active transfer busy flag")
  val LAST_BRESP = REG_STATUS.field(Bits(2 bits), RO, doc = "Last AXI write response status")

  BUSY := dmaBusy
  LAST_BRESP := brespVal

  // 0x08: BASE_ADDR_LOW (RW)
  val REG_BASE_LOW = busif.newRegAt(address = 0x08, doc = "DRAM Frame Base Address (Lower 32 bits)")
  val BASE_ADDR_LOW = REG_BASE_LOW.field(
    UInt(32 bits),
    RW,
    resetValue = 0,
    doc = "Lower 32 bits of Frame Destination Base Address (4KB aligned: bits 11..0 = 0)"
  )

  // 0x0C: BASE_ADDR_HIGH (RW)
  val REG_BASE_HIGH = busif.newRegAt(address = 0x0c, doc = "DRAM Frame Base Address (Upper 32 bits)")
  val BASE_ADDR_HIGH = REG_BASE_HIGH.field(
    UInt(32 bits),
    RW,
    resetValue = 0,
    doc = "Upper 32 bits of Frame Base Address (64-bit DRAM space)"
  )

  // 0x10: STRIDE (RW)
  val REG_STRIDE = busif.newRegAt(address = 0x10, doc = "DRAM Frame Row Byte Stride")
  val STRIDE =
    REG_STRIDE.field(UInt(32 bits), RW, resetValue = 0, doc = "Framebuffer row byte stride / pitch (power of 2)")

  // 0x14: TILES_X
  val REG_TILES_X = if (config.useAutoAdvance) busif.newRegAt(address = 0x14, doc = "Horizontal Tiles Count") else null
  val TILES_X =
    if (config.useAutoAdvance)
      REG_TILES_X.field(UInt(16 bits), RW, resetValue = 1, doc = "Total number of horizontal tiles per frame row")
    else null

  // 0x18: TILES_Y
  val REG_TILES_Y =
    if (config.useAutoAdvance) busif.newRegAt(address = 0x18, doc = "Vertical Tile Rows Count") else null
  val TILES_Y =
    if (config.useAutoAdvance)
      REG_TILES_Y.field(UInt(16 bits), RW, resetValue = 1, doc = "Total number of vertical tile rows per frame")
    else null

  // 0x1C: CUR_POS (RW) - Bits 15..0 = curTileX, bits 31..16 = curTileY
  val REG_CUR_POS = if (config.useAutoAdvance) busif.newRegAt(address = 0x1c, doc = "Current Tile Position") else null
  val CUR_TILE_X =
    if (config.useAutoAdvance)
      REG_CUR_POS.field(UInt(16 bits), RW, resetValue = 0, doc = "Current horizontal tile index (0..TILES_X-1)")
    else null
  val CUR_TILE_Y =
    if (config.useAutoAdvance)
      REG_CUR_POS.field(UInt(16 bits), RW, resetValue = 0, doc = "Current vertical tile row index (0..TILES_Y-1)")
    else null

  // 0x20: CURRENT_LINE (RO)
  val REG_LINE = busif.newRegAt(address = 0x20, doc = "Current Line Progress")
  val CURRENT_LINE =
    REG_LINE.field(UInt(log2Up(config.tileHeight + 1) bits), RO, doc = "Current line index within active tile (0..127)")
  CURRENT_LINE := curLineIdx

  // 0x24: PERF_CYCLES (RO)
  val REG_PERF = busif.newRegAt(address = 0x24, doc = "Performance Cycle Counter")
  val PERF_CYCLES = REG_PERF.field(UInt(32 bits), RO, doc = "Elapsed clock cycles during active tile transfer")
  PERF_CYCLES := cycleCounter

  // ===========================================================================
  // INTERRUPT FACTORY REGISTERS (0x40 - 0x4C)
  // RegGroup RFMS4:
  //   0x40: INTR_INT_RAW    - Raw interrupt event flags (W1C: write 1 to clear)
  //   0x44: INTR_INT_FORCE  - Software debug force (W1S: write 1 to trigger raw)
  //   0x48: INTR_INT_MASK   - Mask register (RW: 1 = masked/off, 0 = unmasked/open)
  //   0x4C: INTR_INT_STATUS - Masked status register (RO: raw && !mask)
  // Fields:
  //   Bit 0: done           - Tile transfer complete
  //   Bit 1: error          - AXI bus write error (SLVERR/DECERR)
  //   Bit 2: frame_done     - Final tile of frame complete (Auto-Advance wrap)
  // ===========================================================================
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

  // Built-in Documentation & Header Generators (exported to targetDirectory)
  busif.accept(HtmlGenerator("axi_tile_dma_regmap", "AXI_TILE_DMA"))
  busif.accept(CHeaderGenerator("axi_tile_dma_regmap", "AXI_TILE_DMA"))
  busif.accept(JsonGenerator("axi_tile_dma_regmap"))

  // Full 64-bit / 32-bit combined frame base address (4KB aligned: bits 11..0 are 0)
  val fullBaseAddr = UInt(config.addressWidth bits)
  if (config.addressWidth > 32) {
    fullBaseAddr := (BASE_ADDR_HIGH @@ BASE_ADDR_LOW(31 downto 12) @@ U(0, 12 bits)).resized
  } else {
    fullBaseAddr := (BASE_ADDR_LOW(31 downto 12) @@ U(0, 12 bits)).resized
  }

  // ---------------------------------------------------------------------------
  // 2D Raster Tile Stepping Coordinates & Origin Tracking
  // ---------------------------------------------------------------------------
  val tileOriginAddr = Reg(UInt(config.addressWidth bits)) init (0)
  val rowStartTileAddr = Reg(UInt(config.addressWidth bits)) init (0)

  val shadowStride = Reg(UInt(32 bits)) init (0)
  val shadowTilesX = if (config.useAutoAdvance) Reg(UInt(16 bits)) init (1) else null
  val shadowTilesY = if (config.useAutoAdvance) Reg(UInt(16 bits)) init (1) else null

  if (config.useAutoAdvance) {
    // Reset tile position to (0,0)
    when(RESET_POS) {
      CUR_TILE_X := 0
      CUR_TILE_Y := 0
      tileOriginAddr := fullBaseAddr
      rowStartTileAddr := fullBaseAddr
    }
  }

  when(dmaBusy) {
    cycleCounter := cycleCounter + 1
  }

  // ---------------------------------------------------------------------------
  // Asynchronous Dual-Clock CDC FIFO (ramClock -> axiClock)
  // ---------------------------------------------------------------------------
  val cdcFifo = StreamFifoCC(
    dataType = Bits(config.dataWidth bits),
    depth = config.cdcDepth,
    pushClock = ramClock,
    popClock = ClockDomain.current // axiClock
  )
  cdcFifo.ram.addAttribute("ram_style", "distributed")

  // Pulse to start reading the 128x128 tile in the high-frequency ramClock domain
  val startTilePulseAxi = False

  // Cross single-cycle start pulse from AXI domain to RAM domain
  val startPulseRam = PulseCCByToggle(
    input = startTilePulseAxi,
    clockIn = ClockDomain.current,
    clockOut = ramClock
  )

  // ===========================================================================
  // HIGH-FREQUENCY RAM CLOCK DOMAIN LOGIC (ramClock, optimized for 600+ MHz)
  // Fully pipelined credit tracking: registers pushOccupancy to cut the Gray-to-Binary
  // XOR path, and drives tile_ram.en and tile_ram.addr directly from registers.
  // ===========================================================================
  val ramArea = new ClockingArea(ramClock) {
    val ramReadsLeft = Reg(UInt(log2Up(config.totalBeats + 1) bits)) init (0)
    val ramPushesLeft = Reg(UInt(log2Up(config.totalBeats + 1) bits)) init (0)
    val ramAddrReg = Reg(UInt(config.ramWordAddrWidth bits)) init (0)
    val ramEnReg = RegInit(False)
    val active = RegInit(False)

    // Pipeline Stage 1: Register CDC FIFO push occupancy to isolate the popToPushGray XOR path
    val pushOccupancyReg = RegNext(cdcFifo.io.pushOccupancy) init (0)

    // In-flight tracking: counts reads in pipeline between schedule and retirement into pushOccupancyReg
    // Sized to cdcDepth + 1 to prevent counter overflow during bursts
    val inFlight = Reg(UInt(log2Up(config.cdcDepth + 1) bits)) init (0)
    val inFlightRetire = Delay(ramEnReg, cycleCount = config.bramReadLatency + 2, init = False)

    // Pipeline Stage 2: Fast 5-bit comparison with registered occupancy
    val headroom = config.bramReadLatency + 3
    val totalCommitted = pushOccupancyReg + inFlight
    val hasRoom = totalCommitted < (config.cdcDepth - headroom)
    val nextCanRead = active && (ramReadsLeft > 0) && hasRoom

    // Direct Register Outputs to BRAM pins (Zero combinational gates, optimal for 600 MHz)
    // BRAM is continuously enabled throughout active transfer; disabled when idle
    io.tile_ram.en := active
    io.tile_ram.addr := (ramAddrReg @@ U(0, config.beatShift bits)).resized

    // Delay read enable by bramReadLatency cycles to align with BRAM dout valid timing
    val readValid = Delay(ramEnReg, cycleCount = config.bramReadLatency, init = False)

    when(startPulseRam) {
      ramReadsLeft := config.totalBeats
      ramPushesLeft := config.totalBeats
      ramAddrReg := 0
      ramEnReg := False
      inFlight := 0
      active := True
    }.otherwise {
      // Register read enable for cycle T+1
      ramEnReg := nextCanRead

      // Address increments whenever a read is active
      when(ramEnReg) {
        ramAddrReg := ramAddrReg + 1
      }

      // Reads remaining decrements when a read is scheduled
      when(nextCanRead) {
        ramReadsLeft := ramReadsLeft - 1
      }

      // In-flight read tracking: increment on schedule, decrement on retirement into pushOccupancyReg
      when(nextCanRead =/= inFlightRetire) {
        when(nextCanRead) {
          inFlight := inFlight + 1
        }.otherwise {
          when(inFlight > 0) {
            inFlight := inFlight - 1
          }
        }
      }

      // Pushes remaining decrements when data enters CDC FIFO
      when(readValid) {
        ramPushesLeft := ramPushesLeft - 1
      }

      // Deassert active when all beats have entered CDC FIFO
      when(active && ramPushesLeft === 0) {
        active := False
      }
    }

    // Push BRAM data directly into the dual-clock CDC FIFO
    cdcFifo.io.push.valid := readValid
    cdcFifo.io.push.payload := io.tile_ram.dout
  }

  // ===========================================================================
  // AXI4 MASTER WRITE CHANNELS & FSM LOGIC (axiClock domain)
  // ===========================================================================
  object DmaState extends SpinalEnum {
    val IDLE, SEND_AW, READ_CDC_AND_WRITE_W, WAIT_RESP = newElement()
    val ADVANCE_CALC_1 = if (config.useAutoAdvance) newElement() else null
    val ADVANCE_CALC_2 = if (config.useAutoAdvance) newElement() else null
  }
  val state = RegInit(DmaState.IDLE)

  val currentLineInTile = Reg(UInt(log2Up(config.tileHeight + 1) bits)) init (0)
  val lineStartDramAddr = Reg(UInt(config.addressWidth bits)) init (0)
  val curDramAddr = Reg(UInt(config.addressWidth bits)) init (0)
  val beatsLeftInLine = Reg(UInt(log2Up(config.beatsPerRow + 1) bits)) init (0)

  // Pipelined registers for multi-cycle tile-advance calculation
  val advTileHeightStride = if (config.useAutoAdvance) Reg(UInt(config.addressWidth bits)) init (0) else null
  val advStepRightAddr = if (config.useAutoAdvance) Reg(UInt(config.addressWidth bits)) init (0) else null
  val advIsLastInRow = if (config.useAutoAdvance) RegInit(False) else null
  val advIsLastInFrame = if (config.useAutoAdvance) RegInit(False) else null

  // Burst length calculation:
  // With 4KB-aligned base address and power-of-2 stride (multiple of 512 bytes),
  // tile rows (512 bytes = 32 beats @ 128-bit) never cross 4KB boundaries.
  val currentBurstLen = UInt(16 bits)
  if (config.beatsPerRow <= config.maxBurstLength) {
    currentBurstLen := beatsLeftInLine.resize(16)
  } else {
    currentBurstLen := beatsLeftInLine.resize(16).min(U(config.maxBurstLength, 16 bits))
  }

  val activeBurstLen = Reg(UInt(16 bits)) init (0)
  val wBeatsRemaining = Reg(UInt(16 bits)) init (0)

  val outstandingBursts = Reg(UInt(log2Up(config.maxOutstanding + 1) bits)) init (0)

  // ---------------------------------------------------------------------------
  // AXI4 AW (Write Address) Channel
  // ---------------------------------------------------------------------------
  io.m_axi.aw.valid := (state === DmaState.SEND_AW) && (outstandingBursts < config.maxOutstanding)
  io.m_axi.aw.addr := curDramAddr
  io.m_axi.aw.len := (currentBurstLen - 1).resized
  io.m_axi.aw.size := U(config.beatShift, 3 bits)
  io.m_axi.aw.burst := Axi4.burst.INCR

  if (config.idWidth > 0) io.m_axi.aw.id.clearAll()
  if (config.getAxi4Config.useRegion) io.m_axi.aw.region.clearAll()
  if (config.getAxi4Config.useLock) io.m_axi.aw.lock.clearAll()
  if (config.getAxi4Config.useCache) io.m_axi.aw.cache := B"0011"
  if (config.getAxi4Config.useProt) io.m_axi.aw.prot.clearAll()
  if (config.getAxi4Config.useQos) io.m_axi.aw.qos.clearAll()

  // ---------------------------------------------------------------------------
  // AXI4 W (Write Data) Channel - Pop from Async CDC FIFO
  // ---------------------------------------------------------------------------
  val wFire = io.m_axi.w.fire

  io.m_axi.w.valid := (state === DmaState.READ_CDC_AND_WRITE_W) && cdcFifo.io.pop.valid
  io.m_axi.w.data := cdcFifo.io.pop.payload
  io.m_axi.w.strb.setAll()
  io.m_axi.w.last := (wBeatsRemaining === 1)

  cdcFifo.io.pop.ready := (state === DmaState.READ_CDC_AND_WRITE_W) && io.m_axi.w.ready

  // ---------------------------------------------------------------------------
  // AXI4 B (Write Response) Channel
  // ---------------------------------------------------------------------------
  io.m_axi.b.ready := True

  val awFire = io.m_axi.aw.fire
  val bFire = io.m_axi.b.fire

  when(awFire && !bFire) {
    outstandingBursts := outstandingBursts + 1
  }.elsewhen(!awFire && bFire) {
    outstandingBursts := outstandingBursts - 1
  }

  when(bFire) {
    when(io.m_axi.b.resp === Axi4.resp.SLVERR || io.m_axi.b.resp === Axi4.resp.DECERR) {
      intrError := True
      brespVal := io.m_axi.b.resp
    }
  }

  // ---------------------------------------------------------------------------
  // Main DMA FSM Control (AXI Clock Domain)
  // ---------------------------------------------------------------------------
  when(SOFT_RESET) {
    state := DmaState.IDLE
    dmaBusy := False
    outstandingBursts := 0
    tileOriginAddr := 0
    rowStartTileAddr := 0
    if (config.useAutoAdvance) {
      CUR_TILE_X := 0
      CUR_TILE_Y := 0
    }
  }.otherwise {
    switch(state) {
      is(DmaState.IDLE) {
        when(START && !dmaBusy) {
          dmaBusy := True
          cycleCounter := 0

          shadowStride := STRIDE
          if (config.useAutoAdvance) {
            shadowTilesX := (TILES_X === 0) ? U(1, 16 bits) | TILES_X
            shadowTilesY := (TILES_Y === 0) ? U(1, 16 bits) | TILES_Y

            val currentOrigin =
              ((CUR_TILE_X === 0 && CUR_TILE_Y === 0) || tileOriginAddr === 0) ? fullBaseAddr | tileOriginAddr
            tileOriginAddr := currentOrigin
            rowStartTileAddr := (CUR_TILE_X === 0) ? currentOrigin | rowStartTileAddr
            lineStartDramAddr := currentOrigin
            curDramAddr := currentOrigin
          } else {
            tileOriginAddr := fullBaseAddr
            rowStartTileAddr := fullBaseAddr
            lineStartDramAddr := fullBaseAddr
            curDramAddr := fullBaseAddr
          }

          currentLineInTile := 0
          curLineIdx := 0

          // Trigger high-frequency RAM reader in ramClock domain via CDC pulse
          startTilePulseAxi := True

          beatsLeftInLine := config.beatsPerRow
          state := DmaState.SEND_AW
        }
      }

      is(DmaState.SEND_AW) {
        when(io.m_axi.aw.fire) {
          activeBurstLen := currentBurstLen
          wBeatsRemaining := currentBurstLen
          state := DmaState.READ_CDC_AND_WRITE_W
        }
      }

      is(DmaState.READ_CDC_AND_WRITE_W) {
        when(wFire) {
          wBeatsRemaining := wBeatsRemaining - 1

          when(wBeatsRemaining === 1) {
            // Burst finished on W channel
            val burstBytes = activeBurstLen << config.beatShift
            val nextDram = curDramAddr + burstBytes
            val remBeats = beatsLeftInLine - activeBurstLen.resized

            when(remBeats === 0) {
              // Finished all bursts for current line in tile
              when(currentLineInTile === (config.tileHeight - 1)) {
                // All 128 rows of this tile finished!
                state := DmaState.WAIT_RESP
              }.otherwise {
                // Advance to next line within current tile (single register addition)
                val nextLineIdx = currentLineInTile + 1
                val nextLineDram = lineStartDramAddr + shadowStride.resized

                currentLineInTile := nextLineIdx
                curLineIdx := nextLineIdx
                lineStartDramAddr := nextLineDram
                curDramAddr := nextLineDram
                beatsLeftInLine := config.beatsPerRow
                state := DmaState.SEND_AW
              }
            }.otherwise {
              // More bursts remaining in current line (e.g. split across 4KB boundary)
              curDramAddr := nextDram
              beatsLeftInLine := remBeats
              state := DmaState.SEND_AW
            }
          }
        }
      }

      is(DmaState.WAIT_RESP) {
        when(outstandingBursts === 0) {
          dmaBusy := False
          intrDone := True
          if (config.useAutoAdvance) {
            state := DmaState.ADVANCE_CALC_1
          } else {
            state := DmaState.IDLE
          }
        }
      }

      if (config.useAutoAdvance) {
        // ---------------------------------------------------------------------
        // Multi-Cycle Pipelined 2D Raster Tile Advancing Calculation
        // Cycle 1: Shift-based row jump (stride is power of 2) & comparator step
        // ---------------------------------------------------------------------
        is(DmaState.ADVANCE_CALC_1) {
          // 128 * stride = stride << 7 (Zero multipliers, pure bit shift)
          advTileHeightStride := (shadowStride << config.tileHeightShift).resized
          advStepRightAddr := tileOriginAddr + U(config.bytesPerRow, config.addressWidth bits)
          advIsLastInRow := (CUR_TILE_X === (shadowTilesX - 1))
          advIsLastInFrame := (CUR_TILE_X === (shadowTilesX - 1)) && (CUR_TILE_Y === (shadowTilesY - 1))
          state := DmaState.ADVANCE_CALC_2
        }

        // ---------------------------------------------------------------------
        // Cycle 2: Coordinate & address commit step (Zero timing pressure)
        // ---------------------------------------------------------------------
        is(DmaState.ADVANCE_CALC_2) {
          when(advIsLastInFrame) {
            // Completed entire frame! Wrap around to (0, 0)
            CUR_TILE_X := 0
            CUR_TILE_Y := 0
            tileOriginAddr := fullBaseAddr
            rowStartTileAddr := fullBaseAddr
            intrFrameDone := True
          }.elsewhen(advIsLastInRow) {
            // Wrap down to the next tile row
            CUR_TILE_X := 0
            CUR_TILE_Y := CUR_TILE_Y + 1
            val nextTileRowOrigin = rowStartTileAddr + advTileHeightStride
            rowStartTileAddr := nextTileRowOrigin
            tileOriginAddr := nextTileRowOrigin
          }.otherwise {
            // Advance right to next tile in the current row
            CUR_TILE_X := CUR_TILE_X + 1
            tileOriginAddr := advStepRightAddr
          }

          state := DmaState.IDLE
        }
      }
    }
  }

  // Discrete status & interrupt outputs
  val rawBits = INTR.RAW.readBits
  io.irq := irqMerged
  io.irq_done := rawBits(0)
  io.irq_error := rawBits(1)
  io.irq_frame_done := (if (config.useAutoAdvance) rawBits(2) else False)
}

// =============================================================================
// Helper: Optional Top-Level Wrapper with Integrated 128x128 Tile Dual-Port BRAM
// =============================================================================
class AxiTileDmaWithRam(
    val config: AxiTileDmaConfig = AxiTileDmaConfig(),
    val ramClockDomain: ClockDomain = null
) extends Component {

  val ramClock = if (ramClockDomain != null) ramClockDomain else ClockDomain.external("ram", withReset = true)

  val io = new Bundle {
    val s_apb = slave(Apb3(config.getApbConfig))
    val m_axi = master(Axi4WriteOnly(config.getAxi4Config))
    val irq = out Bool ()
    val irq_done = out Bool ()
    val irq_error = out Bool ()
    val irq_frame_done = out Bool ()

    // External Write Port into Tiled RAM (Operates in high-frequency ramClock domain)
    val ram_wr_en = in Bool ()
    val ram_wr_addr = in UInt (config.ramWordAddrWidth bits)
    val ram_wr_data = in Bits (config.dataWidth bits)
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

  renameAxiForVivado(io.m_axi, "m_axi")

  clockDomain.clock.setName("aclk")
  clockDomain.reset.setName("aresetn")

  val dma = new AxiTileDma(config, ramClockDomain = ramClock)
  dma.io.s_apb <> io.s_apb
  dma.io.m_axi <> io.m_axi
  io.irq := dma.io.irq
  io.irq_done := dma.io.irq_done
  io.irq_error := dma.io.irq_error
  io.irq_frame_done := dma.io.irq_frame_done

  // Dual-Port BRAM in high-frequency ramClock domain
  val ramArea = new ClockingArea(ramClock) {
    val tiledRam = Mem(Bits(config.dataWidth bits), wordCount = config.totalBeats)

    tiledRam.write(
      enable = io.ram_wr_en,
      address = io.ram_wr_addr,
      data = io.ram_wr_data
    )

    // Convert byte-address back to word address for SpinalHDL simulation Mem
    val memRead = tiledRam.readSync(
      enable = dma.io.tile_ram.en,
      address = (dma.io.tile_ram.addr >> config.beatShift).resized
    )
    dma.io.tile_ram.dout := Delay(memRead, cycleCount = config.bramReadLatency - 1)
  }
}

// =============================================================================
// Verilog Generation Entry Point
// =============================================================================
object AxiTileDmaVerilog {
  def main(args: Array[String]): Unit = {
    val config = AxiTileDmaConfig(
      addressWidth = 32,
      pixelsPerClock = 4, // 4 pixels per clock = 128-bit AXI data bus (configurable: 1, 2, 4, 8, 16)
      tileWidth = 128, // 128x128 tile known at compile time
      tileHeight = 128,
      maxBurstLength = 256,
      maxOutstanding = 8,
      idWidth = 4,
      cdcDepth = 16, // 16-entry async CDC
      bramReadLatency = 4
    )

    SpinalConfig(
      targetDirectory = "rtl/AxiTileDma",
      defaultConfigForClockDomains = ClockDomainConfig(resetActiveLevel = LOW)
    ).withPrivateNamespace.generateVerilog(
      new AxiTileDma(
        config = config,
        ramClockDomain = ClockDomain.external("ram", withReset = true)
      )
    )
  }
}
