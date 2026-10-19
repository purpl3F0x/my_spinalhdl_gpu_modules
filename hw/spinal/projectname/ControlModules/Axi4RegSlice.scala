package projectname

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba4.axi._

case class Axi4RegSlice(config: Axi4Config) extends Component {
  val io = new Bundle {
    val input = slave(Axi4(config))
    val output = master(Axi4(config))
  }

  io.output.ar << io.input.ar.stage()
  io.output.aw << io.input.aw.stage()
  io.output.w << io.input.w.stage()
  io.input.r << io.output.r.stage()
  io.input.b << io.output.b.stage()
}

case class Axi4Pipeline(config: Axi4Config, stages: Int = 1) extends Component {
  require(stages >= 0)
  val io = new Bundle {
    val input = slave(Axi4(config))
    val output = master(Axi4(config))
  }

  val stageInsts = Array.fill(stages)(Axi4RegSlice(config))
  if (stages == 0) {
    io.output <> io.input
  } else {
    io.input <> stageInsts.head.io.input
    for (i <- 0 until stages - 1) {
      stageInsts(i).io.output <> stageInsts(i + 1).io.input
    }
    stageInsts.last.io.output <> io.output
  }
}
