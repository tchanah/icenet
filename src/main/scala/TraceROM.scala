package icenet

import chisel3._
import chisel3.util._
import testchipip.StreamChannel

class TraceROM(ifWidth: Int) extends BlackBox {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Bool())
    val stream = Decoupled(new StreamChannel(ifWidth))
    val macAddr = Output(UInt(48.W))
    val length = Output(UInt(32.W))
  })
}
