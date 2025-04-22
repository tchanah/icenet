// Location: src/main/scala/chipyard/example/PacketModifier.scala
// (Can be in the same file as the module or a separate Params.scala)
package icenet.collective

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._

import icenet.{NICKey, NICIOvonly, IceNetConsts, StreamChannel, StreamIO}

// Define a case class for any parameters our PacketModifier might need.
// Even if it needs no parameters currently besides dataWidth (which we get elsewhere),
// having the structure is good practice. We could add 'enable' or 'modificationType' here later.
case class PacketModifierParams(
  // Example parameter: Maybe we want to enable/disable modification via config
  // enableModification: Boolean = true
  // For now, it's empty, just signifying presence.
)

// Define a Key (object extending Field) to access these parameters in the configuration.
// The type is Field[Option[PacketModifierParams]] which allows the module to be optional.
// If the Key is not set (or set to None), the module won't be instantiated.
case object PacketModifierKey extends Field[Option[PacketModifierParams]](None)

/**
 * Revised Chisel module using a plausible StreamChannel interface.
 * It modifies the first byte of the data in the *first beat* of a packet.
 * Uses state to detect the first beat based on the 'last' signal of the previous beat.
 *
 * @param dataWidth The width of the stream data bus (e.g., NET_IF_WIDTH)
 */
class PacketModifier(val dataWidth: Int = IceNetConsts.NET_IF_WIDTH) extends Module {
  val io = IO(new StreamIO(dataWidth))

  // State to track if the previous beat was the last one (or if it's the beginning)
  // If last beat was seen, the next valid beat is the 'first' beat of a new packet.
  val s_idle :: s_busy :: Nil = Enum(2)
  val state = RegInit(s_idle)

  // Default pass-through assignments
  val modified_bits = Wire(new StreamChannel(dataWidth))
  modified_bits := io.in.bits

  val is_first_beat = WireDefault(false.B)

  // State transition and first beat detection
  when(io.in.fire) { // When a beat is successfully transferred IN
    when(state === s_idle) {
      is_first_beat := true.B // First beat since idle
      when(!io.in.bits.last) {
        state := s_busy // Packet continues
      }
    } .elsewhen(state === s_busy) {
      when(io.in.bits.last) {
        state := s_idle // Packet ends, go back to idle
      }
    }
  }

  // Apply modification only on the first beat when valid
  when(io.in.valid && is_first_beat) {
    // Invert the first byte (bits 7:0) of the data payload
    val original_data = io.in.bits.data
    val first_byte = original_data(7, 0)
    val inverted_first_byte = ~first_byte

    // Ensure dataWidth is considered if it's larger than 8 bits
    if (dataWidth > 8) {
        val remaining_data = original_data(dataWidth - 1, 8)
        modified_bits.data := Cat(remaining_data, inverted_first_byte)
    } else {
        modified_bits.data := inverted_first_byte
    }
    // NOTE: We might also need to consider the 'keep' bits. If keep(0) is low,
    // the first byte isn't valid, and maybe shouldn't be modified.
    // For simplicity, this example modifies it regardless of 'keep'.
  }

  // --- Connection Logic ---
  io.out.valid := io.in.valid
  io.out.bits  := modified_bits 

  io.in.ready  := io.out.ready // Handle backpressure
}

// RecursiveDoubling config fragment to set default parameters
class WithPacketModifier() extends Config((site, here, up) => {
    case PacketModifierKey => Some(PacketModifierParams())
})