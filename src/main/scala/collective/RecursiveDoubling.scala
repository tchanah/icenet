package icenet.collective

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._

import icenet.{NICKey, NICIOvonly, IceNetConsts, StreamChannel, StreamIO}

// === Parameters ===

case class RecursiveDoublingParams(
    Levels: Int = 4,   // Example: Max depth of storage/recursion
    DataElements: Int = 256, // Number of data elements per packet
    BytesPerElement: Int = 4, // Size of each data element
    dataWidth: Int = 64         // Width of the streaming interface in bits
) {
    // Derived parameters
    val bytesPerWord: Int = dataWidth / 8
    require(dataWidth % 8 == 0, "dataWidth must be a multiple of 8.")
    require(DataElements > 0, "DataElements must be positive.")
    // Total words = metadata word (1) + data words
    val numDataWords: Int = (DataElements*BytesPerElement) / bytesPerWord 
    val totalWordsPerPacket: Int = 1 + numDataWords
    val elementWidth: Int = 8 * BytesPerElement // Assuming data elements are bytes based on metadata description
    require(DataElements % bytesPerWord == 0, "For simplicity, assuming DataElements is a multiple of bytesPerWord")
    val elementsPerWord: Int = bytesPerWord / BytesPerElement

    // Width required for counters
    val wordCountBits: Int = log2Ceil(totalWordsPerPacket + 1) // Counts words being received/sent
    //val elementCountBits: Int = log2Ceil(DataElements)    // Counts elements during processing/storage access
    //val levelCountBits: Int = log2Ceil(Levels + 1) // Stores recursion level
}

// Key to access parameters in the configuration
case object RecursiveDoublingKey extends Field[Option[RecursiveDoublingParams]](None)


// === Chisel Module ===

class RecursiveDoubling(params: RecursiveDoublingParams) extends Module {
    val io = IO(new StreamIO(params.dataWidth))

    // --- Constants ---
    val BYTES_PER_WORD = params.bytesPerWord
    val ELEMENTS_PER_WORD = params.elementsPerWord
    val NUM_DATA_ELEMENTS = params.DataElements
    val MAX_RECURSION_LEVEL = params.Levels
    val NUM_DATA_WORDS = params.numDataWords
    val TOTAL_WORDS_PER_PACKET = params.totalWordsPerPacket
    val ELEMENT_WIDTH = params.elementWidth

    // --- State Machine ---
    val s_idle :: s_recv_data :: s_process :: s_send_meta :: s_send_data :: Nil = Enum(5)
    val state = RegInit(s_idle)

    // --- Metadata Registers ---
    // Stored when recv_meta completes
    val collectiveIdReg   = Reg(UInt(16.W))
    val collectiveTypeReg = Reg(UInt(8.W))
    val operationReg      = Reg(UInt(8.W))
    // Bytes 4, 5 are empty
    val maxLevelReg       = Reg(UInt(8.W)) // Store Max Level from packet
    val currentLevelReg   = Reg(UInt(8.W)) // Store Current Level from packet

    // --- Data Buffers and Storage ---
    // Temporary buffer for incoming data elements (bytes)
    val incomingDataBuffer = Reg(Vec(NUM_DATA_ELEMENTS, UInt(ELEMENT_WIDTH.W)))
    // Main storage registry: Levels x Elements
    // Note: Using Vec of Regs for synthesis. Consider Mem for larger structures.
    val storageRegistry = Reg(Vec(MAX_RECURSION_LEVEL, Vec(NUM_DATA_ELEMENTS, UInt(ELEMENT_WIDTH.W))))

    // --- Counters ---
    // Counts words received within a packet (metadata + data)
    val receivedWordCount = RegInit(0.U(params.wordCountBits.W))
    // Counts words sent within a packet (metadata + data)
    val sentWordCount = RegInit(0.U(params.wordCountBits.W))

    // --- Processing Wires ---
    val processedData = Reg(Vec(NUM_DATA_ELEMENTS, UInt(ELEMENT_WIDTH.W)))
    val nextLevel = Wire(UInt(8.W))
    val storeData = RegInit(false.B)
    val useProcessedDataForOutput = RegInit(false.B)
    
    // Default control signals
    nextLevel := currentLevelReg + 1.U // Default next level calculation

    // --- Input Handling ---
    val incoming_bits = io.in.bits
    val incoming_valid = io.in.valid
    val incoming_fire = io.in.fire // Convenience wire: io.in.valid && io.in.ready

    // --- Output Handling ---
    val outgoing_bits = Wire(new StreamChannel(params.dataWidth))

    // Assign default values (e.g., 0 or DontCare) to ensure the wire is always driven.
    // Using 0 is often simplest. Chisel's DontCare (:= chisel3.DontCare) can sometimes help synthesis tools.
    outgoing_bits.data := 0.U(params.dataWidth.W)
    outgoing_bits.keep := 0.U(params.bytesPerWord.W)
    outgoing_bits.last := false.B
    
    io.out.bits := outgoing_bits
    io.out.valid := false.B // Default

    // Default pass-through for unmodified metadata parts (can be overridden)
    // Assemble metadata fields into the first 8 bytes (64 bits)
    // Bytes: | 7(MSB) | 6     | 5 | 4 | 3      | 2      | 1    | 0(LSB)|
    // Field: | CurrLvl| MaxLvl| - | - | Op     | Type   | Coll ID     |
    val outgoingMetadataWord = Wire(UInt(params.dataWidth.W))
    outgoingMetadataWord := Cat(
        nextLevel(7,0),                      // Byte 7: Next Level (Calculated)
        maxLevelReg(7,0),                    // Byte 6: Max Level (Stored from input)
        0.U(8.W),                            // Byte 5: Empty
        0.U(8.W),                            // Byte 4: Empty
        operationReg,                        // Byte 3: Operation (Stored from input)
        collectiveTypeReg,                   // Byte 2: Type (Stored from input)
        collectiveIdReg(15, 8),              // Byte 1: Collective ID MSB (Stored from input)
        collectiveIdReg(7, 0)                // Byte 0: Collective ID LSB (Stored from input)
    )

    // --- State Machine Logic ---

    when(state === s_idle) {
        io.out.valid := false.B
        when(incoming_fire) {
            // First beat must be metadata
            // Extract metadata (assuming dataWidth >= 64)
            val metaWord = incoming_bits.data
            collectiveIdReg   := metaWord(15, 0)
            collectiveTypeReg := metaWord(23, 16)
            operationReg      := metaWord(31, 24)
            // Bytes 4, 5 (bits 47:32) are ignored/empty
            maxLevelReg       := metaWord(55, 48)
            currentLevelReg   := metaWord(63, 56)

            // Reset counters and transition
            receivedWordCount := 1.U // We've received the metadata word
            state := s_recv_data
        }
    }
    .elsewhen(state === s_recv_data) {
        io.out.valid := false.B
        when(incoming_fire) {
            // Calculate base index for this word's bytes in the buffer
            val baseElementIndex = (receivedWordCount - 1.U) * ELEMENTS_PER_WORD.U

            // Store incoming data bytes into the buffer
            for (i <- 0 until ELEMENTS_PER_WORD) {
                when((baseElementIndex + i.U) < NUM_DATA_ELEMENTS.U) { // Bounds check
                    incomingDataBuffer(baseElementIndex + i.U) := incoming_bits.data((i + 1) * ELEMENT_WIDTH - 1, i * ELEMENT_WIDTH)
                }
            }

            receivedWordCount := receivedWordCount + 1.U

            // Check if this is the last beat of the packet
            // Assumes 'last' is asserted on the final data beat
            when(incoming_bits.last) {
                state := s_process
                receivedWordCount := 0.U // Reset for next packet
            }
        }
    }
    .elsewhen(state === s_process) {
        // --- Combinational Processing Logic ---
        io.out.valid := false.B // Processing happens here, output starts in s_send_meta

        when(currentLevelReg === 0.U) {
            // Base case: Level 0
            nextLevel := 1.U
            useProcessedDataForOutput := false.B // Send incoming data directly
            storeData := true.B                 // Store incoming data at level 0

        } .otherwise {
            // Recursive case: Level > 0
            nextLevel := currentLevelReg + 1.U
            useProcessedDataForOutput := true.B // Send the calculated sum
            storeData := (currentLevelReg < maxLevelReg)

            // Perform element-wise addition with data from previous level
            val prevLevelData = storageRegistry(currentLevelReg - 1.U)
            for (i <- 0 until NUM_DATA_ELEMENTS) {
                // Assuming simple UInt addition (wraps on overflow)
                processedData(i) := incomingDataBuffer(i) + prevLevelData(i)
            }

            // Store result if not exceeding max level
            // when(currentLevelReg < maxLevelReg) {
            //     storeData := true.B
            // } .otherwise {
            //     storeData := false.B // Max level reached, only send
            // }
        }

        // --- Transition to Sending ---
        state := s_send_meta
        sentWordCount := 0.U // Reset output counter
    }
    .elsewhen(state === s_send_meta) {
        // Send the metadata word first
        io.out.valid := true.B
        outgoing_bits.data := outgoingMetadataWord
        outgoing_bits.last := (NUM_DATA_WORDS == 0).B // Only last if there's no data payload
        outgoing_bits.keep := (~0.U((params.dataWidth / 8).W)) // Assume all bytes valid

        // --- Storage Logic (triggered by storeData) ---
        // This happens combinationally based on the decision above,
        // the registers update at the clock edge when state transitions.
        when(storeData) {
            // Store either incoming data (Level 0) or processed data (Level > 0)
            // at the current level index.
            val dataToStore = Mux(currentLevelReg === 0.U, incomingDataBuffer, processedData)
            when (currentLevelReg < MAX_RECURSION_LEVEL.U) { // Ensure we don't write out of bounds
                storageRegistry(currentLevelReg) := dataToStore
            }
        }

        when(io.out.fire) {
            sentWordCount := 1.U
            if (NUM_DATA_WORDS > 0) {
                state := s_send_data
            } else {
                state := s_idle // No data words to send
            }
        }

    }
    .elsewhen(state === s_send_data) {
        io.out.valid := true.B

        // Select the correct data source based on processing decision
        val dataSource = Mux(useProcessedDataForOutput, processedData, incomingDataBuffer)

        // Assemble the data word to send
        val baseElementIndex = (sentWordCount - 1.U) * ELEMENTS_PER_WORD.U
        val dataWordVec = Wire(Vec(ELEMENTS_PER_WORD, UInt(ELEMENT_WIDTH.W)))
        for (i <- 0 until ELEMENTS_PER_WORD) {
            when((baseElementIndex + i.U) < NUM_DATA_ELEMENTS.U) { // Bounds check
                dataWordVec(i) := dataSource(baseElementIndex + i.U)
            }.otherwise {
                dataWordVec(i) := 0.U // Padding if needed (shouldn't happen with current requires)
            }
        }
        outgoing_bits.data := Cat(dataWordVec.reverse) // Cat expects MSB first

        // Set 'last' signal for the final data word
        val isLastDataWord = sentWordCount === TOTAL_WORDS_PER_PACKET.U - 1.U
        outgoing_bits.last := isLastDataWord
        outgoing_bits.keep := (~0.U((params.dataWidth / 8).W)) // Assume all bytes valid

        when(io.out.fire) {
            sentWordCount := sentWordCount + 1.U
            when(isLastDataWord) {
                state := s_idle // Packet fully sent
                sentWordCount := 0.U // Reset for next packet
            }
            // else remain in s_send_data
        }
    }

    // --- Input Ready Logic ---
    // Ready to accept input if in a receiving state and downstream is ready
    // Note: This connects input ready directly to output ready like the example.
    // For higher throughput, buffer status should be considered.
    io.in.ready := (state === s_idle || state === s_recv_data) && io.out.ready

    // Alternative Input Ready (Decoupled from output, assumes enough buffer space)
    // io.in.ready := (state === s_idle || state === s_recv_data)
}


// === Rocket Chip Config Fragment ===

// Add this config fragment to your Chipyard configuration
// Example: class MyConfig extends Config(
//    ...
//    new icenet.collective.WithRecursiveDoubling ++
//    ...
// )
class WithRecursiveDoubling(
    maxLevel: Int = 4,
    numElements: Int = 256,
    numBytesPerElement: Int = 4
) extends Config((site, here, up) => {
    case RecursiveDoublingKey => Some(RecursiveDoublingParams(
        Levels = maxLevel,
        DataElements = numElements,
        BytesPerElement = numBytesPerElement,
        dataWidth = IceNetConsts.NET_IF_WIDTH // Or get from NICKey if available/appropriate
    ))
    // Optional: If you need to connect this via diplomacy later,
    // you might override NICKey or add custom diplomacy nodes here.
})