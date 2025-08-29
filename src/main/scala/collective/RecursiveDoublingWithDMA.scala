package icenet.collective

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import icenet.{NICKey, NICIOvonly, IceNetConsts, StreamChannel, StreamIO}
import hardfloat._

// === Parameters ===

case class RecursiveDoublingWithDMAParams(
    Levels: Int = 4,   // Max depth of storage/recursion
    DataElements: Int = 256, // Number of data elements per packet
    BytesPerElement: Int = 4, // Size of each data element
    dataWidth: Int = 64,      // Width of the streaming interface in bits
    baseMemoryAddr: BigInt = 0x80000000L, // Base address for DMA operations
    sourceIds: Int = 256,     // Number of TileLink source IDs for concurrent transactions
    EnableDebug: Boolean = true // Elabor-time debug printing (no hardware cost when false)
) {
    // Derived parameters
    val bytesPerWord: Int = dataWidth / 8
    require(dataWidth % 8 == 0, "dataWidth must be a multiple of 8.")
    require(DataElements > 0, "DataElements must be positive.")
    // Total words = metadata word (1) + data words
    val numDataWords: Int = (DataElements*BytesPerElement) / bytesPerWord 
    val totalWordsPerPacket: Int = 1 + numDataWords
    val elementWidth: Int = 8 * BytesPerElement
    require(DataElements % bytesPerWord == 0, "For simplicity, assuming DataElements is a multiple of bytesPerWord")
    val elementsPerWord: Int = bytesPerWord / BytesPerElement

    // Width required for counters
    val wordCountBits: Int = log2Ceil(totalWordsPerPacket + 1)
    val levelCountBits: Int = log2Ceil(Levels + 1)
    
    // Memory layout: Each level gets a contiguous block
    val bytesPerLevel: Int = DataElements * BytesPerElement
    val wordsPerLevel: Int = (bytesPerLevel + bytesPerWord - 1) / bytesPerWord // Round up
}

// Key to access parameters in the configuration
case object RecursiveDoublingWithDMAKey extends Field[Option[RecursiveDoublingWithDMAParams]](None)

// === Chisel Module ===

class RecursiveDoublingWithDMA(val params: RecursiveDoublingWithDMAParams)(implicit p: Parameters) extends LazyModule {
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLMasterParameters.v1(
    sourceId = IdRange(0, params.sourceIds),
    name = "recursive-doubling-dma"
  )))))
  
  lazy val module = new RecursiveDoublingWithDMAModuleImp(this)
}

class RecursiveDoublingWithDMAModuleImp(outer: RecursiveDoublingWithDMA) extends LazyModuleImp(outer) {
    val io = IO(new StreamIO(outer.params.dataWidth))
    // Elabor-time debug helper (no hardware cost when disabled)
    private val dbgEnabled: Boolean = outer.params.EnableDebug
    @inline private def dprintf(msg: Printable): Unit = if (dbgEnabled) { printf(msg) }
    
    // Get the TileLink client interface
    val (tl, edge) = outer.node.out(0)

    // --- FPU Instantiation ---
    // Instantiate a single-precision floating-point adder.
    // Single precision: exponent = 8, significand = 24 (23 stored + 1 implicit)
    val fpAdder = Module(new AddRecFN(expWidth = 8, sigWidth = 24))
    val FPU_LATENCY = 5 // Example latency, check the FPU documentation for exact value

    // Provide default values to prevent "not fully initialized" errors.
    // These will be overridden when the FSM is in the s_fp_add_pipe state.
    fpAdder.io.a := 0.U
    fpAdder.io.b := 0.U
    fpAdder.io.roundingMode   := 0.U
    fpAdder.io.detectTininess := 0.U
    fpAdder.io.subOp          := false.B

    // --- Constants ---
    val BYTES_PER_WORD = outer.params.bytesPerWord
    val ELEMENTS_PER_WORD = outer.params.elementsPerWord
    val NUM_DATA_ELEMENTS = outer.params.DataElements
    val MAX_RECURSION_LEVEL = outer.params.Levels
    val NUM_DATA_WORDS = outer.params.numDataWords
    val TOTAL_WORDS_PER_PACKET = outer.params.totalWordsPerPacket
    val ELEMENT_WIDTH = outer.params.elementWidth
    val WORDS_PER_LEVEL = outer.params.wordsPerLevel
    val BYTES_PER_LEVEL = outer.params.bytesPerLevel

    // --- Data Buffers ---
    // Temporary buffer for incoming data elements
    val incomingDataBuffer = Reg(Vec(NUM_DATA_ELEMENTS, UInt(ELEMENT_WIDTH.W)))
    // Buffer for data read from memory
    val memoryReadBuffer = Reg(Vec(NUM_DATA_ELEMENTS, UInt(ELEMENT_WIDTH.W)))
    // Separate buffer for buffered packet data to avoid conflicts
    val bufferedPacketReadBuffer = Reg(Vec(NUM_DATA_ELEMENTS, UInt(ELEMENT_WIDTH.W)))
    // Buffer for processed data to be written to memory
    val processedDataBuffer = RegInit(VecInit(Seq.fill(NUM_DATA_ELEMENTS)(0.U(ELEMENT_WIDTH.W))))

    // --- Packet Buffer (stored in memory via DMA) ---
    // Local valid register for fast access (stored in memory)
    val packetBufferValid = RegInit(VecInit(Seq.fill(MAX_RECURSION_LEVEL - 1)(false.B)))
    // Sideband metadata for buffered packets (per level index)
    val packetBufferMetaId   = Reg(Vec(MAX_RECURSION_LEVEL - 1, UInt(16.W)))
    val packetBufferMetaType = Reg(Vec(MAX_RECURSION_LEVEL - 1, UInt(8.W)))
    val packetBufferMetaOp   = Reg(Vec(MAX_RECURSION_LEVEL - 1, UInt(8.W)))
    val packetBufferMetaMax  = Reg(Vec(MAX_RECURSION_LEVEL - 1, UInt(8.W)))
    
    // Buffer for writing packet info to memory
    val packetBufferWriteAddr = Reg(UInt(48.W))
    


    // Track validity of each level in memory
    val memoryValid = RegInit(VecInit(Seq.fill(MAX_RECURSION_LEVEL)(false.B)))

    // --- Extended State Machine ---
    val s_idle :: s_recv_data :: s_write_buffer :: s_wait_write_buffer :: s_process :: s_dma_write :: s_wait_write :: s_dma_read :: s_wait_read :: s_wait_read_done :: s_fp_add_pipe :: s_send_meta :: s_send_data :: s_check_buffer :: Nil = Enum(14)
    val state = RegInit(s_idle)
    val prevState = RegInit(s_idle)

    // --- Metadata Registers ---
    val collectiveIdReg   = Reg(UInt(16.W))
    val collectiveTypeReg = Reg(UInt(8.W))
    val operationReg      = Reg(UInt(8.W))
    val maxLevelReg       = Reg(UInt(8.W))
    val currentLevelReg   = Reg(UInt(8.W))

    // --- Counters ---
    val receivedWordCount = RegInit(0.U(outer.params.wordCountBits.W))
    val sentWordCount = RegInit(0.U(outer.params.wordCountBits.W))
    val dmaWordCount = RegInit(0.U(outer.params.wordCountBits.W))
    val waitCounter = RegInit(0.U(8.W))
    // Separate counters for read requests and responses
    val readReqCount = RegInit(0.U(outer.params.wordCountBits.W))
    val readWordCount = RegInit(0.U(outer.params.wordCountBits.W)) // used as response count
    // Add counters for managing the FPU pipeline
    val elementIdx = RegInit(0.U(log2Ceil(NUM_DATA_ELEMENTS + 1).W))

    // --- Processing Wires ---
    val processedData = RegInit(VecInit(Seq.fill(NUM_DATA_ELEMENTS)(0.U(ELEMENT_WIDTH.W))))
    val nextLevel = Wire(UInt(8.W))
    val storeData = RegInit(false.B)
    val useProcessedDataForOutput = RegInit(false.B)
    val dmaReadLevel = RegInit(0.U(outer.params.levelCountBits.W))
    val isReadingBufferedPacket = RegInit(false.B)  // Flag to distinguish between reading previous level data vs buffered packet data
    
    // Default control signals
    nextLevel := currentLevelReg + 1.U

    // --- Input/Output Handling ---
    val incoming_bits = io.in.bits
    val incoming_valid = io.in.valid
    val incoming_fire = io.in.fire

    val outgoing_bits = Wire(new StreamChannel(outer.params.dataWidth))
    outgoing_bits.data := 0.U(outer.params.dataWidth.W)
    outgoing_bits.keep := 0.U(outer.params.bytesPerWord.W)
    outgoing_bits.last := false.B
    
    io.out.bits := outgoing_bits
    io.out.valid := false.B

    // --- TileLink Default Signals ---
    tl.a.valid := false.B
    tl.a.bits := DontCare
    tl.d.ready := false.B

    // --- Memory Address Calculation ---
    def getLevelAddress(level: UInt): UInt = {
        (outer.params.baseMemoryAddr.U(48.W) + (level * BYTES_PER_LEVEL.U))
    }
    
    def getPacketBufferAddress(level: UInt): UInt = {
        // Packet buffer starts after the level data storage
        val levelDataEnd = outer.params.baseMemoryAddr.U(48.W) + (MAX_RECURSION_LEVEL.U * BYTES_PER_LEVEL.U)
        levelDataEnd + (level * (NUM_DATA_ELEMENTS * ELEMENT_WIDTH / 8).U)
    }

    // --- Metadata Assembly ---
    val outgoingMetadataWord = Wire(UInt(outer.params.dataWidth.W))
    outgoingMetadataWord := Cat(
        nextLevel(7,0),
        maxLevelReg(7,0),
        0.U(8.W),
        0.U(8.W),
        operationReg,
        collectiveTypeReg,
        collectiveIdReg(15, 8),
        collectiveIdReg(7, 0)
    )

    // --- State Machine Logic ---

    when(state === s_idle) {
        io.out.valid := false.B
        when(incoming_fire) {
            dprintf(p"[s_idle] Received metadata: level=${incoming_bits.data(63, 56)}, maxLevel=${incoming_bits.data(55, 48)}, collId=0x${Hexadecimal(incoming_bits.data(15, 0))}\n")
            val metaWord = incoming_bits.data
            val newCollectiveId = metaWord(15, 0)
            
            // Clear memory valid flags if this is a new test set (different collective ID)
            when(newCollectiveId =/= collectiveIdReg) {
                dprintf(p"[s_idle] New test set detected (collective ID changed from 0x${Hexadecimal(collectiveIdReg)} to 0x${Hexadecimal(newCollectiveId)}), clearing all valid flags\n")
                memoryValid.foreach(_ := false.B)
                packetBufferValid.foreach(_ := false.B)
                isReadingBufferedPacket := false.B
                storeData := false.B
                useProcessedDataForOutput := false.B

            }
            
            collectiveIdReg   := newCollectiveId
            collectiveTypeReg := metaWord(23, 16)
            operationReg      := metaWord(31, 24)
            maxLevelReg       := metaWord(55, 48)
            currentLevelReg   := metaWord(63, 56)

            // Start word count at 1 since we just received the metadata word
            receivedWordCount := 1.U
            state := s_recv_data
        }
    }
    .elsewhen(state === s_recv_data) {
        io.out.valid := false.B
        when(incoming_fire) {
            val baseElementIndex = (receivedWordCount - 1.U) * ELEMENTS_PER_WORD.U

            for (i <- 0 until ELEMENTS_PER_WORD) {
                when((baseElementIndex + i.U) < NUM_DATA_ELEMENTS.U) {
                    incomingDataBuffer(baseElementIndex + i.U) := incoming_bits.data((i + 1) * ELEMENT_WIDTH - 1, i * ELEMENT_WIDTH)
                }
            }
            
            // Debug: print when we're writing the last word
            when(receivedWordCount === NUM_DATA_WORDS.U) {
                dprintf(p"[s_recv_data] Writing last word: baseElementIndex=${baseElementIndex}, elements[${baseElementIndex}]=0x${Hexadecimal(incoming_bits.data(ELEMENT_WIDTH-1, 0))}, elements[${baseElementIndex + 1.U}]=0x${Hexadecimal(incoming_bits.data(2*ELEMENT_WIDTH-1, ELEMENT_WIDTH))}\n")
            }
            
            // Debug: print constants
            when(receivedWordCount === 1.U) {
                dprintf(p"[s_recv_data] Constants: NUM_DATA_WORDS=${NUM_DATA_WORDS}, WORDS_PER_LEVEL=${WORDS_PER_LEVEL}, ELEMENTS_PER_WORD=${ELEMENTS_PER_WORD}\n")
            }

            receivedWordCount := receivedWordCount + 1.U

            when(incoming_bits.last) {
                // Only level 0 can be processed immediately in a new test set
                // For higher levels, we need to ensure the previous level data is from the current test set
                val canProcess = currentLevelReg === 0.U || 
                                (currentLevelReg > 0.U && memoryValid(currentLevelReg - 1.U) && 
                                // Additional check: ensure we're not processing a higher level before level 0
                                memoryValid(0.U))
                
                when(canProcess) {
                    dprintf(p"[s_recv_data] Can process immediately: level=${currentLevelReg}, canProcess=${canProcess}\n")
                    state := s_process
                } .otherwise {
                    dprintf(p"[s_recv_data] Cannot process immediately, buffering: level=${currentLevelReg}\n")
                    // Set up packet buffer write directly
                    val bufferIndex = currentLevelReg - 1.U
                    packetBufferWriteAddr := getPacketBufferAddress(bufferIndex)
                    
                    // Save sideband metadata locally
                    packetBufferMetaId(bufferIndex)   := collectiveIdReg
                    packetBufferMetaType(bufferIndex) := collectiveTypeReg
                    packetBufferMetaOp(bufferIndex)   := operationReg
                    packetBufferMetaMax(bufferIndex)  := maxLevelReg
                    
                    state := s_write_buffer
                    dmaWordCount := 0.U
                }
                receivedWordCount := 0.U
            }
        }
    }
    .elsewhen(state === s_process) {
        dprintf(p"[s_process] Starting processing: currentLevel=${currentLevelReg}, maxLevel=${maxLevelReg}\n")
        io.out.valid := false.B

        when(currentLevelReg === 0.U) {
            // Base case: Level 0
            nextLevel := 1.U
            useProcessedDataForOutput := false.B
            storeData := true.B
            state := s_dma_write
            dmaWordCount := 0.U
        } .otherwise {
            // Recursive case: Level > 0 - need to read previous level from memory
            nextLevel := currentLevelReg + 1.U
            useProcessedDataForOutput := true.B
            storeData := (currentLevelReg < maxLevelReg)
            dmaReadLevel := currentLevelReg - 1.U
            // Ensure we're reading level data, not buffered packet data
            isReadingBufferedPacket := false.B
            state := s_dma_read
            dmaWordCount := 0.U
            readWordCount := 0.U
            readReqCount := 0.U
        }
    }
    .elsewhen(state === s_write_buffer) {
        // Write packet info to memory using DMA (one outstanding Put at a time)
        when(dmaWordCount < WORDS_PER_LEVEL.U) {
            tl.a.valid := true.B
            tl.a.bits.opcode := TLMessages.PutFullData
            tl.a.bits.param := 0.U
            tl.a.bits.size := log2Ceil(BYTES_PER_WORD).U
            // Serialize, use a fixed safe source ID
            tl.a.bits.source := 0.U
            tl.a.bits.address := packetBufferWriteAddr + (dmaWordCount << log2Ceil(BYTES_PER_WORD).U)
            tl.a.bits.mask := (~0.U(BYTES_PER_WORD.W))

            // Pack data elements into word directly from incomingDataBuffer
            val baseElementIndex = dmaWordCount * ELEMENTS_PER_WORD.U
            val dataWord = Wire(UInt(outer.params.dataWidth.W))
            val dataVec = Wire(Vec(ELEMENTS_PER_WORD, UInt(ELEMENT_WIDTH.W)))
            // Default initialize to zero to avoid uninitialized sinks
            for (i <- 0 until ELEMENTS_PER_WORD) { dataVec(i) := 0.U }
            for (i <- 0 until ELEMENTS_PER_WORD) {
                when((baseElementIndex + i.U) < NUM_DATA_ELEMENTS.U) {
                    dataVec(i) := incomingDataBuffer(baseElementIndex + i.U)
                }
            }
            dataWord := Cat(dataVec.reverse)

            tl.a.bits.data := dataWord

            when(tl.a.fire) {
                dprintf(p"[s_write_buffer] Write packet data: level=${currentLevelReg}, word=${dmaWordCount}, addr=0x${Hexadecimal(packetBufferWriteAddr + (dmaWordCount << log2Ceil(BYTES_PER_WORD).U))}, data=0x${Hexadecimal(dataWord)}\n")
                state := s_wait_write_buffer
            }
        }
    }
    .elsewhen(state === s_wait_write_buffer) {
        // Wait for AccessAck per buffered write
        tl.d.ready := true.B
        when(tl.d.valid && tl.d.bits.opcode === TLMessages.AccessAck && tl.d.bits.source === 0.U) {
            val nextCount = dmaWordCount + 1.U
            dprintf(p"[s_wait_write_buffer] AccessAck received for word ${dmaWordCount} (next=${nextCount})\n")
            dmaWordCount := nextCount
            when(nextCount === WORDS_PER_LEVEL.U) {
                dprintf(p"[s_wait_write_buffer] Packet buffer write complete for level ${currentLevelReg}\n")
                // Mark buffer as valid only after DMA write completes
                val bufferIndex = currentLevelReg - 1.U
                packetBufferValid(bufferIndex) := true.B
                state := s_idle
            }.otherwise {
                state := s_write_buffer
            }
        }.elsewhen(tl.d.valid) {
            // Unexpected response while waiting for write ack
            dprintf(p"[s_wait_write_buffer][WARN] Unexpected D resp opcode=${tl.d.bits.opcode} source=${tl.d.bits.source}\n")
        }
    }
    .elsewhen(state === s_dma_write) {
        // Write data to memory for current level (one outstanding Put at a time)
        when(dmaWordCount < WORDS_PER_LEVEL.U) {
            tl.a.valid := true.B
            tl.a.bits.opcode := TLMessages.PutFullData
            tl.a.bits.param := 0.U
            tl.a.bits.size := log2Ceil(BYTES_PER_WORD).U
            // Use a fixed safe source ID since we serialize writes
            tl.a.bits.source := 0.U
            tl.a.bits.address := getLevelAddress(currentLevelReg) + (dmaWordCount << log2Ceil(BYTES_PER_WORD).U)
            tl.a.bits.mask := (~0.U(BYTES_PER_WORD.W))

            // Pack data elements into word
            val baseElementIndex = dmaWordCount * ELEMENTS_PER_WORD.U
            val dataWord = Wire(UInt(outer.params.dataWidth.W))
            val dataVec = Wire(Vec(ELEMENTS_PER_WORD, UInt(ELEMENT_WIDTH.W)))

            // Select correct data source: incoming data for level 0, processed data for higher levels
            val dataSource = Mux(currentLevelReg === 0.U, incomingDataBuffer, processedDataBuffer)

            // Default initialize to zero to avoid uninitialized sinks
            for (i <- 0 until ELEMENTS_PER_WORD) { dataVec(i) := 0.U }
            for (i <- 0 until ELEMENTS_PER_WORD) {
                when((baseElementIndex + i.U) < NUM_DATA_ELEMENTS.U) {
                    dataVec(i) := dataSource(baseElementIndex + i.U)
                }
            }
            dataWord := Cat(dataVec.reverse)
            tl.a.bits.data := dataWord

            when(tl.a.fire) {
                dprintf(p"[s_dma_write] Write request: level=${currentLevelReg}, word=${dmaWordCount}, addr=0x${Hexadecimal(getLevelAddress(currentLevelReg) + (dmaWordCount << log2Ceil(BYTES_PER_WORD).U))}, data=0x${Hexadecimal(dataWord)}\n")
                // Also print the element indices and values used to form this word
                val eIdx0 = baseElementIndex
                val eIdx1 = baseElementIndex + 1.U
                val e0 = Mux(eIdx0 < NUM_DATA_ELEMENTS.U, dataSource(eIdx0), 0.U)
                val e1 = Mux(eIdx1 < NUM_DATA_ELEMENTS.U, dataSource(eIdx1), 0.U)
                dprintf(p"[s_dma_write] Elements: [${eIdx0}] = 0x${Hexadecimal(e0)}, [${eIdx1}] = 0x${Hexadecimal(e1)}\n")
                // Debug: print processedDataBuffer[255] when writing the last word
                when(dmaWordCount === 127.U) {
                    dprintf(p"[s_dma_write] processedDataBuffer[255] = 0x${Hexadecimal(processedDataBuffer(255))}\n")
                }
                // Wait for AccessAck before issuing the next Put
                state := s_wait_write
            }
        }
    }
    .elsewhen(state === s_wait_write) {
        // Wait for AccessAck of the Put before proceeding
        tl.d.ready := true.B
        when(tl.d.valid && tl.d.bits.opcode === TLMessages.AccessAck && tl.d.bits.source === 0.U) {
            // One write completed
            val nextCount = dmaWordCount + 1.U
            dprintf(p"[s_wait_write] AccessAck received for word ${dmaWordCount} (next=${nextCount})\n")
            dmaWordCount := nextCount
            when(nextCount === WORDS_PER_LEVEL.U) {
                memoryValid(currentLevelReg) := true.B
                state := s_send_meta
                sentWordCount := 0.U
            }.otherwise {
                state := s_dma_write
            }
        }.elsewhen(tl.d.valid) {
            // Unexpected response while waiting for write ack
            dprintf(p"[s_wait_write][WARN] Unexpected D resp opcode=${tl.d.bits.opcode} source=${tl.d.bits.source}\n")
        }
    }
    .elsewhen(state === s_dma_read) {
        // Read data from memory for previous level or buffered packet, one word at a time (no pipelining)
        when(readReqCount < WORDS_PER_LEVEL.U) {
            tl.a.valid := true.B
            tl.a.bits.opcode := TLMessages.Get
            tl.a.bits.param := 0.U
            tl.a.bits.size := log2Ceil(BYTES_PER_WORD).U
            // Keep source ID strictly within allowed range; single outstanding read at a time
            tl.a.bits.source := 0.U

            // Choose address based on what we're reading
            val readAddr = Mux(isReadingBufferedPacket,
                                getPacketBufferAddress(dmaReadLevel) + (readReqCount << log2Ceil(BYTES_PER_WORD).U),
                                getLevelAddress(dmaReadLevel) + (readReqCount << log2Ceil(BYTES_PER_WORD).U))
            tl.a.bits.address := readAddr
            tl.a.bits.mask := (~0.U(BYTES_PER_WORD.W))

            when(tl.a.fire) {
                when(isReadingBufferedPacket) {
                    dprintf(p"[s_dma_read] Read request: level=${dmaReadLevel}, word=${readReqCount}, addr=0x${Hexadecimal(readAddr)} (expecting buffered packet data)\n")
                }.otherwise {
                    dprintf(p"[s_dma_read] Read request: level=${dmaReadLevel}, word=${readReqCount}, addr=0x${Hexadecimal(readAddr)} (expecting data from level ${dmaReadLevel})\n")
                }
                // One request issued; move to wait for its response
                state := s_wait_read
                waitCounter := 0.U
            }
        }
    }
    .elsewhen(state === s_wait_read) {
        // Handle TileLink responses for level data read (one at a time)
        tl.d.ready := true.B
        when(tl.d.valid && tl.d.bits.opcode === TLMessages.AccessAckData && tl.d.bits.source === 0.U) {
            val dataWord = tl.d.bits.data
            dprintf(p"[s_wait_read] Read response: data=0x${Hexadecimal(dataWord)}\n")

            // Current word index equals number of words received so far
            val wordIndex = readWordCount

            // Unpack the data word into individual elements
            for (i <- 0 until ELEMENTS_PER_WORD) {
                val elementIndex = wordIndex * ELEMENTS_PER_WORD.U + i.U
                when(elementIndex < NUM_DATA_ELEMENTS.U) {
                    val elementStart = i * ELEMENT_WIDTH
                    val elementEnd = elementStart + ELEMENT_WIDTH - 1
                    val extractedData = dataWord(elementEnd, elementStart)
                    
                    // Use different buffers based on what we're reading
                    when(isReadingBufferedPacket) {
                        bufferedPacketReadBuffer(elementIndex) := extractedData
                    }.otherwise {
                        memoryReadBuffer(elementIndex) := extractedData
                    }
                    
                    // Debug: print unpacking for the last word
                    when(wordIndex === 127.U && i.U === 1.U) {
                        dprintf(p"[s_wait_read] Unpacking last word: dataWord=0x${Hexadecimal(dataWord)}, elementIndex=${elementIndex}, elementStart=${elementStart}, elementEnd=${elementEnd}, extracted=0x${Hexadecimal(extractedData)}\n")
                    }
                }
            }

            // Acknowledge the response and update counters
            readWordCount := readWordCount + 1.U
            readReqCount := readReqCount + 1.U

            // If more words to fetch, issue next request; else proceed
            when(readWordCount + 1.U < WORDS_PER_LEVEL.U) {
                state := s_dma_read
                waitCounter := 0.U
            }.otherwise {
                dprintf(p"[s_wait_read] All responses received (${readWordCount + 1.U} words).\n")
                // Defer copies/computation by one cycle so the last beat is visible in registers
                state := s_wait_read_done
            }
        }.elsewhen(tl.d.valid) {
            // Unexpected response while waiting for read data
            dprintf(p"[s_wait_read][WARN] Unexpected D resp opcode=${tl.d.bits.opcode} source=${tl.d.bits.source}\n")
        }.otherwise {
            // Simple timeout on a single response wait
            when(waitCounter > 500.U) {
                dprintf(p"[s_wait_read] Timeout waiting for read response at word ${readWordCount}. Aborting read.\n")
                // Fail safe: give up this read to avoid deadlock
                when(isReadingBufferedPacket) {
                    isReadingBufferedPacket := false.B
                    packetBufferValid(currentLevelReg - 1.U) := false.B
                }
                state := s_idle
            }.otherwise {
                waitCounter := waitCounter + 1.U
            }
        }
    }

    .elsewhen(state === s_wait_read_done) {
        // One-cycle handoff after final read response so registered buffers settle
        when(isReadingBufferedPacket) {
            // Now copy buffered packet into incomingDataBuffer
            dprintf(p"[s_wait_read_done] Copy buffered packet: bufferedPacketReadBuffer[255]=0x${Hexadecimal(bufferedPacketReadBuffer(255))}\n")
            incomingDataBuffer := bufferedPacketReadBuffer
            packetBufferValid(currentLevelReg - 1.U) := false.B
            isReadingBufferedPacket := false.B
            state := s_process
        }.otherwise {
            // Data is ready, start feeding the FPU pipeline
            dprintf(p"[s_wait_read_done] Read complete. Starting FP addition pipeline for level ${currentLevelReg}\n")
            state := s_fp_add_pipe
            elementIdx := 0.U
        }
    }
    
    .elsewhen(state === s_fp_add_pipe) {
        // ACTION: Provide inputs, calculate sum, and store the result in the same cycle.
        fpAdder.io.a := incomingDataBuffer(elementIdx)
        fpAdder.io.b := memoryReadBuffer(elementIdx)
        
        // Set control signals
        fpAdder.io.roundingMode   := "b000".U
        fpAdder.io.detectTininess := 1.U
        fpAdder.io.subOp          := false.B

        // Since the FPU is combinational, the result is available immediately.
        val sum = fpAdder.io.out
        processedData(elementIdx) := sum
        when(storeData) {
            processedDataBuffer(elementIdx) := sum
        }

        // TRANSITION: When the last element is processed, move to the next state.
        when(elementIdx === (NUM_DATA_ELEMENTS - 1).U) {
            dprintf(p"[s_fp_add_pipe] All FP results collected.\n")
            if (dbgEnabled) {
                dprintf(p"[s_fp_add_pipe] Last FP sum: 0x${Hexadecimal(sum)}\n")
            }
            when(storeData) {
                dprintf(p"[s_fp_add_pipe] Storing processed data to memory for level ${currentLevelReg}\n")
                state := s_dma_write
                dmaWordCount := 0.U
            } .otherwise {
                dprintf(p"[s_fp_add_pipe] Sending processed data directly (no storage)\n")
                state := s_send_meta
                sentWordCount := 0.U
            }
        } .otherwise {
            elementIdx := elementIdx + 1.U
        }
    }

    .elsewhen(state === s_send_meta) {
        io.out.valid := true.B
        outgoing_bits.data := outgoingMetadataWord
        outgoing_bits.last := (NUM_DATA_WORDS == 0).B
        outgoing_bits.keep := (~0.U((outer.params.dataWidth / 8).W))

        when(io.out.fire) {
            dprintf(p"[s_send_meta] OUT META: nextLevel=${nextLevel}, maxLevel=${maxLevelReg}, op=${operationReg}, type=${collectiveTypeReg}, collId=0x${Hexadecimal(collectiveIdReg)}\n")
            sentWordCount := 1.U
            if (NUM_DATA_WORDS > 0) {
                state := s_send_data
            } else {
                state := s_check_buffer
            }
        }
    }
    .elsewhen(state === s_send_data) {
        io.out.valid := true.B

        // Select correct data source: processed data for recursive cases, incoming data for level 0
        // For the final level (storeData == false), use freshly computed 'processedData' instead of the buffer
        val dataSource = Mux(useProcessedDataForOutput,
            Mux(storeData, processedDataBuffer, processedData),
            incomingDataBuffer)
        val baseElementIndex = (sentWordCount - 1.U) * ELEMENTS_PER_WORD.U
        val dataWordVec = Wire(Vec(ELEMENTS_PER_WORD, UInt(ELEMENT_WIDTH.W)))
        
        for (i <- 0 until ELEMENTS_PER_WORD) {
            when((baseElementIndex + i.U) < NUM_DATA_ELEMENTS.U) {
                dataWordVec(i) := dataSource(baseElementIndex + i.U)
            }.otherwise {
                dataWordVec(i) := 0.U
            }
        }
        outgoing_bits.data := Cat(dataWordVec.reverse)

        val isLastDataWord = sentWordCount === TOTAL_WORDS_PER_PACKET.U - 1.U
        outgoing_bits.last := isLastDataWord
        outgoing_bits.keep := (~0.U((outer.params.dataWidth / 8).W))

        when(io.out.fire) {
            when(isLastDataWord) {
                when(currentLevelReg === maxLevelReg) {
                    memoryValid.foreach(_ := false.B)
                }
                state := s_check_buffer
                sentWordCount := 0.U
            }.otherwise {
                sentWordCount := sentWordCount + 1.U
            }
        }
    }
    .elsewhen(state === s_check_buffer) {
        io.out.valid := false.B
        
        val nextLevel = currentLevelReg + 1.U
        dprintf(p"[s_check_buffer] Checking level ${nextLevel}, buffer valid=${packetBufferValid(nextLevel - 1.U)}\n")
        
        when(nextLevel < MAX_RECURSION_LEVEL.U && packetBufferValid(nextLevel - 1.U)) {
            dprintf(p"[s_check_buffer] Found valid buffer entry, loading into registers\n")
            
            // Load metadata from sideband registers for buffered level
            val bufferedIndex = nextLevel - 1.U
            collectiveIdReg   := packetBufferMetaId(bufferedIndex)
            collectiveTypeReg := packetBufferMetaType(bufferedIndex)
            operationReg      := packetBufferMetaOp(bufferedIndex)
            maxLevelReg       := packetBufferMetaMax(bufferedIndex)
            currentLevelReg   := nextLevel
            
            // Set up DMA read for buffered packet data
            dmaReadLevel := nextLevel - 1.U
            isReadingBufferedPacket := true.B
            state := s_dma_read
            dmaWordCount := 0.U
            waitCounter := 0.U  // Reset timeout counter
            readReqCount := 0.U
            readWordCount := 0.U
            
            // Mark buffer as invalid
            packetBufferValid(nextLevel - 1.U) := false.B
            
            // Do not override the read start; we'll read buffered data first
        } .otherwise {
            dprintf(p"[s_check_buffer] No valid buffer entry, returning to idle\n")
            state := s_idle
        }
    }
    
    // --- Input Ready Logic ---
    io.in.ready := (state === s_idle || state === s_recv_data)

    // --- Debug ---
    when(state =/= prevState) {
        dprintf(p"[STATE] Transition: ${prevState} -> ${state} (level=${currentLevelReg}, recvWord=${receivedWordCount}, sentWord=${sentWordCount})\n")
        prevState := state
    }
}

// === Rocket Chip Config Fragment ===

class WithRecursiveDoublingWithDMA(
    maxLevel: Int = 4,
    numElements: Int = 256,
    numBytesPerElement: Int = 4,
    baseAddr: BigInt = 0x80000000L,
    sourceIds: Int = 8,
    EnableDebug: Boolean = true
) extends Config((site, here, up) => {
    case RecursiveDoublingWithDMAKey => Some(RecursiveDoublingWithDMAParams(
        Levels = maxLevel,
        DataElements = numElements,
        BytesPerElement = numBytesPerElement,
        dataWidth = IceNetConsts.NET_IF_WIDTH,
        baseMemoryAddr = baseAddr,
        sourceIds = sourceIds,
        EnableDebug = EnableDebug
    ))
}) 