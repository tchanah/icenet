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
    Levels: Int             = 4,            // Max depth of storage/recursion
    DataElements: Int       = 256,          // Number of data elements per packet
    BytesPerElement: Int    = 4,            // Size of each data element
    dataWidth: Int          = 64,           // Width of the streaming interface in bits
    baseMemoryAddr: BigInt  = 0x80000000L,  // Base address for DMA operations
    sourceIds: Int          = 256,          // Number of TileLink source IDs for concurrent transactions
    MaxChunks: Int          = 1024,         // Maximum number of logical chunks
    numMemoryBlocks: Int    = 1024,         // For 1MB RAM with 1KB chunks
    EnableDebug: Boolean    = true          // Elabor-time debug printing (no hardware cost when false)
) {
    // Derived parameters
    val bytesPerWord: Int           = dataWidth / 8
    require(dataWidth % 8 == 0, "dataWidth must be a multiple of 8.")
    require(DataElements > 0, "DataElements must be positive.")
    require(MaxChunks > 0, "MaxChunks must be positive.")
    
    // Total words = metadata word (2) + data words
    val numDataWords: Int           = (DataElements*BytesPerElement) / bytesPerWord 
    val totalWordsPerPacket: Int    = 2 + numDataWords
    val elementWidth: Int           = 8 * BytesPerElement
    require(DataElements % bytesPerWord == 0, "For simplicity, assuming DataElements is a multiple of bytesPerWord")
    val elementsPerWord: Int        = bytesPerWord / BytesPerElement

    // Width required for counters
    val wordCountBits: Int          = log2Ceil(totalWordsPerPacket + 1)
    val levelCountBits: Int         = log2Ceil(Levels + 1)
    val chunkIndexBits: Int         = log2Ceil(MaxChunks + 1)
    
    // Memory layout: Each level gets a contiguous block, with space for chunked data
    val bytesPerChunk: Int          = DataElements * BytesPerElement  // 1KB per chunk
    val wordsPerChunk: Int          = (bytesPerChunk + bytesPerWord - 1) / bytesPerWord // Round up
    val maxBytesPerLevel: Int       = MaxChunks * bytesPerChunk  // Up to 1MB per level
    val maxWordsPerLevel: Int       = (maxBytesPerLevel + bytesPerWord - 1) / bytesPerWord
    
    // Legacy compatibility
    val bytesPerLevel: Int          = bytesPerChunk  // For single-chunk operations
    val wordsPerLevel: Int          = wordsPerChunk  // For single-chunk operations
}

// Key to access parameters in the configuration
case object RecursiveDoublingWithDMAKey extends Field[Option[RecursiveDoublingWithDMAParams]](None)

// === Chisel Module ===

class RecursiveDoublingWithDMA(val params: RecursiveDoublingWithDMAParams)(implicit p: Parameters) extends LazyModule {
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLMasterParameters.v1(
    sourceId    = IdRange(0, params.sourceIds),
    name        = "recursive-doubling-dma"
  )))))
  
  lazy val module = new RecursiveDoublingWithDMAModuleImp(this)
}

class RecursiveDoublingWithDMAModuleImp(outer: RecursiveDoublingWithDMA) extends LazyModuleImp(outer) {
    val io      = IO(new StreamIO(outer.params.dataWidth))
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
    fpAdder.io.a                := 0.U
    fpAdder.io.b                := 0.U
    fpAdder.io.roundingMode     := 0.U
    fpAdder.io.detectTininess   := 0.U
    fpAdder.io.subOp            := false.B

    // --- Constants ---
    val BYTES_PER_WORD          = outer.params.bytesPerWord
    val ELEMENTS_PER_WORD       = outer.params.elementsPerWord
    val NUM_DATA_ELEMENTS       = outer.params.DataElements
    val MAX_RECURSION_LEVEL     = outer.params.Levels
    val NUM_DATA_WORDS          = outer.params.numDataWords
    val ELEMENT_WIDTH           = outer.params.elementWidth
    val WORDS_PER_CHUNK         = outer.params.wordsPerChunk
    val BYTES_PER_CHUNK         = outer.params.bytesPerChunk
    val MAX_CHUNKS              = outer.params.MaxChunks
    val CHUNK_INDEX_BITS        = outer.params.chunkIndexBits
    val MAX_BYTES_PER_LEVEL     = outer.params.maxBytesPerLevel
    val WORDS_PER_LEVEL         = outer.params.wordsPerLevel
    val BYTES_PER_LEVEL         = outer.params.bytesPerLevel

    // --- Data Buffers ---
    // Temporary buffer for incoming data elements
    val incomingDataBuffer      = Reg(Vec(NUM_DATA_ELEMENTS, UInt(ELEMENT_WIDTH.W)))
    // Buffer for data read from memory
    val memoryReadBuffer        = Reg(Vec(NUM_DATA_ELEMENTS, UInt(ELEMENT_WIDTH.W)))
    // Separate buffer for buffered packet data to avoid conflicts
    // Buffer for processed data to be written to memory
    val processedDataBuffer     = RegInit(VecInit(Seq.fill(NUM_DATA_ELEMENTS)(0.U(ELEMENT_WIDTH.W))))
    
    // --- Chunk Management ---
    // Track total chunks for current collective operation
    val totalChunksForCollective = RegInit(0.U(CHUNK_INDEX_BITS.W))
    
    // Chunk arrival tracking per level (bit vector indicating which chunks have arrived)
    // Using a memory-efficient representation: each level has a bit vector
    val chunkArrivedBits        = RegInit(VecInit(Seq.fill(MAX_RECURSION_LEVEL)(VecInit(Seq.fill(MAX_CHUNKS)(false.B)))))
    
    // Chunk processing tracking per level (bit vector indicating which chunks have been processed)
    val chunkProcessedBits      = RegInit(VecInit(Seq.fill(MAX_RECURSION_LEVEL)(VecInit(Seq.fill(MAX_CHUNKS)(false.B)))))

    // --- Dynamic Memory Management ---
    val NUM_MEM_BLOCKS = outer.params.numMemoryBlocks
    // A bitmap to track free 1KB blocks in the RAM. true = free.
    val memBlockFree = RegInit(VecInit(Seq.fill(NUM_MEM_BLOCKS)(true.B)))
    val BLOCK_INDEX_BITS = log2Ceil(NUM_MEM_BLOCKS)

    // Tables to store the block index for each chunk's data
    val incomingChunkBlockIndex = Reg(Vec(MAX_RECURSION_LEVEL, Vec(MAX_CHUNKS, UInt(BLOCK_INDEX_BITS.W))))
    val processedChunkBlockIndex = Reg(Vec(MAX_RECURSION_LEVEL, Vec(MAX_CHUNKS, UInt(BLOCK_INDEX_BITS.W))))

    // --- Extended State Machine ---
    val s_idle :: s_recv_meta2 :: s_recv_data :: s_dma_write :: s_wait_write :: s_dma_read :: s_wait_read :: s_wait_read_done :: s_fp_add_pipe :: s_send_meta :: s_send_meta2 :: s_send_data :: s_check_chunks :: Nil = Enum(13)
    val state                   = RegInit(s_idle)
    val prevState               = RegInit(s_idle)

    // --- Metadata Registers ---
    val collectiveIdReg         = Reg(UInt(16.W))
    val collectiveTypeReg       = Reg(UInt(8.W))
    val operationReg            = Reg(UInt(8.W))
    val maxLevelReg             = Reg(UInt(8.W))
    val currentLevelReg         = Reg(UInt(8.W))
    val chunkIndexReg           = Reg(UInt(32.W))
    val totalChunksReg          = Reg(UInt(32.W))
    
    // Additional chunk processing state
    val processingChunkIndex    = RegInit(0.U(CHUNK_INDEX_BITS.W))
    val processingLevel         = RegInit(0.U(outer.params.levelCountBits.W))

    // Register to hold the physical block index of the chunk currently being read
    val blockIndexInFlightReg   = Reg(UInt(BLOCK_INDEX_BITS.W))
    // Register to hold the physical block index for the chunk currently being written
    val blockIndexToWriteReg    = Reg(UInt(BLOCK_INDEX_BITS.W))

    // --- Counters ---
    val receivedDataWordCount   = RegInit(0.U(outer.params.wordCountBits.W))
    val sentDataWordCount       = RegInit(0.U(outer.params.wordCountBits.W))
    val dmaWordCount            = RegInit(0.U(outer.params.wordCountBits.W))
    
    // Add counter for tracking output backpressure
    val outputNotReadyCount     = RegInit(0.U(32.W))
    
    // Separate counters for read requests and responses
    val readReqCount            = RegInit(0.U(outer.params.wordCountBits.W))
    val readWordCount           = RegInit(0.U(outer.params.wordCountBits.W)) // used as response count

    // Add counters for managing the FPU pipeline
    val elementIdx              = RegInit(0.U(log2Ceil(NUM_DATA_ELEMENTS + 1).W))

    // --- Processing Wires ---
    val nextLevel               = Wire(UInt(8.W))
    val isStoringIncomingData   = RegInit(false.B)     // Flag to distinguish storing incoming data vs processed data
    val isReadingInputData      = RegInit(false.B)        // Flag to distinguish reading input data vs previous level data
    
    // Default control signals
    nextLevel := currentLevelReg + 1.U

    // --- Input/Output Handling ---
    val incoming_bits           = io.in.bits
    val incoming_valid          = io.in.valid
    val incoming_fire           = io.in.fire

    val outgoing_bits           = Wire(new StreamChannel(outer.params.dataWidth))
    outgoing_bits.data          := 0.U(outer.params.dataWidth.W)
    outgoing_bits.keep          := 0.U(outer.params.bytesPerWord.W)
    outgoing_bits.last          := false.B
    
    io.out.bits                 := outgoing_bits
    io.out.valid                := false.B

    // --- TileLink Default Signals ---
    tl.a.valid                  := false.B
    tl.a.bits                   := DontCare
    tl.d.ready                  := false.B
    
    // Dynamic addressing replaces static helper functions for address calculation
    
    // Helper function to check if all required chunks are available for processing
    // Note: Level 0 chunks are processed immediately during receive, so this only handles Level 1+
    def canProcessChunk(level: UInt, chunkIndex: UInt): Bool = {
        val canProcess = Wire(Bool())
        // For any level > 0, the chunk is processable if:
        // 1. Its own data has arrived in memory.
        // 2. The result from the PREVIOUS level is ready.
        // 3. It has not ALREADY been processed.
        // 4. Level 0 of this Chunk is already processed
        canProcess := chunkArrivedBits(level)(chunkIndex) && 
                      chunkProcessedBits(level - 1.U)(chunkIndex) &&
                      !chunkProcessedBits(level)(chunkIndex) &&
                      chunkProcessedBits(0.U)(chunkIndex)
        canProcess
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
        io.out.valid            := false.B
        isStoringIncomingData   := false.B
        isReadingInputData      := false.B
        
        when(incoming_fire) {
            dprintf(p"[s_idle] Received metadata: level=${incoming_bits.data(63, 56)}, maxLevel=${incoming_bits.data(55, 48)}, collId=0x${Hexadecimal(incoming_bits.data(15, 0))}\n")
            val metaWord        = incoming_bits.data
            val newCollectiveId = metaWord(15, 0)
            
            // Clear memory valid flags if this is a new test set (different collective ID)
            when(newCollectiveId =/= collectiveIdReg) {
                dprintf(p"[s_idle] New test set detected (collective ID changed from 0x${Hexadecimal(collectiveIdReg)} to 0x${Hexadecimal(newCollectiveId)}), clearing all valid flags\n")

                totalChunksForCollective := 0.U
                
                // Clear chunk tracking for new collective
                for (level <- 0 until MAX_RECURSION_LEVEL) {
                    for (chunk <- 0 until MAX_CHUNKS) {
                        chunkArrivedBits(level)(chunk)      := false.B
                        chunkProcessedBits(level)(chunk)    := false.B
                    }
                }
                // Mark all memory blocks as free for the new collective
                for (i <- 0 until NUM_MEM_BLOCKS) { memBlockFree(i) := true.B }
                dprintf(p"[s_idle] Resetting memory manager: all ${NUM_MEM_BLOCKS} blocks are now free.\n")
            }
            
            collectiveIdReg     := newCollectiveId
            collectiveTypeReg   := metaWord(23, 16)
            operationReg        := metaWord(31, 24)
            maxLevelReg         := metaWord(55, 48)
            currentLevelReg     := metaWord(63, 56)
            
            // Transition to wait for the second metadata word
            state               := s_recv_meta2
        }
    }

    .elsewhen(state === s_recv_meta2) {
        io.out.valid        := false.B
        when(incoming_fire) {
            val metaWord2   = incoming_bits.data
            chunkIndexReg   := metaWord2(31, 0)
            totalChunksReg  := metaWord2(63, 32)

            dprintf(p"[s_recv_meta2] Received metaWord2: currentChunks = 0x${Hexadecimal(metaWord2(31, 0))} totalChunks = 0x${Hexadecimal(metaWord2(63, 32))}\n")
            
            // Update total chunks for collective if this is the first chunk or if it's larger
            when(totalChunksForCollective === 0.U || totalChunksReg > totalChunksForCollective) {
                totalChunksForCollective := metaWord2(63, 32)
                dprintf(p"[s_recv_meta2] Updated totalChunksForCollective to ${metaWord2(63, 32)}\n")
            }

            // Start data word count at 0 since we just finished receiving the metadata word
            receivedDataWordCount   := 0.U
            state                   := s_recv_data
        }
    }

    .elsewhen(state === s_recv_data) {
        io.out.valid        := false.B
        when(incoming_fire) {
            val baseElementIndex = receivedDataWordCount * ELEMENTS_PER_WORD.U

            for (i <- 0 until ELEMENTS_PER_WORD) {
                when((baseElementIndex + i.U) < NUM_DATA_ELEMENTS.U) {
                    when(currentLevelReg === 0.U) {
                        // Level 0: Write directly to processedDataBuffer (no processing needed)
                        processedDataBuffer(baseElementIndex + i.U) := incoming_bits.data((i + 1) * ELEMENT_WIDTH - 1, i * ELEMENT_WIDTH)
                    }.otherwise {
                        // Higher levels: Write to incomingDataBuffer (will be processed later)
                        incomingDataBuffer(baseElementIndex + i.U) := incoming_bits.data((i + 1) * ELEMENT_WIDTH - 1, i * ELEMENT_WIDTH)
                    }
                }
            }

            // Only print for the first and last data word of the packet.
            when(receivedDataWordCount === 0.U || receivedDataWordCount === (NUM_DATA_WORDS.U - 1.U)) {
                val e0  = incoming_bits.data(ELEMENT_WIDTH-1, 0)
                val e1  = incoming_bits.data(2*ELEMENT_WIDTH-1, ELEMENT_WIDTH)
                dprintf(p"[s_recv_data] Recv word ${receivedDataWordCount}: elements[${baseElementIndex}]=0x${Hexadecimal(e0)}, elements[${baseElementIndex+1.U}]=0x${Hexadecimal(e1)}\n")
            }

            receivedDataWordCount := receivedDataWordCount + 1.U

            when(incoming_bits.last) {
                blockIndexToWriteReg := PriorityEncoder(memBlockFree.asUInt) // LATCH the destination block index
                state                   := s_dma_write
                dmaWordCount            := 0.U
                receivedDataWordCount   := 0.U
                isStoringIncomingData   := Mux(currentLevelReg === 0.U, false.B, true.B)

                // Conditional debug print
                when(currentLevelReg === 0.U) {
                    dprintf(p"[s_recv_data] Level 0 optimization: data already in processedDataBuffer, ready to send\n")
                }.otherwise {
                    dprintf(p"[s_recv_data] Store first: storing chunk ${chunkIndexReg} for level ${currentLevelReg} to memory\n")
                }
            }
        }
    }
    
    .elsewhen(state === s_dma_write) {
        // A block is available if the one we pre-selected is still free (for word 0)
        // or if we are continuing a write to a block we've already claimed.
        val blockAvailable = memBlockFree(blockIndexToWriteReg) || (dmaWordCount > 0.U)

        when(dmaWordCount < WORDS_PER_CHUNK.U && !blockAvailable) {
            dprintf(p"[s_dma_write] STALL: Memory is full. Waiting for a block to be freed.\n")
        }

        // Write data to memory, but only if a free block is available
        when(dmaWordCount < WORDS_PER_CHUNK.U && blockAvailable) {
            tl.a.valid          := true.B
            tl.a.bits.opcode    := TLMessages.PutFullData
            tl.a.bits.param     := 0.U
            tl.a.bits.size      := log2Ceil(BYTES_PER_WORD).U
            tl.a.bits.source    := 0.U // Serialize writes

            // Calculate address dynamically based on the first available free block
            val baseAddr = outer.params.baseMemoryAddr.U + (blockIndexToWriteReg << log2Ceil(BYTES_PER_CHUNK).U)
            tl.a.bits.address   := baseAddr + (dmaWordCount << log2Ceil(BYTES_PER_WORD).U)
            tl.a.bits.mask      := (~0.U(BYTES_PER_WORD.W))

            val baseElementIndex = dmaWordCount * ELEMENTS_PER_WORD.U
            val dataWord        = Wire(UInt(outer.params.dataWidth.W))
            val dataSource      = Mux(isStoringIncomingData, incomingDataBuffer, processedDataBuffer)
            val dataVec         = Wire(Vec(ELEMENTS_PER_WORD, UInt(ELEMENT_WIDTH.W)))
            for (i <- 0 until ELEMENTS_PER_WORD) {
                dataVec(i) := Mux((baseElementIndex + i.U) < NUM_DATA_ELEMENTS.U, dataSource(baseElementIndex + i.U), 0.U)
            }
            dataWord            := Cat(dataVec.reverse)
            tl.a.bits.data      := dataWord

            when(tl.a.fire) {
                // On the first word, update the memory management tables
                when(dmaWordCount === 0.U) {
                    memBlockFree(blockIndexToWriteReg) := false.B
                    when(isStoringIncomingData) {
                        incomingChunkBlockIndex(currentLevelReg)(chunkIndexReg) := blockIndexToWriteReg
                        dprintf(p"[s_dma_write] Allocating block ${blockIndexToWriteReg} for INCOMING L${currentLevelReg}C${chunkIndexReg}\n")
                    }.otherwise {
                        processedChunkBlockIndex(currentLevelReg)(chunkIndexReg) := blockIndexToWriteReg
                        dprintf(p"[s_dma_write] Allocating block ${blockIndexToWriteReg} for PROCESSED L${currentLevelReg}C${chunkIndexReg}\n")
                    }
                }

                // Debug prints for first and last word, including elements
                when(dmaWordCount === 0.U || dmaWordCount === (WORDS_PER_CHUNK.U - 1.U)) {
                    when(isStoringIncomingData) {
                        dprintf(p"[s_dma_write] Write request: incoming data, level=${currentLevelReg}, chunk=${chunkIndexReg}, word=${dmaWordCount}, addr=0x${Hexadecimal(tl.a.bits.address)}\n")
                    }.otherwise {
                        dprintf(p"[s_dma_write] Write request: processed data, level=${currentLevelReg}, chunk=${chunkIndexReg}, word=${dmaWordCount}, addr=0x${Hexadecimal(tl.a.bits.address)}\n")
                    }
                    val eIdx0 = baseElementIndex
                    val eIdx1 = baseElementIndex + 1.U
                    val e0    = Mux(eIdx0 < NUM_DATA_ELEMENTS.U, dataSource(eIdx0), 0.U)
                    val e1    = Mux(eIdx1 < NUM_DATA_ELEMENTS.U, dataSource(eIdx1), 0.U)
                    dprintf(p"[s_dma_write]   Elements: [${eIdx0}]=0x${Hexadecimal(e0)}, [${eIdx1}]=0x${Hexadecimal(e1)}\n")
                }
                state           := s_wait_write
            }
        }.elsewhen(dmaWordCount < WORDS_PER_CHUNK.U && !blockAvailable) {
            dprintf(p"[s_dma_write] STALL: Memory is full. Waiting for a block to be freed.\n")
        }
    }

    .elsewhen(state === s_wait_write) {
        // Wait for AccessAck of the Put before proceeding
        tl.d.ready      := true.B
        when(tl.d.valid && tl.d.bits.opcode === TLMessages.AccessAck && tl.d.bits.source === 0.U) {
            // One write completed
            val nextCount   = dmaWordCount + 1.U
            dmaWordCount    := nextCount
            when(nextCount === WORDS_PER_CHUNK.U) {
                when(isStoringIncomingData) {
                    // The chunk data is now officially in memory.
                    // THIS is the correct place to set the arrived flag.
                    chunkArrivedBits(currentLevelReg)(chunkIndexReg) := true.B
                    dprintf(p"[s_wait_write] Incoming Level ${currentLevelReg} Chunk ${chunkIndexReg} stored\n")

                    // Just stored incoming data - now check if processing is possible
                    isStoringIncomingData   := false.B
                    state                   := s_check_chunks
                }.otherwise {
                    dprintf(p"[s_wait_write] Processed Level ${currentLevelReg} Chunk ${chunkIndexReg} stored\n")
                    
                    state                   := s_send_meta
                }
            }.otherwise {
                state       := s_dma_write
            }
        }
    }
    
    .elsewhen(state === s_dma_read) {
        // Read data from memory - address depends on what we're reading
        when(readReqCount < WORDS_PER_CHUNK.U) {
            tl.a.valid          := true.B
            tl.a.bits.opcode    := TLMessages.Get
            tl.a.bits.param     := 0.U
            tl.a.bits.size      := log2Ceil(BYTES_PER_WORD).U
            tl.a.bits.source    := 0.U
            // Calculate the address from the base and block index
            val baseReadAddr = outer.params.baseMemoryAddr.U + (blockIndexInFlightReg << log2Ceil(BYTES_PER_CHUNK).U)
            val readAddr = baseReadAddr + (readReqCount << log2Ceil(BYTES_PER_WORD).U)
            
            tl.a.bits.address   := readAddr
            tl.a.bits.mask      := (~0.U(BYTES_PER_WORD.W))

            when(tl.a.fire) {
                when(readReqCount === 0.U || readReqCount === (WORDS_PER_CHUNK.U - 1.U)) {
                    when(isReadingInputData) {
                        dprintf(p"[s_dma_read] Reading input data: level=${processingLevel}, chunk=${processingChunkIndex}, word=${readReqCount}, addr=0x${Hexadecimal(readAddr)}\n")
                    }.otherwise {
                        dprintf(p"[s_dma_read] Reading previous level data: level=${processingLevel - 1.U}, chunk=${processingChunkIndex}, word=${readReqCount}, addr=0x${Hexadecimal(readAddr)}\n")
                    }
                }
                
                state                   := s_wait_read
            }
        }
    }
    
    .elsewhen(state === s_wait_read) {
        // Handle TileLink responses for level data read (one at a time)
        tl.d.ready      := true.B
        when(tl.d.valid && tl.d.bits.opcode === TLMessages.AccessAckData && tl.d.bits.source === 0.U) {
            val dataWord    = tl.d.bits.data

            // Only print for the first and last word of the DMA transfer.
            when(readWordCount === 0.U || readWordCount === (WORDS_PER_CHUNK.U - 1.U)) {
                dprintf(p"[s_wait_read] DMA Read Response word ${readWordCount}: data=0x${Hexadecimal(dataWord)}\n")
            }

            val wordIndex = readWordCount
            // Unpack into appropriate buffer based on what we're reading
            for (i <- 0 until ELEMENTS_PER_WORD) {
                val elementIndex    = wordIndex * ELEMENTS_PER_WORD.U + i.U
                when(elementIndex < NUM_DATA_ELEMENTS.U) {
                    val elementStart    = i * ELEMENT_WIDTH
                    val elementEnd      = elementStart + ELEMENT_WIDTH - 1
                    val extractedData   = dataWord(elementEnd, elementStart)
                    
                    // Choose buffer based on what we're reading
                    when(isReadingInputData) {
                        incomingDataBuffer(elementIndex)    := extractedData
                    }.otherwise {
                        memoryReadBuffer(elementIndex)      := extractedData
                    }
                }
            }

            readWordCount   := readWordCount + 1.U
            readReqCount    := readReqCount + 1.U

            // If more words to fetch, issue next request; else proceed
            when(readWordCount + 1.U < WORDS_PER_CHUNK.U) {
                state       := s_dma_read
            }.otherwise {
                // Read phase complete. The data is now in internal buffers, so we can free the memory block.
                // The block index was latched during the read request. Use the registered value.
                memBlockFree(blockIndexInFlightReg) := true.B
                dprintf(p"[s_wait_read] Read complete. Freed memory block ${blockIndexInFlightReg}.\n")

                // Now, determine next action
                when(isReadingInputData) {
                    // Phase 1 complete, start Phase 2: read previous level data
                    // Note: Level 0 never reaches read states with our optimization
                    dprintf(p"[s_wait_read] Input data read complete, starting previous level read\n")

                    // Calculate and store the NEXT block index for the second read phase
                    blockIndexInFlightReg   := processedChunkBlockIndex(processingLevel - 1.U)(processingChunkIndex)

                    isReadingInputData  := false.B
                    readWordCount       := 0.U
                    readReqCount        := 0.U
                    state               := s_dma_read
                }.otherwise {
                    dprintf(p"[s_wait_read] Previous level data read complete, ready for FPU\n")
                    // Both reads complete, proceed to FPU processing
                    state               := s_wait_read_done
                }
            }
        }
    }

    .elsewhen(state === s_wait_read_done) {
        // Both input data and previous level data are now in buffers, ready for FPU
        // Note: Level 0 never reaches this state with our optimization
        dprintf(p"[s_wait_read_done] Both reads complete. Starting FP addition pipeline for level ${processingLevel}, chunk ${processingChunkIndex}\n")
        dprintf(p"[s_wait_read_done] incomingDataBuffer[0]=0x${Hexadecimal(incomingDataBuffer(0))}, memoryReadBuffer[0]=0x${Hexadecimal(memoryReadBuffer(0))}\n")

            // Both input data and previous level data are now in buffers, ready for FPU
        dprintf(p"[s_wait_read_done] Both reads complete. Starting FP addition for L${processingLevel}C${processingChunkIndex}\n")

        // DEBUG: Print the first few elements of the input buffers to verify correctness
        dprintf(p"[s_wait_read_done] incomingDataBuffer[0]=0x${Hexadecimal(incomingDataBuffer(0))}, [1]=0x${Hexadecimal(incomingDataBuffer(1))}\n")
        dprintf(p"[s_wait_read_done] memoryReadBuffer[0]  =0x${Hexadecimal(memoryReadBuffer(0))}, [1]=0x${Hexadecimal(memoryReadBuffer(1))}\n")


        state           := s_fp_add_pipe
        elementIdx      := 0.U
    }
    
    .elsewhen(state === s_fp_add_pipe) {
        // ACTION: Provide inputs, calculate sum, and store the result in the same cycle.
        fpAdder.io.a    := incomingDataBuffer(elementIdx)
        fpAdder.io.b    := memoryReadBuffer(elementIdx)
        
        // Set control signals
        fpAdder.io.roundingMode   := "b000".U
        fpAdder.io.detectTininess := 1.U
        fpAdder.io.subOp          := false.B

        // Since the FPU is combinational, the result is available immediately.
        val sum         = fpAdder.io.out
        // Always store to processedDataBuffer for immediate use
        processedDataBuffer(elementIdx) := sum

        // TRANSITION: When the last element is processed, move to the next state.
        when(elementIdx === (NUM_DATA_ELEMENTS - 1).U) {
            dprintf(p"[s_fp_add_pipe] All FP results collected.\n")
            dprintf(p"[s_fp_add_pipe] processedDataBuffer[0]=0x${Hexadecimal(processedDataBuffer(0))}, [1]=0x${Hexadecimal(processedDataBuffer(1))}\n")
            dprintf(p"[s_fp_add_pipe] Last FP sum: 0x${Hexadecimal(sum)}\n")
            
            when(processingLevel < maxLevelReg) {
                // Store processed data back to memory
                dprintf(p"[s_fp_add_pipe] Chunk ${processingChunkIndex} FPU processing complete for level ${processingLevel}, storing to memory\n")
                blockIndexToWriteReg    := PriorityEncoder(memBlockFree.asUInt) // LATCH the destination block index
                isStoringIncomingData   := false.B  // This is processed data, not incoming data
                currentLevelReg         := processingLevel
                chunkIndexReg           := processingChunkIndex
                state                   := s_dma_write
                dmaWordCount            := 0.U
            }.otherwise {
                // Final level - send response directly without storing
                dprintf(p"[s_fp_add_pipe] Final level chunk ${processingChunkIndex} complete, sending response directly\n")
                currentLevelReg         := processingLevel
                chunkIndexReg           := processingChunkIndex
                state                   := s_send_meta
            }
        } .otherwise {
            elementIdx              := elementIdx + 1.U
        }
    }

    .elsewhen(state === s_send_meta) {
        io.out.valid            := true.B
        outgoing_bits.data      := outgoingMetadataWord
        outgoing_bits.last      := false.B
        outgoing_bits.keep      := (~0.U((outer.params.dataWidth / 8).W))

        when(io.out.fire) {
            // Mark chunk as complete and start sending response
            // Note: chunkProcessedBits is set here because last packet don't go to DMA Write and all packets come here.
            chunkProcessedBits(currentLevelReg)(chunkIndexReg) := true.B
            dprintf(p"[s_send_meta] OUT META: nextLevel=${nextLevel}, maxLevel=${maxLevelReg}, op=${operationReg}, type=${collectiveTypeReg}, collId=0x${Hexadecimal(collectiveIdReg)}\n")
            state                   := s_send_meta2
        }.elsewhen(io.out.valid && !io.out.ready) {
            // Track when output is not ready
            outputNotReadyCount     := outputNotReadyCount + 1.U
            when(outputNotReadyCount(7, 0) === 0.U) { // Print every 256 cycles
                dprintf(p"[s_send_meta] Output not ready, backpressure count: ${outputNotReadyCount}\n")
            }
        }
        // Stay in this state until the output is accepted
    }

    .elsewhen(state === s_send_meta2) {
        io.out.valid        := true.B
        // Assemble and send the SECOND word
        outgoing_bits.data  := Cat(totalChunksReg, chunkIndexReg)
        outgoing_bits.last  := (NUM_DATA_WORDS == 0).B
        outgoing_bits.keep  := (~0.U((outer.params.dataWidth / 8).W))

        when(io.out.fire) {
            sentDataWordCount := 0.U
            if (NUM_DATA_WORDS > 0) {
                state   := s_send_data
            } else {
                state   := s_check_chunks
            }
        }.elsewhen(io.out.valid && !io.out.ready) {
            // Track when output is not ready
            outputNotReadyCount := outputNotReadyCount + 1.U
            when(outputNotReadyCount(7, 0) === 0.U) { // Print every 256 cycles
                dprintf(p"[s_send_meta2] Output not ready, backpressure count: ${outputNotReadyCount}\n")
            }
        }
        // Stay in this state until the output is accepted
    }

    .elsewhen(state === s_send_data) {
        io.out.valid            := true.B

        // processedDataBuffer always contain the expected output data. Either the FP addition or when in Level 0, it contains the incoming data.
        val dataSource          = processedDataBuffer
        val baseElementIndex    = sentDataWordCount * ELEMENTS_PER_WORD.U
        val dataWordVec         = Wire(Vec(ELEMENTS_PER_WORD, UInt(ELEMENT_WIDTH.W)))
        
        for (i <- 0 until ELEMENTS_PER_WORD) {
            when((baseElementIndex + i.U) < NUM_DATA_ELEMENTS.U) {
                dataWordVec(i)      := dataSource(baseElementIndex + i.U)
            }.otherwise {
                dataWordVec(i)      := 0.U
            }
        }
        outgoing_bits.data      := Cat(dataWordVec.reverse)
        
        val isLastDataWord      = sentDataWordCount === NUM_DATA_WORDS.U - 1.U
        outgoing_bits.last      := isLastDataWord
        outgoing_bits.keep      := (~0.U((outer.params.dataWidth / 8).W))

        when(io.out.fire) {            
            when(isLastDataWord) {
                dprintf(p"[s_send_data] Finished sending chunk ${chunkIndexReg} for level ${currentLevelReg}\n")
                
                // Check if this was the last level for this chunk
                when(currentLevelReg === maxLevelReg) {
                    dprintf(p"[s_send_data] Completed max level ${maxLevelReg} for chunk ${chunkIndexReg}\n")
                }
                
                // Continue checking for more chunks to process
                state               := s_check_chunks
                sentDataWordCount   := 0.U
            }.otherwise {
                sentDataWordCount   := sentDataWordCount + 1.U
            }
        }.elsewhen(io.out.valid && !io.out.ready) {
            // Track when output is not ready
            outputNotReadyCount     := outputNotReadyCount + 1.U
            when(outputNotReadyCount(7, 0) === 0.U) { // Print every 256 cycles
                dprintf(p"[s_send_data] Output not ready, backpressure count: ${outputNotReadyCount}, word: ${sentDataWordCount}\n")
            }
        }
        // Stay in this state until the current word is accepted
    }
    
    .elsewhen(state === s_check_chunks) {
        // Check for chunks that are ready to be processed
        io.out.valid        := false.B
        
        val foundChunk      = Wire(Bool())
        val foundLevel      = Wire(UInt(outer.params.levelCountBits.W))
        val foundChunkIndex = Wire(UInt(CHUNK_INDEX_BITS.W))
        
        // Create a vector to collect all valid candidates, then pick the first one
        val validCandidates = Wire(Vec(MAX_RECURSION_LEVEL * MAX_CHUNKS, Bool()))
        val candidateLevel  = Wire(Vec(MAX_RECURSION_LEVEL * MAX_CHUNKS, UInt(outer.params.levelCountBits.W)))
        val candidateChunk  = Wire(Vec(MAX_RECURSION_LEVEL * MAX_CHUNKS, UInt(CHUNK_INDEX_BITS.W)))
        
        // Initialize all candidates as invalid
        for (i <- 0 until (MAX_RECURSION_LEVEL * MAX_CHUNKS)) {
            validCandidates(i)  := false.B
            candidateLevel(i)   := 0.U
            candidateChunk(i)   := 0.U
        }
        
        // Priority: process lower levels first, then lower chunk indices
        // Note: Level 0 is handled directly in s_recv_data, so we start from Level 1
        var candidateIndex  = 0
        for (level <- 1 until MAX_RECURSION_LEVEL) {
            for (chunk <- 0 until MAX_CHUNKS) {
                val canProcess = chunk.U < totalChunksForCollective && canProcessChunk(level.U, chunk.U)
                
                // --- NEW, CLEANER PRINT LOGIC ---
                // Only print for the chunks we are actually using in this collective
                when(chunk.U < totalChunksForCollective) {
                    dprintf(p"[s_check_chunks] Checking L${level}, C${chunk.U}: canProcess=${canProcess}\n")
                }

                validCandidates(candidateIndex)     := canProcess
                candidateLevel(candidateIndex)      := level.U
                candidateChunk(candidateIndex)      := chunk.U
                candidateIndex                      += 1
            }
        }
        
        // Find the first valid candidate (priority encoding)
        foundChunk          := validCandidates.asUInt.orR
        foundLevel          := PriorityMux(validCandidates, candidateLevel)
        foundChunkIndex     := PriorityMux(validCandidates, candidateChunk)
        
        when(foundChunk) {
            dprintf(p"[s_check_chunks] Found processable chunk ${foundChunkIndex} at level ${foundLevel}\n")

            // === OPTIMIZATION: Calculate and store the block index ONCE here ===
            // Note: For any level > 0, the first read is always the "incoming" data.
            blockIndexInFlightReg   := incomingChunkBlockIndex(foundLevel)(foundChunkIndex)

            processingChunkIndex    := foundChunkIndex
            processingLevel         := foundLevel
            isReadingInputData      := true.B
            state                   := s_dma_read
            dmaWordCount            := 0.U
            readWordCount           := 0.U
            readReqCount            := 0.U
        }.otherwise {
            dprintf(p"[s_check_chunks] No processable chunks found, returning to idle\n")
            state                   := s_idle
        }
    }
    
    // --- Input Ready Logic ---
    val memoryHasSpace = memBlockFree.asUInt.orR
    io.in.ready     := (state === s_idle || state === s_recv_meta2 || state === s_recv_data) && memoryHasSpace

    // --- Debug ---
    when(state =/= prevState) {
        // dprintf(p"[STATE] Transition: ${prevState} -> ${state} (level=${currentLevelReg}, receivedDataWordCount=${receivedDataWordCount}, sentDataWordCount=${sentDataWordCount})\n")
        prevState := state
    }
}

// === Rocket Chip Config Fragment ===

class WithRecursiveDoublingWithDMA(
    maxLevel: Int           = 4,
    numElements: Int        = 256,
    numBytesPerElement: Int = 4,
    baseAddr: BigInt        = 0x80000000L,
    sourceIds: Int          = 8,
    maxChunks: Int          = 1024,
    numMemoryBlocks: Int    = 1024,
    EnableDebug: Boolean    = true
) extends Config((site, here, up) => {
    case RecursiveDoublingWithDMAKey => Some(RecursiveDoublingWithDMAParams(
        Levels              = maxLevel,
        DataElements        = numElements,
        BytesPerElement     = numBytesPerElement,
        dataWidth           = IceNetConsts.NET_IF_WIDTH,
        baseMemoryAddr      = baseAddr,
        sourceIds           = sourceIds,
        MaxChunks           = maxChunks,
        numMemoryBlocks     = numMemoryBlocks,
        EnableDebug         = EnableDebug
    ))
}) 