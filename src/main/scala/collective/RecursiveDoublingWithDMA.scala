package icenet.collective

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import icenet.{NICKey, NICIOvonly, IceNetConsts, StreamChannel, StreamIO}
import hardfloat._
import midas.targetutils.SynthesizePrintf

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
    val io                      = IO(new Bundle {
        val in = Flipped(Decoupled(new StreamChannel(outer.params.dataWidth)))
        val out = Decoupled(new StreamChannel(outer.params.dataWidth))
        val dstMacAddr = Output(UInt(IceNetConsts.ETH_MAC_BITS.W))  // Destination MAC for outgoing packets
        val level0SrcMac = Input(UInt(IceNetConsts.ETH_MAC_BITS.W))  // Source MAC from Level 0 packets (for Level 4 destination)
        val nodeRank = Output(UInt(8.W)) // Output driven by internal register
    })
    // Elabor-time debug helper (no hardware cost when disabled)
    private val dbgEnabled: Boolean = outer.params.EnableDebug
    @inline private def dprintf(msg: Printable): Unit = if (dbgEnabled) { printf(msg) }
    
    // Get the TileLink client interface
    val (tl, edge)              = outer.node.out(0)
    
    // Node Rank Logic: Initialized to 0, updated by Setup Packet (opCode=0xFE)
    // The Setup packet is the ONLY reliable way to set rank in FireSim multi-node simulation.
    // PlusArgs don't work, and the connector's nodeRank parameter is a compile-time constant.
    val myRankReg = RegInit(0.U(8.W))
    val myRank = myRankReg
    io.nodeRank := myRankReg // Drive the output for NIC to use
    private val AccelMacOffset = 0x22.U(8.W)

    // --- FPU Instantiation ---
    // Instantiate a single-precision floating-point adder.
    // Single precision: exponent = 8, significand = 24 (23 stored + 1 implicit)
    val fpAdder                 = Module(new AddRecFN(expWidth = 8, sigWidth = 24))

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
    // Buffer for processed data to be written to memory (no global reset init)
    val processedDataBuffer     = Reg(Vec(NUM_DATA_ELEMENTS, UInt(ELEMENT_WIDTH.W)))
    
    // --- Chunk Management ---
    // Track total chunks for current collective operation
    val totalChunksForCollective = RegInit(0.U(CHUNK_INDEX_BITS.W))
    
    // Chunk arrival and processing tracking per level using bitmaps (UInt)
    // Each level has a bitmap of MAX_CHUNKS bits.
    // Use plain Reg (no RegInit) to avoid huge aggregate constant in reset; we clear explicitly on new collective.
    val chunkArrivedBitmap      = Reg(Vec(MAX_RECURSION_LEVEL, UInt(MAX_CHUNKS.W)))
    val chunkProcessedBitmap    = Reg(Vec(MAX_RECURSION_LEVEL, UInt(MAX_CHUNKS.W)))

    // --- Dynamic Memory Management ---
    val NUM_MEM_BLOCKS          = outer.params.numMemoryBlocks
    // Free-block bitmap: 1 = free, 0 = allocated. Initialize all blocks as free.
    val memFreeBitmap           = RegInit(~0.U(NUM_MEM_BLOCKS.W))
    val BLOCK_INDEX_BITS        = log2Ceil(NUM_MEM_BLOCKS)
    val numFreeBlocks           = RegInit(NUM_MEM_BLOCKS.U((BLOCK_INDEX_BITS + 1).W))
    val allocSearchPtr          = RegInit(0.U(BLOCK_INDEX_BITS.W))
    val foundBlockValid         = RegInit(false.B)
    val foundBlockIndex         = Reg(UInt(BLOCK_INDEX_BITS.W))

    // --- Background Chunk Searcher ---
    val chunkSearchLevel        = RegInit(1.U(outer.params.levelCountBits.W))
    val chunkSearchChunk        = RegInit(0.U(CHUNK_INDEX_BITS.W))
    val foundChunkValid         = RegInit(false.B)
    val foundChunkLevel         = Reg(UInt(outer.params.levelCountBits.W))
    val foundChunkIndex         = Reg(UInt(CHUNK_INDEX_BITS.W))
    val isProcessing            = RegInit(false.B)

    // Tables to store the block index for each chunk's data
    // Packed per-level fields to avoid large Vec-of-Vec structures
    // Each level holds MAX_CHUNKS fields of width BLOCK_INDEX_BITS
    private val BLOCK_FIELDS_PER_LEVEL_WIDTH = (MAX_CHUNKS * BLOCK_INDEX_BITS)
    private val SHAMT_BITS      = log2Ceil(BLOCK_FIELDS_PER_LEVEL_WIDTH)
    val incomingBlockFields     = Reg(Vec(MAX_RECURSION_LEVEL, UInt(BLOCK_FIELDS_PER_LEVEL_WIDTH.W)))
    val processedBlockFields    = Reg(Vec(MAX_RECURSION_LEVEL, UInt(BLOCK_FIELDS_PER_LEVEL_WIDTH.W)))

    // --- Extended State Machine ---
    val s_idle :: s_recv_meta2 :: s_recv_data :: s_dma_write :: s_wait_write :: s_dma_read :: s_wait_read :: s_wait_read_done :: s_fp_add_pipe :: s_wait_alloc :: s_send_meta :: s_send_meta2 :: s_send_data :: Nil = Enum(13)
    val state                   = RegInit(s_idle)
    val prevState               = RegInit(s_idle)

    // --- Metadata Registers ---
    // Initialize to impossible value to ensure first packet triggers initialization
    val collectiveIdReg         = RegInit(0xFFFF.U(16.W))
    val collectiveTypeReg       = Reg(UInt(8.W))
    val operationReg            = Reg(UInt(8.W))
    val maxLevelReg             = Reg(UInt(8.W))
    val currentLevelReg         = Reg(UInt(8.W))
    val chunkIndexReg           = Reg(UInt(32.W))
    val totalChunksReg          = Reg(UInt(32.W))
    
    // Flag to track pending Setup ACK - set when Setup packet detected,
    // ACK sent after draining full incoming packet
    val pendingSetupAck         = RegInit(false.B)
    
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
    
    // Destination MAC address for outgoing packets
    // This is set before packet transmission starts to ensure it's stable
    val outgoingDstMac           = RegInit(IceNetConsts.ETH_BCAST_MAC)
    io.dstMacAddr                := outgoingDstMac
    
    // Store source MAC from Level 0 packets (from PyTorch)
    // This will be used as destination for Level 4 packets
    // Since the NIC strips headers before the accelerator, the harness provides the extracted MAC.
    val level0SourceMac          = RegInit(0.U(IceNetConsts.ETH_MAC_BITS.W))
    val level0SourceMacValid     = RegInit(false.B)

    // --- TileLink Default Signals ---
    tl.a.valid                  := false.B
    tl.a.bits                   := DontCare
    tl.d.ready                  := false.B
    
    // Dynamic addressing replaces static helper functions for address calculation
    
    // Helper function to check if all required chunks are available for processing
    // Note: Level 0 chunks are processed immediately during receive, so this only handles Level 1+
    def canProcessChunk(level: UInt, chunkIndex: UInt): Bool = {
        val bitIdx                 = chunkIndex(CHUNK_INDEX_BITS-1, 0)
        val mask                   = (1.U(MAX_CHUNKS.W)) << bitIdx
        val arrivedBit             = (chunkArrivedBitmap(level) & mask) =/= 0.U
        val prevLevelProcessedBit  = (chunkProcessedBitmap(level - 1.U) & mask) =/= 0.U
        val thisLevelProcessedBit  = (chunkProcessedBitmap(level) & mask) =/= 0.U
        val level0ProcessedBit     = (chunkProcessedBitmap(0.U) & mask) =/= 0.U
        arrivedBit && prevLevelProcessedBit && !thisLevelProcessedBit && level0ProcessedBit
    }

    // --- Partner Calculation ---
    // Calculate partner rank for recursive doubling: partner = myRank XOR (1 << level)
    def calculatePartnerRank(level: UInt, rank: UInt): UInt = {
        val distance = 1.U << level
        rank ^ distance
    }
    
    // Calculate partner MAC address (base MAC 00:12:6D:00:00:00 + partner rank + 2)
    // MAC addresses in FireSim start at 02 (node 0 = 00:12:6D:00:00:02), so we add 2 to the rank
    def calculatePartnerMac(level: UInt, rank: UInt): UInt = {
        val partnerRank = calculatePartnerRank(level, rank)
        val baseMac = (0x00126D000000L).U(48.W)  // Base: 00:12:6D:00:00:00
        val macLow = partnerRank(7, 0) + AccelMacOffset
        baseMac | Cat(0.U(40.W), macLow(7, 0))
    }
    
    // --- Metadata Assembly ---
    // Include sender rank in reserved byte for partner verification
    // Also include destination MAC lowest byte for debugging routing
    // Note: We'll calculate the destination MAC in the state machine where values are guaranteed to be correct
    // Register to hold the destination MAC lowest byte for metadata
    val dstMacLowestByteReg = RegInit(0.U(8.W))
    
    val outgoingMetadataWord = Wire(UInt(outer.params.dataWidth.W))
    // Metadata with destination MAC from register (set in state machine)
    outgoingMetadataWord := Cat(
        nextLevel(7,0),
        maxLevelReg(7,0),
        myRank(7,0),        // Sender rank in reserved byte (for partner verification)
        dstMacLowestByteReg,   // Destination MAC lowest byte (set in state machine before sending)
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
            val metaWord            = incoming_bits.data
            // Define variables reused later
            val newCollectiveId     = metaWord(15, 0)
            val senderRank          = metaWord(47, 40)
            val receivedLevel       = metaWord(63, 56)
            val opCode              = metaWord(31, 24)
            
            // DEBUG: Print raw packet info for every incoming packet
            SynthesizePrintf {
                printf("[s_idle] RECV: opCode=0x%x, level=%d, collId=0x%x, senderRank=%d, myRank=%d\n",
                       opCode, receivedLevel, newCollectiveId, senderRank, myRankReg)
            }
            
            // Check for Setup Packet (OP_SETUP = 0xFE)
            // COMBINED SETUP+WARMUP: If collID=0xFFFF, send acknowledgment response
            when(opCode === 0xFE.U) {
                val newRank             = metaWord(47, 40) 
                myRankReg               := newRank
                
                // Latch source MAC for sending ACK back
                level0SourceMac         := io.level0SrcMac
                level0SourceMacValid    := true.B
                
                SynthesizePrintf {
                    printf("[s_idle] SETUP PACKET RECEIVED: oldRank=%d, newRank=%d, collId=0x%x\n", myRankReg, newRank, newCollectiveId)
                }
                
                // If collID is 0xFFFF, this is a combined Setup+Warmup - need to send ACK after draining packet
                when(newCollectiveId === 0xFFFF.U) {
                    // Prepare ACK response metadata (will send after draining incoming packet)
                    collectiveIdReg         := 0xFFFF.U
                    collectiveTypeReg       := 0.U
                    operationReg            := 0xFE.U  // Echo Setup opCode to indicate ACK
                    maxLevelReg             := 0.U
                    currentLevelReg         := 0.U  // nextLevel=currentLevelReg+1=1 for ACK response
                    chunkIndexReg           := 0.U
                    totalChunksReg          := 1.U
                    
                    // Destination is the sender of the Setup packet
                    outgoingDstMac          := io.level0SrcMac
                    dstMacLowestByteReg     := io.level0SrcMac(7, 0)
                    
                    // Zero out data buffer for consistent full-sized ACK packet
                    for (i <- 0 until NUM_DATA_ELEMENTS) {
                        processedDataBuffer(i)  := 0.U
                    }
                    sentDataWordCount       := 0.U
                    
                    SynthesizePrintf {
                        printf("[s_idle] SETUP+WARMUP: Will send ACK after draining packet. Dst=0x%x, rank=%d\n", 
                               io.level0SrcMac, newRank)
                    }
                    
                    // Set flag to send ACK after draining full packet
                    pendingSetupAck         := true.B
                    
                    // Continue to drain remaining packet words (meta2 + data)
                    state                   := s_recv_meta2
                }
                // If collID != 0xFFFF, just update rank silently (no ACK needed)
            } .otherwise {
                // Standard Logic for Data Packets
                
                when(receivedLevel === 0.U) {
                    // Latch the source MAC. 
                    // We MUST latch it because io.level0SrcMac might change later 
                    // when Level 1/2/3 packets arrive from other partners.
                    level0SourceMac         := io.level0SrcMac
                    level0SourceMacValid    := true.B
                    
                    // Debug print to confirm what we latched
                    dprintf(p"[s_idle] Level 0 Packet: Latched Source MAC for Final Result: 0x${Hexadecimal(io.level0SrcMac)}\n")
                }
            
                // Verify packet source based on level
                // Level 0: From PyTorch/external (no partner verification needed)
                // Level 1-4: From partner node (verify sender matches expected partner)
                when(receivedLevel > 0.U) {
                    // For levels 1-4, calculate expected partner from previous level
                    // If we're at level L, we expect packet from the partner we sent to at level L-1
                    val expectedPartnerRank = calculatePartnerRank(receivedLevel - 1.U, myRank)
                    when(senderRank === expectedPartnerRank) {
                        dprintf(p"[s_idle] Packet from expected partner rank=${senderRank} at level=${receivedLevel}\n")
                    }.otherwise {
                        dprintf(p"[s_idle] WARNING: Packet from unexpected sender. Level=${receivedLevel}, expected rank=${expectedPartnerRank}, got ${senderRank}\n")
                        // Still process - might be from different collective or out-of-order
                    }
                }
                
                // Clear memory valid flags if this is a new test set (different collective ID)
                when(newCollectiveId =/= collectiveIdReg) {
                    dprintf(p"[s_idle] New test set detected (collective ID changed from 0x${Hexadecimal(collectiveIdReg)} to 0x${Hexadecimal(newCollectiveId)}), clearing all valid flags\n")

                    totalChunksForCollective := 0.U
                    
                    // Clear chunk tracking for new collective (bitmap form)
                    for (level <- 0 until MAX_RECURSION_LEVEL) {
                        chunkArrivedBitmap(level)   := 0.U
                        chunkProcessedBitmap(level) := 0.U
                    }
                    // Mark all memory blocks as free for the new collective
                    memFreeBitmap := (~0.U(NUM_MEM_BLOCKS.W))
                    dprintf(p"[s_idle] Resetting memory manager: all ${NUM_MEM_BLOCKS} blocks are now free.\n")

                    numFreeBlocks           := NUM_MEM_BLOCKS.U
                    allocSearchPtr          := 0.U
                    foundBlockValid         := false.B
                    
                    // Reset background chunk searcher
                    chunkSearchLevel        := 1.U
                    chunkSearchChunk        := 0.U
                    foundChunkValid         := false.B
                    isProcessing            := false.B
                }
                
                collectiveIdReg         := newCollectiveId
                collectiveTypeReg       := metaWord(23, 16)
                operationReg            := metaWord(31, 24)
                maxLevelReg             := metaWord(55, 48)
                currentLevelReg         := metaWord(63, 56)
                
                // Transition to wait for the second metadata word
                state                   := s_recv_meta2
            }
            
        // PRIORITY 2: No incoming packet, but the background search found a chunk.
        } .elsewhen(foundChunkValid) {
            dprintf(p"[s_idle] Consuming chunk C${foundChunkIndex} at L${foundChunkLevel} found by background search.\n")

            // Consume the found chunk
            processingLevel         := foundChunkLevel
            processingChunkIndex    := foundChunkIndex
            // Extract incoming block index for this level/chunk (packed fields)
            val extractedBlock = (incomingBlockFields(foundChunkLevel) >> (foundChunkIndex * BLOCK_INDEX_BITS.U))(BLOCK_INDEX_BITS-1, 0)
            blockIndexInFlightReg   := extractedBlock
            
            // SYNTH DEBUG: Track chunk processing start
            SynthesizePrintf {
                printf("[s_idle] Chunk Found: L%d C%d blk=%d (background)\n",
                       foundChunkLevel, foundChunkIndex, extractedBlock)
            }
            
            // Invalidate the found chunk to re-enable the background search.
            foundChunkValid         := false.B
            isProcessing            := true.B

            // Jump directly to processing
            isReadingInputData      := true.B
            state                   := s_dma_read
            readWordCount           := 0.U
            readReqCount            := 0.U
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
                        processedDataBuffer(baseElementIndex + i.U)     := incoming_bits.data((i + 1) * ELEMENT_WIDTH - 1, i * ELEMENT_WIDTH)
                    }.otherwise {
                        // Higher levels: Write to incomingDataBuffer (will be processed later)
                        incomingDataBuffer(baseElementIndex + i.U)      := incoming_bits.data((i + 1) * ELEMENT_WIDTH - 1, i * ELEMENT_WIDTH)
                    }
                }
            }

            // Only print for the first and last data word of the packet.
            when(receivedDataWordCount === 0.U || receivedDataWordCount === (NUM_DATA_WORDS.U - 1.U)) {
                val e0  = incoming_bits.data(ELEMENT_WIDTH-1, 0)
                val e1  = incoming_bits.data(2*ELEMENT_WIDTH-1, ELEMENT_WIDTH)
                dprintf(p"[s_recv_data] Recv word ${receivedDataWordCount}: elements[${baseElementIndex}]=0x${Hexadecimal(e0)}, elements[${baseElementIndex+1.U}]=0x${Hexadecimal(e1)}\n")
            }
            
            // DEBUG: Track elements 242-255 (words 121-127) - the tail end where 244 fails
            // when(receivedDataWordCount >= 121.U && receivedDataWordCount <= 127.U) {
            //     val e0 = incoming_bits.data(ELEMENT_WIDTH-1, 0)
            //     val e1 = incoming_bits.data(2*ELEMENT_WIDTH-1, ELEMENT_WIDTH)
            //     val elemBase = receivedDataWordCount * 2.U
            //     SynthesizePrintf {
            //         printf("[TAIL_RECV] L%d C%d w%d: e[%d]=0x%x, e[%d]=0x%x\n",
            //                currentLevelReg, chunkIndexReg, receivedDataWordCount,
            //                elemBase, e0, elemBase + 1.U, e1)
            //     }
            // }

            receivedDataWordCount := receivedDataWordCount + 1.U

            when(incoming_bits.last) {
                // Check if we need to send Setup ACK after draining this packet
                when(pendingSetupAck) {
                    // SynthesizePrintf {
                    //     printf("[s_recv_data] Setup packet fully drained. Transitioning to s_send_meta for ACK.\n")
                    // }
                    pendingSetupAck := false.B
                    receivedDataWordCount := 0.U
                    state := s_send_meta
                } .otherwise {
                    // Normal data packet processing
                    // Determine if storing incoming or processed (always incoming here, except L0)
                    isStoringIncomingData   := Mux(currentLevelReg === 0.U, false.B, true.B)

                    // Check if allocator is ready
                    when(foundBlockValid) { // Allocator has a block ready NOW
                        dprintf(p"[s_recv_data] Allocator ready. Latching block ${foundBlockIndex} and proceeding directly to DMA write.\n")
                        blockIndexToWriteReg    := foundBlockIndex
                        foundBlockValid         := false.B
                        allocSearchPtr          := foundBlockIndex + 1.U
                        state                   := s_dma_write // Go directly to write
                    } .otherwise { // Allocator not ready, need to wait
                        dprintf(p"[s_recv_data] Allocator not ready. Transitioning to s_wait_alloc.\n")
                        state                   := s_wait_alloc // Go to waiting state
                    }
                    // Reset counters regardless of next state
                    dmaWordCount            := 0.U
                    receivedDataWordCount   := 0.U

                    // Conditional debug print
                    when(currentLevelReg === 0.U) {
                        dprintf(p"[s_recv_data] Level 0 optimization: data already in processedDataBuffer, ready to send\n")
                    }.otherwise {
                        dprintf(p"[s_recv_data] Store first: storing chunk ${chunkIndexReg} for level ${currentLevelReg} to memory\n")
                    }

                    SynthesizePrintf {
                        printf("[s_recv_data] Received: myRank=%d, currentLevel=%d, chunk=%d, nextLevel=%d\n",
                            myRank, currentLevelReg, chunkIndexReg, nextLevel)
                    }
                }
            }
        }
    }
    
    .elsewhen(state === s_dma_write) {
        // Write data to memory
        when(dmaWordCount < WORDS_PER_CHUNK.U) {
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
                    // Mark allocated block as busy in bitmap
                    {
                        val idx         = blockIndexToWriteReg(BLOCK_INDEX_BITS-1, 0)
                        val mask        = (1.U(NUM_MEM_BLOCKS.W)) << idx
                        memFreeBitmap   := memFreeBitmap & (~mask)
                    }
                    numFreeBlocks                       := numFreeBlocks - 1.U
                    dprintf(p"[s_dma_write] Decremented free blocks to ${numFreeBlocks - 1.U}\n")

                    when(isStoringIncomingData) {
                        // Write incoming block index field into packed vector
                        val cur      = incomingBlockFields(currentLevelReg)
                        val shamtRaw = (chunkIndexReg(CHUNK_INDEX_BITS-1, 0) * BLOCK_INDEX_BITS.U)
                        val shamt    = shamtRaw(SHAMT_BITS-1, 0)
                        val mask     = ~(((BigInt(1) << BLOCK_INDEX_BITS) - 1).U(BLOCK_FIELDS_PER_LEVEL_WIDTH.W) << shamt)
                        val zext     = Cat(0.U((BLOCK_FIELDS_PER_LEVEL_WIDTH - BLOCK_INDEX_BITS).W), blockIndexToWriteReg.asUInt)
                        val insert   = (zext << shamt)
                        incomingBlockFields(currentLevelReg) := (cur & mask) | insert
                        dprintf(p"[s_dma_write] Allocating block ${blockIndexToWriteReg} for INCOMING L${currentLevelReg}C${chunkIndexReg}\n")
                    }.otherwise {
                        // Write processed block index field into packed vector
                        val cur      = processedBlockFields(currentLevelReg)
                        val shamtRaw = (chunkIndexReg(CHUNK_INDEX_BITS-1, 0) * BLOCK_INDEX_BITS.U)
                        val shamt    = shamtRaw(SHAMT_BITS-1, 0)
                        val mask     = ~(((BigInt(1) << BLOCK_INDEX_BITS) - 1).U(BLOCK_FIELDS_PER_LEVEL_WIDTH.W) << shamt)
                        val zext     = Cat(0.U((BLOCK_FIELDS_PER_LEVEL_WIDTH - BLOCK_INDEX_BITS).W), blockIndexToWriteReg.asUInt)
                        val insert   = (zext << shamt)
                        processedBlockFields(currentLevelReg) := (cur & mask) | insert
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
                
                // DEBUG: Track elements 242-255 (words 121-127) during DMA write
                // when(dmaWordCount >= 121.U && dmaWordCount <= 127.U) {
                //     val elemBase = dmaWordCount * 2.U
                //     val e0 = Mux(elemBase < NUM_DATA_ELEMENTS.U, dataSource(elemBase), 0.U)
                //     val e1 = Mux((elemBase + 1.U) < NUM_DATA_ELEMENTS.U, dataSource(elemBase + 1.U), 0.U)
                //     SynthesizePrintf {
                //         printf("[TAIL_WRITE] L%d C%d w%d isInc=%d blk=%d addr=0x%x: e[%d]=0x%x, e[%d]=0x%x\n",
                //                currentLevelReg, chunkIndexReg, dmaWordCount,
                //                isStoringIncomingData, blockIndexToWriteReg, tl.a.bits.address,
                //                elemBase, e0, elemBase + 1.U, e1)
                //     }
                // }
                
                state           := s_wait_write
            }
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
                    {
                        val bitIdx  = chunkIndexReg(CHUNK_INDEX_BITS-1, 0)
                        val bitMask = (1.U(MAX_CHUNKS.W)) << bitIdx
                        chunkArrivedBitmap(currentLevelReg) := chunkArrivedBitmap(currentLevelReg) | bitMask
                    }
                    dprintf(p"[s_wait_write] Incoming Level ${currentLevelReg} Chunk ${chunkIndexReg} stored\n")

                    // Just stored incoming data - now check if processing is possible
                    isStoringIncomingData   := false.B
                    
                    // Fast-track check: Is this incoming chunk now processable?
                    val incomingChunkProcessable = canProcessChunk(currentLevelReg, chunkIndexReg)
                    when(incomingChunkProcessable) {
                        dprintf(p"[s_wait_write] FAST-TRACK: Incoming chunk L${currentLevelReg}C${chunkIndexReg} is immediately processable\n")
                        // Set up for immediate processing
                        processingChunkIndex    := chunkIndexReg
                        processingLevel         := currentLevelReg
                        blockIndexInFlightReg   := (incomingBlockFields(currentLevelReg) >> (chunkIndexReg * BLOCK_INDEX_BITS.U))(BLOCK_INDEX_BITS-1, 0)
                        isProcessing            := true.B
                        isReadingInputData      := true.B
                        readWordCount           := 0.U
                        readReqCount            := 0.U
                        state                   := s_dma_read
                    }.otherwise {
                        state                   := s_idle
                    }
                }.otherwise {
                    dprintf(p"[s_wait_write] Processed Level ${currentLevelReg} Chunk ${chunkIndexReg} stored\n")
                    
                    // Calculate destination MAC for intermediate level (send to partner)
                    val partnerRankForSend = calculatePartnerRank(currentLevelReg, myRank)
                    val partnerMacForSend = calculatePartnerMac(currentLevelReg, myRank)
                    outgoingDstMac := partnerMacForSend
                    dstMacLowestByteReg := partnerMacForSend(7, 0)
                    
                    SynthesizePrintf {
                        printf("[RecursiveDoubling] Intermediate level: myRank=%d, currentLevel=%d, partnerRank=%d, dstMAC=0x%x\n",
                            myRank, currentLevelReg, partnerRankForSend, partnerMacForSend)
                    }
                    
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

            // DEBUG: Track elements 242-255 (words 121-127) - the tail end where 244 fails
            // when(readWordCount >= 121.U && readWordCount <= 127.U) {
            //     val e0 = dataWord(ELEMENT_WIDTH-1, 0)
            //     val e1 = dataWord(2*ELEMENT_WIDTH-1, ELEMENT_WIDTH)
            //     val elemBase = readWordCount * 2.U
            //     SynthesizePrintf {
            //         printf("[TAIL_READ] L%d C%d w%d isInc=%d blk=%d: e[%d]=0x%x, e[%d]=0x%x\n",
            //                processingLevel, processingChunkIndex, readWordCount,
            //                isReadingInputData, blockIndexInFlightReg,
            //                elemBase, e0, elemBase + 1.U, e1)
            //     }
            // }

            readWordCount   := readWordCount + 1.U
            readReqCount    := readReqCount + 1.U


            // If more words to fetch, issue next request; else proceed
            when(readWordCount + 1.U < WORDS_PER_CHUNK.U) {
                state       := s_dma_read
            }.otherwise {
                // Read phase complete. The data is now in internal buffers, so we can free the memory block.
                // The block index was latched during the read request. Use the registered value.
                // Mark block as free again after read complete
                {
                    val idx  = blockIndexInFlightReg(BLOCK_INDEX_BITS-1, 0)
                    val mask = (1.U(NUM_MEM_BLOCKS.W)) << idx
                    memFreeBitmap := memFreeBitmap | mask
                }
                numFreeBlocks                       := numFreeBlocks + 1.U
                dprintf(p"[s_wait_read] Read complete. Freed block ${blockIndexInFlightReg}. Free blocks: ${numFreeBlocks + 1.U}\n")

                // Now, determine next action
                when(isReadingInputData) {
                    // Phase 1 complete, start Phase 2: read previous level data
                    // Note: Level 0 never reaches read states with our optimization
                    dprintf(p"[s_wait_read] Input data read complete, starting previous level read\n")

                    // Calculate and store the NEXT block index for the second read phase
                    blockIndexInFlightReg   := (processedBlockFields(processingLevel - 1.U) >> (processingChunkIndex * BLOCK_INDEX_BITS.U))(BLOCK_INDEX_BITS-1, 0)

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
        // Convert IEEE-754 inputs to HardFloat recoded format (combinational helpers)
        val aRec        = hardfloat.recFNFromFN(8, 24, incomingDataBuffer(elementIdx))
        val bRec        = hardfloat.recFNFromFN(8, 24, memoryReadBuffer(elementIdx))
        fpAdder.io.a    := aRec
        fpAdder.io.b    := bRec
        
        // Set control signals
        fpAdder.io.roundingMode   := "b000".U
        fpAdder.io.detectTininess := 1.U
        fpAdder.io.subOp          := false.B

        // Since the FPU is combinational, the result is available immediately.
        val sumRec      = fpAdder.io.out
        val sum         = hardfloat.fNFromRecFN(8, 24, sumRec)
        // Always store IEEE result to processedDataBuffer for immediate use
        processedDataBuffer(elementIdx) := sum
        
        // DEBUG: Track elements 242-255 - the tail end where 244 fails
        // when(elementIdx >= 242.U && elementIdx <= 255.U) {
        //     SynthesizePrintf {
        //         printf("[TAIL_FPU] L%d C%d e[%d]: a=0x%x, b=0x%x, sum=0x%x\n",
        //                processingLevel, processingChunkIndex, elementIdx,
        //                incomingDataBuffer(elementIdx), memoryReadBuffer(elementIdx), sum)
        //     }
        // }

        // TRANSITION: When the last element is processed, move to the next state.
        when(elementIdx === (NUM_DATA_ELEMENTS - 1).U) {
            dprintf(p"[s_fp_add_pipe] All FP results collected.\n")
            dprintf(p"[s_fp_add_pipe] processedDataBuffer[0]=0x${Hexadecimal(processedDataBuffer(0))}, [1]=0x${Hexadecimal(processedDataBuffer(1))}\n")
            dprintf(p"[s_fp_add_pipe] Last FP sum: 0x${Hexadecimal(sum)}\n")
            
            when(processingLevel < maxLevelReg) {
                // Store processed data back to memory
                dprintf(p"[s_fp_add_pipe] Chunk ${processingChunkIndex} FPU complete for L${processingLevel}. Checking allocator...\n")
                
                // Set up info needed for DMA write *before* checking allocator
                isStoringIncomingData   := false.B // This is always processed data
                currentLevelReg         := processingLevel
                chunkIndexReg           := processingChunkIndex

                // Check if allocator is ready
                when(foundBlockValid) { // Allocator ready NOW
                    dprintf(p"[s_fp_add_pipe] Allocator ready. Latching block ${foundBlockIndex} and proceeding directly to DMA write.\n")
                    blockIndexToWriteReg    := foundBlockIndex
                    foundBlockValid         := false.B
                    allocSearchPtr          := foundBlockIndex + 1.U
                    state                   := s_dma_write // Go directly to write
                } .otherwise { // Allocator not ready, need to wait
                    dprintf(p"[s_fp_add_pipe] Allocator not ready. Transitioning to s_wait_alloc.\n")
                    state                   := s_wait_alloc // Go to waiting state
                }
                // Reset counter regardless of next state
                dmaWordCount            := 0.U
            }.otherwise {
                // Final level - send response directly without storing
                dprintf(p"[s_fp_add_pipe] Final level chunk ${processingChunkIndex} complete, sending response directly\n")
                currentLevelReg         := processingLevel
                chunkIndexReg           := processingChunkIndex
                
                // Calculate destination MAC for final level (send back to Level 0 source)
                val dstMacForFinal = Mux(level0SourceMacValid, level0SourceMac, IceNetConsts.ETH_BCAST_MAC)
                outgoingDstMac := dstMacForFinal
                dstMacLowestByteReg := dstMacForFinal(7, 0)
                
                // SynthesizePrintf {
                //     printf("[RecursiveDoubling] Final level: myRank=%d, dstMAC=0x%x\n", myRank, dstMacForFinal)
                // }
                
                state                   := s_send_meta
            }
        } .otherwise {
            elementIdx              := elementIdx + 1.U
        }
    }

    .elsewhen(state === s_wait_alloc) {
        io.out.valid := false.B // Ensure no output during wait

        when(foundBlockValid) {
            dprintf(p"[s_wait_alloc] Allocator ready! Latching block ${foundBlockIndex} and proceeding to DMA write.\n")
            // Consume the block info
            blockIndexToWriteReg    := foundBlockIndex
            foundBlockValid         := false.B
            allocSearchPtr          := foundBlockIndex + 1.U

            // Transition to DMA write (already set up counters and flags in previous state)
            state                   := s_dma_write
        } .otherwise {
            // Stay in this state, keep waiting
            state := s_wait_alloc
        }
    }

    .elsewhen(state === s_send_meta) {
        io.out.valid            := true.B
        outgoing_bits.data      := outgoingMetadataWord
        outgoing_bits.last      := false.B
        outgoing_bits.keep      := (~0.U((outer.params.dataWidth / 8).W))

        when(io.out.fire) {
            // Mark chunk as complete and start sending response
            // Note: set processed bit here (responses do not go to DMA write)
            {
                val bitIdx  = chunkIndexReg(CHUNK_INDEX_BITS-1, 0)
                val bitMask = (1.U(MAX_CHUNKS.W)) << bitIdx
                chunkProcessedBitmap(currentLevelReg) := chunkProcessedBitmap(currentLevelReg) | bitMask
            }
            
            // Invalidate background search if it was pointing to this chunk
            when(foundChunkValid && foundChunkLevel === currentLevelReg && foundChunkIndex === chunkIndexReg) {
                foundChunkValid := false.B
                dprintf(p"[s_send_meta] Invalidated background search result for L${currentLevelReg}C${chunkIndexReg}\n")
            }
            
            // Clear processing flag when chunk is complete
            isProcessing := false.B
            
            // Calculate partner for routing
            // Note: Module never sends Level 0 (only receives it from PyTorch)
            // Module sends Level 1-4:
            //   - Level 1-3: Send to partner node (intermediate levels)
            //   - Level 4: Send back to PyTorch/external (final result)
            val partnerRank = calculatePartnerRank(currentLevelReg, myRank)
            val partnerMac = calculatePartnerMac(currentLevelReg, myRank)
            
            // Set destination MAC for this packet
            // Level 1-3: Send to partner node
            // Level 4: Send back to Level 0 source (PyTorch) using stored MAC (fallback to broadcast if unknown)
            val dstMacForPacket = Mux(currentLevelReg < maxLevelReg, 
                                     partnerMac, 
                                     Mux(level0SourceMacValid, level0SourceMac, IceNetConsts.ETH_BCAST_MAC))
            // Destination MAC should already be set before entering s_send_meta state
            // This printf is just for verification (should match what was set at transition)
            SynthesizePrintf {
                printf("[s_send_meta] Sent: myRank=%d, currentLevel=%d, chunk=%d, nextLevel=%d\n",
                    myRank, currentLevelReg, chunkIndexReg, nextLevel)
            }
            
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
                state   := s_idle
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
        
        // DEBUG: Track elements 242-255 (words 121-127) - the tail end where 244 fails
        // when(sentDataWordCount >= 121.U && sentDataWordCount <= 127.U) {
        //     val elemBase = sentDataWordCount * 2.U
        //     SynthesizePrintf {
        //         printf("[TAIL_SEND] L%d C%d w%d: e[%d]=0x%x, e[%d]=0x%x\n",
        //                currentLevelReg, chunkIndexReg, sentDataWordCount,
        //                elemBase, processedDataBuffer(elemBase), 
        //                elemBase + 1.U, processedDataBuffer(elemBase + 1.U))
        //     }
        // }
        
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
                // Fast-track check: Does completing this chunk enable the next level?
                val nextLevelChunkProcessable = (nextLevel < MAX_RECURSION_LEVEL.U) && canProcessChunk(nextLevel, chunkIndexReg)
                when(nextLevelChunkProcessable) {
                    dprintf(p"[s_send_data] FAST-TRACK: Completing L${currentLevelReg}C${chunkIndexReg} enables next level L${nextLevel}C${chunkIndexReg}\n")
                    // Set up for immediate processing
                    processingChunkIndex    := chunkIndexReg
                    processingLevel         := nextLevel
                    blockIndexInFlightReg   := (incomingBlockFields(nextLevel) >> (chunkIndexReg * BLOCK_INDEX_BITS.U))(BLOCK_INDEX_BITS-1, 0)
                    isProcessing            := true.B
                    isReadingInputData      := true.B
                    readWordCount           := 0.U
                    readReqCount            := 0.U
                    state                   := s_dma_read
                }.otherwise {
                    state                   := s_idle
                }
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
    
    
    // --- Background Memory Block Allocator ---
    // This runs concurrently and continuously searches for a free memory block
    // whenever one hasn't already been found and latched.
    when(!foundBlockValid && numFreeBlocks > 0.U) { // Only search if needed and if blocks are potentially available
        val probeIdx  = allocSearchPtr(BLOCK_INDEX_BITS-1, 0)
        val probeMask = (1.U(NUM_MEM_BLOCKS.W)) << probeIdx
        when((memFreeBitmap & probeMask) =/= 0.U) {
            // Found a free block! Latch it and pause searching.
            foundBlockValid := true.B
            foundBlockIndex := allocSearchPtr
            dprintf(p"[BG_ALLOC] Found and latched free block index ${allocSearchPtr}\n")
        } .otherwise {
            // Block is busy, move to the next index for the next cycle
            allocSearchPtr := allocSearchPtr + 1.U // Wraps around automatically
        }
    }
    
    // --- Background Chunk Searcher ---
    // This runs concurrently and continuously searches for processable chunks
    // whenever one hasn't already been found and latched.
    when(!foundChunkValid && totalChunksForCollective > 0.U) { // Only search if needed and if chunks are potentially available
        val currentChunkProcessable = chunkSearchChunk < totalChunksForCollective && canProcessChunk(chunkSearchLevel, chunkSearchChunk)
        val currentlyProcessingThisChunk = isProcessing && (chunkSearchLevel === processingLevel) && (chunkSearchChunk === processingChunkIndex)
        
        when(currentChunkProcessable && !currentlyProcessingThisChunk) {
            // Found a processable chunk that's not currently being processed! Latch it and pause searching.
            foundChunkValid := true.B
            foundChunkLevel := chunkSearchLevel
            foundChunkIndex := chunkSearchChunk
            dprintf(p"[BG_CHUNK] Found and latched processable chunk C${chunkSearchChunk} at L${chunkSearchLevel}\n")
        } .otherwise {
            // Either chunk is not processable OR it's currently being processed - advance search pointer
            when(currentChunkProcessable && currentlyProcessingThisChunk) {
                dprintf(p"[BG_CHUNK] Found chunk C${chunkSearchChunk} at L${chunkSearchLevel} but it's currently being processed, skipping\n")
            }
            
            // Advance search pointer
            val nextChunk = chunkSearchChunk + 1.U
            when(nextChunk < totalChunksForCollective) {
                chunkSearchChunk := nextChunk
            } .otherwise {
                // Move to next level
                val nextSearchLevel = chunkSearchLevel + 1.U
                chunkSearchChunk := 0.U
                when(nextSearchLevel < MAX_RECURSION_LEVEL.U) {
                    chunkSearchLevel := nextSearchLevel
                } .otherwise {
                    // Wrap around to level 1 (level 0 is handled directly in s_recv_data)
                    chunkSearchLevel := 1.U
                }
            }
        }
    }
    
    // --- Input Ready Logic ---
    val memoryHasSpace = numFreeBlocks > 0.U
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