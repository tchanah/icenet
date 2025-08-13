package icenet.collective

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import icenet._
import icenet.IceNetConsts._

/**
 * Simple DMA Controller for fixed-size packet processing
 * 
 * This module provides a simple DMA implementation that:
 * - Receives fixed-size packets (64 bytes)
 * - Performs DMA write to memory
 * - Performs DMA read from memory  
 * - Sends packets back to network
 * 
 * Uses direct TileLink operations for simplicity and reliability.
 * 
 * Architecture:
 * - Single packet processing (one at a time)
 * - Direct TileLink client interface
 * - Simple state machine for DMA operations
 * - Fixed memory address (0x80000000)
 * - 4 source IDs for concurrent transactions
 */
class SimpleDmaController(implicit p: Parameters) extends LazyModule {
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLMasterParameters.v1(
    sourceId = IdRange(0, 4), // Use 4 source IDs
    name = "simple-dma-controller"
  )))))
  
  lazy val module = new SimpleDmaControllerModuleImp(this)
}

class SimpleDmaControllerModuleImp(outer: SimpleDmaController) extends LazyModuleImp(outer) {
  val io = IO(new Bundle {
    val net_in = Flipped(Decoupled(new StreamChannel(NET_IF_WIDTH)))
    val net_out = Decoupled(new StreamChannel(NET_IF_WIDTH))
  })
  
  // Get the TileLink client interface
  val (tl, edge) = outer.node.out(0)
  
  // Buffer for incoming packets
  val inputQueue = Module(new Queue(chiselTypeOf(io.net_in.bits), 4))
  inputQueue.io.enq <> io.net_in
  
  // --- State Machine Definition ---
  val s_IDLE :: s_WRITE_DATA :: s_WAIT_WRITE :: s_READ_DATA :: s_WAIT_READ :: s_SEND_DATA :: Nil = Enum(6)
  val state = RegInit(s_IDLE)
  
  // DMA parameters
  val dma_addr = RegInit("h80000000".U(48.W))
  val packet_len_bytes = 64.U(16.W) // Changed from 60 to 64 to match the full packet
  val words_to_write = (packet_len_bytes + 7.U) >> 3 // Round up to 8-byte words
  
  // Counters
  val word_count = RegInit(0.U(8.W))
  val data_buffer = Reg(Vec(16, UInt(64.W))) // Increased buffer size to 16 words
  
  // --- Default Connections ---
  io.net_out.valid := false.B
  io.net_out.bits := DontCare
  
  // Default TileLink signals
  tl.a.valid := false.B
  tl.a.bits := DontCare
  tl.d.ready := true.B
  
  // --- State Machine Logic ---
  when(state === s_IDLE) {
    when(inputQueue.io.deq.valid) {
      // Store the first word immediately
      data_buffer(0) := inputQueue.io.deq.bits.data
      inputQueue.io.deq.ready := true.B
      printf("[SimpleDmaController] Stored first word: data=0x%x at buffer[0]\n", inputQueue.io.deq.bits.data)
      
      // Start writing data to memory
      state := s_WRITE_DATA
      word_count := 0.U
      printf("[SimpleDmaController] Starting DMA write for %d words\n", words_to_write)
    }
  }
  .elsewhen(state === s_WRITE_DATA) {
    // Write data to memory using TileLink PutFullData
    when(word_count < words_to_write) {
      tl.a.valid := true.B
      tl.a.bits.opcode := TLMessages.PutFullData
      tl.a.bits.param := 0.U
      tl.a.bits.size := 3.U // 8 bytes
      tl.a.bits.source := word_count(1, 0) // Use word_count as source ID
      tl.a.bits.address := dma_addr + (word_count << 3)
      tl.a.bits.mask := "b11111111".U(8.W) // Full 8-byte mask
      tl.a.bits.data := data_buffer(word_count) // Use data from buffer instead of input
      
      when(tl.a.fire) {
        word_count := word_count + 1.U
        
        printf("[SimpleDmaController] Write request sent: word=%d, addr=0x%x, data=0x%x\n", 
          word_count, dma_addr + (word_count << 3), data_buffer(word_count))
        
        when(word_count === words_to_write - 1.U) {
          state := s_WAIT_WRITE
          printf("[SimpleDmaController] All write requests sent, waiting briefly\n")
        }
      }
    }
    
    // Continue storing incoming data in buffer
    when(inputQueue.io.deq.valid && word_count < words_to_write) {
      data_buffer(word_count + 1.U) := inputQueue.io.deq.bits.data
      inputQueue.io.deq.ready := true.B
      printf("[SimpleDmaController] Stored word %d: data=0x%x at buffer[%d]\n", 
        word_count + 1.U, inputQueue.io.deq.bits.data, word_count + 1.U)
    }
  }
  .elsewhen(state === s_WAIT_WRITE) {
    // Wait a few cycles for writes to complete, then proceed
    val wait_counter = RegInit(0.U(8.W))
    wait_counter := wait_counter + 1.U
    
    when(wait_counter > 10.U) {
      state := s_READ_DATA
      word_count := 0.U
      wait_counter := 0.U
      printf("[SimpleDmaController] Write wait complete, starting read\n")
    }
  }
  .elsewhen(state === s_READ_DATA) {
    // Read data back from memory
    when(word_count < words_to_write) {
      tl.a.valid := true.B
      tl.a.bits.opcode := TLMessages.Get
      tl.a.bits.param := 0.U
      tl.a.bits.size := 3.U // 8 bytes
      tl.a.bits.source := word_count(1, 0) + 4.U // Use different source IDs for reads
      tl.a.bits.address := dma_addr + (word_count << 3)
      tl.a.bits.mask := "b11111111".U(8.W)
      
      when(tl.a.fire) {
        word_count := word_count + 1.U
        printf("[SimpleDmaController] Read request sent: word=%d, addr=0x%x\n", 
          word_count, dma_addr + (word_count << 3))
        
        when(word_count === words_to_write - 1.U) {
          state := s_WAIT_READ
          printf("[SimpleDmaController] All read requests sent, waiting briefly\n")
        }
      }
    }
  }
  .elsewhen(state === s_WAIT_READ) {
    // Wait a few cycles for reads to complete, then proceed
    val wait_counter = RegInit(0.U(8.W))
    wait_counter := wait_counter + 1.U
    
    when(wait_counter > 10.U) {
      state := s_SEND_DATA
      word_count := 0.U
      wait_counter := 0.U
      printf("[SimpleDmaController] Read wait complete, starting send\n")
    }
  }
  .elsewhen(state === s_SEND_DATA) {
    // Send data back to network
    io.net_out.valid := true.B
    io.net_out.bits.data := data_buffer(word_count)
    io.net_out.bits.last := (word_count === words_to_write - 1.U)
    io.net_out.bits.keep := "b11111111".U(8.W)
    
    when(io.net_out.fire) {
      printf("[SimpleDmaController] Sending data: word=%d, data=0x%x, last=%d\n", 
        word_count, data_buffer(word_count), word_count === words_to_write - 1.U)
      
      word_count := word_count + 1.U
      when(word_count === words_to_write - 1.U) {
        state := s_IDLE
        printf("[SimpleDmaController] DMA operation complete, returning to idle\n")
      }
    }
  }
  
  // Input ready logic
  inputQueue.io.deq.ready := (state === s_IDLE) || (state === s_WRITE_DATA && tl.a.fire)
  
  // Debug prints
  when(state =/= RegNext(state)) {
    printf("[SimpleDmaController] STATE: %d\n", state)
  }
  
  when(io.net_in.fire) {
    printf("[SimpleDmaController] NET_IN: data=0x%x, last=%d\n", io.net_in.bits.data, io.net_in.bits.last)
  }
  
  when(io.net_out.fire) {
    printf("[SimpleDmaController] NET_OUT: data=0x%x, last=%d\n", io.net_out.bits.data, io.net_out.bits.last)
  }
} 