// filename: SimpleDMAController.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._
import freechips.rocketchip.rocket.constants.MemoryOpConstants 

// AcceleratorConfig is now defined in AcceleratorConfig.scala
// Ensure AcceleratorConfig.scala is in the same package or imported correctly.

// --- Simple Memory Interface Bundles (defined in this file) ---
class DmaSimpleMemReq(val addrWidth: Int, val dataWidth: Int, val beatBytes: Int) extends Bundle {
  val addr    = UInt(addrWidth.W)
  val data    = UInt(dataWidth.W)
  val isWrite = Bool()
  val size    = UInt(log2Ceil(log2Ceil(beatBytes) + 1).W) // lg(beatBytes)
}

class DmaSimpleMemResp(val dataWidth: Int) extends Bundle {
  val data = UInt(dataWidth.W)
}
// --- End Simple Memory Interface Bundles ---

// DMA request bundle (remains the same)
class DMARequest(val coreMaxAddrBits: Int, val spadAddrWidthMax: Int) extends Bundle {
  val main_mem_addr       = UInt(coreMaxAddrBits.W)
  val spad_base_addr      = UInt(spadAddrWidthMax.W)
  val length_bytes        = UInt(16.W)
  val is_write_to_main_mem  = Bool()
  val spad_target_is_ifm    = Bool()
  val spad_target_is_kernel = Bool()
}

class SimpleDMAControllerIO(val config: AcceleratorConfig) extends Bundle {
  val dma_req = Flipped(Decoupled(new DMARequest(config.coreMaxAddrBits, log2Ceil(config.ifmDepth max config.kernelMaxDepth max config.ofmDepth)))) // spadAddrWidthMax derived example

  val mem_req  = Decoupled(new DmaSimpleMemReq(config.coreMaxAddrBits, config.tileLinkBeatBytes * 8, config.tileLinkBeatBytes))
  val mem_resp = Flipped(Valid(new DmaSimpleMemResp(config.tileLinkBeatBytes * 8)))

  // SPAD interfaces - Address widths need to be sufficient for their respective depths
  // Example derivation for address widths (adjust as needed for your SPAD implementation)
  val spad_ifm_addr_width = log2Ceil(config.ifmDepth)
  val spad_kernel_addr_width = log2Ceil(config.kernelMaxDepth)
  val spad_ofm_addr_width = log2Ceil(config.ofmDepth)


  val spad_ifm_addr    = Output(UInt(spad_ifm_addr_width.W))
  val spad_ifm_data_in = Output(UInt(config.ifmDataType.dataWidth.W))
  val spad_ifm_wen     = Output(Bool())

  val spad_kernel_addr    = Output(UInt(spad_kernel_addr_width.W))
  val spad_kernel_data_in = Output(UInt(config.kernelDataType.dataWidth.W))
  val spad_kernel_wen     = Output(Bool())

  val spad_ofm_addr     = Output(UInt(spad_ofm_addr_width.W))
  val spad_ofm_data_out = Input(UInt(config.ofmDataType.dataWidth.W))

  val done  = Output(Bool())
  val error = Output(Bool())
  val busy  = Output(Bool())
}

class SimpleDMAController(val config: AcceleratorConfig) extends Module {
  // Deriving spadAddrWidthMax for internal register, matching DMARequest definition in IO
  val spadAddrWidthMaxDerived = log2Ceil(config.ifmDepth max config.kernelMaxDepth max config.ofmDepth)
  val io = IO(new SimpleDMAControllerIO(config))


  val sIdle :: sReadElementsFromSpad :: sWriteBeatToSpad :: sReqMemReadBeat :: sWaitForMemReadBeatData :: sReqMemWriteBeat :: sWaitForMemWriteBeatResp :: sError :: Nil = Enum(8)
  val state = RegInit(sIdle)

  val current_dma_req              = Reg(new DMARequest(config.coreMaxAddrBits, spadAddrWidthMaxDerived))
  val beat_buffer                  = Reg(UInt((config.tileLinkBeatBytes * 8).W))
  val current_main_mem_addr_for_beat = Reg(UInt(config.coreMaxAddrBits.W))
  val current_spad_addr_for_element = Reg(UInt(spadAddrWidthMaxDerived.W))
  val bytes_transferred_in_current_beat = Reg(UInt(log2Ceil(config.tileLinkBeatBytes + 1).W))
  val bytes_transferred_total      = Reg(UInt(io.dma_req.bits.length_bytes.getWidth.W))
  val total_bytes_to_transfer      = Reg(UInt(io.dma_req.bits.length_bytes.getWidth.W))

  // Default outputs
  io.dma_req.ready    := false.B
  io.spad_ifm_wen     := false.B
  io.spad_ifm_addr    := current_spad_addr_for_element // Connect to the common element address tracker
  io.spad_ifm_data_in := 0.U
  io.spad_kernel_wen  := false.B
  io.spad_kernel_addr := current_spad_addr_for_element // Connect to the common element address tracker
  io.spad_kernel_data_in := 0.U
  io.spad_ofm_addr    := current_spad_addr_for_element // Connect to the common element address tracker

  io.mem_req.valid         := false.B
  io.mem_req.bits.addr     := current_main_mem_addr_for_beat
  io.mem_req.bits.data     := beat_buffer
  io.mem_req.bits.isWrite  := false.B
  io.mem_req.bits.size     := log2Ceil(config.tileLinkBeatBytes).U

  io.done  := false.B
  io.error := false.B
  io.busy  := (state =/= sIdle)


  switch(state) {
    is(sIdle) {
      io.dma_req.ready := true.B
      when(io.dma_req.valid) {
        current_dma_req                := io.dma_req.bits
        current_main_mem_addr_for_beat := io.dma_req.bits.main_mem_addr
        current_spad_addr_for_element  := io.dma_req.bits.spad_base_addr // SPAD base address is per element
        total_bytes_to_transfer        := io.dma_req.bits.length_bytes
        bytes_transferred_total        := 0.U
        bytes_transferred_in_current_beat := 0.U
        beat_buffer                    := 0.U

        when(io.dma_req.bits.length_bytes === 0.U) {
          io.done := true.B
        } .elsewhen(io.dma_req.bits.is_write_to_main_mem) {
          state := sReadElementsFromSpad
        } .otherwise {
          state := sReqMemReadBeat
        }
      }
    }

    is(sReadElementsFromSpad) { // Assemble one beat from OFM SPAD
      val bytes_per_ofm_element = config.ofmDataType.dataWidth / 8
      val current_beat_offset_bits = bytes_transferred_in_current_beat << 3

      io.spad_ofm_addr := current_spad_addr_for_element

      // This assumes io.spad_ofm_data_out is combinatorially available or latched correctly
      // LSB of OFM element data goes to LSB of its segment in beat_buffer.
      // Ensure masking if beat_buffer is wider than needed for this element.
      beat_buffer := beat_buffer | (io.spad_ofm_data_out.asUInt << current_beat_offset_bits)

      val next_bytes_in_beat = bytes_transferred_in_current_beat + bytes_per_ofm_element.U
      val next_total_bytes_overall = bytes_transferred_total + next_bytes_in_beat

      when(next_bytes_in_beat >= config.tileLinkBeatBytes.U || next_total_bytes_overall >= total_bytes_to_transfer) {
        // Beat is full, or this is the last element for the transfer
        current_spad_addr_for_element := current_spad_addr_for_element + bytes_per_ofm_element.U // Update for the element just read
        bytes_transferred_in_current_beat := next_bytes_in_beat // Capture bytes for this beat
        state := sReqMemWriteBeat
      } .otherwise {
        bytes_transferred_in_current_beat := next_bytes_in_beat
        current_spad_addr_for_element := current_spad_addr_for_element + bytes_per_ofm_element.U
        state := sReadElementsFromSpad
      }
    }

    is(sWriteBeatToSpad) { // Write one beat from beat_buffer to IFM or Kernel SPAD
      val shifted_beat_buffer = beat_buffer >> (bytes_transferred_in_current_beat << 3)
      val data_to_write_spad = Wire(UInt()) // 让 Chisel 推断或稍后连接

      // 根据 spad_target 选择正确的宽度 (Scala Int) 和进行提取
      val ifm_data_width = config.ifmDataType.dataWidth // Scala Int
      val kernel_data_width = config.kernelDataType.dataWidth // Scala Int
      val bytes_per_spad_element_write = Wire(UInt((log2Ceil(config.tileLinkBeatBytes) + 1).W)) // 假设最大元素宽度不超过beatBytes

      when(current_dma_req.spad_target_is_ifm) {
        data_to_write_spad := shifted_beat_buffer(ifm_data_width - 1, 0)
        bytes_per_spad_element_write := (ifm_data_width / 8).U

        io.spad_ifm_addr    := current_spad_addr_for_element
        io.spad_ifm_data_in := data_to_write_spad
        io.spad_ifm_wen     := true.B
      } .elsewhen(current_dma_req.spad_target_is_kernel) {
        data_to_write_spad := shifted_beat_buffer(kernel_data_width - 1, 0)
        bytes_per_spad_element_write := (kernel_data_width / 8).U

        io.spad_kernel_addr    := current_spad_addr_for_element
        io.spad_kernel_data_in := data_to_write_spad
        io.spad_kernel_wen     := true.B
      } .otherwise {
        data_to_write_spad := 0.U // Default or error
        bytes_per_spad_element_write := 0.U
        state := sError // Invalid SPAD target
      }

      // 后续逻辑使用 bytes_per_spad_element_write (它现在是一个 UInt Wire，根据条件被赋值)
      val next_bytes_in_beat = bytes_transferred_in_current_beat + bytes_per_spad_element_write

      val all_bytes_for_this_mem_beat_written_to_spad = next_bytes_in_beat >= config.tileLinkBeatBytes.U
      // Check if all bytes for the *entire transfer* that came *from this specific memory beat* have been written to SPAD
      val effective_bytes_from_mem_this_beat = Mux((total_bytes_to_transfer - (bytes_transferred_total - config.tileLinkBeatBytes.U)) < config.tileLinkBeatBytes.U,
                                                   total_bytes_to_transfer - (bytes_transferred_total - config.tileLinkBeatBytes.U),
                                                   config.tileLinkBeatBytes.U)
      val all_relevant_bytes_from_mem_beat_written = next_bytes_in_beat >= effective_bytes_from_mem_beat_processed

      when(all_relevant_bytes_from_mem_beat_written) {
        current_spad_addr_for_element := current_spad_addr_for_element + bytes_per_spad_element_write // Update for element just written
        bytes_transferred_in_current_beat := 0.U
        // beat_buffer cleared before next mem read if needed (or naturally overwritten)

        when(bytes_transferred_total >= total_bytes_to_transfer) {
          io.done := true.B
          io.dma_req.ready := true.B
          state := sIdle
        } .otherwise {
          state := sReqMemReadBeat
        }
      } .otherwise {
        bytes_transferred_in_current_beat := next_bytes_in_beat
        current_spad_addr_for_element := current_spad_addr_for_element + bytes_per_spad_element_write
        state := sWriteBeatToSpad
      }
    }

    is(sReqMemReadBeat) {
      io.mem_req.valid         := true.B
      io.mem_req.bits.isWrite  := false.B
      io.mem_req.bits.addr     := current_main_mem_addr_for_beat
      io.mem_req.bits.size     := log2Ceil(config.tileLinkBeatBytes).U

      when(io.mem_req.fire) {
        state := sWaitForMemReadBeatData
      }
    }

    is(sWaitForMemReadBeatData) {
      when(io.mem_resp.valid) {
        beat_buffer := io.mem_resp.bits.data
        current_main_mem_addr_for_beat := current_main_mem_addr_for_beat + config.tileLinkBeatBytes.U
        
        // Determine actual bytes received relevant to transfer if this is the last beat
        val bytes_just_read_from_mem = config.tileLinkBeatBytes.U
        val remaining_bytes = total_bytes_to_transfer - bytes_transferred_total
        val effective_bytes_this_beat = Mux(remaining_bytes < bytes_just_read_from_mem, remaining_bytes, bytes_just_read_from_mem)
        
        bytes_transferred_total := bytes_transferred_total + effective_bytes_this_beat
        bytes_transferred_in_current_beat := 0.U // Reset for writing this new beat to SPAD
        state := sWriteBeatToSpad
      }
    }

    is(sReqMemWriteBeat) {
      io.mem_req.valid         := true.B
      io.mem_req.bits.isWrite  := true.B
      io.mem_req.bits.addr     := current_main_mem_addr_for_beat
      io.mem_req.bits.data     := beat_buffer // beat_buffer was filled in sReadElementsFromSpad
      io.mem_req.bits.size     := log2Ceil(config.tileLinkBeatBytes).U

      when(io.mem_req.fire) {
        state := sWaitForMemWriteBeatResp
      }
    }

    is(sWaitForMemWriteBeatResp) {
      when(io.mem_resp.valid) {
        // bytes_transferred_in_current_beat was the amount assembled into beat_buffer
        // This amount is what's relevant for the current transfer, even if a full beat was written to mem.
        bytes_transferred_total := bytes_transferred_total + bytes_transferred_in_current_beat
        current_main_mem_addr_for_beat := current_main_mem_addr_for_beat + config.tileLinkBeatBytes.U
        
        bytes_transferred_in_current_beat := 0.U // Reset for next beat assembly
        beat_buffer := 0.U // Clear buffer for next assembly

        when(bytes_transferred_total >= total_bytes_to_transfer) {
          io.done := true.B
          io.dma_req.ready := true.B
          state   := sIdle
        } .otherwise {
          state := sReadElementsFromSpad
        }
      }
    }

    is(sError) {
      io.error := true.B
      io.dma_req.ready := true.B // Allow new requests to potentially clear error
      when(io.dma_req.valid) {
          state := sIdle // Or a specific reset state
      }
    }
  }
  // Helper for SPAD write state, needs to be calculated based on bytes_transferred_total updated after mem read
  val effective_bytes_from_mem_beat_processed = Wire(UInt(log2Ceil(config.tileLinkBeatBytes + 1).W))
  val bytes_total_before_this_mem_read = RegNext(bytes_transferred_total, init=0.U) // Capture total before current mem beat was added in sWaitForMemReadBeatData

  when(state === sWriteBeatToSpad) {
    val bytes_remaining_for_transfer = total_bytes_to_transfer - bytes_total_before_this_mem_read
    effective_bytes_from_mem_beat_processed := Mux(bytes_remaining_for_transfer < config.tileLinkBeatBytes.U,
                                                 bytes_remaining_for_transfer,
                                                 config.tileLinkBeatBytes.U)
  } .otherwise {
    effective_bytes_from_mem_beat_processed := config.tileLinkBeatBytes.U // Default or don't care
  }

}