// filename: SimpleDMAController.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._
import freechips.rocketchip.tile.{HasCoreParameters} // Removed TileKey, HasNonDiplomaticTileParameters
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.rocket.HellaCacheIO
import org.chipsalliance.cde.config.Parameters


// DMA request bundle initiated by CNNController
class DMARequest(val coreMaxAddrBits: Int, val spadAddrWidthMax: Int) extends Bundle {
  val main_mem_addr       = UInt(coreMaxAddrBits.W)
  val spad_base_addr      = UInt(spadAddrWidthMax.W) // Base address for the target SPAD (IFM, Kernel, or OFM starting point)
  val length_bytes        = UInt(16.W) // Total bytes to transfer
  val is_write_to_main_mem  = Bool()     // True if reading from SPAD and writing to Main Memory
  val spad_target_is_ifm    = Bool()     // True if writing to IFM SPAD (main_mem -> IFM)
  val spad_target_is_kernel = Bool()     // True if writing to Kernel SPAD (main_mem -> Kernel)
                                        // If is_write_to_main_mem is true, these two are false (OFM -> main_mem)
}

class SimpleDMAControllerIO(val config: AcceleratorConfig)(implicit p: Parameters) extends Bundle {
  // Request from CNNController
  val dma_req = Flipped(Decoupled(new DMARequest(config.coreMaxAddrBits, log2Ceil(config.ifmDepth.max(config.kernelMaxDepth).max(config.ofmDepth)))))

  // Status back to CNNController
  val busy  = Output(Bool())
  val done  = Output(Bool())
  val error = Output(Bool())

  // Interface to RoCC's memory port (HellaCacheIO)
  val mem = new HellaCacheIO()(p)

  // Scratchpad Write Port
  val spad_write_en   = Output(Bool())
  val spad_write_addr = Output(UInt(log2Ceil(config.ifmDepth.max(config.kernelMaxDepth)).W)) // Max for IFM/Kernel
  val spad_write_data = Output(UInt(config.ifmDataType.dataWidth.max(config.kernelDataType.dataWidth).W))

  // Scratchpad Read Port
  val spad_read_en    = Output(Bool())
  // Address width for OFM read or general SPAD read if consolidating address spaces.
  val spad_read_addr  = Output(UInt(log2Ceil(config.ifmDepth.max(config.kernelMaxDepth).max(config.ofmDepth)).W))
  // Data width should be max of readable SPAD regions if data paths are shared. Here, OFM is the source for main_mem writes.
  val spad_read_data  = Input(UInt(config.ofmDataType.dataWidth.max(config.ifmDataType.dataWidth).max(config.kernelDataType.dataWidth).W))
}


class SimpleDMAController(val config: AcceleratorConfig)(implicit val p: Parameters) extends Module
  with HasCoreParameters with MemoryOpConstants {

  val io = IO(new SimpleDMAControllerIO(config)(p))

  val sIdle :: sReqMemReadBeat :: sWaitForMemReadBeatResp :: sWriteElementsToSpad :: sReadElementsFromSpad :: sReqMemWriteBeat :: sWaitForMemWriteBeatResp :: sError :: Nil = Enum(8)
  val state = RegInit(sIdle)

  val maxSpadAddrWidth = log2Ceil(config.ifmDepth.max(config.kernelMaxDepth).max(config.ofmDepth))
  val current_req = Reg(new DMARequest(config.coreMaxAddrBits, maxSpadAddrWidth))

  val total_bytes_to_transfer = Reg(UInt(16.W))
  val bytes_transferred_total = RegInit(0.U(16.W)) // Total bytes transferred for the current DMARequest

  val current_main_mem_addr_for_beat = Reg(UInt(config.coreMaxAddrBits.W)) // Beat-aligned address for memory op
  val current_spad_addr_element_wise = Reg(UInt(maxSpadAddrWidth.W)) // Element-wise address for SPAD op

  val spad_element_idx_in_beat = RegInit(0.U(log2Ceil(config.tileLinkBeatBytes + 1).W)) // Tracks elements within the current beat

  // Buffer to hold one beat of data from/to main memory
  val beat_buffer = Reg(UInt((config.tileLinkBeatBytes * 8).W))

  // Determine properties based on current_req
  val bytes_per_spad_element = Wire(UInt(8.W))

  when(current_req.is_write_to_main_mem) { // Reading from OFM SPAD
    bytes_per_spad_element := (config.ofmDataType.dataWidth / 8).U
  } .elsewhen(current_req.spad_target_is_ifm) { // Writing to IFM SPAD
    bytes_per_spad_element := (config.ifmDataType.dataWidth / 8).U
  } .elsewhen(current_req.spad_target_is_kernel) { // Writing to Kernel SPAD
    bytes_per_spad_element := (config.kernelDataType.dataWidth / 8).U
  } .otherwise { // Default case
    bytes_per_spad_element := (config.ifmDataType.dataWidth / 8).U // Default to IFM
  }

  val elements_per_beat = config.tileLinkBeatBytes.U / bytes_per_spad_element
  require(config.tileLinkBeatBytes > 0, "tileLinkBeatBytes must be greater than 0")
  // Ensure bytes_per_spad_element is not zero before division if it can be dynamically zero.
  // For fixed configs, this should be fine.

  // Default outputs
  io.dma_req.ready := (state === sIdle)
  io.busy  := (state =/= sIdle) && (state =/= sError)
  io.done  := false.B
  io.error := (state === sError)

  // HellaCacheIO request defaults
  io.mem.req.valid   := false.B
  io.mem.req.bits.addr  := current_main_mem_addr_for_beat
  io.mem.req.bits.tag   := 0.U // Simple DMA, one outstanding request to cache
  io.mem.req.bits.cmd   := M_XRD
  io.mem.req.bits.size  := log2Ceil(config.tileLinkBeatBytes).U // Always transfer a full beat
  io.mem.req.bits.signed := false.B
  io.mem.req.bits.data  := beat_buffer // For writes
  io.mem.req.bits.phys  := false.B
  io.mem.req.bits.no_alloc := false.B
  io.mem.req.bits.no_xcpt  := false.B

  // Scratchpad connections
  io.spad_write_en   := false.B
  io.spad_write_addr := current_req.spad_base_addr + current_spad_addr_element_wise
  // Data to SPAD comes from beat_buffer, sliced per element
  val data_to_spad_slice = beat_buffer >> (spad_element_idx_in_beat * bytes_per_spad_element * 8.U)

  // Corrected: Use MuxCase to select the slice using Scala Ints for slice boundaries
  io.spad_write_data := MuxCase(
    data_to_spad_slice(config.ifmDataType.dataWidth - 1, 0), // Default if other conditions not met
    Array(
      current_req.spad_target_is_ifm    -> data_to_spad_slice(config.ifmDataType.dataWidth - 1, 0),
      current_req.spad_target_is_kernel -> data_to_spad_slice(config.kernelDataType.dataWidth - 1, 0)
      // is_write_to_main_mem (false here) means writing to SPAD. Target IFM or Kernel.
    )
  )

  io.spad_read_en     := false.B
  io.spad_read_addr   := current_req.spad_base_addr + current_spad_addr_element_wise
  // Data from SPAD (io.spad_read_data) is assembled into beat_buffer

  val bytes_remaining_in_req = total_bytes_to_transfer - bytes_transferred_total

  // State Machine Logic
  switch(state) {
    is(sIdle) {
      when(io.dma_req.valid) {
        current_req := io.dma_req.bits
        total_bytes_to_transfer    := io.dma_req.bits.length_bytes
        bytes_transferred_total    := 0.U
        current_main_mem_addr_for_beat := io.dma_req.bits.main_mem_addr // Assume start addr is beat-aligned or cache handles it
        current_spad_addr_element_wise := 0.U // Relative to spad_base_addr
        spad_element_idx_in_beat   := 0.U
        io.dma_req.ready          := false.B

        when(io.dma_req.bits.length_bytes === 0.U) { // No data to transfer
            io.done := true.B
            io.dma_req.ready := true.B
            state := sIdle
        }.elsewhen(io.dma_req.bits.is_write_to_main_mem) {
            state := sReadElementsFromSpad
        } .otherwise {
            state := sReqMemReadBeat
        }
      }
    }

    is(sReqMemReadBeat) { // Request to read one beat from Main Memory
      io.mem.req.valid  := true.B
      io.mem.req.bits.cmd  := M_XRD
      // addr and size are already set

      when(io.mem.req.fire) {
        state := sWaitForMemReadBeatResp
      }
    }

    is(sWaitForMemReadBeatResp) { // Waiting for beat from Main Memory
      when(io.mem.resp.valid) {
        // Assuming xLen (coreDataBits from HasCoreParameters) == config.tileLinkBeatBytes * 8
        beat_buffer := io.mem.resp.bits.data // Store the received beat
        spad_element_idx_in_beat := 0.U      // Reset for processing elements from the new beat
        state := sWriteElementsToSpad
      }
    }

    is(sWriteElementsToSpad) { // Write elements from beat_buffer to Scratchpad
      val more_elements_in_beat = spad_element_idx_in_beat < elements_per_beat
      val more_bytes_to_transfer = bytes_transferred_total < total_bytes_to_transfer

      when(more_elements_in_beat && more_bytes_to_transfer) {
        io.spad_write_en := true.B
        // spad_write_addr and spad_write_data are connected

        // After this cycle, SPAD write will occur. Update counters for the next element/state.
        bytes_transferred_total    := bytes_transferred_total + bytes_per_spad_element
        current_spad_addr_element_wise := current_spad_addr_element_wise + 1.U
        spad_element_idx_in_beat   := spad_element_idx_in_beat + 1.U
        state := sWriteElementsToSpad // Stay to write next element from buffer
      } .otherwise { // Done with current beat or all data transferred
        when(bytes_transferred_total >= total_bytes_to_transfer) {
          io.done := true.B
          io.dma_req.ready := true.B
          state   := sIdle
        } .otherwise { // More beats to read
          current_main_mem_addr_for_beat := current_main_mem_addr_for_beat + config.tileLinkBeatBytes.U
          state := sReqMemReadBeat
        }
      }
    }

    is(sReadElementsFromSpad) { // Read elements from SPAD to fill beat_buffer
      val more_elements_to_fill_beat = spad_element_idx_in_beat < elements_per_beat
      val more_bytes_for_req = bytes_transferred_total < total_bytes_to_transfer

      when(more_elements_to_fill_beat && more_bytes_for_req) {
        io.spad_read_en := true.B // Assert read enable for current SPAD address

        // Corrected: Use MuxCase to select the slice using Scala Ints for slice boundaries
        val data_from_spad = MuxCase(
            io.spad_read_data(config.ofmDataType.dataWidth - 1, 0), // Default, should be OFM if is_write_to_main_mem
            Array(
                // This state implies current_req.is_write_to_main_mem is true, so data is from OFM SPAD
                current_req.is_write_to_main_mem  -> io.spad_read_data(config.ofmDataType.dataWidth - 1, 0)
            )
        )

        val current_shift = spad_element_idx_in_beat * bytes_per_spad_element * 8.U

        // Corrected: Determine the width of the data being read from SPAD dynamically for mask generation
        val current_element_data_width_for_mask = Wire(UInt(log2Ceil(config.tileLinkBeatBytes * 8 + 1).W))
        when(current_req.is_write_to_main_mem) { // Reading from OFM
            current_element_data_width_for_mask := config.ofmDataType.dataWidth.U
        } .otherwise {
            // This case should ideally not be hit if logic is correct for sReadElementsFromSpad state
            // Defaulting to IFM width as a fallback, but this path needs review if other SPADs can be read from.
            // For now, this state is entered only when is_write_to_main_mem = true (i.e. reading OFM)
            current_element_data_width_for_mask := config.ofmDataType.dataWidth.U
        }
        // Corrected: Create mask dynamically based on element width
        val mask = ((1.U << current_element_data_width_for_mask) - 1.U) << current_shift


        // Clear the region in beat_buffer and then OR with new data
        beat_buffer := (beat_buffer & (~mask).asUInt) | (data_from_spad << current_shift)

        // Update counters for the next element to be read from SPAD
        bytes_transferred_total := bytes_transferred_total + bytes_per_spad_element
        current_spad_addr_element_wise := current_spad_addr_element_wise + 1.U
        spad_element_idx_in_beat := spad_element_idx_in_beat + 1.U

        val is_last_element_for_beat_or_req = (spad_element_idx_in_beat + 1.U === elements_per_beat) ||
                                              (bytes_transferred_total + bytes_per_spad_element >= total_bytes_to_transfer)

        when(is_last_element_for_beat_or_req) {
          state := sReqMemWriteBeat
        } .otherwise {
          state := sReadElementsFromSpad // Stay to read next element into buffer
        }
      } .otherwise { // Beat buffer is full or all data is collected
        when(bytes_transferred_total >= total_bytes_to_transfer && bytes_transferred_total > 0.U) {
            state := sReqMemWriteBeat
        } .elsewhen (bytes_transferred_total === 0.U && total_bytes_to_transfer === 0.U) { // Should be caught by sIdle
            io.done := true.B
            io.dma_req.ready := true.B
            state := sIdle
        } .otherwise {
            state := sError // Should have data to send or be done
        }
      }
    }


    is(sReqMemWriteBeat) { // Request to write one beat (from beat_buffer) to Main Memory
      io.mem.req.valid := true.B
      io.mem.req.bits.cmd  := M_XWR
      io.mem.req.bits.data := beat_buffer

      when(io.mem.req.fire) {
        state := sWaitForMemWriteBeatResp
      }
    }

    is(sWaitForMemWriteBeatResp) { // Waiting for Main Memory write acknowledgment
      when(io.mem.resp.valid) { // Write ack received
        spad_element_idx_in_beat := 0.U // Reset for next beat assembly

        when(bytes_transferred_total >= total_bytes_to_transfer) {
          io.done := true.B
          io.dma_req.ready := true.B
          state   := sIdle
        } .otherwise { // More beats to write
          current_main_mem_addr_for_beat := current_main_mem_addr_for_beat + config.tileLinkBeatBytes.U
          beat_buffer := 0.U // Clear buffer for next assembly
          state := sReadElementsFromSpad
        }
      }
    }

    is(sError) {
      io.error := true.B
      // Consider if reset to sIdle is desired after error or manual reset needed
    }
  }

  // Handle memory exceptions
  // Corrected: Check s2_xcpt by casting to UInt and checking for non-zero
  when(io.mem.s2_xcpt.asUInt =/= 0.U) {
    state := sError
  }
}