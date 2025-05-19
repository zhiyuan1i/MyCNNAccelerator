// filename: SimpleDMAEngine.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile.HasCoreParameters // For coreMaxAddrBits, xLen etc.
// It's good practice to ensure TileKey is available if p(TileKey) is used,
// but here we pass coreMaxAddrBits explicitly to the IO bundle.
// import freechips.rocketchip.tile.TileKey

import CNNAcceleratorISA._

// IO Bundle for the implementation part of the LazyModule
class SimpleDMATLModuleIO(
    val config: AcceleratorConfig,
    val coreMaxAddrBitsParam: Int // Explicitly pass the address width
)(implicit p: Parameters) extends Bundle {
  // Control from MyCNNRoCC (or a higher-level controller)
  val start = Input(Bool())
  val mem_base_addr = Input(UInt(coreMaxAddrBitsParam.W)) // Use passed width
  val length_bytes = Input(UInt(CNNAcceleratorISA.dmaConfigLenBits.W))
  // The following line was causing: type mismatch; found: chisel3.UInt required: chisel3.ActualDirection
  // This usually means the Chisel compiler is confused, often due to other errors.
  // The syntax itself is correct. If the error persists after other fixes,
  // we might try renaming 'direction' or using a literal for the width to isolate.
  val directionBits = Input(UInt(CNNAcceleratorISA.dmaConfigDirBits.W))
  
  val buffer_id = Input(UInt(CNNAcceleratorISA.dmaConfigBufIdBits.W))

  // Status to MyCNNRoCC
  val busy = Output(Bool())
  val done = Output(Bool())
  val dma_error = Output(Bool())

  // Scratchpad (Internal Buffer) Interface - driven by DMA to be connected by RoCC
  val spad_write_en = Output(Bool())
  val spad_write_addr = Output(UInt(32.W)) // Generic width, RoCC maps this to buffer
  val spad_write_data = Output(SInt(config.dataWidth.W))

  val spad_read_addr = Output(UInt(32.W))  // Generic width, RoCC maps this from buffer
  val spad_read_data = Input(SInt(config.dataWidth.W))
}

// SimpleDMAEngine as a LazyModule for TileLink integration
class SimpleDMAEngine(val accConfig: AcceleratorConfig, numOutstandingReqs: Int = 4)(implicit p: Parameters) extends LazyModule {
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLClientParameters(
    name = "simple-dma-engine",
    sourceId = IdRange(0, numOutstandingReqs),
    requestFifo = true
  )))))

  lazy val module = new SimpleDMAEngineModuleImp(this)
}

class SimpleDMAEngineModuleImp(outer: SimpleDMAEngine)(implicit p: Parameters) extends LazyModuleImp(outer)
  with HasCoreParameters { // Provides xLen, coreMaxAddrBits, etc.

  // Instantiate the IO, passing the coreMaxAddrBits from HasCoreParameters
  val io = IO(new SimpleDMATLModuleIO(outer.accConfig, coreMaxAddrBits)(p))

  val (tl, edge) = outer.node.out.head

  val sIdle :: sTLRead_ReqAddr :: sTLRead_WaitData :: sTLRead_WriteToBuf :: sTLWrite_ReadFromBuf :: sTLWrite_ReqAddr :: sTLWrite_WaitAck :: sDone :: sError :: Nil = Enum(9)
  val state = RegInit(sIdle)

  val mem_base_addr_reg = Reg(UInt(coreMaxAddrBits.W))
  val length_bytes_reg = Reg(UInt(CNNAcceleratorISA.dmaConfigLenBits.W))
  val direction_reg = Reg(UInt(CNNAcceleratorISA.dmaConfigDirBits.W))
  // val buffer_id_reg = Reg(UInt(CNNAcceleratorISA.dmaConfigBufIdBits.W)) // DMA uses io.buffer_id directly

  val bytes_transferred_count = RegInit(0.U(CNNAcceleratorISA.dmaConfigLenBits.W))
  val current_mem_addr = Reg(UInt(coreMaxAddrBits.W))
  val current_spad_addr = Reg(UInt(32.W)) // Assuming spad addresses are local to each buffer concept

  val beatBytes = tl.a.bits.data.getWidth / 8
  // Ensure buffer_elems_per_beat is at least 1, and handle dataWidth being 0 (though unlikely for practical data)
  val buffer_elems_per_beat = if (outer.accConfig.dataWidth > 0) {
    val elems = beatBytes * 8 / outer.accConfig.dataWidth
    if (elems > 0) elems else 1
  } else {
    1
  }


  io.busy := (state =/= sIdle) && (state =/= sDone) && (state =/= sError)
  io.done := (state === sDone)
  io.dma_error := (state === sError)

  io.spad_write_en := false.B
  io.spad_write_addr := 0.U // Default, will be current_spad_addr in active states
  io.spad_write_data := 0.S
  io.spad_read_addr := 0.U  // Default, will be current_spad_addr in active states

  tl.a.valid := false.B
  tl.a.bits := DontCare
  tl.b.ready := true.B
  tl.c.valid := false.B
  tl.c.bits := DontCare
  tl.d.ready := false.B
  tl.e.valid := false.B
  tl.e.bits := DontCare

  val mem_data_beat_reg = Reg(UInt((beatBytes * 8).W))
  val spad_data_to_write_beat_reg = Reg(UInt((beatBytes*8).W)) // For BUF_TO_MEM
  // val sub_beat_counter = RegInit(0.U(log2Ceil(buffer_elems_per_beat + 1).W)) // May not be needed if writing full beat

  val dma_start_reg = RegNext(io.start, false.B) // Detect rising edge of start
  val dma_started_this_cycle = io.start && !dma_start_reg


  switch(state) {
    is(sIdle) {
      when(dma_started_this_cycle) { // Use registered start to avoid re-triggering if start held high
        mem_base_addr_reg := io.mem_base_addr
        length_bytes_reg := io.length_bytes
        direction_reg := io.directionBits
        // buffer_id_reg := io.buffer_id // Latch buffer_id for the current operation
        current_mem_addr := io.mem_base_addr
        current_spad_addr := 0.U
        bytes_transferred_count := 0.U
        // sub_beat_counter := 0.U // Reset if used
        mem_data_beat_reg := 0.U

        when(io.length_bytes === 0.U) {
          state := sDone
        } .elsewhen(io.directionBits === DMADirection.MEM_TO_BUF) {
          state := sTLRead_ReqAddr
        } .elsewhen(io.directionBits === DMADirection.BUF_TO_MEM) {
          state := sTLWrite_ReadFromBuf // Enable write path
        } .otherwise {
          state := sError
        }
      }
    }

    is(sTLRead_ReqAddr) {
      val (legal, get) = edge.Get(
        fromSource = 0.U, // Manage source IDs for multiple outstanding requests
        toAddress = current_mem_addr,
        lgSize = log2Ceil(beatBytes).U
      )
      when(legal) {
        tl.a.valid := true.B
        tl.a.bits := get
        when(tl.a.ready) {
          state := sTLRead_WaitData
        }
      } .otherwise {
        state := sError
      }
    }

    is(sTLRead_WaitData) {
      tl.d.ready := true.B
      when(tl.d.fire) {
        when(tl.d.bits.denied || tl.d.bits.corrupt) {
            state := sError
        } .otherwise {
            mem_data_beat_reg := tl.d.bits.data
            state := sTLRead_WriteToBuf
        }
      }
    }

    is(sTLRead_WriteToBuf) {
      // Simplified: write one beat from mem_data_beat_reg to spad.
      // Assumes dataWidth matches beatBytes*8 or RoCC handles sub-beat packing/unpacking.
      // For now, let's assume RoCC handles data element extraction from spad_write_data
      // if spad_write_data is wider than one element. Or DMA does packing if spad_write_data is one element.
      // The current SimpleDMATLModuleIO has spad_write_data as config.dataWidth.
      // This implies the DMA engine must break down the 'mem_data_beat_reg' (beatBytes*8 wide)
      // into 'config.dataWidth' chunks and write them sequentially to SPAD.
      // This adds complexity with a sub_beat_counter.
      // For simplicity, let's assume config.dataWidth IS beatBytes*8 for this example,
      // or that spad_write_data is a Vec.
      // Given current io.spad_write_data is SInt(config.dataWidth.W), DMA must iterate.
      // This part is not fully implemented here for brevity.
      // A common simplification: spad_write_data is beat-wide, RoCC unpacks.
      // Or, this state iterates for each element within the beat.
      // For now, we'll pretend one write is one element of config.dataWidth, and advance spad_addr by 1 element.
      // And assume beatBytes is one element. THIS IS A MAJOR SIMPLIFICATION.

      io.spad_write_en := true.B
      io.spad_write_addr := current_spad_addr
      // This assignment is problematic if beatBytes*8 != config.dataWidth
      // For now, taking the LSBs, assuming RoCC will handle it or config matches.
      io.spad_write_data := mem_data_beat_reg(outer.accConfig.dataWidth - 1, 0).asSInt

      // This state should ideally last for buffer_elems_per_beat cycles if dataWidth < beatBytes*8
      // current_spad_addr needs to increment per element written.
      // bytes_transferred_count increments per memory beat.

      // After one "write to buffer" cycle (could be one element or one full beat based on above simplification)
      val next_spad_addr = current_spad_addr + 1.U // Increment by 1 element address
      val next_bytes_transferred = bytes_transferred_count + beatBytes.U // One full beat from memory processed

      current_spad_addr := next_spad_addr
      // current_mem_addr should advance only when a full beat is consumed and a new one is needed.
      // If we are processing elements within a beat, mem_addr doesn't change yet.
      // The FSM implies one memory read maps to one spad write cycle.
      current_mem_addr := current_mem_addr + beatBytes.U
      bytes_transferred_count := next_bytes_transferred


      when(next_bytes_transferred >= length_bytes_reg) {
        state := sDone
      } .elsewhen (next_bytes_transferred < length_bytes_reg) {
        state := sTLRead_ReqAddr // Request next beat
      } .otherwise { // Should not happen if length_bytes_reg is multiple of beatBytes or handled
        state := sDone // Or error
      }
    }

    is(sTLWrite_ReadFromBuf) {
        // Read data from SPAD via io.spad_read_data
        // This state needs to assert io.spad_read_addr
        io.spad_read_addr := current_spad_addr
        // Data will be available on io.spad_read_data on the next cycle (combinatorial read from buffer + RegNext in MinimalBuffer)
        // Or if buffer is sync read, it takes a cycle. Assume MinimalBuffer's RegNext makes it available "next cycle" relative to addr.
        // For TL Put, we need to prepare data for tl.a.bits.data.
        // This might require a cycle to read from spad and then a cycle to send.

        // Let's assume data is read and available in the same cycle for simplicity of FSM state count,
        // or that MyCNNRoCCModuleImp ensures data is ready for spad_read_data when DMA wants it.
        // A more realistic FSM would have: sTLWrite_SetSpadAddr, sTLWrite_LatchSpadData, then sTLWrite_ReqAddr.
        // For now, assume io.spad_read_data is valid when this state is entered (after current_spad_addr is set).

        // This simplified FSM will take spad_read_data and directly try to send it.
        // This again assumes spad_read_data provides a full beatBytes chunk.
        // If config.dataWidth < beatBytes*8, packing logic is needed here.
        // Taking LSBs for now.
        spad_data_to_write_beat_reg := io.spad_read_data.asUInt // Zero-extend if narrower

        state := sTLWrite_ReqAddr
    }

    is(sTLWrite_ReqAddr) {
        // Prepare a TileLink PutFullData message (write request)
        val (legal, put) = edge.Put(
            fromSource = 0.U,
            toAddress = current_mem_addr,
            lgSize = log2Ceil(beatBytes).U,
            data = spad_data_to_write_beat_reg // Assumes spad_data_to_write_beat_reg is prepared and beatBytes wide
        )
        // For PutPartialData: edge.Put(..., mask = ...)

        when(legal) {
            tl.a.valid := true.B
            tl.a.bits := put
            when(tl.a.ready) { // TL slave accepts the request
                // For PutFullData, data is on channel A. Slave grants on D.
                state := sTLWrite_WaitAck
            }
        } .otherwise {
            state := sError
        }
    }

    is(sTLWrite_WaitAck) {
        // Wait for Grant/Ack on Channel D
        tl.d.ready := true.B // We are ready to accept the grant/response
        when(tl.d.fire) {
            // Check tl.d.bits.denied or tl.d.bits.corrupt for errors
            when(tl.d.bits.denied || tl.d.bits.corrupt) { // Check for errors from slave
                state := sError
            } .otherwise {
                // Write successful for this beat
                val next_spad_addr = current_spad_addr + 1.U // Increment by 1 element address
                val next_bytes_transferred = bytes_transferred_count + beatBytes.U

                current_spad_addr := next_spad_addr
                current_mem_addr := current_mem_addr + beatBytes.U
                bytes_transferred_count := next_bytes_transferred

                when(next_bytes_transferred >= length_bytes_reg) {
                    state := sDone
                } .elsewhen (next_bytes_transferred < length_bytes_reg) {
                    state := sTLWrite_ReadFromBuf // Read next data from SPAD
                } .otherwise {
                     state := sDone // Or error
                }
            }
        }
    }

    is(sDone) {
      // io.start is a level signal. Wait for it to go low before returning to sIdle,
      // or ensure controller only pulses start.
      // If start is kept high, we might immediately re-start.
      // Using dma_started_this_cycle for sIdle transition handles this.
      state := sIdle
    }
    is(sError) {
      state := sIdle // Or a persistent error state needing explicit clear
    }
  }
}