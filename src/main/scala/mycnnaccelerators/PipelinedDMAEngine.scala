// filename: PipelinedDMAEngine.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile.HasCoreParameters

import CNNAcceleratorISA._

class PipelinedDMAEngine(val accConfig: AcceleratorConfig, val numOutstandingReqs: Int = 4)(implicit p: Parameters) extends LazyModule {
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLClientParameters(
    name = "pipelined-dma-engine",
    sourceId = IdRange(0, numOutstandingReqs),
    requestFifo = true
  )))))

  override lazy val module = new PipelinedDMAEngineModuleImp(this)
}

class PipelinedDMAEngineModuleImp(outer: PipelinedDMAEngine)(implicit p: Parameters) extends LazyModuleImp(outer)
  with HasCoreParameters {

  val io = IO(new SimpleDMATLModuleIO(outer.accConfig, coreMaxAddrBits)(p))
  val (tl, edge) = outer.node.out.head

  val enableDebugPrints = false.B // Set to true to see detailed logs

  // --- DMA Parameters ---
  val dataWidthVal = outer.accConfig.dataWidth
  val elementBytesVal = if (dataWidthVal > 0) dataWidthVal / 8 else 1
  val beatDataBits = tl.a.bits.data.getWidth
  val beatBytesVal = beatDataBits / 8
  val elemsPerBeat = if (elementBytesVal > 0) beatBytesVal / elementBytesVal else 1

  // --- Main Control FSM ---
  val sIdle :: sBusy :: sDone :: sError :: Nil = Enum(4)
  val state = RegInit(sIdle)
  val dma_start_pulse = io.start && !RegNext(io.start, false.B)

  val mem_base_addr_reg = Reg(UInt(coreMaxAddrBits.W))
  val length_bytes_reg = Reg(UInt(CNNAcceleratorISA.dmaConfigLenBits.W))
  val direction_reg = Reg(UInt(CNNAcceleratorISA.dmaConfigDirBits.W))

  io.busy := state === sBusy
  io.done := state === sDone
  io.dma_error := state === sError

  // --- Source ID Management ---
  val source_id_busy = RegInit(VecInit(Seq.fill(outer.numOutstandingReqs)(false.B)))
  val next_free_source_id = PriorityEncoder(source_id_busy.map(!_))
  val source_id_available = source_id_busy.contains(false.B)
  val no_reqs_outstanding = !source_id_busy.contains(true.B)

  val isReadTransfer  = state === sBusy && direction_reg === DMADirection.MEM_TO_BUF
  val isWriteTransfer = state === sBusy && direction_reg === DMADirection.BUF_TO_MEM

  // =================================================================
  // == READ PIPELINE (Memory -> Scratchpad)
  // =================================================================
  val read_addr_gen_bytes   = Reg(UInt(CNNAcceleratorISA.dmaConfigLenBits.W))
  val spad_write_addr_gen   = Reg(UInt(32.W))
  val spad_beat_unpack_idx  = Reg(UInt(log2Ceil(elemsPerBeat+1).W))
  val spad_write_beat_reg   = Reg(UInt(beatDataBits.W))
  val spad_is_writing_beat  = RegInit(false.B)
  val read_data_queue       = Module(new Queue(UInt(beatDataBits.W), 4))

  val all_reads_issued = read_addr_gen_bytes >= length_bytes_reg

  // Stage 1: Issue TL Get requests
  val (legal_get, get_op) = edge.Get(next_free_source_id, mem_base_addr_reg + read_addr_gen_bytes, log2Ceil(beatBytesVal).U)

  // Stage 3: Unpack beats from queue and write elements to SPAD
  io.spad_write_en    := spad_is_writing_beat
  io.spad_write_addr  := spad_write_addr_gen
  io.spad_write_data  := (spad_write_beat_reg >> (spad_beat_unpack_idx * dataWidthVal.U)).asSInt

  read_data_queue.io.deq.ready := !spad_is_writing_beat
  when(read_data_queue.io.deq.fire) {
    spad_is_writing_beat := true.B
    spad_write_beat_reg  := read_data_queue.io.deq.bits
    spad_beat_unpack_idx := 0.U
  }

  when(spad_is_writing_beat) {
    spad_write_addr_gen  := spad_write_addr_gen + 1.U
    spad_beat_unpack_idx := spad_beat_unpack_idx + 1.U
    when(spad_beat_unpack_idx === (elemsPerBeat - 1).U) {
      spad_is_writing_beat := false.B
    }
  }

  // Read Completion Check
  val read_finished = all_reads_issued && no_reqs_outstanding && read_data_queue.io.count === 0.U && !spad_is_writing_beat
  
  // =================================================================
  // == WRITE PIPELINE (Scratchpad -> Memory)
  // =================================================================
  val write_addr_gen_bytes  = Reg(UInt(CNNAcceleratorISA.dmaConfigLenBits.W))
  val spad_read_addr_gen    = Reg(UInt(32.W))
  val write_beat_pack_idx   = Reg(UInt(log2Ceil(elemsPerBeat+1).W))
  val spad_read_beat_reg    = Reg(UInt(beatDataBits.W))
  val write_req_queue       = Module(new Queue(new Bundle {
    val addr = UInt(coreMaxAddrBits.W)
    val data = UInt(beatDataBits.W)
  }, 4))

  val all_writes_assembled = (spad_read_addr_gen * elementBytesVal.U) >= length_bytes_reg

  // Stage 1: Read SPAD and Assemble Beats (FSM to handle 1-cycle latency)
  val sWriteIdle :: sWriteReadSPAD :: sWriteAssemble :: Nil = Enum(3)
  val writeAssemblyState = RegInit(sWriteIdle)

  io.spad_read_addr := spad_read_addr_gen
  write_req_queue.io.enq.valid := false.B
  write_req_queue.io.enq.bits  := DontCare

  when(isWriteTransfer) {
    switch(writeAssemblyState) {
      is(sWriteIdle) {
        when(!all_writes_assembled) {
          writeAssemblyState := sWriteReadSPAD
        } .elsewhen(write_beat_pack_idx =/= 0.U && write_req_queue.io.enq.ready) { // Flush last partial beat
          write_req_queue.io.enq.valid := true.B
          write_req_queue.io.enq.bits.addr := mem_base_addr_reg + write_addr_gen_bytes
          write_req_queue.io.enq.bits.data := spad_read_beat_reg
          write_beat_pack_idx := 0.U // Mark flush as complete
        }
      }
      is(sWriteReadSPAD) {
        writeAssemblyState := sWriteAssemble
      }
      is(sWriteAssemble) {
        val element = io.spad_read_data.asUInt
        val new_beat = spad_read_beat_reg | (element << (write_beat_pack_idx * dataWidthVal.U))
        val beat_is_full = write_beat_pack_idx === (elemsPerBeat - 1).U
        
        spad_read_addr_gen := spad_read_addr_gen + 1.U
        spad_read_beat_reg := Mux(beat_is_full, 0.U, new_beat)
        write_beat_pack_idx := Mux(beat_is_full, 0.U, write_beat_pack_idx + 1.U)

        when(beat_is_full) {
          when(write_req_queue.io.enq.ready) {
            write_req_queue.io.enq.valid := true.B
            write_req_queue.io.enq.bits.addr := mem_base_addr_reg + write_addr_gen_bytes
            write_req_queue.io.enq.bits.data := new_beat
            write_addr_gen_bytes := write_addr_gen_bytes + beatBytesVal.U
            writeAssemblyState := sWriteIdle
          }
        } .otherwise {
          writeAssemblyState := sWriteIdle
        }
      }
    }
  }

  // Stage 2: Issue TL Put requests from queue
  val (legal_put, put_op) = edge.Put(next_free_source_id, write_req_queue.io.deq.bits.addr, log2Ceil(beatBytesVal).U, write_req_queue.io.deq.bits.data)
  write_req_queue.io.deq.ready := isWriteTransfer && source_id_available && tl.a.ready && legal_put

  // Write Completion Check
  val write_finished = all_writes_assembled && (write_beat_pack_idx === 0.U) && write_req_queue.io.count === 0.U && no_reqs_outstanding

  // =================================================================
  // == Main State & Shared TL Logic
  // =================================================================

  // --- Main FSM Transitions ---
  when(dma_start_pulse) {
    mem_base_addr_reg := io.mem_base_addr
    length_bytes_reg  := io.length_bytes
    direction_reg     := io.directionBits
    state             := Mux(io.length_bytes === 0.U, sDone, sBusy)
    // Reset all pipeline states and counters
    read_addr_gen_bytes   := 0.U
    spad_write_addr_gen   := 0.U
    spad_is_writing_beat  := false.B
    write_addr_gen_bytes  := 0.U
    spad_read_addr_gen    := 0.U
    writeAssemblyState    := sWriteIdle
    write_beat_pack_idx   := 0.U
    spad_read_beat_reg    := 0.U
  }

  when((isReadTransfer && read_finished) || (isWriteTransfer && write_finished)) {
    state := sDone
  }
  when(state === sDone) {
    state := sIdle
  }
  
  // --- TileLink A-Channel (Requests) ---
  when(isReadTransfer) {
    tl.a.valid := !all_reads_issued && source_id_available && legal_get
    tl.a.bits  := get_op
    when(!all_reads_issued && !legal_get) { state := sError }
  } .elsewhen(isWriteTransfer) {
    tl.a.valid := write_req_queue.io.deq.valid && source_id_available && legal_put
    tl.a.bits  := put_op
    when(write_req_queue.io.deq.valid && !legal_put) { state := sError }
  } .otherwise {
    tl.a.valid := false.B
    tl.a.bits  := DontCare
  }

  when(tl.a.fire) {
    source_id_busy(tl.a.bits.source) := true.B
    when(isReadTransfer) {
      read_addr_gen_bytes := read_addr_gen_bytes + beatBytesVal.U
    }
  }

  // --- TileLink D-Channel (Responses) ---
  // BUG FIX: Must be ready for ACKs during writes, and for data during reads (if queue has space)
  tl.d.ready := (isReadTransfer && read_data_queue.io.enq.ready) || isWriteTransfer
  
  read_data_queue.io.enq.valid := isReadTransfer && tl.d.valid && tl.d.bits.opcode === TLMessages.AccessAckData
  read_data_queue.io.enq.bits  := tl.d.bits.data
  
  when(tl.d.fire) {
    source_id_busy(tl.d.bits.source) := false.B
    when(tl.d.bits.denied || tl.d.bits.corrupt) {
      state := sError
    }
  }

  // --- Debug Prints ---
  if(enableDebugPrints.litToBoolean) {
    when(dma_start_pulse) {
        printf(p"DMA_START: New transfer. Addr=0x${Hexadecimal(io.mem_base_addr)}, Len=${io.length_bytes}, Dir=${io.directionBits}\n")
    }
    when(state =/= RegNext(state)) {
        printf(p"DMA_STATE: ${RegNext(state)} -> ${state}\n")
    }
    when(isReadTransfer) {
      when(!all_reads_issued && source_id_available && !legal_get) { printf("DMA_ERROR: Illegal GET address\n")}
      when(read_finished) { printf(p"DMA_INFO: Read transfer finished.\n") }
    }
    when(isWriteTransfer) {
        printf(p"DMA_WRITE_TICK: state=${writeAssemblyState}, spad_addr=${spad_read_addr_gen}, beat_idx=${write_beat_pack_idx}, all_read=${all_writes_assembled}, q_count=${write_req_queue.io.count}\n")
        when(write_finished) { printf(p"DMA_INFO: Write transfer finished.\n") }
    }
    when(tl.a.fire) {
        printf(p"DMA_TL_A_FIRE: isRead=${isReadTransfer}, Addr=0x${Hexadecimal(tl.a.bits.address)}, SrcID=${tl.a.bits.source}\n")
    }
    when(tl.d.fire) {
        printf(p"DMA_TL_D_FIRE: isRead=${isReadTransfer}, SrcID=${tl.d.bits.source}, Op=${tl.d.bits.opcode}, Denied=${tl.d.bits.denied}\n")
    }
  }
}