// filename: SimpleDMAEngine.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile.HasCoreParameters // For coreMaxAddrBits, xLen etc.
// It's good practice to ensure TileKey is available if p(TileKey) is used,
// but here we pass coreMaxAddrBitsParam explicitly to the IO bundle.
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

  val io = IO(new SimpleDMATLModuleIO(outer.accConfig, coreMaxAddrBits)(p))
  val (tl, edge) = outer.node.out.head

  // ***** MODIFICATION START *****
  val enableDmaDebugPrints = true.B // Ensure this is true.B
  // ***** MODIFICATION END *****

  val dmaCycleCount = RegInit(0.U(32.W))
  if (enableDmaDebugPrints.litToBoolean) { // This condition will now be true
    dmaCycleCount := dmaCycleCount + 1.U
  }

  // Define FSM States
  val sIdle :: sTLRead_ReqAddr :: sTLRead_WaitData :: sTLRead_WriteToBuf :: sTLWrite_SetSpadAddr :: sTLWrite_LatchSpadData :: sTLWrite_ReqAddr :: sTLWrite_WaitAck :: sDone :: sError :: Nil = Enum(10)
  val state = RegInit(sIdle)

  // Configuration registers
  val mem_base_addr_reg = Reg(UInt(coreMaxAddrBits.W))
  val length_bytes_reg = Reg(UInt(CNNAcceleratorISA.dmaConfigLenBits.W))
  val direction_reg = Reg(UInt(CNNAcceleratorISA.dmaConfigDirBits.W))

  // Operation tracking registers
  val total_useful_bytes_transferred_reg = RegInit(0.U(CNNAcceleratorISA.dmaConfigLenBits.W))
  val current_mem_addr_reg = Reg(UInt(coreMaxAddrBits.W))
  val current_spad_elem_addr_reg = Reg(UInt(32.W))

  // Derived parameters
  val dataWidthVal = outer.accConfig.dataWidth // Scala Int
  val elementBytesVal = outer.accConfig.dataWidth / 8 // Scala Int

  val beatDataBits = tl.a.bits.data.getWidth // Chisel Int (constant at elaboration)
  val beatBytesVal = beatDataBits / 8         // Scala Int (derived from Chisel Int constant)

  val elemsPerBeatLit: Int = if (elementBytesVal > 0) {
    // CORRECTED: Directly use beatBytesVal as it's already a Scala Int
    require(beatBytesVal >= elementBytesVal, s"DMA beatBytes (${beatBytesVal}) must be >= element_bytes ($elementBytesVal)")
    require(beatBytesVal % elementBytesVal == 0, "DMA beatBytes must be a multiple of element_bytes for this simplified DMA")
    beatBytesVal / elementBytesVal
  } else {
    1
  }
  val elemsPerBeatValU = elemsPerBeatLit.U((log2Ceil(elemsPerBeatLit + 1)).W)


  val beat_elem_idx_reg = RegInit(0.U(log2Ceil(elemsPerBeatLit + 1).W))

  // Data holding registers
  val mem_data_beat_reg = Reg(UInt(beatDataBits.W))
  val spad_data_to_write_beat_reg = RegInit(0.U(beatDataBits.W))

  val dma_start_reg = RegNext(io.start, false.B)
  val dma_started_this_cycle = io.start && !dma_start_reg

  // Default outputs
  io.busy := (state =/= sIdle) && (state =/= sDone) && (state =/= sError)
  io.done := (state === sDone)
  io.dma_error := (state === sError)

  io.spad_write_en := false.B
  io.spad_write_addr := 0.U
  io.spad_write_data := 0.S
  io.spad_read_addr := 0.U

  tl.a.valid := false.B
  tl.a.bits := DontCare
  tl.b.ready := true.B
  tl.c.valid := false.B
  tl.c.bits := DontCare
  tl.d.ready := false.B
  tl.e.valid := false.B
  tl.e.bits := DontCare

  if(enableDmaDebugPrints.litToBoolean){
      when(RegNext(state) =/= state) {
        printf(p"RoCC DMA Cycle[${dmaCycleCount}]: State Change ${RegNext(state)} -> ${state}\n")
      }
      printf(p"RoCC DMA Cycle[${dmaCycleCount}]: State=${state} useful_bytes_transferred=${total_useful_bytes_transferred_reg} len_bytes=${length_bytes_reg} mem_addr=0x${Hexadecimal(current_mem_addr_reg)} spad_addr=${current_spad_elem_addr_reg} beat_elem_idx=${beat_elem_idx_reg}\n")
      printf(p"RoCC DMA Info: beatBytes=${beatBytesVal.U}, elementBytes=${elementBytesVal.U}, elemsPerBeat=${elemsPerBeatValU}\n")
  }


  switch(state) {
    is(sIdle) {
      when(dma_started_this_cycle) {
        if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sIdle: DMA Start. Len=${io.length_bytes}, Dir=${io.directionBits}, MemAddr=0x${Hexadecimal(io.mem_base_addr)}\n")}
        mem_base_addr_reg := io.mem_base_addr
        length_bytes_reg := io.length_bytes
        direction_reg := io.directionBits
        current_mem_addr_reg := io.mem_base_addr
        current_spad_elem_addr_reg := 0.U
        total_useful_bytes_transferred_reg := 0.U
        beat_elem_idx_reg := 0.U
        mem_data_beat_reg := 0.U
        spad_data_to_write_beat_reg := 0.U

        when(io.length_bytes === 0.U || elementBytesVal.U === 0.U) {
          if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sIdle: Zero length or zero element_bytes. To sDone.\n")}
          state := sDone
        } .elsewhen(io.directionBits === DMADirection.MEM_TO_BUF) {
          if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sIdle: Dir MEM_TO_BUF. To sTLRead_ReqAddr.\n")}
          state := sTLRead_ReqAddr
        } .elsewhen(io.directionBits === DMADirection.BUF_TO_MEM) {
          if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sIdle: Dir BUF_TO_MEM. To sTLWrite_SetSpadAddr.\n")}
          state := sTLWrite_SetSpadAddr
        } .otherwise {
          if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sIdle: Invalid DMA direction. To sError.\n")}
          state := sError
        }
      }
    }

    is(sTLRead_ReqAddr) {
      when(current_mem_addr_reg - mem_base_addr_reg >= length_bytes_reg && length_bytes_reg > 0.U && beat_elem_idx_reg === 0.U) {
          if (enableDmaDebugPrints.litToBoolean) {printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sTLRead_ReqAddr: All memory for useful bytes covered based on mem_addr_reg. To sDone.\n")}
          state := sDone
      } .elsewhen(total_useful_bytes_transferred_reg >= length_bytes_reg) { // Primary check for completion
          state := sDone
      }.otherwise {
        if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sTLRead_ReqAddr: Requesting mem read at 0x${Hexadecimal(current_mem_addr_reg)}\n")}
        val (legal, get_op) = edge.Get(
          fromSource = 0.U,
          toAddress = current_mem_addr_reg,
          lgSize = log2Ceil(beatBytesVal).U
        )
        tl.a.valid := legal
        tl.a.bits := get_op
        when(!legal) {
            if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sTLRead_ReqAddr: Illegal address 0x${Hexadecimal(current_mem_addr_reg)}. To sError.\n")}
            state := sError
        }
        when(tl.a.fire) {
          state := sTLRead_WaitData
        }
      }
    }

    is(sTLRead_WaitData) {
      tl.d.ready := true.B
      when(tl.d.fire) {
        if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sTLRead_WaitData: TL.D fired. Opcode=${tl.d.bits.opcode} Denied=${tl.d.bits.denied} Data=0x${Hexadecimal(tl.d.bits.data)}\n")}
        when(tl.d.bits.denied || tl.d.bits.corrupt || tl.d.bits.opcode =/= TLMessages.AccessAckData) {
          state := sError
        } .otherwise {
          mem_data_beat_reg := tl.d.bits.data
          beat_elem_idx_reg := 0.U
          current_mem_addr_reg := current_mem_addr_reg + beatBytesVal.U
          state := sTLRead_WriteToBuf
        }
      }
    }

    is(sTLRead_WriteToBuf) {
      when(total_useful_bytes_transferred_reg >= length_bytes_reg) {
        state := sDone
      } .elsewhen(beat_elem_idx_reg < elemsPerBeatValU) {
        io.spad_write_en := true.B
        io.spad_write_addr := current_spad_elem_addr_reg

        val shiftAmount = beat_elem_idx_reg * dataWidthVal.U
        val extracted_element = (mem_data_beat_reg >> shiftAmount)(dataWidthVal - 1, 0)
        io.spad_write_data := extracted_element.asSInt

        if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sTLRead_WriteToBuf: Writing elem ${beat_elem_idx_reg} to SPAD_addr ${current_spad_elem_addr_reg}, data 0x${Hexadecimal(extracted_element)}\n")}

        current_spad_elem_addr_reg := current_spad_elem_addr_reg + 1.U
        total_useful_bytes_transferred_reg := total_useful_bytes_transferred_reg + elementBytesVal.U
        beat_elem_idx_reg := beat_elem_idx_reg + 1.U
        state := sTLRead_WriteToBuf
      } .otherwise { // beat_elem_idx_reg >= elemsPerBeatValU (all elements from current beat written)
        // CORRECTED: Check total_useful_bytes_transferred_reg with 'when'
        when (total_useful_bytes_transferred_reg >= length_bytes_reg) {
            state := sDone
        } .otherwise {
            if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sTLRead_WriteToBuf: Current beat processed. To sTLRead_ReqAddr for next beat.\n")}
            state := sTLRead_ReqAddr
        }
      }
    }

    is(sTLWrite_SetSpadAddr) {
      when(total_useful_bytes_transferred_reg >= length_bytes_reg && beat_elem_idx_reg === 0.U) { // If all done AND current beat is empty
        state := sDone
      } .elsewhen(beat_elem_idx_reg < elemsPerBeatValU && total_useful_bytes_transferred_reg < length_bytes_reg) { // If beat not full AND more data needed
        io.spad_read_addr := current_spad_elem_addr_reg
        if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sTLWrite_SetSpadAddr: Reading SPAD_addr ${current_spad_elem_addr_reg} for beat_elem ${beat_elem_idx_reg}\n")}
        state := sTLWrite_LatchSpadData
      } .otherwise { // Beat is full OR (all useful data collected AND beat_elem_idx_reg is not 0, meaning partial beat data is ready to be sent)
        if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sTLWrite_SetSpadAddr: Beat full or final partial beat. To sTLWrite_ReqAddr.\n")}
        state := sTLWrite_ReqAddr
      }
    }

    is(sTLWrite_LatchSpadData) {
      val elementFromSpad = io.spad_read_data.asUInt
      val shiftAmount = beat_elem_idx_reg * dataWidthVal.U
      spad_data_to_write_beat_reg := spad_data_to_write_beat_reg | (elementFromSpad << shiftAmount)

      if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sTLWrite_LatchSpadData: Latched SPAD_elem ${beat_elem_idx_reg} data 0x${Hexadecimal(elementFromSpad)}. BeatReg now 0x${Hexadecimal(spad_data_to_write_beat_reg)}\n")}

      current_spad_elem_addr_reg := current_spad_elem_addr_reg + 1.U
      total_useful_bytes_transferred_reg := total_useful_bytes_transferred_reg + elementBytesVal.U
      beat_elem_idx_reg := beat_elem_idx_reg + 1.U

      state := sTLWrite_SetSpadAddr
    }

    is(sTLWrite_ReqAddr) {
      if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sTLWrite_ReqAddr: Writing beat to mem 0x${Hexadecimal(current_mem_addr_reg)}, data 0x${Hexadecimal(spad_data_to_write_beat_reg)}\n")}
      val (legal, put_op) = edge.Put(
        fromSource = 0.U,
        toAddress = current_mem_addr_reg,
        lgSize = log2Ceil(beatBytesVal).U,
        data = spad_data_to_write_beat_reg
      )
      tl.a.valid := legal
      tl.a.bits := put_op
      when(!legal) {
          if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sTLWrite_ReqAddr: Illegal address 0x${Hexadecimal(current_mem_addr_reg)}. To sError.\n")}
          state := sError
      }
      when(tl.a.fire) {
        current_mem_addr_reg := current_mem_addr_reg + beatBytesVal.U
        spad_data_to_write_beat_reg := 0.U // Clear beat register for next beat
        beat_elem_idx_reg := 0.U           // Reset beat element index for next beat
        state := sTLWrite_WaitAck
      }
    }

    is(sTLWrite_WaitAck) {
      tl.d.ready := true.B
      when(tl.d.fire) {
        if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sTLWrite_WaitAck: TL.D fired. Opcode=${tl.d.bits.opcode} Denied=${tl.d.bits.denied}\n")}
        when(tl.d.bits.denied || tl.d.bits.corrupt || tl.d.bits.opcode =/= TLMessages.AccessAck) {
          state := sError
        } .otherwise {
          when(total_useful_bytes_transferred_reg >= length_bytes_reg) {
            state := sDone
          } .otherwise {
            state := sTLWrite_SetSpadAddr // Go back to fill next beat from SPAD
          }
        }
      }
    }

    is(sDone) {
      if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sDone: DMA Complete. total_useful_bytes_transferred_reg=${total_useful_bytes_transferred_reg}. To sIdle.\n")}
      state := sIdle
    }
    is(sError) {
      if(enableDmaDebugPrints.litToBoolean){printf(p"RoCC DMA Cycle[${dmaCycleCount}]: sError: DMA Error. To sIdle.\n")}
      state := sIdle
    }
  }
}