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

  val enableDmaDebugPrints = false.B // Keep this true for focused debugging

  val dmaCycleCount = RegInit(0.U(32.W))
  if (enableDmaDebugPrints.litToBoolean) {
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
  val current_spad_elem_addr_reg = Reg(UInt(32.W)) // Kept 32.W for generic SPAD addressing

  // Derived parameters
  val dataWidthVal = outer.accConfig.dataWidth // Scala Int
  val elementBytesVal = outer.accConfig.dataWidth / 8 // Scala Int

  val beatDataBits = tl.a.bits.data.getWidth // Chisel Int (constant at elaboration)
  val beatBytesVal = beatDataBits / 8       // Scala Int (derived from Chisel Int constant)

  val elemsPerBeatLit: Int = if (elementBytesVal > 0) {
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
  tl.d.ready := false.B // Default to false, enable only when waiting for D channel response
  tl.e.valid := false.B
  tl.e.bits := DontCare

  // General DMA state print (rate-limited) - MOVED TO THE END OF THE MODULE
  // Specific, highly relevant prints will be placed inside state logic.

  switch(state) {
    is(sIdle) {
      when(dma_started_this_cycle) {
        if(enableDmaDebugPrints.litToBoolean){printf(p"DMA_CTRL Cycle[${dmaCycleCount}]: sIdle: DMA Start asserted. Len=${io.length_bytes}, Dir=${io.directionBits}, MemAddr=0x${Hexadecimal(io.mem_base_addr)}, BufID=${io.buffer_id}\n")}
        mem_base_addr_reg := io.mem_base_addr
        length_bytes_reg := io.length_bytes
        direction_reg := io.directionBits // Capture direction from RoCC
        current_mem_addr_reg := io.mem_base_addr
        current_spad_elem_addr_reg := 0.U
        total_useful_bytes_transferred_reg := 0.U
        beat_elem_idx_reg := 0.U
        mem_data_beat_reg := 0.U
        spad_data_to_write_beat_reg := 0.U

        when(io.length_bytes === 0.U || elementBytesVal.U === 0.U) {
          if(enableDmaDebugPrints.litToBoolean){printf(p"DMA_CTRL Cycle[${dmaCycleCount}]: sIdle: Zero length or zero element_bytes. To sDone.\n")}
          state := sDone
        } .elsewhen(io.directionBits === DMADirection.MEM_TO_BUF) { // Use io.directionBits for initial decision
          if(enableDmaDebugPrints.litToBoolean){printf(p"DMA_CTRL Cycle[${dmaCycleCount}]: sIdle: Dir MEM_TO_BUF. To sTLRead_ReqAddr.\n")}
          state := sTLRead_ReqAddr
        } .elsewhen(io.directionBits === DMADirection.BUF_TO_MEM) { // Use io.directionBits for initial decision
          if(enableDmaDebugPrints.litToBoolean){printf(p"DMA_CTRL Cycle[${dmaCycleCount}]: sIdle: Dir BUF_TO_MEM. To sTLWrite_SetSpadAddr.\n")}
          state := sTLWrite_SetSpadAddr
        } .otherwise {
          if(enableDmaDebugPrints.litToBoolean){printf(p"DMA_CTRL Cycle[${dmaCycleCount}]: sIdle: Invalid DMA direction. To sError.\n")}
          state := sError
        }
      }
    }

    is(sTLRead_ReqAddr) {
      // ... (keep existing logic and prints if desired, or make them more conditional)
      when(total_useful_bytes_transferred_reg >= length_bytes_reg) {
         state := sDone
      }.otherwise {
        if(enableDmaDebugPrints.litToBoolean){printf(p"DMA_MEM_OP Cycle[${dmaCycleCount}]: sTLRead_ReqAddr: Requesting mem read at 0x${Hexadecimal(current_mem_addr_reg)}\n")}
        val (legal, get_op) = edge.Get(
          fromSource = 0.U,
          toAddress = current_mem_addr_reg,
          lgSize = log2Ceil(beatBytesVal).U
        )
        tl.a.valid := legal
        tl.a.bits := get_op
        when(!legal) {
          if(enableDmaDebugPrints.litToBoolean){printf(p"DMA_ERROR Cycle[${dmaCycleCount}]: sTLRead_ReqAddr: Illegal address 0x${Hexadecimal(current_mem_addr_reg)}. To sError.\n")}
          state := sError
        }
        when(tl.a.fire) {
          state := sTLRead_WaitData
        }
      }
    }

    is(sTLRead_WaitData) {
      tl.d.ready := true.B // Always ready to accept data when in this state
      when(tl.d.fire) {
        if(enableDmaDebugPrints.litToBoolean){printf(p"DMA_MEM_OP Cycle[${dmaCycleCount}]: sTLRead_WaitData: TL.D fired. Opcode=${tl.d.bits.opcode} Denied=${tl.d.bits.denied} Data=0x${Hexadecimal(tl.d.bits.data)}\n")}
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
      // ... (keep existing logic and prints, or make more conditional)
      when(total_useful_bytes_transferred_reg >= length_bytes_reg) {
        state := sDone
      } .elsewhen(beat_elem_idx_reg < elemsPerBeatValU) {
        io.spad_write_en := true.B
        io.spad_write_addr := current_spad_elem_addr_reg
        val shiftAmount = beat_elem_idx_reg * dataWidthVal.U
        val extracted_element = (mem_data_beat_reg >> shiftAmount)(dataWidthVal - 1, 0)
        io.spad_write_data := extracted_element.asSInt
        if(enableDmaDebugPrints.litToBoolean){printf(p"DMA_SPAD_OP Cycle[${dmaCycleCount}]: sTLRead_WriteToBuf: Writing elem ${beat_elem_idx_reg} to SPAD_addr ${current_spad_elem_addr_reg}, data 0x${Hexadecimal(extracted_element)}\n")}
        current_spad_elem_addr_reg := current_spad_elem_addr_reg + 1.U
        total_useful_bytes_transferred_reg := total_useful_bytes_transferred_reg + elementBytesVal.U
        beat_elem_idx_reg := beat_elem_idx_reg + 1.U
        // state := sTLRead_WriteToBuf // Stays in this state by default unless changed by outer when
      } .otherwise {
        when (total_useful_bytes_transferred_reg >= length_bytes_reg) {
           state := sDone
        } .otherwise {
           state := sTLRead_ReqAddr
        }
      }
    }

    is(sTLWrite_SetSpadAddr) {
      // ... (keep existing logic and prints)
      when(total_useful_bytes_transferred_reg >= length_bytes_reg && beat_elem_idx_reg === 0.U) {
        state := sDone
      } .elsewhen(beat_elem_idx_reg < elemsPerBeatValU && total_useful_bytes_transferred_reg < length_bytes_reg) {
        io.spad_read_addr := current_spad_elem_addr_reg
        // No change to general printf here, keep if useful
        state := sTLWrite_LatchSpadData
      } .otherwise {
        state := sTLWrite_ReqAddr
      }
    }

    is(sTLWrite_LatchSpadData) {
      val elementFromSpad = io.spad_read_data.asUInt
      val currentShiftAmount = beat_elem_idx_reg * dataWidthVal.U // Calculate before beat_elem_idx_reg increments
      val updated_spad_data_to_write_beat_reg = spad_data_to_write_beat_reg | (elementFromSpad << currentShiftAmount)
      spad_data_to_write_beat_reg := updated_spad_data_to_write_beat_reg

      if (enableDmaDebugPrints.litToBoolean) {
        // *** This is a highly relevant log for OFM store debugging ***
        // Only print for the first beat being assembled during an OFM (BUF_TO_MEM) transfer
        val is_assembling_first_ofm_beat = (direction_reg === DMADirection.BUF_TO_MEM) &&
                                           (current_mem_addr_reg === mem_base_addr_reg) // Address of current beat being assembled is still base for the first one

        when(is_assembling_first_ofm_beat && beat_elem_idx_reg < elemsPerBeatValU) { // Check beat_elem_idx_reg before it increments
          printf(p"DMA_BEAT_ASSM Cycle[${dmaCycleCount}]: sTLWrite_LatchSpadData (OFM First Beat): ElemAddrInSpad=${current_spad_elem_addr_reg}, IdxInBeat=${beat_elem_idx_reg}, ElemFromSpad=0x${Hexadecimal(elementFromSpad)}, BeatRegAfter=0x${Hexadecimal(updated_spad_data_to_write_beat_reg)}\n")
        }
      }

      current_spad_elem_addr_reg := current_spad_elem_addr_reg + 1.U
      total_useful_bytes_transferred_reg := total_useful_bytes_transferred_reg + elementBytesVal.U
      beat_elem_idx_reg := beat_elem_idx_reg + 1.U
      state := sTLWrite_SetSpadAddr
    }

    is(sTLWrite_ReqAddr) {
      // *** This is the MOST relevant log for OFM store debugging ***
      // Print the data beat *before* it's potentially sent and *before* spad_data_to_write_beat_reg is cleared
      if (enableDmaDebugPrints.litToBoolean) {
        val is_first_ofm_beat_to_send = (direction_reg === DMADirection.BUF_TO_MEM) && (current_mem_addr_reg === mem_base_addr_reg)
        when(is_first_ofm_beat_to_send) {
          printf(p"DMA_PUT_PREP Cycle[${dmaCycleCount}]: sTLWrite_ReqAddr: Preparing FIRST OFM PUT to Addr=0x${Hexadecimal(current_mem_addr_reg)}, Data Beat=0x${Hexadecimal(spad_data_to_write_beat_reg)}, BeatElemIdxState=${beat_elem_idx_reg}\n")
        }
      }

      val (legal, put_op) = edge.Put(
        fromSource = 0.U,
        toAddress = current_mem_addr_reg,
        lgSize = log2Ceil(beatBytesVal).U,
        data = spad_data_to_write_beat_reg
      )
      tl.a.valid := legal
      tl.a.bits := put_op

      when(!legal) {
        if(enableDmaDebugPrints.litToBoolean){printf(p"DMA_ERROR Cycle[${dmaCycleCount}]: sTLWrite_ReqAddr: Illegal address 0x${Hexadecimal(current_mem_addr_reg)}. To sError.\n")}
        state := sError
      }
      when(tl.a.fire) {
        // This printf shows the data that was effectively on tl.a.bits.data
        // spad_data_to_write_beat_reg will be cleared in this same block AFTER this printf (if it were to use current value)
        // So, we should capture the values if we want to print what was truly sent accurately *after* fire.
        // The "DMA_PUT_PREP" above is more reliable for seeing the data intended for the Put.
        // However, the user's existing conditional print was already inside when(tl.a.fire).
        // Let's refine it to use RegNext to be sure about what was sent for the first OFM beat.
        if (enableDmaDebugPrints.litToBoolean) {
            val addr_that_fired = RegNext(current_mem_addr_reg)
            val data_that_fired = RegNext(spad_data_to_write_beat_reg) // Capture data before it's cleared
            val dir_that_fired  = RegNext(direction_reg)

            when(dir_that_fired === DMADirection.BUF_TO_MEM && addr_that_fired === mem_base_addr_reg) {
              printf(p"DMA_PUT_FIRE Cycle[${dmaCycleCount}]: sTLWrite_ReqAddr: FIRST OFM PUT Fired for Addr=0x${Hexadecimal(addr_that_fired)}, Data Beat Sent=0x${Hexadecimal(data_that_fired)}\n")
            }
        }
        current_mem_addr_reg := current_mem_addr_reg + beatBytesVal.U
        spad_data_to_write_beat_reg := 0.U // Clear beat register for next beat
        beat_elem_idx_reg := 0.U           // Reset beat element index for next beat
        state := sTLWrite_WaitAck
      }
    }

    is(sTLWrite_WaitAck) {
      tl.d.ready := true.B // Always ready for ack
      when(tl.d.fire) {
        if(enableDmaDebugPrints.litToBoolean){printf(p"DMA_MEM_OP Cycle[${dmaCycleCount}]: sTLWrite_WaitAck: TL.D fired. Opcode=${tl.d.bits.opcode} Denied=${tl.d.bits.denied}\n")}
        when(tl.d.bits.denied || tl.d.bits.corrupt || tl.d.bits.opcode =/= TLMessages.AccessAck) {
          state := sError
        } .otherwise {
          when(total_useful_bytes_transferred_reg >= length_bytes_reg) {
            state := sDone
          } .otherwise {
            state := sTLWrite_SetSpadAddr
          }
        }
      }
    }

    is(sDone) {
      if(enableDmaDebugPrints.litToBoolean){printf(p"DMA_CTRL Cycle[${dmaCycleCount}]: sDone: DMA Complete. total_useful_bytes_transferred_reg=${total_useful_bytes_transferred_reg}. To sIdle.\n")}
      state := sIdle
    }
    is(sError) {
      if(enableDmaDebugPrints.litToBoolean){printf(p"DMA_ERROR Cycle[${dmaCycleCount}]: sError: DMA Error. To sIdle.\n")}
      state := sIdle
    }
  }

  // General DMA state print (rate-limited)
  if(enableDmaDebugPrints.litToBoolean){
    val print_counter = RegInit(0.U(10.W)) // For further rate limiting if needed, e.g. 10.W for every 1024 cycles
    val dma_state_prev_for_print = RegNext(state, sIdle) // Detect state change

    when(state =/= dma_state_prev_for_print || print_counter === 0.U) {
      printf(p"DMA_GENERAL_STATE Cycle[${dmaCycleCount}]: State=${state} useful_bytes_transferred=${total_useful_bytes_transferred_reg} len_bytes=${length_bytes_reg} mem_addr=0x${Hexadecimal(current_mem_addr_reg)} spad_addr=${current_spad_elem_addr_reg} beat_elem_idx=${beat_elem_idx_reg}\n")
    }
    print_counter := Mux(print_counter === ((1<<10)-1).U, 0.U, print_counter + 1.U) // Make counter wrap for rate limiting

    // Optional: Less verbose general info, print only once or on config change
    // when(dma_started_this_cycle) { // Or some other less frequent trigger
    //   printf(p"DMA_STATIC_INFO: beatBytes=${beatBytesVal.U}, elementBytes=${elementBytesVal.U}, elemsPerBeat=${elemsPerBeatValU}\n")
    // }
  }
}