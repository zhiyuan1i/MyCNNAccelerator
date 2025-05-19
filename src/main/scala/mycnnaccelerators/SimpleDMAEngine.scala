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

  // Define FSM States
  // sTLWrite_ReadFromBuf is replaced by sTLWrite_SetSpadAddr and sTLWrite_LatchSpadData
  val sIdle :: sTLRead_ReqAddr :: sTLRead_WaitData :: sTLRead_WriteToBuf :: sTLWrite_SetSpadAddr :: sTLWrite_LatchSpadData :: sTLWrite_ReqAddr :: sTLWrite_WaitAck :: sDone :: sError :: Nil = Enum(10)
  val state = RegInit(sIdle)

  val mem_base_addr_reg = Reg(UInt(coreMaxAddrBits.W))
  val length_bytes_reg = Reg(UInt(CNNAcceleratorISA.dmaConfigLenBits.W))
  val direction_reg = Reg(UInt(CNNAcceleratorISA.dmaConfigDirBits.W))
  // val buffer_id_reg = Reg(UInt(CNNAcceleratorISA.dmaConfigBufIdBits.W)) // DMA uses io.buffer_id directly

  val bytes_transferred_count = RegInit(0.U(CNNAcceleratorISA.dmaConfigLenBits.W))
  val current_mem_addr = Reg(UInt(coreMaxAddrBits.W))
  val current_spad_addr = Reg(UInt(32.W)) // Assuming spad addresses are local to each buffer concept

  val beatBytes = tl.a.bits.data.getWidth / 8
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

  val mem_data_beat_reg = Reg(UInt((beatBytes * 8).W))
  val spad_data_to_write_beat_reg = Reg(UInt((beatBytes*8).W)) // For BUF_TO_MEM

  val dma_start_reg = RegNext(io.start, false.B) // Detect rising edge of start
  val dma_started_this_cycle = io.start && !dma_start_reg


  switch(state) {
    is(sIdle) {
      when(dma_started_this_cycle) {
        mem_base_addr_reg := io.mem_base_addr
        length_bytes_reg := io.length_bytes
        direction_reg := io.directionBits
        current_mem_addr := io.mem_base_addr
        current_spad_addr := 0.U
        bytes_transferred_count := 0.U
        mem_data_beat_reg := 0.U

        when(io.length_bytes === 0.U) {
          state := sDone
        } .elsewhen(io.directionBits === DMADirection.MEM_TO_BUF) {
          state := sTLRead_ReqAddr
        } .elsewhen(io.directionBits === DMADirection.BUF_TO_MEM) {
          state := sTLWrite_SetSpadAddr // Start SPAD read sequence
        } .otherwise {
          state := sError
        }
      }
    }

    is(sTLRead_ReqAddr) {
      val (legal, get) = edge.Get(
        fromSource = 0.U,
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
      // This state writes data from mem_data_beat_reg to the SPAD.
      // Current simplification: one write per cycle, data width handled by RoCC or matches.
      io.spad_write_en := true.B
      io.spad_write_addr := current_spad_addr
      // Assuming dataWidth matches beat, or RoCC handles sub-beat packing/unpacking from this.
      io.spad_write_data := mem_data_beat_reg(outer.accConfig.dataWidth - 1, 0).asSInt

      val next_spad_addr = current_spad_addr + 1.U
      val next_bytes_transferred = bytes_transferred_count + beatBytes.U

      current_spad_addr := next_spad_addr
      current_mem_addr := current_mem_addr + beatBytes.U
      bytes_transferred_count := next_bytes_transferred

      when(next_bytes_transferred >= length_bytes_reg) {
        state := sDone
      } .elsewhen (next_bytes_transferred < length_bytes_reg) {
        state := sTLRead_ReqAddr
      } .otherwise {
        state := sDone
      }
    }

    is(sTLWrite_SetSpadAddr) {
      // Assert SPAD read address. Data will be available on io.spad_read_data next cycle.
      io.spad_read_addr := current_spad_addr
      state := sTLWrite_LatchSpadData
    }

    is(sTLWrite_LatchSpadData) {
      // Latch data from SPAD. io.spad_read_data is now valid.
      // This assumes MinimalBuffer's RegNext behavior.
      spad_data_to_write_beat_reg := io.spad_read_data.asUInt
      // Packing logic might be needed here if config.dataWidth < beatBytes*8.
      // Current simplification: one element from spad_read_data is used for one beat.
      state := sTLWrite_ReqAddr
    }

    is(sTLWrite_ReqAddr) {
      val (legal, put) = edge.Put(
          fromSource = 0.U,
          toAddress = current_mem_addr,
          lgSize = log2Ceil(beatBytes).U,
          data = spad_data_to_write_beat_reg // Use latched data
      )
      when(legal) {
          tl.a.valid := true.B
          tl.a.bits := put
          when(tl.a.ready) {
              state := sTLWrite_WaitAck
          }
      } .otherwise {
          state := sError
      }
    }

    is(sTLWrite_WaitAck) {
      tl.d.ready := true.B
      when(tl.d.fire) {
        when(tl.d.bits.denied || tl.d.bits.corrupt) {
          state := sError
        } .otherwise {
          val next_spad_addr = current_spad_addr + 1.U
          val next_bytes_transferred = bytes_transferred_count + beatBytes.U

          current_spad_addr := next_spad_addr
          current_mem_addr := current_mem_addr + beatBytes.U
          bytes_transferred_count := next_bytes_transferred

          when(next_bytes_transferred >= length_bytes_reg) {
            state := sDone
          } .elsewhen (next_bytes_transferred < length_bytes_reg) {
            state := sTLWrite_SetSpadAddr // Go back to set SPAD address for next read
          } .otherwise {
            state := sDone
          }
        }
      }
    }

    is(sDone) {
      state := sIdle
    }
    is(sError) {
      state := sIdle
    }
  }
}