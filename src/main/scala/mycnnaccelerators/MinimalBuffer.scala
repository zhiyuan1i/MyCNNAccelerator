// filename: MinimalBuffer.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

class MinimalBufferIO(addrWidth: Int, dataWidth: Int) extends Bundle {
  // Write port
  val write_en   = Input(Bool())
  val write_addr = Input(UInt(addrWidth.W))
  val write_data = Input(SInt(dataWidth.W))

  // Read port
  val read_addr  = Input(UInt(addrWidth.W))
  val read_data  = Output(SInt(dataWidth.W))
}

class MinimalBuffer(depth: Int, dataWidth: Int) extends Module {
  val addrWidth = log2Ceil(depth)
  val io = IO(new MinimalBufferIO(addrWidth, dataWidth))

  require(dataWidth > 0, "dataWidth must be positive")
  require(dataWidth % 8 == 0, "dataWidth must be a multiple of 8 for this SRAM implementation style")
  val numBytes = dataWidth / 8
  val byteVecType = Vec(numBytes, UInt(8.W))

  // Use SyncReadMem with a byte-vector data type, similar to Gemmini's successful pattern.
  // SyncReadMem.ReadFirst defines behavior for same-cycle read/write to the same address:
  // read returns the old value. This mimics the original Reg(Vec) + RegNext behavior.
  val sram_mem = SyncReadMem(depth, byteVecType, SyncReadMem.ReadFirst)

  // Write operation
  when(io.write_en) {
    // Convert the SInt write_data to Vec[UInt(8.W)]
    // .asUInt is a bit-preserving cast, then .asTypeOf reinterprets the bits.
    val write_data_as_bytes = io.write_data.asUInt.asTypeOf(byteVecType)
    
    // Create a full mask to write all bytes
    val full_write_mask = VecInit(Seq.fill(numBytes)(true.B))
    
    sram_mem.write(io.write_addr, write_data_as_bytes, full_write_mask)
  }

  // Read operation
  // Read is always enabled based on read_addr, data available next cycle.
  val read_enable = true.B 
  val read_bytes_from_mem = sram_mem.read(io.read_addr, read_enable)

  // Convert the Vec[UInt(8.W)] read from memory back to SInt(dataWidth.W)
  // .asUInt concatenates the bytes, then .asSInt reinterprets as SInt.
  io.read_data := read_bytes_from_mem.asUInt.asSInt
}