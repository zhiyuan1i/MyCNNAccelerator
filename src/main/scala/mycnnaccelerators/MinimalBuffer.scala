// filename: MinimalBuffer.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

class MinimalBufferIO(addrWidth: Int, dataWidth: Int) extends Bundle {
  // Write port
  val write_en   = Input(Bool())
  val write_addr = Input(UInt(addrWidth.W))
  val write_data = Input(UInt(dataWidth.W))

  // Read port
  // For SyncReadMem, read data is available one cycle after 'read_addr' is asserted
  // if an explicit read_en for the mem itself is true, or if read is always enabled based on addr.
  val read_addr  = Input(UInt(addrWidth.W))
  val read_data  = Output(UInt(dataWidth.W)) // Data will be valid next cycle
}

class MinimalBuffer(depth: Int, dataWidth: Int) extends Module {
  val addrWidth = log2Ceil(depth)
  val io = IO(new MinimalBufferIO(addrWidth, dataWidth))

  val mem = SyncReadMem(depth, UInt(dataWidth.W))

  when(io.write_en) {
    mem.write(io.write_addr, io.write_data)
  }

  // The read operation from SyncReadMem is registered.
  // Data appears on io.read_data in the cycle *after* io.read_addr is set.
  // The read enable for SyncReadMem.read is implicitly true here.
  io.read_data := mem.read(io.read_addr, true.B)
}