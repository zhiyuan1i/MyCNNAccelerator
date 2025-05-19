// filename: CNNAcceleratorISA.scala
package mycnnaccelerators

import chisel3._

object CNNAcceleratorISA {
  // Funct values for RoCC custom instructions (7-bit)
  // rs1 generally for address, rs2 for data (if applicable)
  // rd generally for response data (if applicable)

  val CMD_SET_IFM_ADDR_DATA   = 0.U(7.W) // rs1: address in IFM buffer, rs2: data
  val CMD_SET_KERNEL_ADDR_DATA= 1.U(7.W) // rs1: address in Kernel buffer, rs2: data
  val CMD_START_COMPUTE       = 2.U(7.W) // No operands needed if data is pre-loaded
  val CMD_GET_OFM_ADDR_DATA   = 3.U(7.W) // rs1: address in OFM buffer, rd: data from OFM
  val CMD_GET_STATUS          = 4.U(7.W) // rd: status code

  // Status codes (ensure fits in xLen, typically 8-bits are plenty)
  val STATUS_IDLE             = 0.U(8.W)
  val STATUS_LOADING_IFM      = 1.U(8.W) // Optional: if we want finer grain status
  val STATUS_LOADING_KERNEL   = 2.U(8.W) // Optional
  val STATUS_COMPUTING        = 3.U(8.W)
  val STATUS_COMPUTE_DONE     = 4.U(8.W) // Computation finished, OFM is ready
  val STATUS_ERROR            = 255.U(8.W) // General error
}