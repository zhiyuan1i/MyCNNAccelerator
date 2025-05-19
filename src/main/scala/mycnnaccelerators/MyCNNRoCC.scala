// filename: MyCNNRoCC.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.tile._
import freechips.rocketchip.diplomacy.LazyModule
import CNNAcceleratorISA._ // Import ISA definitions

// Key for configuration
case object MyCNNAcceleratorKey extends Field[AcceleratorConfig](DefaultAcceleratorConfig)

class MyCNNRoCC(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  val baseConfig = p(MyCNNAcceleratorKey) // Get base config from Parameters
  // Actual config is created in MyCNNRoCCModuleImp once xLen is known
  override lazy val module = new MyCNNRoCCModuleImp(this, baseConfig)
}

class MyCNNRoCCModuleImp(outer: MyCNNRoCC, defaultConfig: AcceleratorConfig)(implicit p: Parameters)
    extends LazyRoCCModuleImp(outer)
    with HasCoreParameters { // HasCoreParameters provides xLen, etc.

  // Create the actual config using xLen from HasCoreParameters
  val config = defaultConfig.copy(xLen = xLen)

  // Instantiate internal modules
  val ifm_buffer = Module(new MinimalBuffer(config.ifmDepth, config.dataWidth))
  val kernel_buffer = Module(new MinimalBuffer(config.kernelDepth, config.dataWidth))
  val ofm_buffer = Module(new MinimalBuffer(config.ofmDepth, config.dataWidth))
  val compute_unit = Module(new ComputeUnit(config))

  // RoCC FSM states
  val sIdle :: sWaitOFMRead :: sRespond :: Nil = Enum(3)
  val rocc_state = RegInit(sIdle)

  // Registers for RoCC interaction
  val resp_data_reg = Reg(UInt(config.xLen.W))
  val resp_rd_reg = Reg(UInt(5.W))
  val resp_valid_reg = RegInit(false.B)

  val accelerator_status_reg = RegInit(STATUS_IDLE) // For GET_STATUS command
  val ofm_read_addr_reg = Reg(UInt(config.ofmAddrWidth.W)) // For CMD_GET_OFM_ADDR_DATA

  // Default outputs for RoCC interface
  io.cmd.ready := (rocc_state === sIdle) && !resp_valid_reg
  io.resp.valid := resp_valid_reg
  io.resp.bits.rd := resp_rd_reg
  io.resp.bits.data := resp_data_reg
  io.busy := (compute_unit.io.busy || accelerator_status_reg === STATUS_COMPUTING) && (rocc_state =/= sIdle) // Simplified busy logic for now
  io.interrupt := false.B // No interrupts for this simple version

  // Default connections for buffer write ports (controlled by RoCC FSM)
  ifm_buffer.io.write_en := false.B
  ifm_buffer.io.write_addr := 0.U
  ifm_buffer.io.write_data := 0.S(config.dataWidth.W) // CORRECTED: SInt literal
  // ifm_buffer.io.read_addr is driven by compute_unit

  kernel_buffer.io.write_en := false.B
  kernel_buffer.io.write_addr := 0.U
  kernel_buffer.io.write_data := 0.S(config.dataWidth.W) // CORRECTED: SInt literal
  // kernel_buffer.io.read_addr is driven by compute_unit

  // OFM buffer connections
  ofm_buffer.io.write_en := compute_unit.io.ofm_write_en // CU writes to OFM
  ofm_buffer.io.write_addr := compute_unit.io.ofm_write_addr
  ofm_buffer.io.write_data := compute_unit.io.ofm_write_data // SInt to SInt (OK)
  ofm_buffer.io.read_addr := 0.U // Controlled by RoCC FSM for readback via CMD_GET_OFM_ADDR_DATA

  // Default connections for compute_unit inputs (controlled by RoCC FSM or wired below)
  compute_unit.io.start := false.B
  // compute_unit.io.ifm_read_addr is an OUTPUT of compute_unit
  // compute_unit.io.ifm_read_data is an INPUT to compute_unit, connected below
  // compute_unit.io.kernel_read_addr is an OUTPUT of compute_unit
  // compute_unit.io.kernel_read_data is an INPUT to compute_unit, connected below

  // RoCC Command Handling FSM
  when(resp_valid_reg && io.resp.ready) {
    resp_valid_reg := false.B // Clear response valid when CPU accepts it
  }

  switch(rocc_state) {
    is(sIdle) {
      accelerator_status_reg := Mux(compute_unit.io.busy, STATUS_COMPUTING,
                                Mux(compute_unit.io.done && accelerator_status_reg === STATUS_COMPUTING, STATUS_COMPUTE_DONE,
                                    Mux(accelerator_status_reg === STATUS_COMPUTE_DONE, STATUS_COMPUTE_DONE, STATUS_IDLE)))

      when(io.cmd.valid && !resp_valid_reg) {
        val cmd = io.cmd.bits
        resp_rd_reg := cmd.inst.rd // Capture destination register for all commands

        io.cmd.ready := false.B // Will be set back to true if cmd processed in one cycle

        switch(cmd.inst.funct) {
          is(CMD_SET_IFM_ADDR_DATA) {
            when(cmd.rs1 < config.ifmDepth.U) {
              ifm_buffer.io.write_en   := true.B
              ifm_buffer.io.write_addr := cmd.rs1
              ifm_buffer.io.write_data := cmd.rs2(config.dataWidth - 1, 0).asSInt // CORRECTED: Cast to SInt
              accelerator_status_reg := STATUS_LOADING_IFM
            } .otherwise {
              accelerator_status_reg := STATUS_ERROR
            }
            io.cmd.ready := true.B
          }
          is(CMD_SET_KERNEL_ADDR_DATA) {
            when(cmd.rs1 < config.kernelDepth.U) {
              kernel_buffer.io.write_en   := true.B
              kernel_buffer.io.write_addr := cmd.rs1
              kernel_buffer.io.write_data := cmd.rs2(config.dataWidth - 1, 0).asSInt // CORRECTED: Cast to SInt
              accelerator_status_reg := STATUS_LOADING_KERNEL
            } .otherwise {
              accelerator_status_reg := STATUS_ERROR
            }
            io.cmd.ready := true.B
          }
          is(CMD_START_COMPUTE) {
            when(!compute_unit.io.busy && accelerator_status_reg =/= STATUS_COMPUTING) { // ensure not already busy
              compute_unit.io.start := true.B
              accelerator_status_reg := STATUS_COMPUTING
            }
            io.cmd.ready := true.B
          }
          is(CMD_GET_OFM_ADDR_DATA) {
            when(cmd.rs1 < config.ofmDepth.U) {
              ofm_buffer.io.read_addr := cmd.rs1
              // ofm_read_addr_reg := cmd.rs1 // Not strictly needed if data is read next cycle directly
              rocc_state := sWaitOFMRead
            } .otherwise {
              resp_data_reg := STATUS_ERROR.asUInt // Ensure STATUS_ERROR can fit or is truncated appropriately
              resp_valid_reg := true.B
              accelerator_status_reg := STATUS_ERROR
              rocc_state := sRespond // Go to respond to send error
            }
          }
          is(CMD_GET_STATUS) {
            resp_data_reg := accelerator_status_reg // Status reg is already UInt(8.W), will be zero-extended by assignment to UInt(xLen.W)
            resp_valid_reg := true.B
            rocc_state := sRespond // Go to respond state
          }
        }
      }
    }

    is(sWaitOFMRead) {
      // Data from ofm_buffer.io.read_data is valid in this cycle
      // CORRECTED: Handle SInt from buffer to UInt RoCC response register with sign extension
      val data_from_ofm = Wire(SInt(config.xLen.W))
      data_from_ofm := ofm_buffer.io.read_data // Sign-extends from dataWidth to xLen
      resp_data_reg := data_from_ofm.asUInt    // Convert to UInt

      resp_valid_reg := true.B
      rocc_state := sRespond
    }

    is(sRespond) {
      // Wait for resp_valid_reg to be cleared by CPU io.resp.ready
      when(!resp_valid_reg) {
        io.cmd.ready := true.B
        rocc_state := sIdle
      }
    }
  }

  // Connect Compute Unit to Buffers
  // Buffers provide data to CU (input to CU)
  compute_unit.io.ifm_read_data    := ifm_buffer.io.read_data    // SInt to SInt (OK)
  compute_unit.io.kernel_read_data := kernel_buffer.io.read_data // SInt to SInt (OK)

  // CU drives its own read addresses to the buffers (output from CU)
  ifm_buffer.io.read_addr    := compute_unit.io.ifm_read_addr
  kernel_buffer.io.read_addr := compute_unit.io.kernel_read_addr

  // When compute unit is done, and RoCC was in computing state, update RoCC's status
  when(compute_unit.io.done && accelerator_status_reg === STATUS_COMPUTING) {
      accelerator_status_reg := STATUS_COMPUTE_DONE
  }
}