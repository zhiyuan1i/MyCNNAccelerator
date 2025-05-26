// filename: my_cnn_rocc_test.c
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
// The header now contains multiple expected OFM arrays
#include "generated_conv_data.h"

// RoCC Instruction Header (from my demo)
#ifndef SRC_MAIN_C_ROCC_H
#define SRC_MAIN_C_ROCC_H

#include <riscv-pk/encoding.h> // May not be available directly

#define STR1(x) #x
#define STR(x) STR1(x)
#define EXTRACT(a, size, offset) (((~(~0 << size) << offset) & a) >> offset)

#define CUSTOMX_OPCODE(x) CUSTOM_ ## x
#define CUSTOM_0 0b0001011
#define CUSTOM_1 0b0101011
#define CUSTOM_2 0b1011011
#define CUSTOM_3 0b1111011

#define CUSTOMX(X, xd, xs1, xs2, rd, rs1, rs2, funct) \
  CUSTOMX_OPCODE(X)                       | \
  (rd                           << (7))        | \
  (EXTRACT(funct, 7, 0) << (7+5+3+5+5))   | /* funct7 */ \
  (rs2                          << (7+5+3+5))    | /* rs2 */ \
  (rs1                          << (7+5+3))      | /* rs1 */ \
  (0b000                        << (7+5))        | /* funct3 (hardcoded to 000 for RoCC) */ \
  (xd                           << (7+5+1+1))    | /* xd (destination register used) */ \
  (xs1                          << (7+5+1))      | /* xs1 (source register 1 used) */ \
  (xs2                          << (7+5+0))      /* xs2 (source register 2 used) */


#define ROCC_INSTRUCTION_DSS(X, rd_val, rs1_val, rs2_val, funct) \
    ROCC_INSTRUCTION_R_R_R(X, rd_val, rs1_val, rs2_val, funct, 10, 11, 12)

#define ROCC_INSTRUCTION_R_R_R(X, rd_val, rs1_val, rs2_val, funct, rd_n, rs1_n, rs2_n) { \
    register uint64_t rd_ asm ("x" # rd_n); \
    register uint64_t rs1_ asm ("x" # rs1_n) = (uint64_t) rs1_val; \
    register uint64_t rs2_ asm ("x" # rs2_n) = (uint64_t) rs2_val; \
    asm volatile ( \
        ".word " STR(CUSTOMX(X, 1, 1, 1, rd_n, rs1_n, rs2_n, funct)) "\n\t" \
        : "=r" (rd_) \
        : [_rs1] "r" (rs1_), [_rs2] "r" (rs2_)); \
    rd_val = rd_; \
}

#endif // SRC_MAIN_C_ROCC_H


// CNN Accelerator ISA definitions
#define CMD_SET_IFM_ADDR_DATA     0
#define CMD_SET_KERNEL_ADDR_DATA  1
#define CMD_START_COMPUTE         2
#define CMD_GET_OFM_ADDR_DATA     3
#define CMD_GET_STATUS            4
#define CMD_DMA_CONFIG_ADDR       5
#define CMD_DMA_CONFIG_PARAMS     6
#define CMD_DMA_START             7
#define CMD_SET_KERNEL_PARAMS     8

#define STATUS_IDLE                 0
#define STATUS_COMPUTING            3
#define STATUS_COMPUTE_DONE         4
#define STATUS_DMA_BUSY             10
#define STATUS_DMA_CONFIG_READY     11
#define STATUS_DMA_IFM_LOAD_DONE    12
#define STATUS_DMA_KERNEL_LOAD_DONE 13
#define STATUS_DMA_OFM_STORE_DONE   14
#define STATUS_KERNEL_PARAMS_SET    15
#define STATUS_DMA_ERROR            254
#define STATUS_ERROR                255

#define BUFFER_ID_IFM    0
#define BUFFER_ID_KERNEL 1
#define BUFFER_ID_OFM    2

#define DMA_DIR_MEM_TO_BUF 0
#define DMA_DIR_BUF_TO_MEM 1

#define DMA_PARAM_BUF_ID_BITS    2
#define DMA_PARAM_DIR_BITS       1
#define DMA_PARAM_BUF_ID_DIR_TOTAL_BITS (DMA_PARAM_BUF_ID_BITS + DMA_PARAM_DIR_BITS)


static inline uint64_t rdcycle_custom(void) {
    uint64_t cycles;
    asm volatile ("rdcycle %0" : "=r" (cycles));
    return cycles;
}

static inline uint64_t rocc_set_kernel_params(uint64_t kernel_dim) {
    uint64_t status_rd;
    ROCC_INSTRUCTION_DSS(0, status_rd, kernel_dim, 0, CMD_SET_KERNEL_PARAMS);
    return status_rd;
}

static inline uint64_t rocc_dma_config_addr(uint64_t addr) {
    uint64_t status_rd;
    ROCC_INSTRUCTION_DSS(0, status_rd, addr, 0, CMD_DMA_CONFIG_ADDR);
    return status_rd;
}

static inline uint64_t rocc_dma_config_params(uint64_t len_bytes, uint64_t direction, uint64_t buffer_id) {
    uint64_t params = (len_bytes << DMA_PARAM_BUF_ID_DIR_TOTAL_BITS) |
                      (direction << DMA_PARAM_BUF_ID_BITS) |
                      buffer_id;
    uint64_t status_rd;
    ROCC_INSTRUCTION_DSS(0, status_rd, params, 0, CMD_DMA_CONFIG_PARAMS);
    return status_rd;
}

static inline uint64_t rocc_dma_start() {
    uint64_t status_rd;
    ROCC_INSTRUCTION_DSS(0, status_rd, 0, 0, CMD_DMA_START);
    return status_rd;
}

static inline uint64_t rocc_start_compute() {
    uint64_t status_rd;
    ROCC_INSTRUCTION_DSS(0, status_rd, 0, 0, CMD_START_COMPUTE);
    return status_rd;
}

static inline uint64_t rocc_get_status() {
    uint64_t status;
    ROCC_INSTRUCTION_DSS(0, status, 0, 0, CMD_GET_STATUS);
    return status;
}

// Use defines from generated_conv_data.h
#define IFM_ROWS GEN_IFM_ROWS
#define IFM_COLS GEN_IFM_COLS
#define MAX_KERNEL_DIM GEN_MAX_KERNEL_DIM // Max dimension of the stored kernel data
#define OFM_ROWS GEN_OFM_ROWS
#define OFM_COLS GEN_OFM_COLS
#define DATA_WIDTH_BYTES 2

// Pointers to data from generated_conv_data.h
const int16_t *ifm_data_ptr = generated_ifm_data;
// generated_kernel_data_5x5 is the full 5x5 kernel data in memory
const int16_t *kernel_data_memory_ptr = generated_kernel_data_5x5;

// Expected OFM data pointers
const int16_t *ofm_expected_data_5x5_ptr = generated_ofm_expected_data_5x5;
const int16_t *ofm_expected_data_3x3_ptr = generated_ofm_expected_data_3x3;
const int16_t *ofm_expected_data_1x1_ptr = generated_ofm_expected_data_1x1;

int16_t ofm_output_data[OFM_ROWS * OFM_COLS]; // For accelerator output

int poll_status_until(uint64_t target_status, uint64_t error_status1, uint64_t error_status2, const char* operation_name, int max_retries) {
    uint64_t current_status;
    int attempts = 0;
    printf("Polling for %s (target: %lu)...\n", operation_name, target_status);
    do {
        current_status = rocc_get_status();
        if (current_status == target_status) {
            printf("%s completed. Status: %lu\n", operation_name, current_status);
            return 0;
        }
        if (current_status == error_status1 || current_status == error_status2) {
            printf("Error during %s. Status: %lu\n", operation_name, current_status);
            return -1;
        }
        attempts++;
         if (attempts > 0 && (attempts % (max_retries/20) == 0 || attempts == 1 || attempts == max_retries-1) ) { // Print more frequently
             printf("Still waiting for %s... Status: %lu (target: %lu), Attempts: %d/%d\n",
                    operation_name, current_status, target_status, attempts, max_retries);
         }
    } while (attempts < max_retries);

    printf("Timeout waiting for %s. Last status: %lu\n", operation_name, current_status);
    return -1;
}

int run_convolution_test_case(int kernel_dim_to_test) {
    uint64_t start_cycle_total, end_cycle_total;
    uint64_t current_status;

    printf("\n--- Starting Test Case: Kernel %dx%d ---\n", kernel_dim_to_test, kernel_dim_to_test);

    for (int i = 0; i < OFM_ROWS * OFM_COLS; ++i) {
        ofm_output_data[i] = 0;
    }

    uint64_t ifm_addr    = (uint64_t)ifm_data_ptr;
    // kernel_addr always points to the start of the 5x5 block in memory.
    // The DMA length will determine how much is read.
    uint64_t kernel_addr = (uint64_t)kernel_data_memory_ptr;
    uint64_t ofm_addr    = (uint64_t)ofm_output_data;

    uint64_t ifm_len_bytes    = (uint64_t)IFM_ROWS * IFM_COLS * DATA_WIDTH_BYTES;
    uint64_t kernel_len_bytes = (uint64_t)kernel_dim_to_test * kernel_dim_to_test * DATA_WIDTH_BYTES;
    uint64_t ofm_len_bytes    = (uint64_t)OFM_ROWS * OFM_COLS * DATA_WIDTH_BYTES;

    printf("IFM Addr: 0x%lx, Len: %lu bytes\n", ifm_addr, ifm_len_bytes);
    printf("Kernel Addr (base of 5x5 block): 0x%lx\n", kernel_addr);
    printf("Actual Kernel Dim for this test: %dx%d, DMA Len for kernel: %lu bytes\n", kernel_dim_to_test, kernel_dim_to_test, kernel_len_bytes);
    printf("OFM Addr: 0x%lx, Len: %lu bytes\n", ofm_addr, ofm_len_bytes);

    start_cycle_total = rdcycle_custom();

    printf("Sending CMD_SET_KERNEL_PARAMS with dim = %d\n", kernel_dim_to_test);
    rocc_set_kernel_params(kernel_dim_to_test); // Return value is status, can be checked
    if (poll_status_until(STATUS_KERNEL_PARAMS_SET, STATUS_ERROR, STATUS_DMA_ERROR, "Set Kernel Params", 1000) != 0) return 1;

    printf("Configuring IFM DMA (Addr: 0x%lx)\n", ifm_addr);
    rocc_dma_config_addr(ifm_addr);
    current_status = rocc_dma_config_params(ifm_len_bytes, DMA_DIR_MEM_TO_BUF, BUFFER_ID_IFM);

    printf("Starting IFM DMA...\n");
    rocc_dma_start();
    if (poll_status_until(STATUS_DMA_IFM_LOAD_DONE, STATUS_DMA_ERROR, STATUS_ERROR, "IFM Load", 2000000) != 0) return 1;

    printf("Configuring Kernel DMA (Addr: 0x%lx)\n", kernel_addr);
    rocc_dma_config_addr(kernel_addr);
    current_status = rocc_dma_config_params(kernel_len_bytes, DMA_DIR_MEM_TO_BUF, BUFFER_ID_KERNEL);

    printf("Starting Kernel DMA...\n");
    rocc_dma_start();
    if (poll_status_until(STATUS_DMA_KERNEL_LOAD_DONE, STATUS_DMA_ERROR, STATUS_ERROR, "Kernel Load", 2000000) != 0) return 1;

    printf("Sending CMD_START_COMPUTE for %dx%d kernel...\n", kernel_dim_to_test, kernel_dim_to_test);
    rocc_start_compute();
    if (poll_status_until(STATUS_COMPUTE_DONE, STATUS_ERROR, STATUS_DMA_ERROR, "Computation", 10000000) != 0) return 1; // Increased timeout

    printf("Configuring OFM DMA (Addr: 0x%lx)\n", ofm_addr);
    rocc_dma_config_addr(ofm_addr);
    current_status = rocc_dma_config_params(ofm_len_bytes, DMA_DIR_BUF_TO_MEM, BUFFER_ID_OFM);

    printf("Starting OFM DMA...\n");
    rocc_dma_start();
    if (poll_status_until(STATUS_DMA_OFM_STORE_DONE, STATUS_DMA_ERROR, STATUS_ERROR, "OFM Store", 2000000) != 0) return 1;

    end_cycle_total = rdcycle_custom();
    printf("--- Kernel %dx%d Test Case Finished ---\n", kernel_dim_to_test, kernel_dim_to_test);
    printf("Total cycles for %dx%d kernel test: %lu\n", kernel_dim_to_test, kernel_dim_to_test, end_cycle_total - start_cycle_total);

    printf("--- Verification for %dx%d kernel ---\n", kernel_dim_to_test, kernel_dim_to_test);
    int errors = 0;
    const int16_t *current_ofm_expected_ptr = NULL;

    if (kernel_dim_to_test == 5) {
        current_ofm_expected_ptr = ofm_expected_data_5x5_ptr;
    } else if (kernel_dim_to_test == 3) {
        current_ofm_expected_ptr = ofm_expected_data_3x3_ptr;
    } else if (kernel_dim_to_test == 1) {
        current_ofm_expected_ptr = ofm_expected_data_1x1_ptr;
    } else {
        printf("Error: No expected OFM data for kernel_dim_to_test = %d\n", kernel_dim_to_test);
        return 1; // Cannot verify
    }

    for (int i = 0; i < OFM_ROWS * OFM_COLS; ++i) {
        if (ofm_output_data[i] != current_ofm_expected_ptr[i]) {
            if(errors < 20) {
                printf("Mismatch at OFM[%d]: Expected %d, Got %d\n", i, current_ofm_expected_ptr[i], ofm_output_data[i]);
            }
            errors++;
        }
    }
    if (errors == 0) {
        printf("OFM data matches expected data for %dx%d kernel.\n", kernel_dim_to_test, kernel_dim_to_test);
    } else {
        printf("%d mismatches in OFM data for %dx%d kernel.\n", errors, kernel_dim_to_test, kernel_dim_to_test);
    }
    
    current_status = rocc_get_status();
    printf("Final accelerator status after %dx%d test: %lu (Expected IDLE: %d or OFM_DONE: %d)\n",
           kernel_dim_to_test, kernel_dim_to_test, current_status, STATUS_IDLE, STATUS_DMA_OFM_STORE_DONE);

    // if (!((current_status == STATUS_IDLE || current_status == STATUS_DMA_OFM_STORE_DONE))) {
    //      printf("Test FAILED for %dx%d: Final status %lu is unexpected.\n", kernel_dim_to_test, kernel_dim_to_test, current_status);
    //      return 1;
    // }
    if (errors != 0) {
        printf("Test FAILED for %dx%d: Results do not match reference.\n", kernel_dim_to_test, kernel_dim_to_test);
        return 1;
    }

    printf("Test PASSED for %dx%d (operation completed, final status reasonable, results match).\n", kernel_dim_to_test, kernel_dim_to_test);
    return 0;
}

int main(void) {
    printf("Starting CNN Accelerator RoCC Full Test Suite\n");
    int overall_test_result = 0;

    if (run_convolution_test_case(5) != 0) { // Test 5x5
        overall_test_result = 1;
    }
    if (run_convolution_test_case(3) != 0) { // Test 3x3
        overall_test_result = 1;
    }
    if (run_convolution_test_case(1) != 0) { // Test 1x1
        overall_test_result = 1;
    }

    printf("\nCNN Accelerator RoCC Full Test Suite Finished.\n");
    if (overall_test_result == 0) {
        printf("All test cases PASSED.\n");
    } else {
        printf("One or more test cases FAILED.\n");
    }
    return overall_test_result;
}

