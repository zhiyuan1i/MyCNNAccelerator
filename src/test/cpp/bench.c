// filename: bench.c
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "generated_conv_data.h"

// --- RoCC Instruction Header and ISA Definitions (Same as before) ---
#ifndef SRC_MAIN_C_ROCC_H
#define SRC_MAIN_C_ROCC_H
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
  (rd                           << (7))          | \
  (EXTRACT(funct, 7, 0) << (7+5+3+5+5))   | \
  (rs2                          << (7+5+3+5))      | \
  (rs1                          << (7+5+3))        | \
  (0b000                        << (7+5))          | \
  (xd                           << (7+5+1+1))      | \
  (xs1                          << (7+5+1))        | \
  (xs2                          << (7+5+0))
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

#define CMD_SET_IFM_ADDR_DATA     0
#define CMD_SET_KERNEL_ADDR_DATA  1
#define CMD_START_COMPUTE         2
#define CMD_GET_OFM_ADDR_DATA     3
#define CMD_GET_STATUS            4
#define CMD_DMA_CONFIG_ADDR       5
#define CMD_DMA_CONFIG_PARAMS     6
#define CMD_DMA_START             7
#define CMD_SET_KERNEL_PARAMS     8

#define STATUS_IDLE               0
#define STATUS_COMPUTING          3
#define STATUS_COMPUTE_DONE       4
#define STATUS_DMA_BUSY           10
#define STATUS_DMA_CONFIG_READY   11
#define STATUS_DMA_IFM_LOAD_DONE  12
#define STATUS_DMA_KERNEL_LOAD_DONE 13
#define STATUS_DMA_OFM_STORE_DONE 14
#define STATUS_KERNEL_PARAMS_SET  15
#define STATUS_DMA_ERROR          254
#define STATUS_ERROR              255

#define BUFFER_ID_IFM     0
#define BUFFER_ID_KERNEL  1
#define BUFFER_ID_OFM     2

#define DMA_DIR_MEM_TO_BUF 0
#define DMA_DIR_BUF_TO_MEM 1

#define DMA_PARAM_BUF_ID_BITS     2
#define DMA_PARAM_DIR_BITS        1
#define DMA_PARAM_BUF_ID_DIR_TOTAL_BITS (DMA_PARAM_BUF_ID_BITS + DMA_PARAM_DIR_BITS)
// --- End RoCC Instruction Header and ISA Definitions ---

#define F_BITS_FOR_CPU_SCALING 8

#ifndef IFM_ROWS
#define IFM_ROWS GEN_IFM_ROWS
#endif
#ifndef IFM_COLS
#define IFM_COLS GEN_IFM_COLS
#endif
#ifndef MAX_KERNEL_DIM
#define MAX_KERNEL_DIM GEN_MAX_KERNEL_DIM
#endif
#ifndef OFM_ROWS
#define OFM_ROWS GEN_OFM_ROWS
#endif
#ifndef OFM_COLS
#define OFM_COLS GEN_OFM_COLS
#endif
#ifndef DATA_WIDTH_BYTES
    #error "DATA_WIDTH_BYTES is not defined. Please define in generated_conv_data.h (e.g., #define DATA_WIDTH_BYTES 2)"
#endif

// Global variable to control RoCC polling verbosity
// Set this to 1 for detailed prints, 0 for less verbose
#define g_verbose_rocc_polling 0

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

const int16_t *ifm_data_ptr = generated_ifm_data;
const int16_t *kernel_data_memory_ptr = generated_kernel_data_5x5;
int16_t ofm_rocc_output_data[OFM_ROWS * OFM_COLS];
int16_t ofm_cpu_output_data[OFM_ROWS * OFM_COLS];

// Restored verbose poll_status_until function
int poll_status_until(uint64_t target_status, uint64_t error_status1, uint64_t error_status2, const char* operation_name, int max_retries) {
    uint64_t current_status;
    int attempts = 0;
    // if (g_verbose_rocc_polling) {
    //     printf("Polling for %s (target: %lu)...\n", operation_name, target_status);
    // }
    do {
        current_status = rocc_get_status();
        if (current_status == target_status) {
            if (g_verbose_rocc_polling) {
                printf("%s completed. Status: %lu\n", operation_name, current_status);
            }
            return 0; // Success
        }
        // if (current_status == error_status1 || current_status == error_status2) {
        //     // This error is critical, so always print it
        //     printf("Error during %s. Status: %lu\n", operation_name, current_status);
        //     return -1; // Error
        // }
        attempts++;
        // if (g_verbose_rocc_polling && attempts > 0) {
        //     int print_interval = max_retries / 20;
        //     if (print_interval == 0) print_interval = 1; // Avoid division by zero and ensure some prints

        //     if (attempts == 1 || attempts == (max_retries -1) || (attempts % print_interval == 0) ) {
        //          printf("Still waiting for %s... Status: %lu (target: %lu), Attempts: %d/%d\n",
        //                operation_name, current_status, target_status, attempts, max_retries);
        //     }
        // }
    } while (attempts < max_retries);

    // This timeout is critical, so always print it
    printf("Timeout waiting for %s. Last status: %lu (Target: %lu), Attempts: %d\n", operation_name, current_status, target_status, attempts);
    return -1; // Timeout
}

void cpu_convolution_same_padding(const int16_t *ifm,
                                  const int16_t *kernel_base,
                                  int16_t *ofm,
                                  int ifm_r, int ifm_c,
                                  int actual_k_dim,
                                  int stored_k_dim_stride) {
    int pad = (actual_k_dim - 1) / 2;
    for (int or = 0; or < ifm_r; ++or) {
        for (int oc = 0; oc < ifm_c; ++oc) {
            int32_t accumulator = 0;
            for (int kr = 0; kr < actual_k_dim; ++kr) {
                for (int kc = 0; kc < actual_k_dim; ++kc) {
                    int ifm_eff_r = or - pad + kr;
                    int ifm_eff_c = oc - pad + kc;
                    int16_t ifm_val = 0;
                    if (ifm_eff_r >= 0 && ifm_eff_r < ifm_r && ifm_eff_c >= 0 && ifm_eff_c < ifm_c) {
                        ifm_val = ifm[ifm_eff_r * ifm_c + ifm_eff_c];
                    }
                    int16_t kernel_val = kernel_base[kr * stored_k_dim_stride + kc];
                    accumulator += (int32_t)ifm_val * kernel_val;
                }
            }
            ofm[or * ifm_c + oc] = (int16_t)(accumulator >> F_BITS_FOR_CPU_SCALING);
        }
    }
}

// run_rocc_convolution with conditional prints
int run_rocc_convolution(int kernel_dim_to_test, uint64_t *rocc_cycles) {
    uint64_t start_cycle_total, end_cycle_total;

    for (int i = 0; i < OFM_ROWS * OFM_COLS; ++i) {
        ofm_rocc_output_data[i] = 0;
    }

    uint64_t ifm_addr    = (uint64_t)ifm_data_ptr;
    uint64_t kernel_addr = (uint64_t)kernel_data_memory_ptr;
    uint64_t ofm_addr    = (uint64_t)ofm_rocc_output_data;

    uint64_t ifm_len_bytes    = (uint64_t)IFM_ROWS * IFM_COLS * DATA_WIDTH_BYTES;
    uint64_t kernel_len_bytes = (uint64_t)kernel_dim_to_test * kernel_dim_to_test * DATA_WIDTH_BYTES;
    uint64_t ofm_len_bytes    = (uint64_t)OFM_ROWS * OFM_COLS * DATA_WIDTH_BYTES;

    if (g_verbose_rocc_polling) {
        printf("RoCC Info: Starting test for %dx%d kernel.\n", kernel_dim_to_test, kernel_dim_to_test);
        printf("RoCC Info: IFM Addr: 0x%lx, Len: %lu bytes\n", ifm_addr, ifm_len_bytes);
        printf("RoCC Info: Kernel Addr (base of %dx%d block): 0x%lx, DMA Len: %lu bytes\n", MAX_KERNEL_DIM, MAX_KERNEL_DIM, kernel_addr, kernel_len_bytes);
        printf("RoCC Info: OFM Addr: 0x%lx, Len: %lu bytes\n", ofm_addr, ofm_len_bytes);
    }

    start_cycle_total = rdcycle_custom();

    if (g_verbose_rocc_polling) printf("RoCC Step: Sending CMD_SET_KERNEL_PARAMS with dim = %d\n", kernel_dim_to_test);
    (void)rocc_set_kernel_params(kernel_dim_to_test);
    if (poll_status_until(STATUS_KERNEL_PARAMS_SET, STATUS_ERROR, STATUS_DMA_ERROR, "Set Kernel Params", 10000) != 0) return 1;

    if (g_verbose_rocc_polling) printf("RoCC Step: Configuring IFM DMA (Addr: 0x%lx)\n", ifm_addr);
    (void)rocc_dma_config_addr(ifm_addr);
    (void)rocc_dma_config_params(ifm_len_bytes, DMA_DIR_MEM_TO_BUF, BUFFER_ID_IFM);
    if (g_verbose_rocc_polling) printf("RoCC Step: Starting IFM DMA...\n");
    (void)rocc_dma_start();
    if (poll_status_until(STATUS_DMA_IFM_LOAD_DONE, STATUS_DMA_ERROR, STATUS_ERROR, "IFM Load", 2000000) != 0) return 1;

    if (g_verbose_rocc_polling) printf("RoCC Step: Configuring Kernel DMA (Addr: 0x%lx)\n", kernel_addr);
    (void)rocc_dma_config_addr(kernel_addr);
    (void)rocc_dma_config_params(kernel_len_bytes, DMA_DIR_MEM_TO_BUF, BUFFER_ID_KERNEL);
    if (g_verbose_rocc_polling) printf("RoCC Step: Starting Kernel DMA...\n");
    (void)rocc_dma_start();
    if (poll_status_until(STATUS_DMA_KERNEL_LOAD_DONE, STATUS_DMA_ERROR, STATUS_ERROR, "Kernel Load", 2000000) != 0) return 1;

    if (g_verbose_rocc_polling) printf("RoCC Step: Sending CMD_START_COMPUTE for %dx%d kernel...\n", kernel_dim_to_test, kernel_dim_to_test);
    (void)rocc_start_compute();
    if (poll_status_until(STATUS_COMPUTE_DONE, STATUS_ERROR, STATUS_DMA_ERROR, "Computation", 20000000) != 0) return 1;

    if (g_verbose_rocc_polling) printf("RoCC Step: Configuring OFM DMA (Addr: 0x%lx)\n", ofm_addr);
    (void)rocc_dma_config_addr(ofm_addr);
    (void)rocc_dma_config_params(ofm_len_bytes, DMA_DIR_BUF_TO_MEM, BUFFER_ID_OFM);
    if (g_verbose_rocc_polling) printf("RoCC Step: Starting OFM DMA...\n");
    (void)rocc_dma_start();
    if (poll_status_until(STATUS_DMA_OFM_STORE_DONE, STATUS_DMA_ERROR, STATUS_ERROR, "OFM Store", 2000000) != 0) return 1;

    end_cycle_total = rdcycle_custom();
    *rocc_cycles = end_cycle_total - start_cycle_total;
    
    uint64_t final_status = rocc_get_status();
    if (g_verbose_rocc_polling) {
         printf("RoCC Info: Kernel %dx%d Test Case Finished. Final Status: %lu (Expected IDLE: %d or OFM_DONE: %d)\n",
            kernel_dim_to_test, kernel_dim_to_test, final_status, STATUS_IDLE, STATUS_DMA_OFM_STORE_DONE);
    }
    // Critical errors (timeout/dma error from polling) already return 1.
    // This is a softer check for unexpected final state if all polls passed.
    if (!((final_status == STATUS_IDLE || final_status == STATUS_DMA_OFM_STORE_DONE))) {
        // This printf is important if polling passed but final state is odd.
        printf("Warning: RoCC K=%d final status %lu is unexpected after successful polling.\n", kernel_dim_to_test, final_status);
        // Consider if this should also return 1 to mark the run as problematic.
        // For now, only polling failures cause a hard fail (return 1).
    }
    return 0;
}

int main(void) {
    uint64_t cpu_cycles, rocc_op_cycles;

    printf("Starting CNN Accelerator Benchmark (IFM: %dx%d, OFM: %dx%d)\n",
           IFM_ROWS, IFM_COLS, OFM_ROWS, OFM_COLS);
    if (g_verbose_rocc_polling) {
        printf("Verbose RoCC polling is ENABLED.\n");
    } else {
        printf("Verbose RoCC polling is DISABLED.\n");
    }
    printf("CPU convolution uses F_BITS_FOR_CPU_SCALING = %d\n\n", F_BITS_FOR_CPU_SCALING);

    int kernel_dims_to_test[] = {1, 3, 5};
    int num_kernel_dims = sizeof(kernel_dims_to_test) / sizeof(kernel_dims_to_test[0]);

    for (int i = 0; i < num_kernel_dims; ++i) {
        int k_dim = kernel_dims_to_test[i];
        printf("--- Kernel Dimension: %dx%d ---\n", k_dim, k_dim);

        // RoCC Convolution
        if (g_verbose_rocc_polling) {
            printf("Running RoCC for %dx%d kernel...\n", k_dim, k_dim);
        }
        if (run_rocc_convolution(k_dim, &rocc_op_cycles) == 0) {
            printf("RoCC %dx%d Kernel Cycles: %lu\n", k_dim, k_dim, rocc_op_cycles);
        } else {
            printf("RoCC %dx%d Kernel Test: FAILED (see error/timeout messages above)\n", k_dim, k_dim);
            rocc_op_cycles = 0; 
        }

        // CPU Convolution
        if (g_verbose_rocc_polling) {
            printf("Running CPU for %dx%d kernel...\n", k_dim, k_dim);
        }
        uint64_t start_cpu = rdcycle_custom();
        cpu_convolution_same_padding(ifm_data_ptr, kernel_data_memory_ptr, ofm_cpu_output_data,
                                     IFM_ROWS, IFM_COLS, k_dim, MAX_KERNEL_DIM);
        uint64_t end_cpu = rdcycle_custom();
        cpu_cycles = end_cpu - start_cpu;
        printf("CPU %dx%d Kernel Cycles: %lu\n", k_dim, k_dim, cpu_cycles);

        if (cpu_cycles > 0 && rocc_op_cycles > 0) {
            printf("RoCC Speedup vs CPU for %dx%d: %.2f\n", k_dim, k_dim, (double)cpu_cycles / rocc_op_cycles);
        }
        printf("\n");
    }

    printf("CNN Accelerator Benchmark Finished.\n");
    return 0;
}