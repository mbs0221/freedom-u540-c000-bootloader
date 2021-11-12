// See LICENSE for license details.

#ifndef _SIFIVE_PLATFORM_FPGA_H
#define _SIFIVE_PLATFORM_FPGA_H

// #include "const.h"
// #include "devices/clint.h"
// #include "devices/gpio.h"
// #include "devices/plic.h"
// #include "devices/spi.h"
// #include "devices/uart.h"

#include "sifive/const.h"
#include "sifive/devices/ccache.h"
#include "sifive/devices/clint.h"
#include "sifive/devices/ememoryotp.h"
#include "sifive/devices/gpio.h"
#include "sifive/devices/i2c.h"
#include "sifive/devices/spi.h"
#include "sifive/devices/uart.h"
#include "sifive/devices/ux00prci.h"
#include "tl_clock.h"

 // Some things missing from the official encoding.h
#if __riscv_xlen == 32
  #define MCAUSE_INT         0x80000000UL
  #define MCAUSE_CAUSE       0x7FFFFFFFUL
#else
   #define MCAUSE_INT         0x8000000000000000UL
   #define MCAUSE_CAUSE       0x7FFFFFFFFFFFFFFFUL
#endif

/****************************************************************************
 * Platform definitions
 *****************************************************************************/

// CPU info
#define NUM_CORES 6
#define MAX_HART_ID 6
#define GLOBAL_INT_SIZE 39
#define GLOBAL_INT_MAX_PRIORITY 7
#define RTC_FREQUENCY_HZ TL_CLK
#define RTC_PERIOD_NS 1000/TL_CLK

// Memory map
#define AXI_PCIE_HOST_1_00_A_CTRL_ADDR _AC(0x50000000,UL)
#define AXI_PCIE_HOST_1_00_A_CTRL_SIZE _AC(0x4000000,UL)
#define CCACHE_SIDEBAND_ADDR _AC(0x90000000,UL)
#define CCACHE_SIDEBAND_SIZE _AC(0x1e0000,UL)
#define CLINT_CTRL_ADDR _AC(0x2000000,UL)
#define CLINT_CTRL_SIZE _AC(0x10000,UL)
#define DEBUG_CTRL_ADDR _AC(0x0,UL)
#define DEBUG_CTRL_SIZE _AC(0x1000,UL)
#define ERROR_MEM_ADDR _AC(0x3000,UL)
#define ERROR_MEM_SIZE _AC(0x1000,UL)
#define GPIO0_CTRL_ADDR _AC(0x64002000,UL)
#define GPIO0_CTRL_SIZE _AC(0x1000,UL)
#define GPIO1_CTRL_ADDR _AC(0x64007000,UL)
#define GPIO1_CTRL_SIZE _AC(0x1000,UL)
#define MASKROM_MEM_ADDR _AC(0x78000000,UL)
#define MASKROM_MEM_SIZE _AC(0x10000,UL)
#define MEMORY_MEM_ADDR _AC(0x80000000,UL)
#define MEMORY_MEM_SIZE _AC(0x100000000,UL)
#define PLIC_CTRL_ADDR _AC(0xc000000,UL)
#define PLIC_CTRL_SIZE _AC(0x4000000,UL)
#define SPI0_CTRL_ADDR _AC(0x64001000,UL)
#define SPI0_CTRL_SIZE _AC(0x1000,UL)
#define SPI1_CTRL_ADDR _AC(0x64004000,UL)
#define SPI1_CTRL_SIZE _AC(0x1000,UL)
#define TEST_CTRL_ADDR _AC(0x4000,UL)
#define TEST_CTRL_SIZE _AC(0x1000,UL)
#define UART0_CTRL_ADDR _AC(0x64000000,UL)
#define UART0_CTRL_SIZE _AC(0x1000,UL)
#define UART1_CTRL_ADDR _AC(0x64003000,UL)
#define UART1_CTRL_SIZE _AC(0x1000,UL)
#define UART2_CTRL_ADDR _AC(0x64006000,UL)
#define UART2_CTRL_SIZE _AC(0x1000,UL)
#define NVDLA_CTRL_ADDT _AC(0x10040000,UL)
#define NVDLA_CTRL_SIZE _AC(0x40000,UL)
// IOF masks

#ifndef SPI_CTRL_ADDR
  #define SPI_CTRL_ADDR SPI0_CTRL_ADDR
#endif
#ifndef SPI_CTRL_SIZE
  #define SPI_CTRL_SIZE SPI0_CTRL_SIZE
#endif

#ifndef UART_CTRL_ADDR
  #define UART_CTRL_ADDR UART1_CTRL_ADDR
#endif
#ifndef UART_CTRL_SIZE
  #define UART_CTRL_SIZE UART1_CTRL_SIZE
#endif

// Interrupt numbers
#define I2C_INT_BASE 1
#define UART0_INT_BASE 2
#define UART1_INT_BASE 3
#define UART2_INT_BASE 4
#define GPIO0_INT_BASE 5
#define GPIO1_INT_BASE 26
#define SPI0_INT_BASE 36
#define SPI1_INT_BASE 37
#define NVDLA_INT_BASE 38
#define AXI_PCIE_HOST_1_00_A_INT_BASE 39

#ifndef SPI_INT_BASE
  #define SPI_INT_BASE SPI0_INT_BASE
#endif
#ifndef UART_INT_BASE
  #define UART_INT_BASE UART0_INT_BASE
#endif

// Helper functions
#define _REG64(p, i) (*(volatile uint64_t *)((p) + (i)))
#define _REG32(p, i) (*(volatile uint32_t *)((p) + (i)))
#define _REG16(p, i) (*(volatile uint16_t *)((p) + (i)))
// Bulk set bits in `reg` to either 0 or 1.
// E.g. SET_BITS(MY_REG, 0x00000007, 0) would generate MY_REG &= ~0x7
// E.g. SET_BITS(MY_REG, 0x00000007, 1) would generate MY_REG |= 0x7
#define SET_BITS(reg, mask, value) if ((value) == 0) { (reg) &= ~(mask); } else { (reg) |= (mask); }

// #if __riscv_xlen == 32
  #define AXI_PCIE_HOST_1_00_A_REG(offset) _REG32(AXI_PCIE_HOST_1_00_A_CTRL_ADDR, offset)
  #define CLINT_REG(offset) _REG32(CLINT_CTRL_ADDR, offset)
  #define DEBUG_REG(offset) _REG32(DEBUG_CTRL_ADDR, offset)
  #define ERROR_REG(offset) _REG32(ERROR_CTRL_ADDR, offset)
  #define GPIO0_REG(offset) _REG32(GPIO0_CTRL_ADDR, offset)
  #define GPIO1_REG(offset) _REG32(GPIO1_CTRL_ADDR, offset)
  #define MASKROM_REG(offset) _REG32(MASKROM_CTRL_ADDR, offset)
  #define MEMORY_REG(offset) _REG32(MEMORY_CTRL_ADDR, offset)
  #define PLIC_REG(offset) _REG32(PLIC_CTRL_ADDR, offset)
  #define SPI0_REG(offset) _REG32(SPI0_CTRL_ADDR, offset)
  #define SPI1_REG(offset) _REG32(SPI1_CTRL_ADDR, offset)
  #define TEST_REG(offset) _REG32(TEST_CTRL_ADDR, offset)
  #define UART0_REG(offset) _REG32(UART0_CTRL_ADDR, offset)
  #define UART1_REG(offset) _REG32(UART1_CTRL_ADDR, offset)
  #define UART2_REG(offset) _REG32(UART2_CTRL_ADDR, offset)
// #else
  #define AXI_PCIE_HOST_1_00_A_REG64(offset) _REG64(AXI_PCIE_HOST_1_00_A_CTRL_ADDR, offset)
  #define CLINT_REG64(offset) _REG64(CLINT_CTRL_ADDR, offset)
  #define DEBUG_REG64(offset) _REG64(DEBUG_CTRL_ADDR, offset)
  #define ERROR_REG64(offset) _REG64(ERROR_CTRL_ADDR, offset)
  #define GPIO0_REG64(offset) _REG64(GPIO0_CTRL_ADDR, offset)
  #define GPIO1_REG64(offset) _REG64(GPIO1_CTRL_ADDR, offset)
  #define MASKROM_REG64(offset) _REG64(MASKROM_CTRL_ADDR, offset)
  #define MEMORY_REG64(offset) _REG64(MEMORY_CTRL_ADDR, offset)
  #define PLIC_REG64(offset) _REG64(PLIC_CTRL_ADDR, offset)
  #define SPI0_REG64(offset) _REG64(SPI0_CTRL_ADDR, offset)
  #define SPI1_REG64(offset) _REG64(SPI1_CTRL_ADDR, offset)
  #define TEST_REG64(offset) _REG64(TEST_CTRL_ADDR, offset)
  #define UART0_REG64(offset) _REG64(UART0_CTRL_ADDR, offset)
  #define UART1_REG64(offset) _REG64(UART1_CTRL_ADDR, offset)
  #define UART2_REG64(offset) _REG64(UART2_CTRL_ADDR, offset)
// #endif
// Misc

#ifndef SPI_REG
  #define SPI_REG(offset) SPI0_REG(offset)
#endif
#ifndef SPI_REG64
  #define SPI_REG64(offset) SPI0_REG64(offset)
#endif

#ifndef UART_REG
  #define UART_REG(offset) UART0_REG(offset)
#endif
#ifndef UART_REG64
  #define UART_REG64(offset) UART0_REG64(offset)
#endif

#ifndef GPIO_REG
  #define GPIO_REG(offset) GPIO0_REG(offset)
#endif
#ifndef GPIO_REG64
  #define GPIO_REG64(offset) GPIO0_REG64(offset)
#endif

// Helpers for getting and setting individual bit fields, shifting the values
// for you.
#define GET_FIELD(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define SET_FIELD(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

// Misc
#define ALOE 
#define SPI0_CS_WIDTH 1
#define SPI0_SCKDIV_WIDTH 16

#endif /* _SIFIVE_PLATFORM_H */
