/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_memorymap.h
 *
 * RK3576 Peripheral Base Addresses
 * Verified against RK3576 TRM where possible.
 * Addresses marked [VERIFY] need hardware confirmation.
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_MEMORYMAP_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_MEMORYMAP_H

#include <nuttx/config.h>

/* RK3576 BUS Peripheral Space (0xff100000 - 0xff3fffff) */

/* UART Controllers (DW APB UART, 12 ports) */

#define RK3576_UART0_ADDR          0xff170000
#define RK3576_UART1_ADDR          0xff180000
#define RK3576_UART2_ADDR          0xff190000  /* Debug console */
#define RK3576_UART3_ADDR          0xff1a0000
#define RK3576_UART4_ADDR          0xff1b0000
/* UART5-UART11 exist but addresses need TRM verification */

/* I2C Controllers */

#define RK3576_I2C0_ADDR           0xff1c0000
#define RK3576_I2C1_ADDR           0xff1d0000
#define RK3576_I2C2_ADDR           0xff1e0000
#define RK3576_I2C3_ADDR           0xff1f0000
#define RK3576_I2C4_ADDR           0xff200000

/* SPI Controllers */

#define RK3576_SPI0_ADDR           0xff210000
#define RK3576_SPI1_ADDR           0xff220000
#define RK3576_SPI2_ADDR           0xff230000
#define RK3576_SPI3_ADDR           0xff240000

/* PWM Controllers */

#define RK3576_PWM0_ADDR           0xff250000
#define RK3576_PWM1_ADDR           0xff260000
#define RK3576_PWM2_ADDR           0xff270000
#define RK3576_PWM3_ADDR           0xff280000

/* Display Controllers */

#define RK3576_VOP0_ADDR           0xff2a0000
#define RK3576_VOP1_ADDR           0xff2b0000
#define RK3576_DSI0_ADDR           0xff2c0000
#define RK3576_DSI1_ADDR           0xff2d0000
#define RK3576_HDMI_ADDR           0xff2e0000

/* I2S Audio Controllers (BUS space, verified from TRM) */

#define RK3576_I2S0_ADDR           0xff2f0000
#define RK3576_I2S1_ADDR           0xff300000  /* [VERIFY] */
#define RK3576_I2S2_ADDR           0xff310000  /* [VERIFY] */

/* USB Controllers */

#define RK3576_USB0_ADDR           0xff320000
#define RK3576_USB1_ADDR           0xff330000

/* SDMMC Controllers */

#define RK3576_SDMMC0_ADDR         0xff340000
#define RK3576_SDMMC1_ADDR         0xff350000
#define RK3576_EMMC_ADDR           0xff360000

/* DMA Controller */

#define RK3576_DMAC0_ADDR          0xff370000  /* [VERIFY] */

/* SPI NOR Flash Controller */

#define RK3576_SPIFLASH_ADDR       0xff380000

/* Timer Controllers */

#define RK3576_TIMER0_ADDR         0xff390000
#define RK3576_TIMER1_ADDR         0xff3a0000

/* Watchdog Timer */

#define RK3576_WDT_ADDR            0xff3b0000

/* SARADC (General Purpose ADC) */

#define RK3576_SARADC_ADDR         0xff3c0000

/* Crypto Engine */

#define RK3576_CRYPTO_ADDR         0xff3d0000

/* NPU (Neural Processing Unit) */

#define RK3576_NPU_ADDR            0xff3e0000

/* GPU (Mali-G51) */

#define RK3576_GPU_ADDR            0xff3f0000

/* MIPI CSI Camera Interfaces (VIO space, separate from I2S) */

#define RK3576_CSI0_ADDR           0xff460000  /* [VERIFY] */
#define RK3576_CSI1_ADDR           0xff470000  /* [VERIFY] */

/* RK3576 PMU/Clock/Register Space (0xff400000+) */

/* PMU (Power Management Unit) */

#define RK3576_PMU_ADDR            0xff400000

/* PMU RTC (inside PMU domain, sub-register offset) */

#define RK3576_RTC_ADDR           0xff400000  /* RTC is inside PMU */
#define RK3576_PMU_RTC_ADDR       0xff400000

/* CRU (Clock and Reset Unit) */

#define RK3576_CRU_ADDR            0xff410000

/* GRF (General Register File) */

#define RK3576_GRF_ADDR            0xff420000

/* PMUGRF */

#define RK3576_PMUGRF_ADDR         0xff430000

/* SysGrf */

#define RK3576_SYSGRF_ADDR         0xff440000

/* HDMI CEC (inside HDMI block, not separate) */

#define RK3576_CEC_ADDR            0xff2e0000  /* Shares HDMI base, offset 0x400+ */

/* S/PDIF Audio Output */

#define RK3576_SPDIF_ADDR          0xff480000  /* [VERIFY] */

/* PDMA (DMA for audio) */

#define RK3576_PDMA0_ADDR          0xff490000  /* [VERIFY] */
#define RK3576_PDMA1_ADDR          0xff4a0000  /* [VERIFY] */

/* Ethernet MAC Controllers (GMAC) */

#define RK3576_ETH0_ADDR           0xfe1c0000
#define RK3576_ETH1_ADDR           0xfe010000

/* Default GPIO base */

#define RK3576_PIO_ADDR            RK3576_GPIO0_ADDR

/* GPIO Controllers */

#define RK3576_GPIO0_ADDR          0xff750000
#define RK3576_GPIO1_ADDR          0xff760000
#define RK3576_GPIO2_ADDR          0xff770000
#define RK3576_GPIO3_ADDR          0xff780000
#define RK3576_GPIO4_ADDR          0xff790000

/* DDR Controller */

#define RK3576_DDRPCTL_ADDR        0xfe000000  /* [VERIFY] */

/* PCIe Controllers (Configuration space) */

#define RK3576_PCIE0_ADDR          0xf8000000
#define RK3576_PCIE1_ADDR          0xf9000000

#endif /* __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_MEMORYMAP_H */
