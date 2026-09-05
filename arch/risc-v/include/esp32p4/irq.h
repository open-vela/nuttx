/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <nuttx/config.h>

#define LP_RTC_INTR_SOURCE                    0
#define LP_WDT_INTR_SOURCE                    1
#define LP_TIMER_REG0_INTR_SOURCE             2
#define LP_TIMER_REG1_INTR_SOURCE             3
#define MB_HP_INTR_SOURCE                     4
#define MB_LP_INTR_SOURCE                     5
#define PMU_0_INTR_SOURCE                     6
#define PMU_1_INTR_SOURCE                     7
#define LP_ANAPERI_INTR_SOURCE                8
#define LP_ADC_INTR_SOURCE                    9
#define LP_GPIO_INTR_SOURCE                   10
#define LP_I2C_INTR_SOURCE                    11
#define LP_I2S_INTR_SOURCE                    12
#define LP_SPI_INTR_SOURCE                    13
#define LP_TOUCH_INTR_SOURCE                  14
#define LP_TSENS_INTR_SOURCE                  15
#define TEMPERATURE_SENSOR_INTR_SOURCE        15
#define LP_UART_INTR_SOURCE                   16
#define LP_EFUSE_INTR_SOURCE                  17
#define LP_SW_INTR_SOURCE                     18
#define LP_SYSREG_INTR_SOURCE                 19
#define LP_HUK_INTR_SOURCE                    20
#define SYS_ICM_INTR_SOURCE                   21
#define USB_SERIAL_JTAG_INTR_SOURCE           22
#define SDIO_HOST_INTR_SOURCE                 23
#define DW_GDMA_INTR_SOURCE                   24
#define SPI2_INTR_SOURCE                      25
#define SPI3_INTR_SOURCE                      26
#define I2S0_INTR_SOURCE                      27
#define I2S1_INTR_SOURCE                      28
#define I2S2_INTR_SOURCE                      29
#define UHCI0_INTR_SOURCE                     30
#define UART0_INTR_SOURCE                     31
#define UART1_INTR_SOURCE                     32
#define UART2_INTR_SOURCE                     33
#define UART3_INTR_SOURCE                     34
#define UART4_INTR_SOURCE                     35
#define LCD_CAM_INTR_SOURCE                   36
#define ADC_INTR_SOURCE                       37
#define PWM0_INTR_SOURCE                      38
#define PWM1_INTR_SOURCE                      39
#define TWAI0_INTR_SOURCE                     40
#define TWAI1_INTR_SOURCE                     41
#define TWAI2_INTR_SOURCE                     42
#define RMT_INTR_SOURCE                       43
#define I2C0_INTR_SOURCE                      44
#define I2C1_INTR_SOURCE                      45
#define TG0_T0_INTR_SOURCE                    46
#define TG0_T1_INTR_SOURCE                    47
#define TG0_WDT_LEVEL_INTR_SOURCE             48
#define TG1_T0_INTR_SOURCE                    49
#define TG1_T1_INTR_SOURCE                    50
#define TG1_WDT_LEVEL_INTR_SOURCE             51
#define LEDC_INTR_SOURCE                      52
#define SYSTIMER_TARGET0_INTR_SOURCE          53
#define SYSTIMER_TARGET1_INTR_SOURCE          54
#define SYSTIMER_TARGET2_INTR_SOURCE          55
#define AHB_PDMA_IN_CH0_INTR_SOURCE           56
#define AHB_PDMA_IN_CH1_INTR_SOURCE           57
#define AHB_PDMA_IN_CH2_INTR_SOURCE           58
#define AHB_PDMA_OUT_CH0_INTR_SOURCE          59
#define AHB_PDMA_OUT_CH1_INTR_SOURCE          60
#define AHB_PDMA_OUT_CH2_INTR_SOURCE          61
#define AXI_PDMA_IN_CH0_INTR_SOURCE           62
#define AXI_PDMA_IN_CH1_INTR_SOURCE           63
#define AXI_PDMA_IN_CH2_INTR_SOURCE           64
#define AXI_PDMA_OUT_CH0_INTR_SOURCE          65
#define AXI_PDMA_OUT_CH1_INTR_SOURCE          66
#define AXI_PDMA_OUT_CH2_INTR_SOURCE          67
#define RSA_INTR_SOURCE                       68
#define AES_INTR_SOURCE                       69
#define SHA_INTR_SOURCE                       70
#define ECC_INTR_SOURCE                       71
#define ECDSA_INTR_SOURCE                     72
#define KM_INTR_SOURCE                        73
#define GPIO_INTR0_SOURCE                     74
#define GPIO_INTR1_SOURCE                     75
#define GPIO_INTR2_SOURCE                     76
#define GPIO_INTR3_SOURCE                     77
#define GPIO_PAD_COMP_INTR_SOURCE             78
#define FROM_CPU_INTR0_SOURCE                 79
#define FROM_CPU_INTR1_SOURCE                 80
#define FROM_CPU_INTR2_SOURCE                 81
#define FROM_CPU_INTR3_SOURCE                 82
#define CACHE_INTR_SOURCE                     83
#define MSPI_INTR_SOURCE                      84
#define CSI_BRIDGE_INTR_SOURCE                85
#define DSI_BRIDGE_INTR_SOURCE                86
#define CSI_INTR_SOURCE                       87
#define DSI_INTR_SOURCE                       88
#define GMII_PHY_INTR_SOURCE                  89
#define LPI_INTR_SOURCE                       90
#define PMT_INTR_SOURCE                       91
#define ETH_MAC_INTR_SOURCE                   92
#define USB_OTG_INTR_SOURCE                   93
#define USB_OTG_ENDP_MULTI_PROC_INTR_SOURCE   94
#define JPEG_INTR_SOURCE                      95
#define PPA_INTR_SOURCE                       96
#define CORE0_TRACE_INTR_SOURCE               97
#define CORE1_TRACE_INTR_SOURCE               98
#define HP_CORE_CTRL_INTR_SOURCE              99
#define ISP_INTR_SOURCE                       100
#define I3C_MST_INTR_SOURCE                   101
#define I3C_SLV_INTR_SOURCE                   102
#define USB_OTG11_CH0_INTR_SOURCE             103
#define DMA2D_IN_CH0_INTR_SOURCE              104
#define DMA2D_IN_CH1_INTR_SOURCE              105
#define DMA2D_OUT_CH0_INTR_SOURCE             106
#define DMA2D_OUT_CH1_INTR_SOURCE             107
#define DMA2D_OUT_CH2_INTR_SOURCE             108
#define PSRAM_MSPI_INTR_SOURCE                109
#define HP_SYSREG_INTR_SOURCE                 110
#define PCNT_INTR_SOURCE                      111
#define HP_PAU_INTR_SOURCE                    112
#define HP_PARLIO_RX_INTR_SOURCE              113
#define HP_PARLIO_TX_INTR_SOURCE              114
#define H264_DMA2D_OUT_CH0_INTR_SOURCE        115
#define H264_DMA2D_OUT_CH1_INTR_SOURCE        116
#define H264_DMA2D_OUT_CH2_INTR_SOURCE        117
#define H264_DMA2D_OUT_CH3_INTR_SOURCE        118
#define H264_DMA2D_OUT_CH4_INTR_SOURCE        119
#define H264_DMA2D_IN_CH0_INTR_SOURCE         120
#define H264_DMA2D_IN_CH1_INTR_SOURCE         121
#define H264_DMA2D_IN_CH2_INTR_SOURCE         122
#define H264_DMA2D_IN_CH3_INTR_SOURCE         123
#define H264_DMA2D_IN_CH4_INTR_SOURCE         124
#define H264_DMA2D_IN_CH5_INTR_SOURCE         125
#define H264_REG_INTR_SOURCE                  126
#define ASSIST_DEBUG_INTR_SOURCE              127

/* Total number of peripherals interrupt sources */

#define ESP_NSOURCES                          128

/* CPU Interrupts */

#define ESP_NCPUINTS                          32

/* The following interrupts are reserved:
 *   - 6, 15 and 15, for TIMER0, TIMER1 and TIMER2
 *   - 7, 29 and 11, for SW0, SW1 and PROFILING
 */

#define ESP_CPUINT_PERIPHSET       0xdffe773f

/* IRQ numbers. */

/* ecall is dispatched like normal interrupts.  It occupies an IRQ number. */

#define RISCV_NIRQ_INTERRUPTS      16  /* Number of RISC-V dispatched interrupts. */
#define ESP_NIRQ_DEMUX             1                                        /* Number of IRQs for the demapping interrupt */
#define ESP_IRQ_FIRSTPERIPH        RISCV_NIRQ_INTERRUPTS + ESP_NIRQ_DEMUX   /* First peripheral IRQ number */

/* Special IRQ for the demapping interrupt */

#define ESP_IRQ_DEMUX              RISCV_NIRQ_INTERRUPTS

/* IRQ numbers for peripheral interrupts coming through the Interrupt
 * Matrix.
 */

#define ESP_IRQ2SOURCE(irq)        ((irq) - ESP_IRQ_FIRSTPERIPH)
#define ESP_SOURCE2IRQ(id)         ((id) + ESP_IRQ_FIRSTPERIPH)

/* Peripheral IRQs */

#define ESP_IRQ_LP_RTC                  ESP_SOURCE2IRQ(LP_RTC_INTR_SOURCE)
#define ESP_IRQ_LP_WDT                  ESP_SOURCE2IRQ(LP_WDT_INTR_SOURCE)
#define ESP_IRQ_LP_TIMER_REG0           ESP_SOURCE2IRQ(LP_TIMER_REG0_INTR_SOURCE)
#define ESP_IRQ_LP_TIMER_REG1           ESP_SOURCE2IRQ(LP_TIMER_REG1_INTR_SOURCE)
#define ESP_IRQ_MB_HP                   ESP_SOURCE2IRQ(MB_HP_INTR_SOURCE)
#define ESP_IRQ_MB_LP                   ESP_SOURCE2IRQ(MB_LP_INTR_SOURCE)
#define ESP_IRQ_PMU_0                   ESP_SOURCE2IRQ(PMU_0_INTR_SOURCE)
#define ESP_IRQ_PMU_1                   ESP_SOURCE2IRQ(PMU_1_INTR_SOURCE)
#define ESP_IRQ_LP_ANAPERI              ESP_SOURCE2IRQ(LP_ANAPERI_INTR_SOURCE)
#define ESP_IRQ_LP_ADC                  ESP_SOURCE2IRQ(LP_ADC_INTR_SOURCE)
#define ESP_IRQ_LP_GPIO                 ESP_SOURCE2IRQ(LP_GPIO_INTR_SOURCE)
#define ESP_IRQ_LP_I2C                  ESP_SOURCE2IRQ(LP_I2C_INTR_SOURCE)
#define ESP_IRQ_LP_I2S                  ESP_SOURCE2IRQ(LP_I2S_INTR_SOURCE)
#define ESP_IRQ_LP_SPI                  ESP_SOURCE2IRQ(LP_SPI_INTR_SOURCE)
#define ESP_IRQ_LP_TOUCH                ESP_SOURCE2IRQ(LP_TOUCH_INTR_SOURCE)
#define ESP_IRQ_LP_TSENS                ESP_SOURCE2IRQ(LP_TSENS_INTR_SOURCE)
#define ESP_IRQ_TEMPERATURE_SENSOR      ESP_SOURCE2IRQ(TEMPERATURE_SENSOR_INTR_SOURCE)
#define ESP_IRQ_LP_UART                 ESP_SOURCE2IRQ(LP_UART_INTR_SOURCE)
#define ESP_IRQ_LP_EFUSE                ESP_SOURCE2IRQ(LP_EFUSE_INTR_SOURCE)
#define ESP_IRQ_LP_SW                   ESP_SOURCE2IRQ(LP_SW_INTR_SOURCE)
#define ESP_IRQ_LP_SYSREG               ESP_SOURCE2IRQ(LP_SYSREG_INTR_SOURCE)
#define ESP_IRQ_LP_HUK                  ESP_SOURCE2IRQ(LP_HUK_INTR_SOURCE)
#define ESP_IRQ_SYS_ICM                 ESP_SOURCE2IRQ(SYS_ICM_INTR_SOURCE)
#define ESP_IRQ_USB_SERIAL_JTAG         ESP_SOURCE2IRQ(USB_SERIAL_JTAG_INTR_SOURCE)
#define ESP_IRQ_SDIO_HOST               ESP_SOURCE2IRQ(SDIO_HOST_INTR_SOURCE)
#define ESP_IRQ_DW_GDMA                 ESP_SOURCE2IRQ(DW_GDMA_INTR_SOURCE)
#define ESP_IRQ_SPI2                    ESP_SOURCE2IRQ(SPI2_INTR_SOURCE)
#define ESP_IRQ_SPI3                    ESP_SOURCE2IRQ(SPI3_INTR_SOURCE)
#define ESP_IRQ_I2S0                    ESP_SOURCE2IRQ(I2S0_INTR_SOURCE)
#define ESP_IRQ_I2S1                    ESP_SOURCE2IRQ(I2S1_INTR_SOURCE)
#define ESP_IRQ_I2S2                    ESP_SOURCE2IRQ(I2S2_INTR_SOURCE)
#define ESP_IRQ_UHCI0                   ESP_SOURCE2IRQ(UHCI0_INTR_SOURCE)
#define ESP_IRQ_UART0                   ESP_SOURCE2IRQ(UART0_INTR_SOURCE)
#define ESP_IRQ_UART1                   ESP_SOURCE2IRQ(UART1_INTR_SOURCE)
#define ESP_IRQ_UART2                   ESP_SOURCE2IRQ(UART2_INTR_SOURCE)
#define ESP_IRQ_UART3                   ESP_SOURCE2IRQ(UART3_INTR_SOURCE)
#define ESP_IRQ_UART4                   ESP_SOURCE2IRQ(UART4_INTR_SOURCE)
#define ESP_IRQ_LCD_CAM                 ESP_SOURCE2IRQ(LCD_CAM_INTR_SOURCE)
#define ESP_IRQ_ADC                     ESP_SOURCE2IRQ(ADC_INTR_SOURCE)
#define ESP_IRQ_PWM0                    ESP_SOURCE2IRQ(PWM0_INTR_SOURCE)
#define ESP_IRQ_PWM1                    ESP_SOURCE2IRQ(PWM1_INTR_SOURCE)
#define ESP_IRQ_TWAI0                   ESP_SOURCE2IRQ(TWAI0_INTR_SOURCE)
#define ESP_IRQ_TWAI1                   ESP_SOURCE2IRQ(TWAI1_INTR_SOURCE)
#define ESP_IRQ_TWAI2                   ESP_SOURCE2IRQ(TWAI2_INTR_SOURCE)
#define ESP_IRQ_RMT                     ESP_SOURCE2IRQ(RMT_INTR_SOURCE)
#define ESP_IRQ_I2C0                    ESP_SOURCE2IRQ(I2C0_INTR_SOURCE)
#define ESP_IRQ_I2C1                    ESP_SOURCE2IRQ(I2C1_INTR_SOURCE)
#define ESP_IRQ_TG0_T0                  ESP_SOURCE2IRQ(TG0_T0_INTR_SOURCE)
#define ESP_IRQ_TG0_T1                  ESP_SOURCE2IRQ(TG0_T1_INTR_SOURCE)
#define ESP_IRQ_TG0_WDT_LEVEL           ESP_SOURCE2IRQ(TG0_WDT_LEVEL_INTR_SOURCE)
#define ESP_IRQ_TG1_T0                  ESP_SOURCE2IRQ(TG1_T0_INTR_SOURCE)
#define ESP_IRQ_TG1_T1                  ESP_SOURCE2IRQ(TG1_T1_INTR_SOURCE)
#define ESP_IRQ_TG1_WDT_LEVEL           ESP_SOURCE2IRQ(TG1_WDT_LEVEL_INTR_SOURCE)
#define ESP_IRQ_LEDC                    ESP_SOURCE2IRQ(LEDC_INTR_SOURCE)
#define ESP_IRQ_SYSTIMER_TARGET0        ESP_SOURCE2IRQ(SYSTIMER_TARGET0_INTR_SOURCE)
#define ESP_IRQ_SYSTIMER_TARGET1        ESP_SOURCE2IRQ(SYSTIMER_TARGET1_INTR_SOURCE)
#define ESP_IRQ_SYSTIMER_TARGET2        ESP_SOURCE2IRQ(SYSTIMER_TARGET2_INTR_SOURCE)
#define ESP_IRQ_AHB_PDMA_IN_CH0         ESP_SOURCE2IRQ(AHB_PDMA_IN_CH0_INTR_SOURCE)
#define ESP_IRQ_AHB_PDMA_IN_CH1         ESP_SOURCE2IRQ(AHB_PDMA_IN_CH1_INTR_SOURCE)
#define ESP_IRQ_AHB_PDMA_IN_CH2         ESP_SOURCE2IRQ(AHB_PDMA_IN_CH2_INTR_SOURCE)
#define ESP_IRQ_AHB_PDMA_OUT_CH0        ESP_SOURCE2IRQ(AHB_PDMA_OUT_CH0_INTR_SOURCE)
#define ESP_IRQ_AHB_PDMA_OUT_CH1        ESP_SOURCE2IRQ(AHB_PDMA_OUT_CH1_INTR_SOURCE)
#define ESP_IRQ_AHB_PDMA_OUT_CH2        ESP_SOURCE2IRQ(AHB_PDMA_OUT_CH2_INTR_SOURCE)
#define ESP_IRQ_AXI_PDMA_IN_CH0         ESP_SOURCE2IRQ(AXI_PDMA_IN_CH0_INTR_SOURCE)
#define ESP_IRQ_AXI_PDMA_IN_CH1         ESP_SOURCE2IRQ(AXI_PDMA_IN_CH1_INTR_SOURCE)
#define ESP_IRQ_AXI_PDMA_IN_CH2         ESP_SOURCE2IRQ(AXI_PDMA_IN_CH2_INTR_SOURCE)
#define ESP_IRQ_AXI_PDMA_OUT_CH0        ESP_SOURCE2IRQ(AXI_PDMA_OUT_CH0_INTR_SOURCE)
#define ESP_IRQ_AXI_PDMA_OUT_CH1        ESP_SOURCE2IRQ(AXI_PDMA_OUT_CH1_INTR_SOURCE)
#define ESP_IRQ_AXI_PDMA_OUT_CH2        ESP_SOURCE2IRQ(AXI_PDMA_OUT_CH2_INTR_SOURCE)
#define ESP_IRQ_RSA                     ESP_SOURCE2IRQ(RSA_INTR_SOURCE)
#define ESP_IRQ_AES                     ESP_SOURCE2IRQ(AES_INTR_SOURCE)
#define ESP_IRQ_SHA                     ESP_SOURCE2IRQ(SHA_INTR_SOURCE)
#define ESP_IRQ_ECC                     ESP_SOURCE2IRQ(ECC_INTR_SOURCE)
#define ESP_IRQ_ECDSA                   ESP_SOURCE2IRQ(ECDSA_INTR_SOURCE)
#define ESP_IRQ_KM                      ESP_SOURCE2IRQ(KM_INTR_SOURCE)
#define ESP_IRQ_GPIO_INTR0              ESP_SOURCE2IRQ(GPIO_INTR0_SOURCE)
#define ESP_IRQ_GPIO_INTR1              ESP_SOURCE2IRQ(GPIO_INTR1_SOURCE)
#define ESP_IRQ_GPIO_INTR2              ESP_SOURCE2IRQ(GPIO_INTR2_SOURCE)
#define ESP_IRQ_GPIO_INTR3              ESP_SOURCE2IRQ(GPIO_INTR3_SOURCE)
#define ESP_IRQ_GPIO_PAD_COMP           ESP_SOURCE2IRQ(GPIO_PAD_COMP_INTR_SOURCE)
#define ESP_IRQ_FROM_CPU_INTR0          ESP_SOURCE2IRQ(FROM_CPU_INTR0_SOURCE)
#define ESP_IRQ_FROM_CPU_INTR1          ESP_SOURCE2IRQ(FROM_CPU_INTR1_SOURCE)
#define ESP_IRQ_FROM_CPU_INTR2          ESP_SOURCE2IRQ(FROM_CPU_INTR2_SOURCE)
#define ESP_IRQ_FROM_CPU_INTR3          ESP_SOURCE2IRQ(FROM_CPU_INTR3_SOURCE)
#define ESP_IRQ_CACHE                   ESP_SOURCE2IRQ(CACHE_INTR_SOURCE)
#define ESP_IRQ_MSPI                    ESP_SOURCE2IRQ(MSPI_INTR_SOURCE)
#define ESP_IRQ_CSI_BRIDGE              ESP_SOURCE2IRQ(CSI_BRIDGE_INTR_SOURCE)
#define ESP_IRQ_DSI_BRIDGE              ESP_SOURCE2IRQ(DSI_BRIDGE_INTR_SOURCE)
#define ESP_IRQ_CSI                     ESP_SOURCE2IRQ(CSI_INTR_SOURCE)
#define ESP_IRQ_DSI                     ESP_SOURCE2IRQ(DSI_INTR_SOURCE)
#define ESP_IRQ_GMII_PHY                ESP_SOURCE2IRQ(GMII_PHY_INTR_SOURCE)
#define ESP_IRQ_LPI                     ESP_SOURCE2IRQ(LPI_INTR_SOURCE)
#define ESP_IRQ_PMT                     ESP_SOURCE2IRQ(PMT_INTR_SOURCE)
#define ESP_IRQ_ETH_MAC                 ESP_SOURCE2IRQ(ETH_MAC_INTR_SOURCE)
#define ESP_IRQ_USB_OTG                 ESP_SOURCE2IRQ(USB_OTG_INTR_SOURCE)
#define ESP_IRQ_USB_OTG_ENDP_MULTI_PROC ESP_SOURCE2IRQ(USB_OTG_ENDP_MULTI_PROC_INTR_SOURCE)
#define ESP_IRQ_JPEG                    ESP_SOURCE2IRQ(JPEG_INTR_SOURCE)
#define ESP_IRQ_PPA                     ESP_SOURCE2IRQ(PPA_INTR_SOURCE)
#define ESP_IRQ_CORE0_TRACE             ESP_SOURCE2IRQ(CORE0_TRACE_INTR_SOURCE)
#define ESP_IRQ_CORE1_TRACE             ESP_SOURCE2IRQ(CORE1_TRACE_INTR_SOURCE)
#define ESP_IRQ_HP_CORE_CTRL            ESP_SOURCE2IRQ(HP_CORE_CTRL_INTR_SOURCE)
#define ESP_IRQ_ISP                     ESP_SOURCE2IRQ(ISP_INTR_SOURCE)
#define ESP_IRQ_I3C_MST                 ESP_SOURCE2IRQ(I3C_MST_INTR_SOURCE)
#define ESP_IRQ_I3C_SLV                 ESP_SOURCE2IRQ(I3C_SLV_INTR_SOURCE)
#define ESP_IRQ_USB_OTG11_CH0           ESP_SOURCE2IRQ(USB_OTG11_CH0_INTR_SOURCE)
#define ESP_IRQ_DMA2D_IN_CH0            ESP_SOURCE2IRQ(DMA2D_IN_CH0_INTR_SOURCE)
#define ESP_IRQ_DMA2D_IN_CH1            ESP_SOURCE2IRQ(DMA2D_IN_CH1_INTR_SOURCE)
#define ESP_IRQ_DMA2D_OUT_CH0           ESP_SOURCE2IRQ(DMA2D_OUT_CH0_INTR_SOURCE)
#define ESP_IRQ_DMA2D_OUT_CH1           ESP_SOURCE2IRQ(DMA2D_OUT_CH1_INTR_SOURCE)
#define ESP_IRQ_DMA2D_OUT_CH2           ESP_SOURCE2IRQ(DMA2D_OUT_CH2_INTR_SOURCE)
#define ESP_IRQ_PSRAM_MSPI              ESP_SOURCE2IRQ(PSRAM_MSPI_INTR_SOURCE)
#define ESP_IRQ_HP_SYSREG               ESP_SOURCE2IRQ(HP_SYSREG_INTR_SOURCE)
#define ESP_IRQ_PCNT                    ESP_SOURCE2IRQ(PCNT_INTR_SOURCE)
#define ESP_IRQ_HP_PAU                  ESP_SOURCE2IRQ(HP_PAU_INTR_SOURCE)
#define ESP_IRQ_HP_PARLIO_RX            ESP_SOURCE2IRQ(HP_PARLIO_RX_INTR_SOURCE)
#define ESP_IRQ_HP_PARLIO_TX            ESP_SOURCE2IRQ(HP_PARLIO_TX_INTR_SOURCE)
#define ESP_IRQ_H264_DMA2D_OUT_CH0      ESP_SOURCE2IRQ(H264_DMA2D_OUT_CH0_INTR_SOURCE)
#define ESP_IRQ_H264_DMA2D_OUT_CH1      ESP_SOURCE2IRQ(H264_DMA2D_OUT_CH1_INTR_SOURCE)
#define ESP_IRQ_H264_DMA2D_OUT_CH2      ESP_SOURCE2IRQ(H264_DMA2D_OUT_CH2_INTR_SOURCE)
#define ESP_IRQ_H264_DMA2D_OUT_CH3      ESP_SOURCE2IRQ(H264_DMA2D_OUT_CH3_INTR_SOURCE)
#define ESP_IRQ_H264_DMA2D_OUT_CH4      ESP_SOURCE2IRQ(H264_DMA2D_OUT_CH4_INTR_SOURCE)
#define ESP_IRQ_H264_DMA2D_IN_CH0       ESP_SOURCE2IRQ(H264_DMA2D_IN_CH0_INTR_SOURCE)
#define ESP_IRQ_H264_DMA2D_IN_CH1       ESP_SOURCE2IRQ(H264_DMA2D_IN_CH1_INTR_SOURCE)
#define ESP_IRQ_H264_DMA2D_IN_CH2       ESP_SOURCE2IRQ(H264_DMA2D_IN_CH2_INTR_SOURCE)
#define ESP_IRQ_H264_DMA2D_IN_CH3       ESP_SOURCE2IRQ(H264_DMA2D_IN_CH3_INTR_SOURCE)
#define ESP_IRQ_H264_DMA2D_IN_CH4       ESP_SOURCE2IRQ(H264_DMA2D_IN_CH4_INTR_SOURCE)
#define ESP_IRQ_H264_DMA2D_IN_CH5       ESP_SOURCE2IRQ(H264_DMA2D_IN_CH5_INTR_SOURCE)
#define ESP_IRQ_H264_REG                ESP_SOURCE2IRQ(H264_REG_INTR_SOURCE)
#define ESP_IRQ_ASSIST_DEBUG            ESP_SOURCE2IRQ(ASSIST_DEBUG_INTR_SOURCE)

#define ESP_NIRQ_PERIPH                 ESP_NSOURCES

/* Second level GPIO interrupts.  GPIO interrupts are decoded and dispatched
 * as a second level of decoding:  The first level dispatches to the GPIO
 * interrupt handler.  The second to the decoded GPIO interrupt handler.
 */

/* TODO: double-check GPIO interrupts */

#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
#  define ESP_NIRQ_GPIO             55
#  define ESP_FIRST_GPIOIRQ         (RISCV_NIRQ_INTERRUPTS + ESP_NIRQ_PERIPH)
#  define ESP_LAST_GPIOIRQ          (ESP_FIRST_GPIOIRQ + ESP_NIRQ_GPIO - 1)
#  define ESP_PIN2IRQ(p)            ((p) + ESP_FIRST_GPIOIRQ)
#  define ESP_IRQ2PIN(i)            ((i) - ESP_FIRST_GPIOIRQ)
#else
#  define ESP_NIRQ_GPIO             0
#endif

/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #pragma once

 #include <nuttx/config.h>

 #define LP_RTC_INTR_SOURCE                    0
 #define LP_WDT_INTR_SOURCE                    1
 #define LP_TIMER_REG0_INTR_SOURCE             2
 #define LP_TIMER_REG1_INTR_SOURCE             3
 #define MB_HP_INTR_SOURCE                     4
 #define MB_LP_INTR_SOURCE                     5
 #define PMU_0_INTR_SOURCE                     6
 #define PMU_1_INTR_SOURCE                     7
 #define LP_ANAPERI_INTR_SOURCE                8
 #define LP_ADC_INTR_SOURCE                    9
 #define LP_GPIO_INTR_SOURCE                   10
 #define LP_I2C_INTR_SOURCE                    11
 #define LP_I2S_INTR_SOURCE                    12
 #define LP_SPI_INTR_SOURCE                    13
 #define LP_TOUCH_INTR_SOURCE                  14
 #define LP_TSENS_INTR_SOURCE                  15
 #define TEMPERATURE_SENSOR_INTR_SOURCE        15
 #define LP_UART_INTR_SOURCE                   16
 #define LP_EFUSE_INTR_SOURCE                  17
 #define LP_SW_INTR_SOURCE                     18
 #define LP_SYSREG_INTR_SOURCE                 19
 #define LP_HUK_INTR_SOURCE                    20
 #define SYS_ICM_INTR_SOURCE                   21
 #define USB_SERIAL_JTAG_INTR_SOURCE           22
 #define SDIO_HOST_INTR_SOURCE                 23
 #define DW_GDMA_INTR_SOURCE                   24
 #define SPI2_INTR_SOURCE                      25
 #define SPI3_INTR_SOURCE                      26
 #define I2S0_INTR_SOURCE                      27
 #define I2S1_INTR_SOURCE                      28
 #define I2S2_INTR_SOURCE                      29
 #define UHCI0_INTR_SOURCE                     30
 #define UART0_INTR_SOURCE                     31
 #define UART1_INTR_SOURCE                     32
 #define UART2_INTR_SOURCE                     33
 #define UART3_INTR_SOURCE                     34
 #define UART4_INTR_SOURCE                     35
 #define LCD_CAM_INTR_SOURCE                   36
 #define ADC_INTR_SOURCE                       37
 #define PWM0_INTR_SOURCE                      38
 #define PWM1_INTR_SOURCE                      39
 #define TWAI0_INTR_SOURCE                     40
 #define TWAI1_INTR_SOURCE                     41
 #define TWAI2_INTR_SOURCE                     42
 #define RMT_INTR_SOURCE                       43
 #define I2C0_INTR_SOURCE                      44
 #define I2C1_INTR_SOURCE                      45
 #define TG0_T0_INTR_SOURCE                    46
 #define TG0_T1_INTR_SOURCE                    47
 #define TG0_WDT_LEVEL_INTR_SOURCE             48
 #define TG1_T0_INTR_SOURCE                    49
 #define TG1_T1_INTR_SOURCE                    50
 #define TG1_WDT_LEVEL_INTR_SOURCE             51
 #define LEDC_INTR_SOURCE                      52
 #define SYSTIMER_TARGET0_INTR_SOURCE          53
 #define SYSTIMER_TARGET1_INTR_SOURCE          54
 #define SYSTIMER_TARGET2_INTR_SOURCE          55
 #define AHB_PDMA_IN_CH0_INTR_SOURCE           56
 #define AHB_PDMA_IN_CH1_INTR_SOURCE           57
 #define AHB_PDMA_IN_CH2_INTR_SOURCE           58
 #define AHB_PDMA_OUT_CH0_INTR_SOURCE          59
 #define AHB_PDMA_OUT_CH1_INTR_SOURCE          60
 #define AHB_PDMA_OUT_CH2_INTR_SOURCE          61
 #define AXI_PDMA_IN_CH0_INTR_SOURCE           62
 #define AXI_PDMA_IN_CH1_INTR_SOURCE           63
 #define AXI_PDMA_IN_CH2_INTR_SOURCE           64
 #define AXI_PDMA_OUT_CH0_INTR_SOURCE          65
 #define AXI_PDMA_OUT_CH1_INTR_SOURCE          66
 #define AXI_PDMA_OUT_CH2_INTR_SOURCE          67
 #define RSA_INTR_SOURCE                       68
 #define AES_INTR_SOURCE                       69
 #define SHA_INTR_SOURCE                       70
 #define ECC_INTR_SOURCE                       71
 #define ECDSA_INTR_SOURCE                     72
 #define KM_INTR_SOURCE                        73
 #define GPIO_INTR0_SOURCE                     74
 #define GPIO_INTR1_SOURCE                     75
 #define GPIO_INTR2_SOURCE                     76
 #define GPIO_INTR3_SOURCE                     77
 #define GPIO_PAD_COMP_INTR_SOURCE             78
 #define FROM_CPU_INTR0_SOURCE                 79
 #define FROM_CPU_INTR1_SOURCE                 80
 #define FROM_CPU_INTR2_SOURCE                 81
 #define FROM_CPU_INTR3_SOURCE                 82
 #define CACHE_INTR_SOURCE                     83
 #define MSPI_INTR_SOURCE                      84
 #define CSI_BRIDGE_INTR_SOURCE                85
 #define DSI_BRIDGE_INTR_SOURCE                86
 #define CSI_INTR_SOURCE                       87
 #define DSI_INTR_SOURCE                       88
 #define GMII_PHY_INTR_SOURCE                  89
 #define LPI_INTR_SOURCE                       90
 #define PMT_INTR_SOURCE                       91
 #define ETH_MAC_INTR_SOURCE                   92
 #define USB_OTG_INTR_SOURCE                   93
 #define USB_OTG_ENDP_MULTI_PROC_INTR_SOURCE   94
 #define JPEG_INTR_SOURCE                      95
 #define PPA_INTR_SOURCE                       96
 #define CORE0_TRACE_INTR_SOURCE               97
 #define CORE1_TRACE_INTR_SOURCE               98
 #define HP_CORE_CTRL_INTR_SOURCE              99
 #define ISP_INTR_SOURCE                       100
 #define I3C_MST_INTR_SOURCE                   101
 #define I3C_SLV_INTR_SOURCE                   102
 #define USB_OTG11_CH0_INTR_SOURCE             103
 #define DMA2D_IN_CH0_INTR_SOURCE              104
 #define DMA2D_IN_CH1_INTR_SOURCE              105
 #define DMA2D_OUT_CH0_INTR_SOURCE             106
 #define DMA2D_OUT_CH1_INTR_SOURCE             107
 #define DMA2D_OUT_CH2_INTR_SOURCE             108
 #define PSRAM_MSPI_INTR_SOURCE                109
 #define HP_SYSREG_INTR_SOURCE                 110
 #define PCNT_INTR_SOURCE                      111
 #define HP_PAU_INTR_SOURCE                    112
 #define HP_PARLIO_RX_INTR_SOURCE              113
 #define HP_PARLIO_TX_INTR_SOURCE              114
 #define H264_DMA2D_OUT_CH0_INTR_SOURCE        115
 #define H264_DMA2D_OUT_CH1_INTR_SOURCE        116
 #define H264_DMA2D_OUT_CH2_INTR_SOURCE        117
 #define H264_DMA2D_OUT_CH3_INTR_SOURCE        118
 #define H264_DMA2D_OUT_CH4_INTR_SOURCE        119
 #define H264_DMA2D_IN_CH0_INTR_SOURCE         120
 #define H264_DMA2D_IN_CH1_INTR_SOURCE         121
 #define H264_DMA2D_IN_CH2_INTR_SOURCE         122
 #define H264_DMA2D_IN_CH3_INTR_SOURCE         123
 #define H264_DMA2D_IN_CH4_INTR_SOURCE         124
 #define H264_DMA2D_IN_CH5_INTR_SOURCE         125
 #define H264_REG_INTR_SOURCE                  126
 #define ASSIST_DEBUG_INTR_SOURCE              127

 /* Total number of peripherals interrupt sources */

 #define ESP_NSOURCES                          128

 /* CPU Interrupts */

 #define ESP_NCPUINTS                          32

 /* The following interrupts are reserved:
  *   - 6, 15 and 15, for TIMER0, TIMER1 and TIMER2
  *   - 7, 29 and 11, for SW0, SW1 and PROFILING
  */

 #define ESP_CPUINT_PERIPHSET       0xdffe773f

 /* IRQ numbers. */

 /* ecall is dispatched like normal interrupts.  It occupies an IRQ number. */

 #define RISCV_NIRQ_INTERRUPTS      16  /* Number of RISC-V dispatched interrupts. */
 #define ESP_NIRQ_DEMUX             1                                        /* Number of IRQs for the demapping interrupt */
 #define ESP_IRQ_FIRSTPERIPH        RISCV_NIRQ_INTERRUPTS + ESP_NIRQ_DEMUX   /* First peripheral IRQ number */

 /* Special IRQ for the demapping interrupt */

 #define ESP_IRQ_DEMUX              RISCV_NIRQ_INTERRUPTS

 /* IRQ numbers for peripheral interrupts coming through the Interrupt
  * Matrix.
  */

 #define ESP_IRQ2SOURCE(irq)        ((irq) - ESP_IRQ_FIRSTPERIPH)
 #define ESP_SOURCE2IRQ(id)         ((id) + ESP_IRQ_FIRSTPERIPH)

 /* Peripheral IRQs */

 #define ESP_IRQ_LP_RTC                  ESP_SOURCE2IRQ(LP_RTC_INTR_SOURCE)
 #define ESP_IRQ_LP_WDT                  ESP_SOURCE2IRQ(LP_WDT_INTR_SOURCE)
 #define ESP_IRQ_LP_TIMER_REG0           ESP_SOURCE2IRQ(LP_TIMER_REG0_INTR_SOURCE)
 #define ESP_IRQ_LP_TIMER_REG1           ESP_SOURCE2IRQ(LP_TIMER_REG1_INTR_SOURCE)
 #define ESP_IRQ_MB_HP                   ESP_SOURCE2IRQ(MB_HP_INTR_SOURCE)
 #define ESP_IRQ_MB_LP                   ESP_SOURCE2IRQ(MB_LP_INTR_SOURCE)
 #define ESP_IRQ_PMU_0                   ESP_SOURCE2IRQ(PMU_0_INTR_SOURCE)
 #define ESP_IRQ_PMU_1                   ESP_SOURCE2IRQ(PMU_1_INTR_SOURCE)
 #define ESP_IRQ_LP_ANAPERI              ESP_SOURCE2IRQ(LP_ANAPERI_INTR_SOURCE)
 #define ESP_IRQ_LP_ADC                  ESP_SOURCE2IRQ(LP_ADC_INTR_SOURCE)
 #define ESP_IRQ_LP_GPIO                 ESP_SOURCE2IRQ(LP_GPIO_INTR_SOURCE)
 #define ESP_IRQ_LP_I2C                  ESP_SOURCE2IRQ(LP_I2C_INTR_SOURCE)
 #define ESP_IRQ_LP_I2S                  ESP_SOURCE2IRQ(LP_I2S_INTR_SOURCE)
 #define ESP_IRQ_LP_SPI                  ESP_SOURCE2IRQ(LP_SPI_INTR_SOURCE)
 #define ESP_IRQ_LP_TOUCH                ESP_SOURCE2IRQ(LP_TOUCH_INTR_SOURCE)
 #define ESP_IRQ_LP_TSENS                ESP_SOURCE2IRQ(LP_TSENS_INTR_SOURCE)
 #define ESP_IRQ_TEMPERATURE_SENSOR      ESP_SOURCE2IRQ(TEMPERATURE_SENSOR_INTR_SOURCE)
 #define ESP_IRQ_LP_UART                 ESP_SOURCE2IRQ(LP_UART_INTR_SOURCE)
 #define ESP_IRQ_LP_EFUSE                ESP_SOURCE2IRQ(LP_EFUSE_INTR_SOURCE)
 #define ESP_IRQ_LP_SW                   ESP_SOURCE2IRQ(LP_SW_INTR_SOURCE)
 #define ESP_IRQ_LP_SYSREG               ESP_SOURCE2IRQ(LP_SYSREG_INTR_SOURCE)
 #define ESP_IRQ_LP_HUK                  ESP_SOURCE2IRQ(LP_HUK_INTR_SOURCE)
 #define ESP_IRQ_SYS_ICM                 ESP_SOURCE2IRQ(SYS_ICM_INTR_SOURCE)
 #define ESP_IRQ_USB_SERIAL_JTAG         ESP_SOURCE2IRQ(USB_SERIAL_JTAG_INTR_SOURCE)
 #define ESP_IRQ_SDIO_HOST               ESP_SOURCE2IRQ(SDIO_HOST_INTR_SOURCE)
 #define ESP_IRQ_DW_GDMA                 ESP_SOURCE2IRQ(DW_GDMA_INTR_SOURCE)
 #define ESP_IRQ_SPI2                    ESP_SOURCE2IRQ(SPI2_INTR_SOURCE)
 #define ESP_IRQ_SPI3                    ESP_SOURCE2IRQ(SPI3_INTR_SOURCE)
 #define ESP_IRQ_I2S0                    ESP_SOURCE2IRQ(I2S0_INTR_SOURCE)
 #define ESP_IRQ_I2S1                    ESP_SOURCE2IRQ(I2S1_INTR_SOURCE)
 #define ESP_IRQ_I2S2                    ESP_SOURCE2IRQ(I2S2_INTR_SOURCE)
 #define ESP_IRQ_UHCI0                   ESP_SOURCE2IRQ(UHCI0_INTR_SOURCE)
 #define ESP_IRQ_UART0                   ESP_SOURCE2IRQ(UART0_INTR_SOURCE)
 #define ESP_IRQ_UART1                   ESP_SOURCE2IRQ(UART1_INTR_SOURCE)
 #define ESP_IRQ_UART2                   ESP_SOURCE2IRQ(UART2_INTR_SOURCE)
 #define ESP_IRQ_UART3                   ESP_SOURCE2IRQ(UART3_INTR_SOURCE)
 #define ESP_IRQ_UART4                   ESP_SOURCE2IRQ(UART4_INTR_SOURCE)
 #define ESP_IRQ_LCD_CAM                 ESP_SOURCE2IRQ(LCD_CAM_INTR_SOURCE)
 #define ESP_IRQ_ADC                     ESP_SOURCE2IRQ(ADC_INTR_SOURCE)
 #define ESP_IRQ_PWM0                    ESP_SOURCE2IRQ(PWM0_INTR_SOURCE)
 #define ESP_IRQ_PWM1                    ESP_SOURCE2IRQ(PWM1_INTR_SOURCE)
 #define ESP_IRQ_TWAI0                   ESP_SOURCE2IRQ(TWAI0_INTR_SOURCE)
 #define ESP_IRQ_TWAI1                   ESP_SOURCE2IRQ(TWAI1_INTR_SOURCE)
 #define ESP_IRQ_TWAI2                   ESP_SOURCE2IRQ(TWAI2_INTR_SOURCE)
 #define ESP_IRQ_RMT                     ESP_SOURCE2IRQ(RMT_INTR_SOURCE)
 #define ESP_IRQ_I2C0                    ESP_SOURCE2IRQ(I2C0_INTR_SOURCE)
 #define ESP_IRQ_I2C1                    ESP_SOURCE2IRQ(I2C1_INTR_SOURCE)
 #define ESP_IRQ_TG0_T0                  ESP_SOURCE2IRQ(TG0_T0_INTR_SOURCE)
 #define ESP_IRQ_TG0_T1                  ESP_SOURCE2IRQ(TG0_T1_INTR_SOURCE)
 #define ESP_IRQ_TG0_WDT_LEVEL           ESP_SOURCE2IRQ(TG0_WDT_LEVEL_INTR_SOURCE)
 #define ESP_IRQ_TG1_T0                  ESP_SOURCE2IRQ(TG1_T0_INTR_SOURCE)
 #define ESP_IRQ_TG1_T1                  ESP_SOURCE2IRQ(TG1_T1_INTR_SOURCE)
 #define ESP_IRQ_TG1_WDT_LEVEL           ESP_SOURCE2IRQ(TG1_WDT_LEVEL_INTR_SOURCE)
 #define ESP_IRQ_LEDC                    ESP_SOURCE2IRQ(LEDC_INTR_SOURCE)
 #define ESP_IRQ_SYSTIMER_TARGET0        ESP_SOURCE2IRQ(SYSTIMER_TARGET0_INTR_SOURCE)
 #define ESP_IRQ_SYSTIMER_TARGET1        ESP_SOURCE2IRQ(SYSTIMER_TARGET1_INTR_SOURCE)
 #define ESP_IRQ_SYSTIMER_TARGET2        ESP_SOURCE2IRQ(SYSTIMER_TARGET2_INTR_SOURCE)
 #define ESP_IRQ_AHB_PDMA_IN_CH0         ESP_SOURCE2IRQ(AHB_PDMA_IN_CH0_INTR_SOURCE)
 #define ESP_IRQ_AHB_PDMA_IN_CH1         ESP_SOURCE2IRQ(AHB_PDMA_IN_CH1_INTR_SOURCE)
 #define ESP_IRQ_AHB_PDMA_IN_CH2         ESP_SOURCE2IRQ(AHB_PDMA_IN_CH2_INTR_SOURCE)
 #define ESP_IRQ_AHB_PDMA_OUT_CH0        ESP_SOURCE2IRQ(AHB_PDMA_OUT_CH0_INTR_SOURCE)
 #define ESP_IRQ_AHB_PDMA_OUT_CH1        ESP_SOURCE2IRQ(AHB_PDMA_OUT_CH1_INTR_SOURCE)
 #define ESP_IRQ_AHB_PDMA_OUT_CH2        ESP_SOURCE2IRQ(AHB_PDMA_OUT_CH2_INTR_SOURCE)
 #define ESP_IRQ_AXI_PDMA_IN_CH0         ESP_SOURCE2IRQ(AXI_PDMA_IN_CH0_INTR_SOURCE)
 #define ESP_IRQ_AXI_PDMA_IN_CH1         ESP_SOURCE2IRQ(AXI_PDMA_IN_CH1_INTR_SOURCE)
 #define ESP_IRQ_AXI_PDMA_IN_CH2         ESP_SOURCE2IRQ(AXI_PDMA_IN_CH2_INTR_SOURCE)
 #define ESP_IRQ_AXI_PDMA_OUT_CH0        ESP_SOURCE2IRQ(AXI_PDMA_OUT_CH0_INTR_SOURCE)
 #define ESP_IRQ_AXI_PDMA_OUT_CH1        ESP_SOURCE2IRQ(AXI_PDMA_OUT_CH1_INTR_SOURCE)
 #define ESP_IRQ_AXI_PDMA_OUT_CH2        ESP_SOURCE2IRQ(AXI_PDMA_OUT_CH2_INTR_SOURCE)
 #define ESP_IRQ_RSA                     ESP_SOURCE2IRQ(RSA_INTR_SOURCE)
 #define ESP_IRQ_AES                     ESP_SOURCE2IRQ(AES_INTR_SOURCE)
 #define ESP_IRQ_SHA                     ESP_SOURCE2IRQ(SHA_INTR_SOURCE)
 #define ESP_IRQ_ECC                     ESP_SOURCE2IRQ(ECC_INTR_SOURCE)
 #define ESP_IRQ_ECDSA                   ESP_SOURCE2IRQ(ECDSA_INTR_SOURCE)
 #define ESP_IRQ_KM                      ESP_SOURCE2IRQ(KM_INTR_SOURCE)
 #define ESP_IRQ_GPIO_INTR0              ESP_SOURCE2IRQ(GPIO_INTR0_SOURCE)
 #define ESP_IRQ_GPIO_INTR1              ESP_SOURCE2IRQ(GPIO_INTR1_SOURCE)
 #define ESP_IRQ_GPIO_INTR2              ESP_SOURCE2IRQ(GPIO_INTR2_SOURCE)
 #define ESP_IRQ_GPIO_INTR3              ESP_SOURCE2IRQ(GPIO_INTR3_SOURCE)
 #define ESP_IRQ_GPIO_PAD_COMP           ESP_SOURCE2IRQ(GPIO_PAD_COMP_INTR_SOURCE)
 #define ESP_IRQ_FROM_CPU_INTR0          ESP_SOURCE2IRQ(FROM_CPU_INTR0_SOURCE)
 #define ESP_IRQ_FROM_CPU_INTR1          ESP_SOURCE2IRQ(FROM_CPU_INTR1_SOURCE)
 #define ESP_IRQ_FROM_CPU_INTR2          ESP_SOURCE2IRQ(FROM_CPU_INTR2_SOURCE)
 #define ESP_IRQ_FROM_CPU_INTR3          ESP_SOURCE2IRQ(FROM_CPU_INTR3_SOURCE)
 #define ESP_IRQ_CACHE                   ESP_SOURCE2IRQ(CACHE_INTR_SOURCE)
 #define ESP_IRQ_MSPI                    ESP_SOURCE2IRQ(MSPI_INTR_SOURCE)
 #define ESP_IRQ_CSI_BRIDGE              ESP_SOURCE2IRQ(CSI_BRIDGE_INTR_SOURCE)
 #define ESP_IRQ_DSI_BRIDGE              ESP_SOURCE2IRQ(DSI_BRIDGE_INTR_SOURCE)
 #define ESP_IRQ_CSI                     ESP_SOURCE2IRQ(CSI_INTR_SOURCE)
 #define ESP_IRQ_DSI                     ESP_SOURCE2IRQ(DSI_INTR_SOURCE)
 #define ESP_IRQ_GMII_PHY                ESP_SOURCE2IRQ(GMII_PHY_INTR_SOURCE)
 #define ESP_IRQ_LPI                     ESP_SOURCE2IRQ(LPI_INTR_SOURCE)
 #define ESP_IRQ_PMT                     ESP_SOURCE2IRQ(PMT_INTR_SOURCE)
 #define ESP_IRQ_ETH_MAC                 ESP_SOURCE2IRQ(ETH_MAC_INTR_SOURCE)
 #define ESP_IRQ_USB_OTG                 ESP_SOURCE2IRQ(USB_OTG_INTR_SOURCE)
 #define ESP_IRQ_USB_OTG_ENDP_MULTI_PROC ESP_SOURCE2IRQ(USB_OTG_ENDP_MULTI_PROC_INTR_SOURCE)
 #define ESP_IRQ_JPEG                    ESP_SOURCE2IRQ(JPEG_INTR_SOURCE)
 #define ESP_IRQ_PPA                     ESP_SOURCE2IRQ(PPA_INTR_SOURCE)
 #define ESP_IRQ_CORE0_TRACE             ESP_SOURCE2IRQ(CORE0_TRACE_INTR_SOURCE)
 #define ESP_IRQ_CORE1_TRACE             ESP_SOURCE2IRQ(CORE1_TRACE_INTR_SOURCE)
 #define ESP_IRQ_HP_CORE_CTRL            ESP_SOURCE2IRQ(HP_CORE_CTRL_INTR_SOURCE)
 #define ESP_IRQ_ISP                     ESP_SOURCE2IRQ(ISP_INTR_SOURCE)
 #define ESP_IRQ_I3C_MST                 ESP_SOURCE2IRQ(I3C_MST_INTR_SOURCE)
 #define ESP_IRQ_I3C_SLV                 ESP_SOURCE2IRQ(I3C_SLV_INTR_SOURCE)
 #define ESP_IRQ_USB_OTG11_CH0           ESP_SOURCE2IRQ(USB_OTG11_CH0_INTR_SOURCE)
 #define ESP_IRQ_DMA2D_IN_CH0            ESP_SOURCE2IRQ(DMA2D_IN_CH0_INTR_SOURCE)
 #define ESP_IRQ_DMA2D_IN_CH1            ESP_SOURCE2IRQ(DMA2D_IN_CH1_INTR_SOURCE)
 #define ESP_IRQ_DMA2D_OUT_CH0           ESP_SOURCE2IRQ(DMA2D_OUT_CH0_INTR_SOURCE)
 #define ESP_IRQ_DMA2D_OUT_CH1           ESP_SOURCE2IRQ(DMA2D_OUT_CH1_INTR_SOURCE)
 #define ESP_IRQ_DMA2D_OUT_CH2           ESP_SOURCE2IRQ(DMA2D_OUT_CH2_INTR_SOURCE)
 #define ESP_IRQ_PSRAM_MSPI              ESP_SOURCE2IRQ(PSRAM_MSPI_INTR_SOURCE)
 #define ESP_IRQ_HP_SYSREG               ESP_SOURCE2IRQ(HP_SYSREG_INTR_SOURCE)
 #define ESP_IRQ_PCNT                    ESP_SOURCE2IRQ(PCNT_INTR_SOURCE)
 #define ESP_IRQ_HP_PAU                  ESP_SOURCE2IRQ(HP_PAU_INTR_SOURCE)
 #define ESP_IRQ_HP_PARLIO_RX            ESP_SOURCE2IRQ(HP_PARLIO_RX_INTR_SOURCE)
 #define ESP_IRQ_HP_PARLIO_TX            ESP_SOURCE2IRQ(HP_PARLIO_TX_INTR_SOURCE)
 #define ESP_IRQ_H264_DMA2D_OUT_CH0      ESP_SOURCE2IRQ(H264_DMA2D_OUT_CH0_INTR_SOURCE)
 #define ESP_IRQ_H264_DMA2D_OUT_CH1      ESP_SOURCE2IRQ(H264_DMA2D_OUT_CH1_INTR_SOURCE)
 #define ESP_IRQ_H264_DMA2D_OUT_CH2      ESP_SOURCE2IRQ(H264_DMA2D_OUT_CH2_INTR_SOURCE)
 #define ESP_IRQ_H264_DMA2D_OUT_CH3      ESP_SOURCE2IRQ(H264_DMA2D_OUT_CH3_INTR_SOURCE)
 #define ESP_IRQ_H264_DMA2D_OUT_CH4      ESP_SOURCE2IRQ(H264_DMA2D_OUT_CH4_INTR_SOURCE)
 #define ESP_IRQ_H264_DMA2D_IN_CH0       ESP_SOURCE2IRQ(H264_DMA2D_IN_CH0_INTR_SOURCE)
 #define ESP_IRQ_H264_DMA2D_IN_CH1       ESP_SOURCE2IRQ(H264_DMA2D_IN_CH1_INTR_SOURCE)
 #define ESP_IRQ_H264_DMA2D_IN_CH2       ESP_SOURCE2IRQ(H264_DMA2D_IN_CH2_INTR_SOURCE)
 #define ESP_IRQ_H264_DMA2D_IN_CH3       ESP_SOURCE2IRQ(H264_DMA2D_IN_CH3_INTR_SOURCE)
 #define ESP_IRQ_H264_DMA2D_IN_CH4       ESP_SOURCE2IRQ(H264_DMA2D_IN_CH4_INTR_SOURCE)
 #define ESP_IRQ_H264_DMA2D_IN_CH5       ESP_SOURCE2IRQ(H264_DMA2D_IN_CH5_INTR_SOURCE)
 #define ESP_IRQ_H264_REG                ESP_SOURCE2IRQ(H264_REG_INTR_SOURCE)
 #define ESP_IRQ_ASSIST_DEBUG            ESP_SOURCE2IRQ(ASSIST_DEBUG_INTR_SOURCE)

 #define ESP_NIRQ_PERIPH                 ESP_NSOURCES

 /* Second level GPIO interrupts.  GPIO interrupts are decoded and dispatched
  * as a second level of decoding:  The first level dispatches to the GPIO
  * interrupt handler.  The second to the decoded GPIO interrupt handler.
  */

 /* TODO: double-check GPIO interrupts */

 #ifdef CONFIG_ESPRESSIF_GPIO_IRQ
 #  define ESP_NIRQ_GPIO             55
 #  define ESP_FIRST_GPIOIRQ         (RISCV_NIRQ_INTERRUPTS + ESP_NIRQ_PERIPH)
 #  define ESP_LAST_GPIOIRQ          (ESP_FIRST_GPIOIRQ + ESP_NIRQ_GPIO - 1)
 #  define ESP_PIN2IRQ(p)            ((p) + ESP_FIRST_GPIOIRQ)
 #  define ESP_IRQ2PIN(i)            ((i) - ESP_FIRST_GPIOIRQ)
 #else
 #  define ESP_NIRQ_GPIO             0
 #endif

/* Total number of IRQs: ecall + Number of peripheral IRQs + GPIOs IRQs. */

#define NR_IRQS  (RISCV_NIRQ_INTERRUPTS + ESP_NIRQ_DEMUX + ESP_NIRQ_PERIPH + ESP_NIRQ_GPIO)

#if CONFIG_ARCH_NUSER_INTERRUPTS < (ESP_IRQ_DEMUX + 1)
#  error "CONFIG_ARCH_NUSER_INTERRUPTS is less than ESP_IRQ_DEMUX + 1. Please increase CONFIG_ARCH_NUSER_INTERRUPTS to at least ESP_IRQ_DEMUX + 1."
#endif
