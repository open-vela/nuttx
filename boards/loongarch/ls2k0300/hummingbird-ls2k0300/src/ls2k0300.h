/****************************************************************************
 * boards/loongarch/ls2k0300/hummingbird-ls2k0300/src/ls2k0300.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __LS2K0300_H
#define __LS2K0300_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdio.h>

#include "ls_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Address conversion macros */

#define UNCACHED_MEMORY_ADDR        0x8000000000000000UL
#define CACHED_MEMORY_ADDR          0x9000000000000000UL
#define PHYS_TO_UNCACHED(x)         (UNCACHED_MEMORY_ADDR | (unsigned long)(x))
#define PHYS_TO_CACHED(x)           (CACHED_MEMORY_ADDR | (unsigned long)(x))

#define REF_CLK_120M

#define CORE_FREQ         1000  /* CPU 1000MHz */
#define DDR_FREQ          800   /* MEM 800Mhz */
#define APB_FREQ          200   /* SB 100~200MHz, for BOOT, USB, APB, SDIO */
#define NET_FREQ          200   /* NETWORK 200~400MHz, for NETWORK, DC */

/* CHIP CONFIG regs */
#define LS_GENERAL_CFG0              PHYS_TO_UNCACHED(0x16000100)
#define LS_GENERAL_CFG1              PHYS_TO_UNCACHED(0x16000104)
#define LS_GENERAL_CFG2              PHYS_TO_UNCACHED(0x16000108)
#define LS_GENERAL_CFG3              PHYS_TO_UNCACHED(0x1600010c)
#define LS_GENERAL_CFG4              PHYS_TO_UNCACHED(0x16000110)
#define LS_GENERAL_CFG5              PHYS_TO_UNCACHED(0x16000114)
#define LS_GENERAL_CFG6              PHYS_TO_UNCACHED(0x16000118)
#define LS_GENERAL_CFG7              PHYS_TO_UNCACHED(0x1600011c)
#define LS_GENERAL_CFG8              PHYS_TO_UNCACHED(0x16000120)
#define LS_GENERAL_CFG9              PHYS_TO_UNCACHED(0x16000124)
#define LS_GENERAL_CFG10             PHYS_TO_UNCACHED(0x16000128)
#define LS_GENERAL_CFG11             PHYS_TO_UNCACHED(0x1600012c)
#define LS_GENERAL_CFG12             PHYS_TO_UNCACHED(0x16000130)
#define LS_GENERAL_CFG13             PHYS_TO_UNCACHED(0x16000134)
#define LS_GENERAL_CFG14             PHYS_TO_UNCACHED(0x16000138)
#define LS_GENERAL_CFG15             PHYS_TO_UNCACHED(0x1600013c)
#define LS_SAMPLE_CFG0               PHYS_TO_UNCACHED(0x16000140)
#define LS_SAMPLE_CFG1               PHYS_TO_UNCACHED(0x16000144)
#define LS_SAMPLE_CFG2               PHYS_TO_UNCACHED(0x16000148)
#define LS_SAMPLE_CFG3               PHYS_TO_UNCACHED(0x1600014c)
#define LS_CHIP_HPT_LO               PHYS_TO_UNCACHED(0x16000150)
#define LS_CHIP_HPT_HI               PHYS_TO_UNCACHED(0x16000154)

#define LS_NODE_PLL_L                PHYS_TO_UNCACHED(0x16000400)
#define LS_NODE_PLL_H                PHYS_TO_UNCACHED(0x16000404)
#define LS_DDR_PLL_L                 PHYS_TO_UNCACHED(0x16000408)
#define LS_DDR_PLL_H                 PHYS_TO_UNCACHED(0x1600040c)
#define LS_PIX0_PLL                  PHYS_TO_UNCACHED(0x16000410)
#define LS_PIX1_PLL                  PHYS_TO_UNCACHED(0x16000414)
#define LS_FREQ_SCALE                PHYS_TO_UNCACHED(0x16000420)

#define NODE_L1DIV_OUT_SHIFT         24
#define NODE_L1DIV_LOOPC_SHIFT       15
#define NODE_L1DIV_REF_SHIFT         8
#define NODE_L1DIV_OUT_WIDTH         7
#define NODE_L1DIV_LOOPC_WIDTH       9
#define NODE_L1DIV_REF_WIDTH         7
#define NODE_L1DIV_OUT_MARK          0x7f
#define NODE_L1DIV_LOOPC_MARK        0x1ff
#define NODE_L1DIV_REF_MARK          0x7f

#define NODE_L2DIV_OUT_I2S_SHIFT     8
#define NODE_L2DIV_OUT_GMAC_SHIFT    0
#define NODE_L2DIV_OUT_I2S_WIDTH     7
#define NODE_L2DIV_OUT_GMAC_WIDTH    7
#define NODE_L2DIV_OUT_I2S_MARK      0x7f
#define NODE_L2DIV_OUT_GMAC_MARK     0x7f

#define DDR_L1DIV_OUT_SHIFT          24
#define DDR_L1DIV_LOOPC_SHIFT        15
#define DDR_L1DIV_REF_SHIFT          8
#define DDR_L1DIV_OUT_WIDTH          7
#define DDR_L1DIV_LOOPC_WIDTH        9
#define DDR_L1DIV_REF_WIDTH          7
#define DDR_L1DIV_OUT_MARK           0x7f
#define DDR_L1DIV_LOOPC_MARK         0x1ff
#define DDR_L1DIV_REF_MARK           0x7f

#define DDR_L2DIV_OUT_MEMDIV_SHIFT   18
#define DDR_L2DIV_OUT_DEV_SHIFT      8
#define DDR_L2DIV_OUT_NET_SHIFT      0
#define DDR_L2DIV_OUT_MEMDIV_WIDTH   7
#define DDR_L2DIV_OUT_DEV_WIDTH      7
#define DDR_L2DIV_OUT_NET_WIDTH      7
#define DDR_L2DIV_OUT_MEMDIV_MARK    0x3
#define DDR_L2DIV_OUT_DEV_MARK       0x7f
#define DDR_L2DIV_OUT_NET_MARK       0x7f

#define PIX_L1DIV_OUT_SHIFT          24
#define PIX_L1DIV_LOOPC_SHIFT        15
#define PIX_L1DIV_REF_SHIFT          8
#define PIX_L1DIV_OUT_WIDTH          7
#define PIX_L1DIV_LOOPC_WIDTH        9
#define PIX_L1DIV_REF_WIDTH          7
#define PIX_L1DIV_OUT_MARK           0x7f
#define PIX_L1DIV_LOOPC_MARK         0x1ff
#define PIX_L1DIV_REF_MARK           0x7f

#define FREQSCALE_SDIO_SHIFT         24
#define FREQSCALE_I2S_SHIFT          20
#define FREQSCALE_APB_SHIFT          16
#define FREQSCALE_USB_SHIFT          12
#define FREQSCALE_BOOT_SHIFT         8
#define FREQSCALE_PIX_SHIFT          4
#define FREQSCALE_NODE_SHIFT         0
#define FREQSCALE_SDIO_MARK          0x7
#define FREQSCALE_I2S_MARK           0x7
#define FREQSCALE_APB_MARK           0x7
#define FREQSCALE_USB_MARK           0x7
#define FREQSCALE_BOOT_MARK          0x7
#define FREQSCALE_PIX_MARK           0x7
#define FREQSCALE_NODE_MARK          0x7

#define LS_GPIO_00_31_DIR            PHYS_TO_UNCACHED(0x16004000)
#define LS_GPIO_32_63_DIR            PHYS_TO_UNCACHED(0x16004000)
#define LS_GPIO_64_95_DIR            PHYS_TO_UNCACHED(0x16004000)
#define LS_GPIO_96_105_DIR           PHYS_TO_UNCACHED(0x16004000)
#define LS_GPIO_00_31_IN             PHYS_TO_UNCACHED(0x16004000)
#define LS_GPIO_32_63_IN             PHYS_TO_UNCACHED(0x16004000)
#define LS_GPIO_00_31_OUT            PHYS_TO_UNCACHED(0x16004000)
#define LS_GPIO_32_63_OUT            PHYS_TO_UNCACHED(0x16004000)

#define LS_GPIO_0_15_MULTI_CFG       PHYS_TO_UNCACHED(0x16000490)
#define LS_GPIO_16_31_MULTI_CFG      PHYS_TO_UNCACHED(0x16000494)
#define LS_GPIO_32_47_MULTI_CFG      PHYS_TO_UNCACHED(0x16000498)
#define LS_GPIO_48_63_MULTI_CFG      PHYS_TO_UNCACHED(0x1600049c)
#define LS_GPIO_64_79_MULTI_CFG      PHYS_TO_UNCACHED(0x160004a0)
#define LS_GPIO_80_95_MULTI_CFG      PHYS_TO_UNCACHED(0x160004a4)
#define LS_GPIO_96_105_MULTI_CFG     PHYS_TO_UNCACHED(0x160004a8)

#define LS_PIXCLK0_CTRL_REG          PHYS_TO_UNCACHED(0x16000410)
#define LS_PIXCLK1_CTRL_REG          PHYS_TO_UNCACHED(0x16000414)

#define PIXCLK_CTRL0_PSTDIV_SET      (1 << 31)
#define PIXCLK_CTRL0_PSTDIV_PD       (1 << 30)
#define PIXCLK_CTRL0_PSTDIV_DF       (0x3f << 24)
#define PIXCLK_CTRL0_PLL_PD          (1 << 7)
#define PIXCLK_CTRL0_PLL_LDF         (0xff << 16)
#define PIXCLK_CTRL0_PLL_ODF         (0x3 << 5)
#define PIXCLK_CTRL0_PLL_IDF         (0x7 << 2)
#define PIXCLK_CTRL0_REF_SEL         0x3

#define writel_reg_bit(addr, clear_bit, bit_val)  ((*(volatile unsigned int*)(addr)) = ((*(volatile unsigned int*)(addr)) & (~clear_bit)) | (bit_val))

/* OTG regs */

/* USB regs */
#define LS_EHCI_BASE                 PHYS_TO_UNCACHED(0x16080000)
#define LS_XHCI_BASE                 PHYS_TO_UNCACHED(0x16088000)

/* GMAC regs */
#define LS_GMAC0_REG_BASE            PHYS_TO_UNCACHED(0x16020000)
#define LS_GMAC0_LIO_IRQ             49

/* HDA regs */

/* GPU regs */

/* DC regs */
#define LS_DC_BASE                   PHYS_TO_UNCACHED(0x16090000)
#define LS_FB_CFG_DVO_REG            (0x1240)
#define LS_FB_CFG_VGA_REG            (0x1250)
#define LS_FB_ADDR0_DVO_REG          (0x1260)
#define LS_FB_ADDR0_VGA_REG          (0x1270)
#define LS_FB_STRI_DVO_REG           (0x1280)
#define LS_FB_STRI_VGA_REG           (0x1290)
#define LS_FB_ADDR1_DVO_REG          (0x1580)
#define LS_FB_ADDR1_VGA_REG          (0x1590)

#define LS_FB_CUR_CFG_REG            (0x1520)
#define LS_FB_CUR_ADDR_REG           (0x1530)
#define LS_FB_CUR_LOC_ADDR_REG       (0x1540)
#define LS_FB_CUR_BACK_REG           (0x1550)
#define LS_FB_CUR_FORE_REG           (0x1560)

#define LS_FB_DAC_CTRL_REG           (0x1600)

/* SPI regs */
#define LS_SPI0_BASE                 PHYS_TO_UNCACHED(0x16010000)
#define LS_SPI1_BASE                 PHYS_TO_UNCACHED(0x16018000)
#define LS_SPI2_BASE                 PHYS_TO_UNCACHED(0x1610c000)
#define LS_SPI3_BASE                 PHYS_TO_UNCACHED(0x1610e000)

/* UART regs */
#define LS_UART0_REG_BASE            PHYS_TO_UNCACHED(0x16100000)
#define LS_UART1_REG_BASE            PHYS_TO_UNCACHED(0x16100400)
#define LS_UART2_REG_BASE            PHYS_TO_UNCACHED(0x16100800)
#define LS_UART3_REG_BASE            PHYS_TO_UNCACHED(0x16100c00)
#define LS_UART4_REG_BASE            PHYS_TO_UNCACHED(0x16101000)
#define LS_UART5_REG_BASE            PHYS_TO_UNCACHED(0x16101400)
#define LS_UART6_REG_BASE            PHYS_TO_UNCACHED(0x16101800)
#define LS_UART7_REG_BASE            PHYS_TO_UNCACHED(0x16101c00)
#define LS_UART8_REG_BASE            PHYS_TO_UNCACHED(0x16102000)
#define LS_UART9_REG_BASE            PHYS_TO_UNCACHED(0x16102400)

/* UART register offsets relative to base address */
#define LS2K_UART_DAT_OFFSET (0)
#define LS2K_UART_IER_OFFSET (1)
#define LS2K_UART_IIR_OFFSET (2)
#define LS2K_UART_FCR_OFFSET (2)
#define LS2K_UART_LCR_OFFSET (3)
#define LS2K_UART_MCR_OFFSET (4)
#define LS2K_UART_LSR_OFFSET (5)
#define LS2K_UART_MSR_OFFSET (6)

#define LS2K_UART_LSB_OFFSET (0)  /* Divisor latch LSB */
#define LS2K_UART_MSB_OFFSET (1)  /* Divisor latch MSB */

/* UART line status register bit fields */
#define LS2K_UART_LSR_TE (1 << 6)
#define LS2K_UART_LSR_TFE (1 << 5)

/* interrupt enable register */
#define IER_IRxE 0x1    /* Received data available interrupt enable */
#define IER_ITxE 0x2    /* Transmitter holding register empty interrupt enable */
#define IER_ILE 0x4     /* Receiver line status interrupt enable */
#define IER_IME 0x8     /* Modem status interrupt enable */

#define IIR_RXTOUT 0xc
#define IIR_RXRDY 0x4
#define LSR_RXRDY 0x01

#define LCR_SPB (1 << 5)
#define LCR_EPS (1 << 4)
#define LCR_PE (1 << 3)
#define LCR_SB (1 << 2)
#define LCR_BEC (0)

/* UART baud rate */
#define UART_REF_CLK (APB_FREQ * MHZ_TO_HZ)  /* MHz */
#define UART_BAUDRATE (115200)               /* Baud rate */
#define UART_DIV_HI (((UART_REF_CLK + (UART_BAUDRATE * 8)) / (UART_BAUDRATE * 16)) >> 8)
#define UART_DIV_LO (((UART_REF_CLK + (UART_BAUDRATE * 8)) / (UART_BAUDRATE * 16)) & 0xff)

/* I2C regs */

/* APB configured addr 0x1fe0, i2c0 addr is 0x1fe01000 */
#define LS_I2C0_REG_BASE             PHYS_TO_UNCACHED(0x16108000)
#define LS_I2C1_REG_BASE             PHYS_TO_UNCACHED(0x16109000)
#define LS_I2C2_REG_BASE             PHYS_TO_UNCACHED(0x1610a000)
#define LS_I2C3_REG_BASE             PHYS_TO_UNCACHED(0x1610b000)

#define MAX_I2C_NUM 3
#define I2C_REF_CLK (APB_FREQ * MHZ_TO_HZ)  /* MHz */

#define I2C_CR1 (0x00)
#define I2C_CR2 (0x04)
#define I2C_OAR (0x08)
#define I2C_DR (0x10)
#define I2C_SR1 (0x14)
#define I2C_SR2 (0x18)
#define I2C_CCR (0x1c)
#define I2C_TRISE (0x20)

#define I2C_CR1_SWRST (15)
#define I2C_CR1_RECOVER (14)
#define I2C_CR1_POS (11)
#define I2C_CR1_ACK (10)
#define I2C_CR1_STOP (9)
#define I2C_CR1_START (8)
#define I2C_CR1_PE (0)

#define I2C_CR2_FREQ_MASK (0x3f)

/* Loongson I2C Status 1 */
#define I2C_SR1_AF (10)
#define I2C_SR1_ARLO (9)
#define I2C_SR1_BERR (8)
#define I2C_SR1_TXE (7)
#define I2C_SR1_RXNE (6)
#define I2C_SR1_BTF (2)
#define I2C_SR1_ADDR (1)
#define I2C_SR1_SB (0)

#define I2C_SR1_ITEVTEN_MASK ((1 << I2C_SR1_BTF) | (1 << I2C_SR1_ADDR) | (1 << I2C_SR1_SB))
#define I2C_SR1_ITBUFEN_MASK ((1 << I2C_SR1_TXE) | (1 << I2C_SR1_RXNE))
#define I2C_SR1_ITERREN_MASK ((1 << I2C_SR1_AF)  | (1 << I2C_SR1_ARLO) | (1 << I2C_SR1_BERR))

/* Loongson I2C Status 2 */
#define I2C_SR2_BUSY (1)

#define I2C_CCR_F_S (15)
#define I2C_CCR_DUTY (14)
#define I2C_CCR_CCR_MASK (0xfff)

#define I2C_TRISE_TRISE_MASK (0x3f)

/* PWM regs */
#define LS_PWM0_REG_BASE             PHYS_TO_UNCACHED(0x1611b000)
#define LS_PWM1_REG_BASE             PHYS_TO_UNCACHED(0x1611b010)
#define LS_PWM2_REG_BASE             PHYS_TO_UNCACHED(0x1611b020)
#define LS_PWM3_REG_BASE             PHYS_TO_UNCACHED(0x1611b030)

#define PWM_LOW_BUFFER (0x4)
#define PWM_FULL_BUFFER (0x8)
#define PWM_CTRL (0xc)

#define PWM_INTE (5)
#define PWM_RST (7)
#define PWM_CAPTE (8)
#define PWM_SINGLE (4)  /* Single pulse control bit */
#define PWM_INT (6)
#define PWM_OE (3)
#define PWM_EN (0)
/**
 * @brief Bit position for PWM inversion control
 *
 * Defines the bit position in PWM control register that controls
 * the inversion of PWM output signal polarity.
 */
#define PWM_INVERT (9)
#define PWM_DZONE (10)

#define PWM_REF_CLK (APB_FREQ * MHZ_TO_HZ)

/* SDIO regs */
#define LS_SDIO0_BASE                PHYS_TO_UNCACHED(0x1ff64000)
#define LS_SDIO1_BASE                PHYS_TO_UNCACHED(0x1ff66000)

/* HPET regs */
#define LS_HPET0_BASE                PHYS_TO_UNCACHED(0x16120000)
#define LS_HPET0_PERIOD              (LS_HPET0_BASE + 0x4)
#define LS_HPET0_CONF                (LS_HPET0_BASE + 0x10)
#define LS_HPET0_MAIN                (LS_HPET0_BASE + 0xf0)

#define LS_HPET1_BASE                PHYS_TO_UNCACHED(0x16121000)
#define LS_HPET2_BASE                PHYS_TO_UNCACHED(0x16122000)
#define LS_HPET3_BASE                PHYS_TO_UNCACHED(0x16123000)

/* I2S regs */
#define LS_I2S_REG_BASE              PHYS_TO_UNCACHED(0x16114000)

/* ACPI regs */

/* not use acpi */
#define NOT_USE_ACPI

#define LS_MAIN_WDT                  PHYS_TO_UNCACHED(0x16124000)
#define LS_RST_CNT_REG               (LS_MAIN_WDT)
#define LS_WD_SET_REG                (LS_MAIN_WDT + 0x4)
#define LS_WD_TIMER_REG              (LS_MAIN_WDT + 0x8)

/* DMA regs */
#define LS_DMA_ORDER0                PHYS_TO_UNCACHED(0x16000c00)
#define LS_DMA_ORDER1                PHYS_TO_UNCACHED(0x16000c10)
#define LS_DMA_ORDER2                PHYS_TO_UNCACHED(0x16000c20)
#define LS_DMA_ORDER3                PHYS_TO_UNCACHED(0x16000c30)

/* GPIO */
#define LS_GPIO_MAX                  105
#define LS_GPIO_B_DIR_BASE           PHYS_TO_UNCACHED(0x16104800)
#define LS_GPIO_B_OUT_BASE           PHYS_TO_UNCACHED(0x16104900)
#define LS_GPIO_B_IN_BASE            PHYS_TO_UNCACHED(0x16104a00)

/* RTC regs */
#define LS_RTC_REG_BASE              PHYS_TO_UNCACHED(0x16128000)
#define LS_TOY_TRIM_REG              (LS_RTC_REG_BASE + 0x0020)
#define LS_TOY_WRITE0_REG            (LS_RTC_REG_BASE + 0x0024)
#define LS_TOY_WRITE1_REG            (LS_RTC_REG_BASE + 0x0028)
#define LS_TOY_READ0_REG             (LS_RTC_REG_BASE + 0x002c)
#define LS_TOY_READ1_REG             (LS_RTC_REG_BASE + 0x0030)
#define LS_TOY_MATCH0_REG            (LS_RTC_REG_BASE + 0x0034)
#define LS_TOY_MATCH1_REG            (LS_RTC_REG_BASE + 0x0038)
#define LS_TOY_MATCH2_REG            (LS_RTC_REG_BASE + 0x003c)
#define LS_RTC_CTRL_REG              (LS_RTC_REG_BASE + 0x0040)
#define LS_RTC_TRIM_REG              (LS_RTC_REG_BASE + 0x0060)
#define LS_RTC_WRITE0_REG            (LS_RTC_REG_BASE + 0x0064)
#define LS_RTC_READ0_REG             (LS_RTC_REG_BASE + 0x0068)
#define LS_RTC_MATCH0_REG            (LS_RTC_REG_BASE + 0x006c)
#define LS_RTC_MATCH1_REG            (LS_RTC_REG_BASE + 0x0070)
#define LS_RTC_MATCH2_REG            (LS_RTC_REG_BASE + 0x0074)

/* ADC */
#define LS_ADC_REG_BASE              PHYS_TO_UNCACHED(0x1611c000)

#define SR    0x00
#define CR1   0x04
#define CR2   0x08
#define SMPR1 0x0c
#define SMPR2 0x10
#define JOFR1 0x14
#define JOFR2 0x18
#define JOFR3 0x1c
#define JOFR4 0x20
#define HTR   0x24
#define LTR   0x28
#define SQR1  0x2c
#define SQR2  0x30
#define SQR3  0x34
#define JSQR  0x38
#define JDR1  0x3c
#define JDR2  0x40
#define JDR3  0x44
#define JDR4  0x48
#define DR    0x4c

#define LS_ADC_SR_REG                (LS_ADC_REG_BASE + 0x0)
#define LS_ADC_CR1_REG               (LS_ADC_REG_BASE + 0x4)
#define LS_ADC_CR2_REG               (LS_ADC_REG_BASE + 0x8)
#define LS_ADC_SMPR1_REG             (LS_ADC_REG_BASE + 0x0c)
#define LS_ADC_SMPR2_REG             (LS_ADC_REG_BASE + 0x10)
#define LS_ADC_JOFR1_REG             (LS_ADC_REG_BASE + 0x14)
#define LS_ADC_JOFR2_REG             (LS_ADC_REG_BASE + 0x18)
#define LS_ADC_JOFR3_REG             (LS_ADC_REG_BASE + 0x1c)
#define LS_ADC_JOFR4_REG             (LS_ADC_REG_BASE + 0x20)
#define LS_ADC_HTR_REG               (LS_ADC_REG_BASE + 0x24)
#define LS_ADC_LTR_REG               (LS_ADC_REG_BASE + 0x28)
#define LS_ADC_SQR1_REG              (LS_ADC_REG_BASE + 0x2c)
#define LS_ADC_SQR2_REG              (LS_ADC_REG_BASE + 0x30)
#define LS_ADC_SQR3_REG              (LS_ADC_REG_BASE + 0x34)
#define LS_ADC_JSQR_REG              (LS_ADC_REG_BASE + 0x38)
#define LS_ADC_JDR1_REG              (LS_ADC_REG_BASE + 0x3c)
#define LS_ADC_JDR2_REG              (LS_ADC_REG_BASE + 0x40)
#define LS_ADC_JDR3_REG              (LS_ADC_REG_BASE + 0x44)
#define LS_ADC_JDR4_REG              (LS_ADC_REG_BASE + 0x48)
#define LS_ADC_DR_REG                (LS_ADC_REG_BASE + 0x4c)

/* SCACHE */
#define LS_SCACHE_LOCK_WIN0_BASE     PHYS_TO_UNCACHED(0x16000200)
#define LS_SCACHE_LOCK_WIN1_BASE     PHYS_TO_UNCACHED(0x16000208)
#define LS_SCACHE_LOCK_WIN2_BASE     PHYS_TO_UNCACHED(0x16000210)
#define LS_SCACHE_LOCK_WIN3_BASE     PHYS_TO_UNCACHED(0x16000218)
#define LS_SCACHE_LOCK_WIN0_MASK     PHYS_TO_UNCACHED(0x16000240)
#define LS_SCACHE_LOCK_WIN1_MASK     PHYS_TO_UNCACHED(0x16000248)
#define LS_SCACHE_LOCK_WIN2_MASK     PHYS_TO_UNCACHED(0x16000250)
#define LS_SCACHE_LOCK_WIN3_MASK     PHYS_TO_UNCACHED(0x16000258)

/* cfg4 pad */
#define PAD_CTRL_JTAG  8
#define PAD_CTRL_DVO   10
#define PAD_CTRL_UART  12
#define PAD_CTRL_GMAC  14
#define PAD_CTRL_SDIO  16
#define PAD_CTRL_SPI   18
#define PAD_CTRL_I2S   20
#define PAD_CTRL_TIMER 22
#define PAD_CTRL_USB   24
#define PAD_CTRL_EMMC  26

/* Core INT regs */
#define CORE_IPISR_REG   PHYS_TO_UNCACHED(0x16001000)  /* RO NA Core IPI_Status register */
#define CORE_IPIEN_REG   PHYS_TO_UNCACHED(0x16001004)  /* RW 0x0 Core IPI_Enable register */
#define CORE_IPISET_REG  PHYS_TO_UNCACHED(0x16001008)  /* WO NA Core IPI_Set register */
#define CORE_IPI_CLR_REG PHYS_TO_UNCACHED(0x1600100c)  /* WO NA Core IPI_Clear register */
#define CORE_INTISR0_REG PHYS_TO_UNCACHED(0x16001040)  /* Low 32-bit interrupt status routed to core */
#define CORE_INTISR1_REG PHYS_TO_UNCACHED(0x16001048)  /* High 32-bit interrupt status routed to core */

/* LIO INT regs */
#define INTISR_0_REG  PHYS_TO_UNCACHED(0x16001420)  /* Low 32-bit interrupt status register */
#define INTSET_0_REG  PHYS_TO_UNCACHED(0x16001428)  /* Low 32-bit interrupt set/enable register */
#define INTIEN_0_REG  PHYS_TO_UNCACHED(0x16001424)  /* Low 32-bit interrupt enable status register */
#define INTPOL_0_REG  PHYS_TO_UNCACHED(0x16001430)  /* Low 32-bit polarity register (level interrupt) */
#define INTEDGE_0_REG PHYS_TO_UNCACHED(0x16001434)  /* Low 32-bit trigger mode register */
#define INTCLR_0_REG  PHYS_TO_UNCACHED(0x1600142c)  /* Low 32-bit interrupt clear register */

#define INTISR_1_REG  PHYS_TO_UNCACHED(0x16001460)  /* High 32-bit interrupt status register */
#define INTSET_1_REG  PHYS_TO_UNCACHED(0x16001468)  /* High 32-bit interrupt set/enable register */
#define INTIEN_1_REG  PHYS_TO_UNCACHED(0x16001464)  /* High 32-bit interrupt enable status register */
#define INTPOL_1_REG  PHYS_TO_UNCACHED(0x16001470)  /* High 32-bit polarity register (level interrupt) */
#define INTEDGE_1_REG PHYS_TO_UNCACHED(0x16001474)  /* High 32-bit trigger mode register */
#define INTCLR_1_REG  PHYS_TO_UNCACHED(0x1600146c)  /* High 32-bit interrupt clear register */

#define ENTRY0_0_REG  PHYS_TO_UNCACHED(0x16001400)  /* 8-bit interrupt routing register [0--7] */
#define ENTRY8_0_REG  PHYS_TO_UNCACHED(0x16001408)  /* 8-bit interrupt routing register [8--15] */
#define ENTRY16_0_REG PHYS_TO_UNCACHED(0x16001410)  /* 8-bit interrupt routing register [16--23] */
#define ENTRY24_0_REG PHYS_TO_UNCACHED(0x16001418)  /* 8-bit interrupt routing register [24--31] */

#define ENTRY0_1_REG  PHYS_TO_UNCACHED(0x16001440)  /* 8-bit interrupt routing register [32--39] */
#define ENTRY8_1_REG  PHYS_TO_UNCACHED(0x16001448)  /* 8-bit interrupt routing register [40--47] */
#define ENTRY16_1_REG PHYS_TO_UNCACHED(0x16001450)  /* 8-bit interrupt routing register [48--55] */
#define ENTRY24_1_REG PHYS_TO_UNCACHED(0x16001458)  /* 8-bit interrupt routing register [56--63] */

/* S3 Need
 *
 * #define STR_XBAR_CONFIG_NODE_a0(OFFSET, BASE, MASK, MMAP) \
 *         daddi   v0, t0, OFFSET;     \
 *         dli     t1, BASE;           \
 *         or      t1, t1, a0;         \
 *         sd      t1, 0x00(v0);       \
 *         dli     t1, MASK;           \
 *         sd      t1, 0x40(v0);       \
 *         dli     t1, MMAP;           \
 *         sd      t1, 0x80(v0);
 */

#endif /* __LS2K0300_H */
