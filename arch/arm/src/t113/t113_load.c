/****************************************************************************
 * arch/arm/src/t113/t113_load.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMPORTANT: This file must be compiled with:
 *   -fno-lto -O3 -fno-stack-protector
 * It runs from SRAM 0x30000 before DDR is initialized.
 * It must NOT reference any global variables or call libc
 * functions.
 *
 ****************************************************************************/

/* Copyright (C) 2023 Xiaomi Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the
 * License.
 */

/****************************************************************************
 *
 * Licensed to the Apache Software Foundation (ASF) under one or
 * more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information regarding
 * copyright ownership.  The ASF licenses this file to you under
 * the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License.  You may
 * obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied.  See the License for the specific
 * language governing permissions and limitations under the
 * License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef NULL
#define NULL 0
#endif

#define T113_CCU_BASE            (0x02001000)
#define R528_CCU_BASE            T113_CCU_BASE

#define CCU_PLL_CPU_CTRL_REG     (0x000)
#define CCU_PLL_DDR_CTRL_REG     (0x010)
#define CCU_PLL_PERI0_CTRL_REG   (0x020)
#define CCU_PLL_PERI1_CTRL_REG   (0x028)
#define CCU_PLL_GPU_CTRL_REG     (0x030)
#define CCU_PLL_VIDEO0_CTRL_REG  (0x040)
#define CCU_PLL_VIDEO1_CTRL_REG  (0x048)
#define CCU_PLL_VE_CTRL          (0x058)
#define CCU_PLL_DE_CTRL          (0x060)
#define CCU_PLL_HSIC_CTRL        (0x070)
#define CCU_PLL_AUDIO0_CTRL_REG  (0x078)
#define CCU_PLL_AUDIO1_CTRL_REG  (0x080)
#define CCU_PLL_DDR_PAT0_CTRL_REG    (0x110)
#define CCU_PLL_DDR_PAT1_CTRL_REG    (0x114)
#define CCU_PLL_PERI0_PAT0_CTRL_REG  (0x120)
#define CCU_PLL_PERI0_PAT1_CTRL_REG  (0x124)
#define CCU_PLL_PERI1_PAT0_CTRL_REG  (0x128)
#define CCU_PLL_PERI1_PAT1_CTRL_REG  (0x12c)
#define CCU_PLL_GPU_PAT0_CTRL_REG    (0x130)
#define CCU_PLL_GPU_PAT1_CTRL_REG    (0x134)
#define CCU_PLL_VIDEO0_PAT0_CTRL_REG (0x140)
#define CCU_PLL_VIDEO0_PAT1_CTRL_REG (0x144)
#define CCU_PLL_VIDEO1_PAT0_CTRL_REG (0x148)
#define CCU_PLL_VIDEO1_PAT1_CTRL_REG (0x14c)
#define CCU_PLL_VE_PAT0_CTRL_REG     (0x158)
#define CCU_PLL_VE_PAT1_CTRL_REG     (0x15c)
#define CCU_PLL_DE_PAT0_CTRL_REG     (0x160)
#define CCU_PLL_DE_PAT1_CTRL_REG     (0x164)
#define CCU_PLL_HSIC_PAT0_CTRL_REG   (0x170)
#define CCU_PLL_HSIC_PAT1_CTRL_REG   (0x174)
#define CCU_PLL_AUDIO0_PAT0_CTRL_REG (0x178)
#define CCU_PLL_AUDIO0_PAT1_CTRL_REG (0x17c)
#define CCU_PLL_AUDIO1_PAT0_CTRL_REG (0x180)
#define CCU_PLL_AUDIO1_PAT1_CTRL_REG (0x184)
#define CCU_PLL_CPU_BIAS_REG     (0x300)
#define CCU_PLL_DDR_BIAS_REG     (0x310)
#define CCU_PLL_PERI0_BIAS_REG   (0x320)
#define CCU_PLL_PERI1_BIAS_REG   (0x328)
#define CCU_PLL_GPU_BIAS_REG     (0x330)
#define CCU_PLL_VIDEO0_BIAS_REG  (0x340)
#define CCU_PLL_VIDEO1_BIAS_REG  (0x348)
#define CCU_PLL_VE_BIAS_REG      (0x358)
#define CCU_PLL_DE_BIAS_REG      (0x360)
#define CCU_PLL_HSIC_BIAS_REG    (0x370)
#define CCU_PLL_AUDIO0_BIAS_REG  (0x378)
#define CCU_PLL_AUDIO1_BIAS_REG  (0x380)
#define CCU_PLL_CPU_TUN_REG      (0x400)
#define CCU_CPU_AXI_CFG_REG      (0x500)
#define CCU_CPU_GATING_REG       (0x504)
#define CCU_PSI_CLK_REG          (0x510)
#define CCU_AHB3_CLK_REG         (0x51c)
#define CCU_APB0_CLK_REG         (0x520)
#define CCU_APB1_CLK_REG         (0x524)
#define CCU_MBUS_CLK_REG         (0x540)
#define CCU_DMA_BGR_REG          (0x70c)
#define CCU_DRAM_CLK_REG         (0x800)
#define CCU_MBUS_MAT_CLK_GATING_REG (0x804)
#define CCU_DRAM_BGR_REG         (0x80c)
#define CCU_RISCV_CLK_REG        (0xd00)
#define CCU_RISCV_GATING_REG     (0xd04)
#define CCU_RISCV_CFG_BGR_REG    (0xd0c)

#define MCTL_COM_BASE            (0x3102000)
#define MCTL_COM_WORK_MODE0      (0x00)
#define MCTL_COM_WORK_MODE1      (0x04)
#define MCTL_COM_DBGCR           (0x08)
#define MCTL_COM_TMR             (0x0c)
#define MCTL_COM_CCCR            (0x14)
#define MCTL_COM_MAER0           (0x20)
#define MCTL_COM_MAER1           (0x24)
#define MCTL_COM_MAER2           (0x28)
#define MCTL_COM_REMAP0          (0x500)
#define MCTL_COM_REMAP1          (0x504)
#define MCTL_COM_REMAP2          (0x508)
#define MCTL_COM_REMAP3          (0x50c)

#define MCTL_PHY_BASE            (0x3103000)
#define MCTL_PHY_PIR             (0x00)
#define MCTL_PHY_PWRCTL          (0x04)
#define MCTL_PHY_MRCTRL0         (0x08)
#define MCTL_PHY_CLKEN           (0x0c)
#define MCTL_PHY_PGSR0           (0x10)
#define MCTL_PHY_PGSR1           (0x14)
#define MCTL_PHY_STATR           (0x18)
#define MCTL_PHY_LP3MR11         (0x2c)
#define MCTL_PHY_DRAM_MR0        (0x30)
#define MCTL_PHY_DRAM_MR1        (0x34)
#define MCTL_PHY_DRAM_MR2        (0x38)
#define MCTL_PHY_DRAM_MR3        (0x3c)
#define MCTL_PHY_PTR0            (0x44)
#define MCTL_PHY_PTR2            (0x4c)
#define MCTL_PHY_PTR3            (0x50)
#define MCTL_PHY_PTR4            (0x54)
#define MCTL_PHY_DRAMTMG0        (0x58)
#define MCTL_PHY_DRAMTMG1        (0x5c)
#define MCTL_PHY_DRAMTMG2        (0x60)
#define MCTL_PHY_DRAMTMG3        (0x64)
#define MCTL_PHY_DRAMTMG4        (0x68)
#define MCTL_PHY_DRAMTMG5        (0x6c)
#define MCTL_PHY_DRAMTMG6        (0x70)
#define MCTL_PHY_DRAMTMG7        (0x74)
#define MCTL_PHY_DRAMTMG8        (0x78)
#define MCTL_PHY_ODTCFG          (0x7c)
#define MCTL_PHY_PITMG0          (0x80)
#define MCTL_PHY_PITMG1          (0x84)
#define MCTL_PHY_LPTPR           (0x88)
#define MCTL_PHY_RFSHCTL0        (0x8c)
#define MCTL_PHY_RFSHTMG         (0x90)
#define MCTL_PHY_RFSHCTL1        (0x94)
#define MCTL_PHY_PWRTMG          (0x98)
#define MCTL_PHY_ASRC            (0x9c)
#define MCTL_PHY_ASRTC           (0xa0)
#define MCTL_PHY_VTFCR           (0xb8)
#define MCTL_PHY_DQSGMR          (0xbc)
#define MCTL_PHY_DTCR            (0xc0)
#define MCTL_PHY_DTAR0           (0xc4)
#define MCTL_PHY_PGCR0           (0x100)
#define MCTL_PHY_PGCR1           (0x104)
#define MCTL_PHY_PGCR2           (0x108)
#define MCTL_PHY_PGCR3           (0x10c)
#define MCTL_PHY_IOVCR0          (0x110)
#define MCTL_PHY_IOVCR1          (0x114)
#define MCTL_PHY_DXCCR           (0x11c)
#define MCTL_PHY_ODTMAP          (0x120)
#define MCTL_PHY_ZQCTL0          (0x124)
#define MCTL_PHY_ZQCTL1          (0x128)
#define MCTL_PHY_ZQCR            (0x140)
#define MCTL_PHY_ZQSR            (0x144)
#define MCTL_PHY_ZQDR0           (0x148)
#define MCTL_PHY_ZQDR1           (0x14c)
#define MCTL_PHY_ZQDR2           (0x150)
#define MCTL_PHY_SCHED           (0x1c0)
#define MCTL_PHY_PERFHPR0        (0x1c4)
#define MCTL_PHY_PERFHPR1        (0x1c8)
#define MCTL_PHY_PERFLPR0        (0x1cc)
#define MCTL_PHY_PERFLPR1        (0x1d0)
#define MCTL_PHY_PERFWR0         (0x1d4)
#define MCTL_PHY_PERFWR1         (0x1d8)
#define MCTL_PHY_ACMDLR          (0x200)
#define MCTL_PHY_ACLDLR          (0x204)
#define MCTL_PHY_ACIOCR0         (0x208)
#define MCTL_PHY_ACIOCR1(x)     (0x210 + 0x4 * (x))
#define MCTL_PHY_DXNMDLR(x)     (0x300 + 0x80 * (x))
#define MCTL_PHY_DXNLDLR0(x)    (0x304 + 0x80 * (x))
#define MCTL_PHY_DXNLDLR1(x)    (0x308 + 0x80 * (x))
#define MCTL_PHY_DXNLDLR2(x)    (0x30c + 0x80 * (x))
#define MCTL_PHY_DXIOCR          (0x310)
#define MCTL_PHY_DATX0IOCR(x)   (0x310 + 0x4 * (x))
#define MCTL_PHY_DATX1IOCR(x)   (0x390 + 0x4 * (x))
#define MCTL_PHY_DATX2IOCR(x)   (0x410 + 0x4 * (x))
#define MCTL_PHY_DATX3IOCR(x)   (0x490 + 0x4 * (x))
#define MCTL_PHY_DXNSDLR6(x)   (0x33c + 0x80 * (x))
#define MCTL_PHY_DXNGTR(x)     (0x340 + 0x80 * (x))
#define MCTL_PHY_DXNGCR0(x)    (0x344 + 0x80 * (x))
#define MCTL_PHY_DXNGSR0(x)    (0x348 + 0x80 * (x))

#define SYS_CONTROL_REG_BASE     (0x3000000)
#define LDO_CTAL_REG             (0x150)
#define ZQ_CAL_CTRL_REG          (0x160)
#define ZQ_RES_CTRL_REG          (0x168)
#define ZQ_RES_STATUS_REG        (0x16c)

#define SYS_SID_BASE             (0x3006000)
#define SYS_CHIPID_REG           (0x200)
#define SYS_EFUSE_REG            (0x228)
#define SYS_LDOB_SID             (0x21c)

#define R_CPUCFG_BASE            (0x7000400)
#define R_CPUCFG_SUP_STAN_FLAG   (0x1d4)

#define R_PRCM_BASE              (0x7010000)
#define VDD_SYS_PWROFF_GATING    (0x250)
#define ANALOG_PWROFF_GATING     (0x254)

#define clrbits_le32(addr, clear) \
  write32(((uint32_t)(addr)), \
          read32(((uint32_t)(addr))) & ~(clear))

#define setbits_le32(addr, set) \
  write32(((uint32_t)(addr)), \
          read32(((uint32_t)(addr))) | (set))

#define clrsetbits_le32(addr, clear, set) \
  write32(((uint32_t)(addr)), \
    (read32(((uint32_t)(addr))) & ~(clear)) | (set))

#define CONFIG_DRAM_BASE         (0x40000000)
#define DIV_ROUND_UP(n, d)       (((n) + (d)-1) / (d))

#define max(a, b) ((a) > (b) ? (a) : (b))

/* Default spi nand page size: 2048(11), 4096(12) */

#define SPINAND_PAGE_BITS  (11)
#define SPINAND_PAGE_MASK  ((1 << SPINAND_PAGE_BITS) - 1)
#define SPINAND_PAGE_SIZE  (1 << SPINAND_PAGE_BITS)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dram_param
{
  uint32_t dram_clk;
  uint32_t dram_type;
  uint32_t dram_zq;
  uint32_t dram_odt_en;
  uint32_t dram_para1;
  uint32_t dram_para2;
  uint32_t dram_mr0;
  uint32_t dram_mr1;
  uint32_t dram_mr2;
  uint32_t dram_mr3;
  uint32_t dram_tpr0;
  uint32_t dram_tpr1;
  uint32_t dram_tpr2;
  uint32_t dram_tpr3;
  uint32_t dram_tpr4;
  uint32_t dram_tpr5;
  uint32_t dram_tpr6;
  uint32_t dram_tpr7;
  uint32_t dram_tpr8;
  uint32_t dram_tpr9;
  uint32_t dram_tpr10;
  uint32_t dram_tpr11;
  uint32_t dram_tpr12;
  uint32_t dram_tpr13;
  uint32_t reserve[8];
};

enum dram_type
{
  DRAM_TYPE_DDR2 = 2,
  DRAM_TYPE_DDR3 = 3,
  DRAM_TYPE_LPDDR2 = 6,
  DRAM_TYPE_LPDDR3 = 7,
};

enum
{
  SPI_GCR = 0x04,
  SPI_TCR = 0x08,
  SPI_IER = 0x10,
  SPI_ISR = 0x14,
  SPI_FCR = 0x18,
  SPI_FSR = 0x1c,
  SPI_WCR = 0x20,
  SPI_CCR = 0x24,
  SPI_MBC = 0x30,
  SPI_MTC = 0x34,
  SPI_BCC = 0x38,
  SPI_TXD = 0x200,
  SPI_RXD = 0x300,
};

enum
{
  BOOT_DEVICE_SPINOR,
  BOOT_DEVICE_SPINAND,
  BOOT_DEVICE_SDCARD,
};

/****************************************************************************
 * Extern Declarations
 ****************************************************************************/

extern unsigned char __image_start[];
extern unsigned char __image_end[];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline __attribute__((__always_inline__))
uint8_t read8(uint32_t addr)
{
  return (*((volatile uint8_t *)(addr)));
}

static inline __attribute__((__always_inline__))
uint16_t read16(uint32_t addr)
{
  return (*((volatile uint16_t *)(addr)));
}

static inline __attribute__((__always_inline__))
uint32_t read32(uint32_t addr)
{
  return (*((volatile uint32_t *)(addr)));
}

static inline __attribute__((__always_inline__))
void write8(uint32_t addr, uint8_t value)
{
  *((volatile uint8_t *)(addr)) = value;
}

static inline __attribute__((__always_inline__))
void write16(uint32_t addr, uint16_t value)
{
  *((volatile uint16_t *)(addr)) = value;
}

static inline __attribute__((__always_inline__))
void write32(uint32_t addr, uint32_t value)
{
  *((volatile uint32_t *)(addr)) = value;
}

static inline void sdelay(int loops)
{
  __asm__ __volatile__("1:\n"
                       "subs %0, %1, #1\n"
                       "bne 1b"
                       : "=r"(loops)
                       : "0"(loops));
}

static void set_pll_cpux_axi(void)
{
  uint32_t val;

  /* AXI: Select cpu clock src to PLL_PERI(1x) */

  write32(R528_CCU_BASE + CCU_CPU_AXI_CFG_REG,
          (4 << 24) | (1 << 0));
  sdelay(10);

  /* Disable pll gating */

  val = read32(R528_CCU_BASE + CCU_PLL_CPU_CTRL_REG);
  val &= ~(1 << 27);
  write32(R528_CCU_BASE + CCU_PLL_CPU_CTRL_REG, val);

  /* Enable pll ldo */

  val = read32(R528_CCU_BASE + CCU_PLL_CPU_CTRL_REG);
  val |= (1 << 30);
  write32(R528_CCU_BASE + CCU_PLL_CPU_CTRL_REG, val);
  sdelay(5);

  /* Set default clk to 1008mhz */

  val = read32(R528_CCU_BASE + CCU_PLL_CPU_CTRL_REG);
  val &= ~((0x3 << 16) | (0xff << 8) | (0x3 << 0));
  val |= (41 << 8);
  write32(R528_CCU_BASE + CCU_PLL_CPU_CTRL_REG, val);

  /* Lock enable */

  val = read32(R528_CCU_BASE + CCU_PLL_CPU_CTRL_REG);
  val |= (1 << 29);
  write32(R528_CCU_BASE + CCU_PLL_CPU_CTRL_REG, val);

  /* Enable pll */

  val = read32(R528_CCU_BASE + CCU_PLL_CPU_CTRL_REG);
  val |= (1 << 31);
  write32(R528_CCU_BASE + CCU_PLL_CPU_CTRL_REG, val);

  /* Wait pll stable */

  while (!(read32(R528_CCU_BASE +
           CCU_PLL_CPU_CTRL_REG) & (0x1 << 28)))
    ;
  sdelay(20);

  /* Enable pll gating */

  val = read32(R528_CCU_BASE + CCU_PLL_CPU_CTRL_REG);
  val |= (1 << 27);
  write32(R528_CCU_BASE + CCU_PLL_CPU_CTRL_REG, val);

  /* Lock disable */

  val = read32(R528_CCU_BASE + CCU_PLL_CPU_CTRL_REG);
  val &= ~(1 << 29);
  write32(R528_CCU_BASE + CCU_PLL_CPU_CTRL_REG, val);
  sdelay(1);

  /* AXI: set and change cpu clk src to PLL_CPUX,
   * PLL_CPUX:AXI0 = 1200MHz:600MHz
   */

  val = read32(R528_CCU_BASE + CCU_CPU_AXI_CFG_REG);
  val &= ~(0x07 << 24 | 0x3 << 16 | 0x3 << 8 | 0xf << 0);
  val |= (0x03 << 24 | 0x0 << 16 | 0x1 << 8 | 0x1 << 0);
  write32(R528_CCU_BASE + CCU_CPU_AXI_CFG_REG, val);
  sdelay(1);
}

static void set_pll_periph0(void)
{
  uint32_t val;

  /* Periph0 has been enabled */

  if (read32(R528_CCU_BASE +
             CCU_PLL_PERI0_CTRL_REG) & (1 << 31))
    {
      return;
    }

  /* Change psi src to osc24m */

  val = read32(R528_CCU_BASE + CCU_PSI_CLK_REG);
  val &= (~(0x3 << 24));
  write32(val, R528_CCU_BASE + CCU_PSI_CLK_REG);

  /* Set default val */

  write32(R528_CCU_BASE + CCU_PLL_PERI0_CTRL_REG,
          0x63 << 8);

  /* Lock enable */

  val = read32(R528_CCU_BASE + CCU_PLL_PERI0_CTRL_REG);
  val |= (1 << 29);
  write32(R528_CCU_BASE + CCU_PLL_PERI0_CTRL_REG, val);

  /* Enabe pll 600m(1x) 1200m(2x) */

  val = read32(R528_CCU_BASE + CCU_PLL_PERI0_CTRL_REG);
  val |= (1 << 31);
  write32(R528_CCU_BASE + CCU_PLL_PERI0_CTRL_REG, val);

  /* Wait pll stable */

  while (!(read32(R528_CCU_BASE +
           CCU_PLL_PERI0_CTRL_REG) & (0x1 << 28)))
    ;
  sdelay(20);

  /* Lock disable */

  val = read32(R528_CCU_BASE + CCU_PLL_PERI0_CTRL_REG);
  val &= ~(1 << 29);
  write32(R528_CCU_BASE + CCU_PLL_PERI0_CTRL_REG, val);
}

static void set_ahb(void)
{
  write32(R528_CCU_BASE + CCU_PSI_CLK_REG,
          (2 << 0) | (0 << 8));
  write32(R528_CCU_BASE + CCU_PSI_CLK_REG,
          read32(R528_CCU_BASE + CCU_PSI_CLK_REG) |
          (0x03 << 24));
  sdelay(1);
}

static void set_apb(void)
{
  write32(R528_CCU_BASE + CCU_APB0_CLK_REG,
          (2 << 0) | (1 << 8));
  write32(R528_CCU_BASE + CCU_APB0_CLK_REG,
          (0x03 << 24) |
          read32(R528_CCU_BASE + CCU_APB0_CLK_REG));
  sdelay(1);
}

static void set_dma(void)
{
  /* Dma reset */

  write32(R528_CCU_BASE + CCU_DMA_BGR_REG,
          read32(R528_CCU_BASE + CCU_DMA_BGR_REG) |
          (1 << 16));
  sdelay(20);

  /* Enable gating clock for dma */

  write32(R528_CCU_BASE + CCU_DMA_BGR_REG,
          read32(R528_CCU_BASE + CCU_DMA_BGR_REG) |
          (1 << 0));
}

static void set_mbus(void)
{
  uint32_t val;

  /* Reset mbus domain */

  val = read32(R528_CCU_BASE + CCU_MBUS_CLK_REG);
  val |= (0x1 << 30);
  write32(R528_CCU_BASE + CCU_MBUS_CLK_REG, val);
  sdelay(1);

  /* Enable mbus master clock gating */

  write32(R528_CCU_BASE + CCU_MBUS_MAT_CLK_GATING_REG,
          0x00000d87);
}

static void set_module(uint32_t addr)
{
  uint32_t val;

  if (!(read32(addr) & (1 << 31)))
    {
      val = read32(addr);
      write32(addr, val | (1 << 31) | (1 << 30));

      /* Lock enable */

      val = read32(addr);
      val |= (1 << 29);
      write32(addr, val);

      /* Wait pll stable */

      while (!(read32(addr) & (0x1 << 28)))
        ;
      sdelay(20);

      /* Lock disable */

      val = read32(addr);
      val &= ~(1 << 29);
      write32(addr, val);
    }
}

static inline int ns_to_t(struct dram_param *para, int ns)
{
  unsigned int freq = para->dram_clk >> 1;
  return DIV_ROUND_UP(freq * ns, 1000);
}

static inline void sid_read_ldob_cal(struct dram_param *para)
{
  uint32_t reg;

  reg = (read32(SYS_SID_BASE + SYS_LDOB_SID) &
         0xff00) >> 8;
  if (reg == 0)
    {
      return;
    }

  /* DDR3 only */

  if (reg > 0x20)
    {
      reg -= 0x16;
    }

  clrsetbits_le32((SYS_CONTROL_REG_BASE + LDO_CTAL_REG),
                  0xff00, reg << 8);
}

static inline void
dram_voltage_set(struct dram_param *para)
{
  int vol;

  vol = 25;  /* DDR3 */
  clrsetbits_le32((SYS_CONTROL_REG_BASE + LDO_CTAL_REG),
                  0x20ff00, vol << 8);
  sdelay(1);
  sid_read_ldob_cal(para);
}

static inline void dram_enable_all_master(void)
{
  write32((MCTL_COM_BASE + MCTL_COM_MAER0),
          0xffffffff);
  write32((MCTL_COM_BASE + MCTL_COM_MAER1),
          0x000000ff);
  write32((MCTL_COM_BASE + MCTL_COM_MAER2),
          0x0000ffff);
  sdelay(10);
}

static inline void dram_disable_all_master(void)
{
  write32((MCTL_COM_BASE + MCTL_COM_MAER0), 0x1);
  write32((MCTL_COM_BASE + MCTL_COM_MAER1), 0x0);
  write32((MCTL_COM_BASE + MCTL_COM_MAER2), 0x0);
  sdelay(10);
}

static void
eye_delay_compensation(struct dram_param *para)
{
  uint32_t delay;
  int i;

  delay = (para->dram_tpr11 & 0xf) << 9;
  delay |= (para->dram_tpr12 & 0xf) << 1;
  for (i = 0; i < 9; i++)
    {
      setbits_le32((MCTL_PHY_BASE +
                    MCTL_PHY_DATX0IOCR(i)), delay);
    }

  delay = (para->dram_tpr11 & 0xf0) << 5;
  delay |= (para->dram_tpr12 & 0xf0) >> 3;
  for (i = 0; i < 9; i++)
    {
      setbits_le32((MCTL_PHY_BASE +
                    MCTL_PHY_DATX1IOCR(i)), delay);
    }

  clrbits_le32((MCTL_PHY_BASE + MCTL_PHY_PGCR0),
               0x04000000);
  delay = (para->dram_tpr11 & 0xf0000) >> 7;
  delay |= (para->dram_tpr12 & 0xf0000) >> 15;
  setbits_le32((MCTL_PHY_BASE +
                MCTL_PHY_DATX0IOCR(9)), delay);
  setbits_le32((MCTL_PHY_BASE +
                MCTL_PHY_DATX0IOCR(10)), delay);
  delay = (para->dram_tpr11 & 0xf00000) >> 11;
  delay |= (para->dram_tpr12 & 0xf00000) >> 19;
  setbits_le32((MCTL_PHY_BASE +
                MCTL_PHY_DATX1IOCR(9)), delay);
  setbits_le32((MCTL_PHY_BASE +
                MCTL_PHY_DATX1IOCR(10)), delay);
  setbits_le32((MCTL_PHY_BASE + MCTL_PHY_DXNSDLR6(0)),
               (para->dram_tpr11 & 0xf0000) << 9);
  setbits_le32((MCTL_PHY_BASE + MCTL_PHY_DXNSDLR6(1)),
               (para->dram_tpr11 & 0xf00000) << 5);
  setbits_le32((MCTL_PHY_BASE + MCTL_PHY_PGCR0),
               (1 << 26));
  sdelay(1);
  delay = (para->dram_tpr10 & 0xf0) << 4;
  for (i = 6; i < 27; ++i)
    {
      setbits_le32((MCTL_PHY_BASE +
                    MCTL_PHY_ACIOCR1(i)), delay);
    }

  setbits_le32((MCTL_PHY_BASE + MCTL_PHY_ACIOCR1(2)),
               (para->dram_tpr10 & 0x0f) << 8);
  setbits_le32((MCTL_PHY_BASE + MCTL_PHY_ACIOCR1(3)),
               (para->dram_tpr10 & 0x0f) << 8);
  setbits_le32((MCTL_PHY_BASE + MCTL_PHY_ACIOCR1(28)),
               (para->dram_tpr10 & 0xf00) >> 4);
}

static void
mctl_set_timing_params(struct dram_param *para)
{
  uint8_t tccd = 2;
  uint8_t tfaw;
  uint8_t trrd;
  uint8_t trcd;
  uint8_t trc;
  uint8_t txp;
  uint8_t twtr;
  uint8_t trtp = 4;
  uint8_t trp;
  uint8_t tras;
  uint16_t trefi;
  uint16_t trfc;
  uint8_t tcksrx;
  uint8_t tckesr;
  uint8_t trd2wr;
  uint8_t twr2rd;
  uint8_t trasmax;
  uint8_t twtp;
  uint8_t tcke;
  uint8_t tmod;
  uint8_t tmrd;
  uint8_t tmrw;
  uint8_t tcl;
  uint8_t tcwl;
  uint8_t t_rdata_en;
  uint8_t wr_latency;
  uint32_t mr0;
  uint32_t mr1;
  uint32_t mr2;
  uint32_t mr3;
  uint32_t tdinit0;
  uint32_t tdinit1;
  uint32_t tdinit2;
  uint32_t tdinit3;

  /* T113-S3 uses DDR3 only (para->dram_type == 3) */

  trfc = ns_to_t(para, 350);
  trefi = ns_to_t(para, 7800) / 32 + 1;
  twtr = ns_to_t(para, 8) + 2;
  trrd = max(ns_to_t(para, 10), 2);
  txp = max(ns_to_t(para, 10), 2);
  if (para->dram_clk <= 800)
    {
      tfaw = ns_to_t(para, 50);
      trcd = ns_to_t(para, 15);
      trp = ns_to_t(para, 15);
      trc = ns_to_t(para, 53);
      tras = ns_to_t(para, 38);
      mr0 = 0x1c70;
      mr2 = 0x18;
      tcl = 6;
      wr_latency = 2;
      tcwl = 4;
      t_rdata_en = 4;
    }
  else
    {
      tfaw = ns_to_t(para, 35);
      trcd = ns_to_t(para, 14);
      trp = ns_to_t(para, 14);
      trc = ns_to_t(para, 48);
      tras = ns_to_t(para, 34);
      mr0 = 0x1e14;
      mr2 = 0x20;
      tcl = 7;
      wr_latency = 3;
      tcwl = 5;
      t_rdata_en = 5;
    }

  trasmax = para->dram_clk / 30;
  twtp = tcwl + 2 + twtr;
  twr2rd = tcwl + twtr;
  tdinit0 = 500 * para->dram_clk + 1;
  tdinit1 = 360 * para->dram_clk / 1000 + 1;
  tdinit2 = 200 * para->dram_clk + 1;
  tdinit3 = 1 * para->dram_clk + 1;
  mr1 = para->dram_mr1;
  mr3 = 0;
  tcke = 3;
  tcksrx = 5;
  tckesr = 4;
  if (((para->dram_tpr13 & 0xc) == 0x04) ||
      para->dram_clk < 912)
    {
      trd2wr = 5;
    }
  else
    {
      trd2wr = 6;
    }

  tmod = 12;
  tmrd = 4;
  tmrw = 0;

  /* Set mode registers */

  write32((MCTL_PHY_BASE + MCTL_PHY_DRAM_MR0), mr0);
  write32((MCTL_PHY_BASE + MCTL_PHY_DRAM_MR1), mr1);
  write32((MCTL_PHY_BASE + MCTL_PHY_DRAM_MR2), mr2);
  write32((MCTL_PHY_BASE + MCTL_PHY_DRAM_MR3), mr3);
  write32((MCTL_PHY_BASE + MCTL_PHY_LP3MR11),
          (para->dram_odt_en >> 4) & 0x3);

  /* Set dram timing DRAMTMG0 - DRAMTMG5 */

  write32((MCTL_PHY_BASE + MCTL_PHY_DRAMTMG0),
          (twtp << 24) | (tfaw << 16) |
          (trasmax << 8) | (tras << 0));
  write32((MCTL_PHY_BASE + MCTL_PHY_DRAMTMG1),
          (txp << 16) | (trtp << 8) | (trc << 0));
  write32((MCTL_PHY_BASE + MCTL_PHY_DRAMTMG2),
          (tcwl << 24) | (tcl << 16) |
          (trd2wr << 8) | (twr2rd << 0));
  write32((MCTL_PHY_BASE + MCTL_PHY_DRAMTMG3),
          (tmrw << 16) | (tmrd << 12) | (tmod << 0));
  write32((MCTL_PHY_BASE + MCTL_PHY_DRAMTMG4),
          (trcd << 24) | (tccd << 16) |
          (trrd << 8) | (trp << 0));
  write32((MCTL_PHY_BASE + MCTL_PHY_DRAMTMG5),
          (tcksrx << 24) | (tcksrx << 16) |
          (tckesr << 8) | (tcke << 0));

  /* Set dual rank timing */

  clrsetbits_le32((MCTL_PHY_BASE + MCTL_PHY_DRAMTMG8),
                  0xf000ffff,
                  (para->dram_clk < 800) ?
                  0xf0006610 : 0xf0007610);

  /* Set phy interface time PITMG0, PTR3, PTR4 */

  write32((MCTL_PHY_BASE + MCTL_PHY_PITMG0),
          (0x2 << 24) | (t_rdata_en << 16) |
          (1 << 8) | (wr_latency << 0));
  write32((MCTL_PHY_BASE + MCTL_PHY_PTR3),
          ((tdinit0 << 0) | (tdinit1 << 20)));
  write32((MCTL_PHY_BASE + MCTL_PHY_PTR4),
          ((tdinit2 << 0) | (tdinit3 << 20)));

  /* Set refresh timing and mode */

  write32((MCTL_PHY_BASE + MCTL_PHY_RFSHTMG),
          (trefi << 16) | (trfc << 0));
  write32((MCTL_PHY_BASE + MCTL_PHY_RFSHCTL1),
          (trefi << 15) & 0x0fff0000);
}

static int ccu_set_pll_ddr_clk(int index,
                                struct dram_param *para)
{
  unsigned int val;
  unsigned int clk;
  unsigned int n;

  if (para->dram_tpr13 & (1 << 6))
    {
      clk = para->dram_tpr9;
    }
  else
    {
      clk = para->dram_clk;
    }

  n = (clk * 2) / 24;
  val = read32((R528_CCU_BASE + CCU_PLL_DDR_CTRL_REG));
  val &= 0xfff800fc;
  val |= (n - 1) << 8;
  val |= 0xc0000000;
  val &= 0xdfffffff;
  write32((R528_CCU_BASE + CCU_PLL_DDR_CTRL_REG),
          val | 0x20000000);
  while ((read32((R528_CCU_BASE +
          CCU_PLL_DDR_CTRL_REG)) & 0x10000000) == 0)
    ;
  sdelay(20);
  val = read32(R528_CCU_BASE);
  val |= 0x08000000;
  write32(R528_CCU_BASE, val);
  val = read32((R528_CCU_BASE + CCU_DRAM_CLK_REG));
  val &= 0xfcfffcfc;
  val |= 0x80000000;
  write32((R528_CCU_BASE + CCU_DRAM_CLK_REG), val);
  return n * 24;
}

static void mctl_sys_init(struct dram_param *para)
{
  clrbits_le32((R528_CCU_BASE + CCU_MBUS_CLK_REG),
               (1 << 30));
  clrbits_le32((R528_CCU_BASE + CCU_DRAM_BGR_REG),
               0x10001);
  clrsetbits_le32((R528_CCU_BASE + CCU_DRAM_CLK_REG),
                  (1 << 31) | (1 << 30), (1 << 27));
  sdelay(10);
  para->dram_clk = ccu_set_pll_ddr_clk(0, para) / 2;
  sdelay(100);
  dram_disable_all_master();
  setbits_le32((R528_CCU_BASE + CCU_DRAM_BGR_REG),
               (1 << 16));
  setbits_le32((R528_CCU_BASE + CCU_MBUS_CLK_REG),
               (1 << 30));
  setbits_le32((R528_CCU_BASE + CCU_DRAM_CLK_REG),
               (1 << 30));
  sdelay(5);
  setbits_le32((R528_CCU_BASE + CCU_DRAM_BGR_REG),
               (1 << 0));
  setbits_le32((R528_CCU_BASE + CCU_DRAM_CLK_REG),
               (1 << 31) | (1 << 27));
  sdelay(5);
  write32((MCTL_PHY_BASE + MCTL_PHY_CLKEN), 0x8000);
  sdelay(10);
}

static void mctl_com_init(struct dram_param *para)
{
  uint32_t ptr;
  uint32_t val;
  uint32_t width;
  uint32_t t;
  int i;

  clrsetbits_le32((MCTL_COM_BASE + MCTL_COM_DBGCR),
                  0x3f00, 0x2000);
  val = read32((MCTL_COM_BASE + MCTL_COM_WORK_MODE0)) &
        ~0x00fff000;
  val |= (para->dram_type & 0x7) << 16;
  val |= (~para->dram_para2 & 0x1) << 12;
  val |= (1 << 22);

  /* DDR3: bit19 controlled by tpr13 */

  if (para->dram_tpr13 & (1 << 5))
    {
      val |= (1 << 19);
    }

  write32((MCTL_COM_BASE + MCTL_COM_WORK_MODE0), val);
  if ((para->dram_para2 & (1 << 8)) &&
      ((para->dram_para2 & 0xf000) != 0x1000))
    {
      width = 32;
    }
  else
    {
      width = 16;
    }

  ptr = (MCTL_COM_BASE + MCTL_COM_WORK_MODE0);
  for (i = 0; i < width; i += 16)
    {
      val = read32(ptr) & 0xfffff000;
      val |= (para->dram_para2 >> 12) & 0x3;
      val |= ((para->dram_para1 >> (i + 12)) << 2) &
             0x4;
      val |= (((para->dram_para1 >> (i + 4)) - 1) <<
             4) & 0xff;
      t = (para->dram_para1 >> i) & 0xf;
      if (t == 8)
        {
          val |= 0xa00;
        }
      else if (t == 4)
        {
          val |= 0x900;
        }
      else if (t == 2)
        {
          val |= 0x800;
        }
      else if (t == 1)
        {
          val |= 0x700;
        }
      else
        {
          val |= 0x600;
        }

      write32(ptr, val);
      ptr += 4;
    }

  val = (read32((MCTL_COM_BASE +
         MCTL_COM_WORK_MODE0)) & 0x1) ? 0x303 : 0x201;
  write32((MCTL_PHY_BASE + MCTL_PHY_ODTMAP), val);
  if (para->dram_para2 & (1 << 0))
    {
      write32((MCTL_PHY_BASE + MCTL_PHY_DXNGCR0(1)),
              0);
    }

  if (para->dram_tpr4)
    {
      setbits_le32((MCTL_COM_BASE +
                    MCTL_COM_WORK_MODE0),
                   (para->dram_tpr4 & 0x3) << 25);
      setbits_le32((MCTL_COM_BASE +
                    MCTL_COM_WORK_MODE1),
                   (para->dram_tpr4 & 0x7fc) << 10);
    }
}

#define AC_REMAP_COLS 22
#define AC_REMAP_ROWS 10

static const uint8_t ac_remapping_tables[] =
{
  /* Row 0 */

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,

  /* Row 1 */

  0x01, 0x09, 0x03, 0x07, 0x08, 0x12,
  0x04, 0x0d, 0x05, 0x06, 0x0a, 0x02,
  0x0e, 0x0c, 0x00, 0x00, 0x15, 0x11,
  0x14, 0x13, 0x0b, 0x16,

  /* Row 2 */

  0x04, 0x09, 0x03, 0x07, 0x08, 0x12,
  0x01, 0x0d, 0x02, 0x06, 0x0a, 0x05,
  0x0e, 0x0c, 0x00, 0x00, 0x15, 0x11,
  0x14, 0x13, 0x0b, 0x16,

  /* Row 3 */

  0x01, 0x07, 0x08, 0x0c, 0x0a, 0x12,
  0x04, 0x0d, 0x05, 0x06, 0x03, 0x02,
  0x09, 0x00, 0x00, 0x00, 0x15, 0x11,
  0x14, 0x13, 0x0b, 0x16,

  /* Row 4 */

  0x04, 0x0c, 0x0a, 0x07, 0x08, 0x12,
  0x01, 0x0d, 0x02, 0x06, 0x03, 0x05,
  0x09, 0x00, 0x00, 0x00, 0x15, 0x11,
  0x14, 0x13, 0x0b, 0x16,

  /* Row 5 */

  0x0d, 0x02, 0x07, 0x09, 0x0c, 0x13,
  0x05, 0x01, 0x06, 0x03, 0x04, 0x08,
  0x0a, 0x00, 0x00, 0x00, 0x15, 0x16,
  0x12, 0x11, 0x0b, 0x14,

  /* Row 6 */

  0x03, 0x0a, 0x07, 0x0d, 0x09, 0x0b,
  0x01, 0x02, 0x04, 0x06, 0x08, 0x05,
  0x0c, 0x00, 0x00, 0x00, 0x14, 0x12,
  0x00, 0x15, 0x16, 0x11,

  /* Row 7 */

  0x03, 0x02, 0x04, 0x07, 0x09, 0x01,
  0x11, 0x0c, 0x12, 0x0e, 0x0d, 0x08,
  0x0f, 0x06, 0x0a, 0x05, 0x13, 0x16,
  0x10, 0x15, 0x14, 0x0b,

  /* Row 8 */

  0x02, 0x13, 0x08, 0x06, 0x0e, 0x05,
  0x14, 0x0a, 0x03, 0x12, 0x0d, 0x0b,
  0x07, 0x0f, 0x09, 0x01, 0x16, 0x15,
  0x11, 0x0c, 0x04, 0x10,

  /* Row 9 */

  0x01, 0x02, 0x0d, 0x08, 0x0f, 0x0c,
  0x13, 0x0a, 0x03, 0x15, 0x06, 0x11,
  0x09, 0x0e, 0x05, 0x10, 0x14, 0x16,
  0x0b, 0x07, 0x04, 0x12,
};

static void
mctl_phy_ac_remapping(struct dram_param *para)
{
  const uint8_t *cfg = &ac_remapping_tables[0];
  uint32_t fuse;
  uint32_t val;
  uint32_t chipid;

  /* DDR3 always continues */

  fuse = (read32(SYS_SID_BASE + SYS_EFUSE_REG) &
          0xf00) >> 8;
  chipid = (read32(SYS_SID_BASE + SYS_CHIPID_REG) &
            0xffff);

  /* DDR3 path only */

  if (para->dram_tpr13 & 0xc0000)
    {
      cfg = &ac_remapping_tables[7 * AC_REMAP_COLS];
    }
  else
    {
      if (fuse == 8)
        {
          cfg = &ac_remapping_tables[2 * AC_REMAP_COLS];
        }
      else if (fuse == 9)
        {
          cfg = &ac_remapping_tables[3 * AC_REMAP_COLS];
        }
      else if (fuse == 10)
        {
          if (chipid == 0x6800)
            {
              cfg = &ac_remapping_tables[0];
            }
          else
            {
              cfg = &ac_remapping_tables[5 * AC_REMAP_COLS];
            }
        }
      else if (fuse == 11)
        {
          cfg = &ac_remapping_tables[4 * AC_REMAP_COLS];
        }
      else if (fuse == 12)
        {
          cfg = &ac_remapping_tables[1 * AC_REMAP_COLS];
        }
      else if (fuse == 13 || fuse == 14)
        {
          cfg = &ac_remapping_tables[0];
        }
    }

  val = (cfg[4] << 25) | (cfg[3] << 20) |
        (cfg[2] << 15) | (cfg[1] << 10) |
        (cfg[0] << 5);
  write32((MCTL_COM_BASE + MCTL_COM_REMAP0), val);
  val = (cfg[10] << 25) | (cfg[9] << 20) |
        (cfg[8] << 15) | (cfg[7] << 10) |
        (cfg[6] << 5) | cfg[5];
  write32((MCTL_COM_BASE + MCTL_COM_REMAP1), val);
  val = (cfg[15] << 20) | (cfg[14] << 15) |
        (cfg[13] << 10) | (cfg[12] << 5) | cfg[11];
  write32((MCTL_COM_BASE + MCTL_COM_REMAP2), val);
  val = (cfg[21] << 25) | (cfg[20] << 20) |
        (cfg[19] << 15) | (cfg[18] << 10) |
        (cfg[17] << 5) | cfg[16];
  write32((MCTL_COM_BASE + MCTL_COM_REMAP3), val);
  val = (cfg[4] << 25) | (cfg[3] << 20) |
        (cfg[2] << 15) | (cfg[1] << 10) |
        (cfg[0] << 5) | 1;
  write32((MCTL_COM_BASE + MCTL_COM_REMAP0), val);
}

static unsigned int
mctl_channel_init(unsigned int ch_index,
                  struct dram_param *para)
{
  unsigned int val;
  unsigned int dqs_gating_mode;

  dqs_gating_mode = (para->dram_tpr13 & 0xc) >> 2;
  clrsetbits_le32((MCTL_COM_BASE + MCTL_COM_TMR),
                  0xfff, (para->dram_clk / 2) - 1);
  clrsetbits_le32((MCTL_PHY_BASE + MCTL_PHY_PGCR2),
                  0xf00, 0x300);
  if (para->dram_odt_en)
    {
      val = 0;
    }
  else
    {
      val = (1 << 5);
    }

  if (para->dram_clk > 672)
    {
      clrsetbits_le32((MCTL_PHY_BASE +
                       MCTL_PHY_DXNGCR0(0)),
                      0xf63e, val);
    }
  else
    {
      clrsetbits_le32((MCTL_PHY_BASE +
                       MCTL_PHY_DXNGCR0(0)),
                      0xf03e, val);
    }

  if (para->dram_clk > 672)
    {
      setbits_le32((MCTL_PHY_BASE +
                    MCTL_PHY_DXNGCR0(0)), 0x400);
      clrsetbits_le32((MCTL_PHY_BASE +
                       MCTL_PHY_DXNGCR0(1)),
                      0xf63e, val);
    }
  else
    {
      clrsetbits_le32((MCTL_PHY_BASE +
                       MCTL_PHY_DXNGCR0(1)),
                      0xf03e, val);
    }

  setbits_le32((MCTL_PHY_BASE + MCTL_PHY_ACIOCR0),
               (1 << 1));
  eye_delay_compensation(para);
  val = read32((MCTL_PHY_BASE + MCTL_PHY_PGCR2));
  if (dqs_gating_mode == 1)
    {
      clrsetbits_le32((MCTL_PHY_BASE +
                       MCTL_PHY_PGCR2), 0xc0, 0);
      clrbits_le32((MCTL_PHY_BASE +
                    MCTL_PHY_DQSGMR), 0x107);
    }
  else if (dqs_gating_mode == 2)
    {
      clrsetbits_le32((MCTL_PHY_BASE +
                       MCTL_PHY_PGCR2), 0xc0, 0x80);
      clrsetbits_le32((MCTL_PHY_BASE +
                       MCTL_PHY_DQSGMR), 0x107,
                      (((para->dram_tpr13 >> 16) &
                      0x1f) - 2) | 0x100);
      clrsetbits_le32((MCTL_PHY_BASE +
                       MCTL_PHY_DXCCR),
                      (1 << 31), (1 << 27));
    }
  else
    {
      clrbits_le32((MCTL_PHY_BASE +
                    MCTL_PHY_PGCR2), 0x40);
      sdelay(10);
      setbits_le32((MCTL_PHY_BASE +
                    MCTL_PHY_PGCR2), 0xc0);
    }

  /* LPDDR2/3 DQS gating removed -- DDR3 only */

  clrsetbits_le32((MCTL_PHY_BASE + MCTL_PHY_DTCR),
                  0x0fffffff,
                  (para->dram_para2 & (1 << 12)) ?
                  0x03000001 : 0x01000007);
  if (read32((R_CPUCFG_BASE +
      R_CPUCFG_SUP_STAN_FLAG)) & (1 << 16))
    {
      clrbits_le32((R_PRCM_BASE +
                    VDD_SYS_PWROFF_GATING), 0x2);
      sdelay(10);
    }

  clrsetbits_le32((MCTL_PHY_BASE + MCTL_PHY_ZQCR),
                  0x3ffffff,
                  (para->dram_zq & 0x00ffffff) |
                  (1 << 25));
  if (dqs_gating_mode == 1)
    {
      write32((MCTL_PHY_BASE + MCTL_PHY_PIR), 0x53);
      while ((read32((MCTL_PHY_BASE +
              MCTL_PHY_PGSR0)) & 0x1) == 0)
        ;
      sdelay(10);
      if (para->dram_type == DRAM_TYPE_DDR3)
        {
          write32((MCTL_PHY_BASE + MCTL_PHY_PIR),
                  0x5a0);
        }
      else
        {
          write32((MCTL_PHY_BASE + MCTL_PHY_PIR),
                  0x520);
        }
    }
  else
    {
      if ((read32((R_CPUCFG_BASE +
          R_CPUCFG_SUP_STAN_FLAG)) &
          (1 << 16)) == 0)
        {
          if (para->dram_type == DRAM_TYPE_DDR3)
            {
              write32((MCTL_PHY_BASE + MCTL_PHY_PIR),
                      0x1f2);
            }
          else
            {
              write32((MCTL_PHY_BASE + MCTL_PHY_PIR),
                      0x172);
            }
        }
      else
        {
          write32((MCTL_PHY_BASE + MCTL_PHY_PIR),
                  0x62);
        }
    }

  setbits_le32((MCTL_PHY_BASE + MCTL_PHY_PIR), 0x1);
  sdelay(10);
  while ((read32((MCTL_PHY_BASE +
          MCTL_PHY_PGSR0)) & 0x1) == 0)
    ;
  if (read32((R_CPUCFG_BASE +
      R_CPUCFG_SUP_STAN_FLAG)) & (1 << 16))
    {
      clrsetbits_le32((MCTL_PHY_BASE +
                       MCTL_PHY_PGCR3),
                      0x06000000, 0x04000000);
      sdelay(10);
      setbits_le32((MCTL_PHY_BASE +
                    MCTL_PHY_PWRCTL), 0x1);
      while ((read32((MCTL_PHY_BASE +
              MCTL_PHY_STATR)) & 0x7) != 0x3)
        ;
      clrbits_le32((R_PRCM_BASE +
                    VDD_SYS_PWROFF_GATING), 0x1);
      sdelay(10);
      clrbits_le32((MCTL_PHY_BASE +
                    MCTL_PHY_PWRCTL), 0x1);
      while ((read32((MCTL_PHY_BASE +
              MCTL_PHY_STATR)) & 0x7) != 0x1)
        ;
      sdelay(15);
      if (dqs_gating_mode == 1)
        {
          clrbits_le32((MCTL_PHY_BASE +
                        MCTL_PHY_PGCR2), 0xc0);
          clrsetbits_le32((MCTL_PHY_BASE +
                           MCTL_PHY_PGCR3),
                          0x06000000, 0x02000000);
          sdelay(1);
          write32((MCTL_PHY_BASE + MCTL_PHY_PIR),
                  0x401);
          while ((read32((MCTL_PHY_BASE +
                  MCTL_PHY_PGSR0)) & 0x1) == 0)
            ;
        }
    }

  if (read32((MCTL_PHY_BASE +
      MCTL_PHY_PGSR0)) & (1 << 20))
    {
      return 0;
    }

  while ((read32((MCTL_PHY_BASE +
          MCTL_PHY_STATR)) & 0x1) == 0)
    ;
  setbits_le32((MCTL_PHY_BASE + MCTL_PHY_RFSHCTL0),
               (1 << 31));
  sdelay(10);
  clrbits_le32((MCTL_PHY_BASE + MCTL_PHY_RFSHCTL0),
               (1 << 31));
  sdelay(10);
  setbits_le32((MCTL_COM_BASE + MCTL_COM_CCCR),
               (1 << 31));
  sdelay(10);
  clrbits_le32((MCTL_PHY_BASE + MCTL_PHY_PGCR3),
               0x06000000);
  if (dqs_gating_mode == 1)
    {
      clrsetbits_le32((MCTL_PHY_BASE +
                       MCTL_PHY_DXCCR), 0xc0, 0x40);
    }

  return 1;
}

static unsigned int
calculate_rank_size(uint32_t regval)
{
  unsigned int bits;

  bits = (regval >> 8) & 0xf;
  bits += (regval >> 4) & 0xf;
  bits += (regval >> 2) & 0x3;
  bits -= 14;
  return 1u << bits;
}

static unsigned int dramc_get_dram_size(void)
{
  uint32_t val;
  unsigned int size;

  val = read32((MCTL_COM_BASE + MCTL_COM_WORK_MODE0));
  size = calculate_rank_size(val);
  if ((val & 0x3) == 0)
    {
      return size;
    }

  val = read32((MCTL_COM_BASE + MCTL_COM_WORK_MODE1));
  if ((val & 0x3) == 0)
    {
      return size * 2;
    }

  return size + calculate_rank_size(val);
}

static int dqs_gate_detect(struct dram_param *para)
{
  uint32_t dx0 = 0;
  uint32_t dx1 = 0;

  if ((read32(MCTL_PHY_BASE + MCTL_PHY_PGSR0) &
       (1 << 22)) == 0)
    {
      para->dram_para2 = (para->dram_para2 & ~0xf) |
                         (1 << 12);
      return 1;
    }

  dx0 = (read32(MCTL_PHY_BASE +
         MCTL_PHY_DXNGSR0(0)) & 0x3000000) >> 24;
  if (dx0 == 0)
    {
      para->dram_para2 = (para->dram_para2 & ~0xf) |
                         0x1001;
      return 1;
    }

  if (dx0 == 2)
    {
      dx1 = (read32(MCTL_PHY_BASE +
             MCTL_PHY_DXNGSR0(1)) &
             0x3000000) >> 24;
      if (dx1 == 2)
        {
          para->dram_para2 =
            para->dram_para2 & ~0xf00f;
        }
      else
        {
          para->dram_para2 =
            (para->dram_para2 & ~0xf00f) | (1 << 0);
        }

      return 1;
    }

  if ((para->dram_tpr13 & (1 << 29)) == 0)
    {
      return 0;
    }

  return 0;
}

static int
dramc_simple_wr_test(unsigned int mem_mb, int len)
{
  unsigned int offs = (mem_mb / 2) << 18;
  unsigned int patt1 = 0x01234567;
  unsigned int patt2 = 0xfedcba98;
  unsigned int *addr;
  unsigned int v1;
  unsigned int v2;
  unsigned int i;

  addr = (unsigned int *)CONFIG_DRAM_BASE;
  for (i = 0; i != len; i++, addr++)
    {
      write32((unsigned long)addr, patt1 + i);
      write32((unsigned long)(addr + offs), patt2 + i);
    }

  addr = (unsigned int *)CONFIG_DRAM_BASE;
  for (i = 0; i != len; i++)
    {
      v1 = read32((unsigned long)(addr + i));
      v2 = patt1 + i;
      if (v1 != v2)
        {
          return 1;
        }

      v1 = read32((unsigned long)(addr + offs + i));
      v2 = patt2 + i;
      if (v1 != v2)
        {
          return 1;
        }
    }

  return 0;
}

static void
mctl_vrefzq_init(struct dram_param *para)
{
  if (para->dram_tpr13 & (1 << 17))
    {
      return;
    }

  clrsetbits_le32((MCTL_PHY_BASE + MCTL_PHY_IOVCR0),
                  0x7f7f7f7f, para->dram_tpr5);
  if ((para->dram_tpr13 & (1 << 16)) == 0)
    {
      clrsetbits_le32((MCTL_PHY_BASE +
                       MCTL_PHY_IOVCR1), 0x7f,
                      para->dram_tpr6 & 0x7f);
    }
}

static int mctl_core_init(struct dram_param *para)
{
  mctl_sys_init(para);
  mctl_vrefzq_init(para);
  mctl_com_init(para);
  mctl_phy_ac_remapping(para);
  mctl_set_timing_params(para);
  return mctl_channel_init(0, para);
}

static int
auto_scan_dram_size(struct dram_param *para)
{
  uint32_t i = 0;
  uint32_t j = 0;
  uint32_t current_rank = 0;
  uint32_t rank_count = 1;
  uint32_t addr_line = 0;
  uint32_t reg_val = 0;
  uint32_t ret = 0;
  uint32_t cnt = 0;
  unsigned long mc_work_mode;
  uint32_t rank1_addr = CONFIG_DRAM_BASE;

  if (mctl_core_init(para) == 0)
    {
      return 0;
    }

  if ((((para->dram_para2 >> 12) & 0xf) == 0x1))
    {
      rank_count = 2;
    }

  for (current_rank = 0; current_rank < rank_count;
       current_rank++)
    {
      mc_work_mode = ((MCTL_COM_BASE +
                       MCTL_COM_WORK_MODE0) +
                      4 * current_rank);
      if (current_rank == 1)
        {
          clrsetbits_le32((MCTL_COM_BASE +
                           MCTL_COM_WORK_MODE0),
                          0xf0c, 0x6f0);
          clrsetbits_le32((MCTL_COM_BASE +
                           MCTL_COM_WORK_MODE1),
                          0xf0c, 0x6f0);
          rank1_addr = CONFIG_DRAM_BASE +
                       (0x1 << 27);
        }

      for (i = 0; i < 64; i++)
        {
          write32(CONFIG_DRAM_BASE + 4 * i,
                  (i % 2)
                    ? (CONFIG_DRAM_BASE + 4 * i)
                    : (~(CONFIG_DRAM_BASE + 4 * i)));
        }

      clrsetbits_le32(mc_work_mode, 0xf0c, 0x6f0);
      sdelay(2);
      for (i = 11; i < 17; i++)
        {
          ret = CONFIG_DRAM_BASE +
                (1 << (i + 2 + 9));
          cnt = 0;
          for (j = 0; j < 64; j++)
            {
              reg_val = (j % 2) ?
                        (rank1_addr + 4 * j) :
                        (~(rank1_addr + 4 * j));
              if (reg_val == read32(ret + j * 4))
                {
                  cnt++;
                }
              else
                {
                  break;
                }
            }

          if (cnt == 64)
            {
              break;
            }
        }

      if (i >= 16)
        {
          i = 16;
        }

      addr_line += i;
      para->dram_para1 &=
        ~(0xffu << (16 * current_rank + 4));
      para->dram_para1 |=
        (i << (16 * current_rank + 4));
      if (current_rank == 1)
        {
          clrsetbits_le32((MCTL_COM_BASE +
                           MCTL_COM_WORK_MODE0),
                          0xffc, 0x6a4);
        }

      clrsetbits_le32(mc_work_mode, 0xffc, 0x6a4);
      sdelay(1);
      for (i = 0; i < 1; i++)
        {
          ret = CONFIG_DRAM_BASE +
                (0x1u << (i + 2 + 9));
          cnt = 0;
          for (j = 0; j < 64; j++)
            {
              reg_val = (j % 2) ?
                        (rank1_addr + 4 * j) :
                        (~(rank1_addr + 4 * j));
              if (reg_val == read32(ret + j * 4))
                {
                  cnt++;
                }
              else
                {
                  break;
                }
            }

          if (cnt == 64)
            {
              break;
            }
        }

      addr_line += i + 2;
      para->dram_para1 &=
        ~(0xfu << (16 * current_rank + 12));
      para->dram_para1 |=
        (i << (16 * current_rank + 12));
      if (current_rank == 1)
        {
          clrsetbits_le32(mc_work_mode,
                          0xffc, 0xaa0);
        }

      clrsetbits_le32(mc_work_mode, 0xffc, 0xaa0);
      sdelay(2);
      for (i = 9; i <= 13; i++)
        {
          ret = CONFIG_DRAM_BASE + (0x1u << i);
          cnt = 0;
          for (j = 0; j < 64; j++)
            {
              reg_val = (j % 2) ?
                        (CONFIG_DRAM_BASE + 4 * j) :
                        (~(CONFIG_DRAM_BASE + 4 * j));
              if (reg_val == read32(ret + j * 4))
                {
                  cnt++;
                }
              else
                {
                  break;
                }
            }

          if (cnt == 64)
            {
              break;
            }
        }

      if (i >= 13)
        {
          i = 13;
        }

      addr_line += i;
      if (i == 9)
        {
          i = 0;
        }
      else
        {
          i = (0x1u << (i - 10));
        }

      para->dram_para1 &=
        ~(0xfu << (16 * current_rank));
      para->dram_para1 |=
        (i << (16 * current_rank));
    }

  if (rank_count == 2)
    {
      para->dram_para2 &= 0xfffff0ff;
      if ((para->dram_para1 & 0xffff) ==
          (para->dram_para1 >> 16))
        {
        }
      else
        {
          para->dram_para2 |= 0x1 << 8;
        }
    }

  return 1;
}

static int
auto_scan_dram_rank_width(struct dram_param *para)
{
  unsigned int s1 = para->dram_tpr13;
  unsigned int s2 = para->dram_para1;

  para->dram_para1 = 0x00b000b0;
  para->dram_para2 = (para->dram_para2 & ~0xf) |
                     (1 << 12);
  para->dram_tpr13 = (para->dram_tpr13 & ~0x8) |
                     (1 << 2) | (1 << 0);
  mctl_core_init(para);
  if (read32((MCTL_PHY_BASE +
      MCTL_PHY_PGSR0)) & (1 << 20))
    {
      return 0;
    }

  if (dqs_gate_detect(para) == 0)
    {
      return 0;
    }

  para->dram_tpr13 = s1;
  para->dram_para1 = s2;
  return 1;
}

static int
auto_scan_dram_config(struct dram_param *para)
{
  if (((para->dram_tpr13 & (1 << 14)) == 0) &&
      (auto_scan_dram_rank_width(para) == 0))
    {
      return 0;
    }

  if (((para->dram_tpr13 & (1 << 0)) == 0) &&
      (auto_scan_dram_size(para) == 0))
    {
      return 0;
    }

  if ((para->dram_tpr13 & (1 << 15)) == 0)
    {
      para->dram_tpr13 |= (1 << 14) | (1 << 13) |
                          (1 << 1) | (1 << 0);
    }

  return 1;
}

static int init_dram(struct dram_param *para)
{
  uint32_t rc;
  uint32_t mem_size_mb;

  if (para->dram_tpr13 & (1 << 16))
    {
      setbits_le32((SYS_CONTROL_REG_BASE +
                    ZQ_CAL_CTRL_REG), (1 << 8));
      write32((SYS_CONTROL_REG_BASE +
               ZQ_RES_CTRL_REG), 0);
      sdelay(10);
    }
  else
    {
      clrbits_le32((SYS_CONTROL_REG_BASE +
                    ZQ_CAL_CTRL_REG), 0x3);
      write32((R_PRCM_BASE + ANALOG_PWROFF_GATING),
              para->dram_tpr13 & (1 << 16));
      sdelay(10);
      clrsetbits_le32((SYS_CONTROL_REG_BASE +
                       ZQ_CAL_CTRL_REG),
                      0x108, (1 << 1));
      sdelay(10);
      setbits_le32((SYS_CONTROL_REG_BASE +
                    ZQ_CAL_CTRL_REG), (1 << 0));
      sdelay(20);
    }

  dram_voltage_set(para);
  if ((para->dram_tpr13 & (1 << 0)) == 0)
    {
      if (auto_scan_dram_config(para) == 0)
        {
          return 0;
        }
    }

  rc = para->dram_mr1;
  if (mctl_core_init(para) == 0)
    {
      return 0;
    }

  rc = para->dram_para2;
  if (rc & (1 << 31))
    {
      rc = (rc >> 16) & ~(1 << 15);
    }
  else
    {
      rc = dramc_get_dram_size();
      para->dram_para2 = (para->dram_para2 &
                          0xffffu) | rc << 16;
    }

  mem_size_mb = rc;
  if (para->dram_tpr13 & (1 << 30))
    {
      rc = para->dram_tpr8;
      if (rc == 0)
        {
          rc = 0x10000200;
        }

      write32((MCTL_PHY_BASE + MCTL_PHY_ASRTC), rc);
      write32((MCTL_PHY_BASE + MCTL_PHY_ASRC), 0x40a);
      setbits_le32((MCTL_PHY_BASE +
                    MCTL_PHY_PWRCTL), (1 << 0));
    }
  else
    {
      clrbits_le32((MCTL_PHY_BASE +
                    MCTL_PHY_ASRTC), 0xffff);
      clrbits_le32((MCTL_PHY_BASE +
                    MCTL_PHY_PWRCTL), 0x1);
    }

  if (para->dram_tpr13 & (1 << 9))
    {
      clrsetbits_le32((MCTL_PHY_BASE +
                       MCTL_PHY_PGCR0),
                      0xf000, 0x5000);
    }
  else
    {
      clrbits_le32((MCTL_PHY_BASE +
                    MCTL_PHY_PGCR0), 0xf000);
    }

  setbits_le32((MCTL_PHY_BASE + MCTL_PHY_ZQCR),
               (1 << 31));
  if (para->dram_tpr13 & (1 << 8))
    {
      write32((MCTL_PHY_BASE + MCTL_PHY_VTFCR),
              read32((MCTL_PHY_BASE +
                      MCTL_PHY_VTFCR)) | 0x300);
    }

  if (para->dram_tpr13 & (1 << 16))
    {
      clrbits_le32((MCTL_PHY_BASE +
                    MCTL_PHY_PGCR2), (1 << 13));
    }
  else
    {
      setbits_le32((MCTL_PHY_BASE +
                    MCTL_PHY_PGCR2), (1 << 13));
    }

  /* LPDDR3 ODT removed -- DDR3 only */

  dram_enable_all_master();
  if (para->dram_tpr13 & (1 << 28))
    {
      if ((read32((R_CPUCFG_BASE +
          R_CPUCFG_SUP_STAN_FLAG)) & (1 << 16)) ||
          dramc_simple_wr_test(mem_size_mb, 4096))
        {
          return 0;
        }
    }

  return mem_size_mb;
}

static void sys_spi_select(void)
{
  uint32_t addr = 0x04025000;
  uint32_t val;

  val = read32(addr + SPI_TCR);
  val &= ~((0x3 << 4) | (0x1 << 7));
  val |= ((0 & 0x3) << 4) | (0x0 << 7);
  write32(addr + SPI_TCR, val);
}

static void sys_spi_deselect(void)
{
  uint32_t addr = 0x04025000;
  uint32_t val;

  val = read32(addr + SPI_TCR);
  val &= ~((0x3 << 4) | (0x1 << 7));
  val |= ((0 & 0x3) << 4) | (0x1 << 7);
  write32(addr + SPI_TCR, val);
}

static void sys_spi_write_txbuf(uint8_t *buf, int len)
{
  uint32_t addr = 0x04025000;
  int i;

  write32(addr + SPI_MTC, len & 0xffffff);
  write32(addr + SPI_BCC, len & 0xffffff);
  if (buf)
    {
      for (i = 0; i < len; i++)
        {
          write8(addr + SPI_TXD, *buf++);
        }
    }
  else
    {
      for (i = 0; i < len; i++)
        {
          write8(addr + SPI_TXD, 0xff);
        }
    }
}

static int
sys_spi_transfer(void *txbuf, void *rxbuf, int len)
{
  uint32_t addr = 0x04025000;
  int count = len;
  uint8_t *tx = txbuf;
  uint8_t *rx = rxbuf;
  uint8_t val;
  int n;
  int i;

  while (count > 0)
    {
      n = (count <= 64) ? count : 64;
      write32(addr + SPI_MBC, n);
      sys_spi_write_txbuf(tx, n);
      write32(addr + SPI_TCR,
              read32(addr + SPI_TCR) | (1 << 31));
      while (read32(addr + SPI_TCR) & (1 << 31))
        ;

      while ((read32(addr + SPI_FSR) & 0xff) < n)
        ;
      for (i = 0; i < n; i++)
        {
          val = read8(addr + SPI_RXD);
          if (rx)
            {
              *rx++ = val;
            }
        }

      if (tx)
        {
          tx += n;
        }

      count -= n;
    }

  return len;
}

static int sys_spi_write_then_read(void *txbuf,
                                   int txlen,
                                   void *rxbuf,
                                   int rxlen)
{
  if (sys_spi_transfer(txbuf, NULL, txlen) != txlen)
    {
      return -1;
    }

  if (sys_spi_transfer(NULL, rxbuf, rxlen) != rxlen)
    {
      return -1;
    }

  return 0;
}

static void sys_spinand_wait(void)
{
  uint8_t tx[2];
  uint8_t rx[1];

  tx[0] = 0x0f;
  tx[1] = 0xc0;
  do
    {
      sys_spi_select();
      sys_spi_write_then_read(tx, 2, rx, 1);
      sys_spi_deselect();
    }
  while ((rx[0] & 0x1) == 0x1);
}

static int get_boot_device(void)
{
  uint8_t s;

  s = *((volatile uint8_t *)(0x00020000 + 0x28));

  if (s == 0x3)
    {
      return BOOT_DEVICE_SPINOR;
    }
  else if (s == 0x4)
    {
      return BOOT_DEVICE_SPINAND;
    }
  else if (s == 0x0)
    {
      return BOOT_DEVICE_SDCARD;
    }

  return BOOT_DEVICE_SPINOR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sys_clock_init(void)
{
  set_pll_cpux_axi();
  set_pll_periph0();
  set_ahb();
  set_apb();
  set_dma();
  set_mbus();
  set_module(R528_CCU_BASE + CCU_PLL_PERI0_CTRL_REG);
  set_module(R528_CCU_BASE + CCU_PLL_VIDEO0_CTRL_REG);
  set_module(R528_CCU_BASE + CCU_PLL_VIDEO1_CTRL_REG);
  set_module(R528_CCU_BASE + CCU_PLL_VE_CTRL);
  set_module(R528_CCU_BASE + CCU_PLL_AUDIO0_CTRL_REG);
  set_module(R528_CCU_BASE + CCU_PLL_AUDIO1_CTRL_REG);
}

void sys_dram_init(void)
{
  struct dram_param para;
  uint8_t *p = (void *)&para;
  int i;

  for (i = 0; i < sizeof(para); i++)
    {
      p[i] = 0;
    }

  para.dram_clk = 792;
  para.dram_type = 3;
  para.dram_zq = 0x7b7bfb;
  para.dram_odt_en = 0x00;
  para.dram_para1 = 0x000010d2;
  para.dram_para2 = 0x0000;
  para.dram_mr0 = 0x1c70;
  para.dram_mr1 = 0x042;
  para.dram_mr2 = 0x18;
  para.dram_mr3 = 0x0;
  para.dram_tpr0 = 0x004a2195;
  para.dram_tpr1 = 0x02423190;
  para.dram_tpr2 = 0x0008b061;
  para.dram_tpr3 = 0xb4787896;
  para.dram_tpr4 = 0x0;
  para.dram_tpr5 = 0x48484848;
  para.dram_tpr6 = 0x00000048;
  para.dram_tpr7 = 0x1620121e;
  para.dram_tpr8 = 0x0;
  para.dram_tpr9 = 0x0;
  para.dram_tpr10 = 0x0;
  para.dram_tpr11 = 0x00340000;
  para.dram_tpr12 = 0x00000046;
  para.dram_tpr13 = 0x34000100;
  init_dram(&para);
}

void sys_spinand_init(void)
{
  uint32_t addr;
  uint32_t val;

  /* Config GPIOC2, GPIOC3, GPIOC4 and GPIOC5 */

  addr = 0x02000060 + 0x00;
  val = read32(addr);
  val &= ~(0xf << ((2 & 0x7) << 2));
  val |= ((0x2 & 0xf) << ((2 & 0x7) << 2));
  write32(addr, val);

  val = read32(addr);
  val &= ~(0xf << ((3 & 0x7) << 2));
  val |= ((0x2 & 0xf) << ((3 & 0x7) << 2));
  write32(addr, val);

  val = read32(addr);
  val &= ~(0xf << ((4 & 0x7) << 2));
  val |= ((0x2 & 0xf) << ((4 & 0x7) << 2));
  write32(addr, val);

  val = read32(addr);
  val &= ~(0xf << ((5 & 0x7) << 2));
  val |= ((0x2 & 0xf) << ((5 & 0x7) << 2));
  write32(addr, val);

  /* Deassert spi0 reset */

  addr = 0x0200196c;
  val = read32(addr);
  val |= (1 << 16);
  write32(addr, val);

  /* Open the spi0 gate */

  addr = 0x02001940;
  val = read32(addr);
  val |= (1 << 31);
  write32(addr, val);

  /* Open the spi0 bus gate */

  addr = 0x0200196c;
  val = read32(addr);
  val |= (1 << 0);
  write32(addr, val);

  /* Select pll-periph0 for spi0 clk */

  addr = 0x02001940;
  val = read32(addr);
  val &= ~(0x3 << 24);
  val |= 0x1 << 24;
  write32(addr, val);

  /* Set clock pre divide ratio, divided by 1 */

  addr = 0x02001940;
  val = read32(addr);
  val &= ~(0x3 << 8);
  val |= 0x0 << 8;
  write32(addr, val);

  /* Set clock divide ratio, divided by 6 */

  addr = 0x02001940;
  val = read32(addr);
  val &= ~(0xf << 0);
  val |= (6 - 1) << 0;
  write32(addr, val);

  /* Set spi clock rate control register,
   * divided by 2
   */

  addr = 0x04025000;
  write32(addr + SPI_CCR, 0x1000);

  /* Enable spi0 and do a soft reset */

  addr = 0x04025000;
  val = read32(addr + SPI_GCR);
  val |= (1 << 31) | (1 << 7) | (1 << 1) | (1 << 0);
  write32(addr + SPI_GCR, val);
  while (read32(addr + SPI_GCR) & (1 << 31))
    ;

  val = read32(addr + SPI_TCR);
  val &= ~(0x3 << 0);
  val |= (1 << 6) | (1 << 2);
  write32(addr + SPI_TCR, val);

  val = read32(addr + SPI_FCR);
  val |= (1 << 31) | (1 << 15);
  write32(addr + SPI_FCR, val);
}

void sys_spinand_exit(void)
{
  uint32_t addr = 0x04025000;
  uint32_t val;

  /* Disable the spi0 controller */

  val = read32(addr + SPI_GCR);
  val &= ~((1 << 1) | (1 << 0));
  write32(addr + SPI_GCR, val);
}

void sys_spinand_read(int addr, void *buf, int count)
{
  uint8_t tx[4];
  uint32_t pa;
  uint32_t ca;
  int n;

  while (count > 0)
    {
      pa = addr >> SPINAND_PAGE_BITS;
      ca = addr & SPINAND_PAGE_MASK;
      n = count > (SPINAND_PAGE_SIZE - ca) ?
          (SPINAND_PAGE_SIZE - ca) : count;
      tx[0] = 0x13;
      tx[1] = (uint8_t)(pa >> 16);
      tx[2] = (uint8_t)(pa >> 8);
      tx[3] = (uint8_t)(pa >> 0);
      sys_spi_select();
      sys_spi_write_then_read(tx, 4, 0, 0);
      sys_spi_deselect();
      sys_spinand_wait();
      tx[0] = 0x03;
      tx[1] = (uint8_t)(ca >> 8);
      tx[2] = (uint8_t)(ca >> 0);
      tx[3] = 0x0;
      sys_spi_select();
      sys_spi_write_then_read(tx, 4, buf, n);
      sys_spi_deselect();
      sys_spinand_wait();
      addr += n;
      buf += n;
      count -= n;
    }
}

void sys_uart_init(void)
{
  uint32_t addr;
  uint32_t val;

  /* Config GPIOE2 and GPIOE3 to txd0 and rxd0 */

  addr = 0x020000c0 + 0x0;
  val = read32(addr);
  val &= ~(0xf << ((2 & 0x7) << 2));
  val |= ((0x6 & 0xf) << ((2 & 0x7) << 2));
  write32(addr, val);

  val = read32(addr);
  val &= ~(0xf << ((3 & 0x7) << 2));
  val |= ((0x6 & 0xf) << ((3 & 0x7) << 2));
  write32(addr, val);

  /* Open the clock gate for uart0 */

  addr = 0x0200190c;
  val = read32(addr);
  val |= 1 << 0;
  write32(addr, val);

  /* Deassert uart0 reset */

  addr = 0x0200190c;
  val = read32(addr);
  val |= 1 << 16;
  write32(addr, val);

  /* Config uart0 to 115200-8-1-0 */

  addr = 0x02500000;
  write32(addr + 0x04, 0x0);
  write32(addr + 0x08, 0xf7);
  write32(addr + 0x10, 0x0);
  val = read32(addr + 0x0c);
  val |= (1 << 7);
  write32(addr + 0x0c, val);
  write32(addr + 0x00, 0xd & 0xff);
  write32(addr + 0x04, (0xd >> 8) & 0xff);
  val = read32(addr + 0x0c);
  val &= ~(1 << 7);
  write32(addr + 0x0c, val);
  val = read32(addr + 0x0c);
  val &= ~0x1f;
  val |= (0x3 << 0) | (0 << 2) | (0x0 << 3);
  write32(addr + 0x0c, val);
}

void sys_jtag_init(void)
{
  uint32_t addr;
  uint32_t val;

  /* Config GPIOF0, GPIOF1, GPIOF3 and GPIOF5 */

  addr = 0x020000f0 + 0x00;
  val = read32(addr);
  val &= ~(0xf << ((0 & 0x7) << 2));
  val |= ((0x3 & 0xf) << ((0 & 0x7) << 2));
  write32(addr, val);

  val = read32(addr);
  val &= ~(0xf << ((1 & 0x7) << 2));
  val |= ((0x3 & 0xf) << ((1 & 0x7) << 2));
  write32(addr, val);

  val = read32(addr);
  val &= ~(0xf << ((3 & 0x7) << 2));
  val |= ((0x3 & 0xf) << ((3 & 0x7) << 2));
  write32(addr, val);

  val = read32(addr);
  val &= ~(0xf << ((5 & 0x7) << 2));
  val |= ((0x3 & 0xf) << ((5 & 0x7) << 2));
  write32(addr, val);
}

void sys_copyself(void)
{
  int d;
  void *mem;
  uint32_t size;

  /* Initial system jtag, uart, clock and ddr */

  sys_jtag_init();
  sys_uart_init();
  sys_clock_init();
  sys_dram_init();

  d = get_boot_device();
  if (d == BOOT_DEVICE_SPINOR)
    {
    }
  else if (d == BOOT_DEVICE_SPINAND)
    {
      mem = (void *)__image_start;
      size = __image_end - __image_start;

      /* sys_mmu_init(); */

      sys_spinand_init();
      sys_spinand_read(1048576, mem, size);
      sys_spinand_exit();
    }
  else if (d == BOOT_DEVICE_SDCARD)
    {
    }
}
