/****************************************************************************
 * arch/loongarch/src/ls2k0300/hardware/ls2k0300_sdio.h
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

#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_SDIO_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_SDIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Each SDIO/eMMC controller owns a 32KB register window:
 *
 *   0x000 - 0x3ff  controller registers
 *   0x400 - 0x7ff  DMA write (memory -> card) configuration
 *   0x800 - 0xfff  DMA read  (card -> memory) configuration
 *
 * See the LS2K0300 user manual, chapter 15 "SDIO controller", section 15.3.
 */

#define LS2K0300_SDIO_WDMA_OFFSET  0x400
#define LS2K0300_SDIO_RDMA_OFFSET  0x800

/* Controller registers */

#define LS2K0300_SDIO_CON          0x00   /* SDI_CON       control      */
#define LS2K0300_SDIO_PRE          0x04   /* SDI_PRE       prescaler    */
#define LS2K0300_SDIO_CMDARG       0x08   /* SDI_CMD_ARG   argument     */
#define LS2K0300_SDIO_CMDCON       0x0c   /* SDI_CMD_CON   command      */
#define LS2K0300_SDIO_CMDSTA       0x10   /* SDI_CMD_STA   status       */
#define LS2K0300_SDIO_RSP0         0x14   /* SDI_RSP0      response 0   */
#define LS2K0300_SDIO_RSP1         0x18   /* SDI_RSP1      response 1   */
#define LS2K0300_SDIO_RSP2         0x1c   /* SDI_RSP2      response 2   */
#define LS2K0300_SDIO_RSP3         0x20   /* SDI_RSP3      response 3   */
#define LS2K0300_SDIO_DTIMER       0x24   /* SDI_DTIMER    data timeout */
#define LS2K0300_SDIO_BSIZE        0x28   /* SDI_BSIZE     block size   */
#define LS2K0300_SDIO_DATCON       0x2c   /* SDI_DAT_CON   data control */
#define LS2K0300_SDIO_DATCNT       0x30   /* SDI_DAT_CNT   data counter */
#define LS2K0300_SDIO_DATSTA       0x34   /* SDI_DAT_STA   data status  */
#define LS2K0300_SDIO_FIFOSTA      0x38   /* SDI_FIFO_STA  FIFO status  */
#define LS2K0300_SDIO_INTSTA       0x3c   /* SDI_INT_MASK  int status   */
#define LS2K0300_SDIO_WRDAT        0x40   /* SDI_DAT       data (DMA)   */
#define LS2K0300_SDIO_INTEN        0x64   /* SDI_INT_EN    int enable   */

/* SDI_CON */

#define SDIO_CON_ENCLK             (1 << 0)   /* SD clock output enable */
#define SDIO_CON_SOFTRST           (1 << 8)   /* module reset, self clear */

/* SDI_PRE */

#define SDIO_PRE_MASK              0xff       /* clock divisor */
#define SDIO_PRE_REVCLOCK          (1 << 31)  /* sample on the falling edge */

/* SDI_CMD_CON */

#define SDIO_CMDCON_INDEX_MASK     0x3f       /* command index */
#define SDIO_CMDCON_SENDERHOST     (1 << 6)   /* host sends the command   */
#define SDIO_CMDCON_START          (1 << 8)   /* start, self clear        */
#define SDIO_CMDCON_WAITRSP        (1 << 9)   /* wait for a response      */
#define SDIO_CMDCON_LONGRSP        (1 << 10)  /* 136 bit response         */
#define SDIO_CMDCON_AUTOSTOP       (1 << 12)  /* send CMD12 in hardware   */
#define SDIO_CMDCON_CHECKCRC       (1 << 13)  /* check the response CRC   */
#define SDIO_CMDCON_SDIOEN         (1 << 14)  /* auto stop uses CMD52     */

/* SDI_CMD_STA */

#define SDIO_CMDSTA_RSPINDEX_MASK  0xff       /* response index           */
#define SDIO_CMDSTA_CMDON          (1 << 8)   /* command in progress      */
#define SDIO_CMDSTA_RSPFIN         (1 << 9)   /* response received        */
#define SDIO_CMDSTA_CMDTOUT        (1 << 10)  /* response timeout         */
#define SDIO_CMDSTA_CMDEND         (1 << 11)  /* command sent             */
#define SDIO_CMDSTA_RSPCRCERR      (1 << 12)  /* response CRC error       */
#define SDIO_CMDSTA_AUTOSTOP       (1 << 13)  /* hardware sent the stop   */
#define SDIO_CMDSTA_CMDSENTFIN     (1 << 14)  /* command + response done  */

/* SDI_DAT_CON */

#define SDIO_DATCON_BLKNUM_MASK    0xfff      /* number of blocks         */
#define SDIO_DATCON_DTST           (1 << 14)  /* start the transfer       */
#define SDIO_DATCON_DMAEN          (1 << 15)  /* use DMA                  */
#define SDIO_DATCON_WIDEMODE       (1 << 16)  /* 4 bit bus                */
#define SDIO_DATCON_WIDEMODE8B     (1 << 26)  /* 8 bit bus                */

/* SDI_DAT_STA */

#define SDIO_DATSTA_RXDATON        (1 << 0)   /* receiving                */
#define SDIO_DATSTA_TXDATON        (1 << 1)   /* transmitting             */
#define SDIO_DATSTA_PROGERR        (1 << 2)   /* programming error        */
#define SDIO_DATSTA_BUSYFIN        (1 << 3)   /* busy released            */
#define SDIO_DATSTA_DATFIN         (1 << 4)   /* transfer finished        */
#define SDIO_DATSTA_DATTOUT        (1 << 5)   /* data timeout             */
#define SDIO_DATSTA_DATCRC         (1 << 6)   /* receive CRC error        */
#define SDIO_DATSTA_CRCSTA         (1 << 7)   /* card reported CRC status */
#define SDIO_DATSTA_SDIOINT        (1 << 8)   /* SDIO interrupt           */
#define SDIO_DATSTA_R1BFIN         (1 << 12)  /* busy released (R1b)      */
#define SDIO_DATSTA_R1BTOUT        (1 << 14)  /* busy timeout (R1b)       */

/* SDI_FIFO_STA */

#define SDIO_FIFOSTA_RXEMPTY       (1 << 7)
#define SDIO_FIFOSTA_RXFULL        (1 << 8)
#define SDIO_FIFOSTA_TXEMPTY       (1 << 10)
#define SDIO_FIFOSTA_TXFULL        (1 << 11)

/* SDI_INT_MASK: interrupt status, write 1 to clear.
 *
 * Bits 6 and 0 are cleared by the hardware.
 */

#define SDIO_INT_DATFIN            (1 << 0)   /* data finished       */
#define SDIO_INT_DATTOUT           (1 << 1)   /* data timeout        */
#define SDIO_INT_DATCRC            (1 << 2)   /* data CRC error      */
#define SDIO_INT_CRCSTA            (1 << 3)   /* card CRC status err */
#define SDIO_INT_PROGERR           (1 << 4)   /* programming error   */
#define SDIO_INT_SDIO              (1 << 5)   /* SDIO interrupt      */
#define SDIO_INT_CMDFIN            (1 << 6)   /* command finished    */
#define SDIO_INT_CMDTOUT           (1 << 7)   /* command timeout     */
#define SDIO_INT_RSPCRC            (1 << 8)   /* response CRC error  */
#define SDIO_INT_R1BFIN            (1 << 9)   /* busy released       */

/* Command phase completion: the controller lumps the command timeout, the
 * response CRC error, the response received and the busy released events
 * into one status word.  All of them are write-1-to-clear.
 */

#define SDIO_INT_CMDMASK   (SDIO_INT_CMDFIN | SDIO_INT_CMDTOUT | \
                            SDIO_INT_RSPCRC | SDIO_INT_R1BFIN)

#define SDIO_INT_CMDERR    (SDIO_INT_CMDTOUT | SDIO_INT_RSPCRC)

/* Data phase completion */

#define SDIO_INT_DATAMASK  (SDIO_INT_DATFIN | SDIO_INT_DATTOUT | \
                            SDIO_INT_DATCRC | SDIO_INT_CRCSTA | \
                            SDIO_INT_PROGERR)

#define SDIO_INT_DATAERR   (SDIO_INT_DATTOUT | SDIO_INT_DATCRC | \
                            SDIO_INT_CRCSTA | SDIO_INT_PROGERR)

#define SDIO_INT_ALL       0x3ff

/* SDI_INT_EN mirrors the bit positions of SDI_INT_MASK */

#define SDIO_INTEN_ALL     0x3ff

/* The transfer size is expressed in 32 bit words and is limited by the
 * 12 bit block counter of SDI_DAT_CON.
 */

#define LS2K0300_SDIO_MAXBLOCKS    4095

/* DMA descriptor (one per contiguous block of data).
 *
 * The descriptor must be 32 byte aligned and lives in memory: the DMA
 * engine fetches it with a bus read.  The layout is the same as the DMA
 * register block at base + 0x400 / base + 0x800.
 */

struct ls2k0300_sdio_dmadesc_s
{
  volatile uint32_t order_addr;     /* 0x00 next descriptor, bit 0 = valid  */
  volatile uint32_t saddr;          /* 0x04 memory address (low 32 bit)     */
  volatile uint32_t daddr;          /* 0x08 device address (SDI_DAT)        */
  volatile uint32_t length;         /* 0x0c transfer length in words        */
  volatile uint32_t step_length;    /* 0x10 gap between two steps           */
  volatile uint32_t step_times;     /* 0x14 number of steps                 */
  volatile uint32_t cmd;            /* 0x18 direction / interrupt mask      */
  volatile uint32_t reserved;       /* 0x1c                                 */
  volatile uint32_t order_addr_hi;  /* 0x20 next descriptor (high 32 bit)   */
  volatile uint32_t saddr_hi;       /* 0x24 memory address (high 32 bit)    */
};

/* DMA_CMD */

#define LS2K0300_DMA_CMD_INTMSK    (1 << 0)   /* unmask the DMA interrupt  */
#define LS2K0300_DMA_CMD_WRITE     (1 << 12)  /* memory -> device          */

/* DMA_ORDER_ADDR_LOW: the descriptor address is 32 byte aligned, the flag
 * bits live in the low 5 bits.
 */

#define LS2K0300_DMA_ORDER_START   (1 << 3)   /* start the DMA             */
#define LS2K0300_DMA_ORDER_64BIT   (1 << 0)   /* 64 bit address mode       */
#define LS2K0300_DMA_ORDER_ALIGN   32

#endif /* __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_SDIO_H */
