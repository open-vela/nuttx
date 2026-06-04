/****************************************************************************
 * arch/arm/src/t113/hardware/t113_ce.h
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
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied.  See the License for the specific language governing
 * permissions and limitations under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_CE_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_CE_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CE base addresses */

#define T113_CE_NS_BASE           0x03040000  /* Non-secure CE */
#define T113_CE_S_BASE            0x03040800  /* Secure CE */
#define T113_CE_KEY_SRAM          0x03041000  /* Key SRAM */

/* Use non-secure CE by default */

#define T113_CE_BASE              T113_CE_NS_BASE

/* CE register offsets */

#define T113_CE_TDA_OFFSET        0x0000  /* Task Descriptor Address */
#define T113_CE_ICR_OFFSET        0x0008  /* Interrupt Control Register */
#define T113_CE_ISR_OFFSET        0x000c  /* Interrupt Status Register */
#define T113_CE_TLR_OFFSET        0x0010  /* Task Load Register */
#define T113_CE_TSR_OFFSET        0x0014  /* Task Status Register */
#define T113_CE_ESR_OFFSET        0x0018  /* Error Status Register */
#define T113_CE_CSA_OFFSET        0x0024  /* Current Source Address */
#define T113_CE_CDA_OFFSET        0x0028  /* Current Destination Address */
#define T113_CE_TPR_OFFSET        0x002c  /* Throughput Register */

/* CE register addresses */

#define T113_CE_TDA               (T113_CE_BASE + T113_CE_TDA_OFFSET)
#define T113_CE_ICR               (T113_CE_BASE + T113_CE_ICR_OFFSET)
#define T113_CE_ISR               (T113_CE_BASE + T113_CE_ISR_OFFSET)
#define T113_CE_TLR               (T113_CE_BASE + T113_CE_TLR_OFFSET)
#define T113_CE_TSR               (T113_CE_BASE + T113_CE_TSR_OFFSET)
#define T113_CE_ESR               (T113_CE_BASE + T113_CE_ESR_OFFSET)
#define T113_CE_CSA               (T113_CE_BASE + T113_CE_CSA_OFFSET)
#define T113_CE_CDA               (T113_CE_BASE + T113_CE_CDA_OFFSET)
#define T113_CE_TPR               (T113_CE_BASE + T113_CE_TPR_OFFSET)

/* CE_ICR bit definitions - Interrupt Control Register */

#define CE_ICR_CHN_IRQ_EN(n)      (1 << (n))  /* Channel n IRQ enable */

/* CE_ISR bit definitions - Interrupt Status Register (W1C) */

#define CE_ISR_CHN_PEND(n)        (1 << (n))  /* Channel n end pending */

/* CE_TLR bit definitions - Task Load Register */

#define CE_TLR_LOAD               (1 << 0)    /* Load task descriptor */

/* CE_TSR bit definitions - Task Status Register */

#define CE_TSR_CHN_MASK           0x3         /* Running channel mask */

/* CE_ESR bit definitions - Error Status Register (W1C)
 * Each channel has 4-bit error field:
 *   bit0: algorithm not supported
 *   bit1: data length error
 *   bit2: keysram access error
 *   bit3: reserved
 */

#define CE_ESR_CHN_SHIFT(n)       ((n) * 4)
#define CE_ESR_CHN_MASK(n)        (0xf << CE_ESR_CHN_SHIFT(n))
#define CE_ESR_ALG_NOTSUP         (1 << 0)
#define CE_ESR_DATA_LEN_ERR       (1 << 1)
#define CE_ESR_KEYSRAM_ERR        (1 << 2)

/* Task Descriptor: Common Control word (word 1) */

/* Algorithm type [6:0] */

#define CE_ALG_AES                0x00
#define CE_ALG_DES                0x01
#define CE_ALG_3DES               0x02
#define CE_ALG_MD5                0x10
#define CE_ALG_SHA1               0x11
#define CE_ALG_SHA224             0x12
#define CE_ALG_SHA256             0x13
#define CE_ALG_SHA384             0x14
#define CE_ALG_SHA512             0x15
#define CE_ALG_HMAC_SHA1          0x16
#define CE_ALG_HMAC_SHA256        0x17
#define CE_ALG_RSA                0x20
#define CE_ALG_TRNG               0x30
#define CE_ALG_PRNG               0x31

/* Operation direction [8] */

#define CE_DIR_ENCRYPT            (0 << 8)
#define CE_DIR_DECRYPT            (1 << 8)

/* Last HMAC plaintext [15] */

#define CE_HMAC_LAST              (1 << 15)

/* IV mode [16] */

#define CE_IV_MODE_CONST          (0 << 16)   /* Use FIPS-180 constants */
#define CE_IV_MODE_INPUT          (1 << 16)   /* Use input IV */

/* Interrupt enable [31] */

#define CE_COMM_INT_EN            (1u << 31)

/* Task Descriptor: Symmetric Control word (word 2) */

/* AES key size [1:0] */

#define CE_AES_KEYSIZE_128        0x00
#define CE_AES_KEYSIZE_192        0x01
#define CE_AES_KEYSIZE_256        0x02

/* CTR width [3:2] */

#define CE_CTR_WIDTH_16           (0 << 2)
#define CE_CTR_WIDTH_32           (1 << 2)
#define CE_CTR_WIDTH_64           (2 << 2)
#define CE_CTR_WIDTH_128          (3 << 2)

/* Algorithm mode [11:8] */

#define CE_MODE_ECB               (0 << 8)
#define CE_MODE_CBC               (1 << 8)
#define CE_MODE_CTR               (2 << 8)
#define CE_MODE_CTS               (3 << 8)
#define CE_MODE_OFB               (4 << 8)
#define CE_MODE_CFB               (5 << 8)

/* CTS last package flag [16] */

#define CE_CTS_LAST               (1 << 16)

/* CFB width [19:18] */

#define CE_CFB_WIDTH_1            (0 << 18)
#define CE_CFB_WIDTH_8            (1 << 18)
#define CE_CFB_WIDTH_64           (2 << 18)
#define CE_CFB_WIDTH_128          (3 << 18)

/* Key select [23:20] */

#define CE_KEY_SELECT_INPUT       (0 << 20)   /* Use CE_KEY (normal) */
#define CE_KEY_SELECT_SSK         (1 << 20)
#define CE_KEY_SELECT_HUK         (2 << 20)
#define CE_KEY_SELECT_RSSK        (3 << 20)

/* Maximum scatter-gather segments */

#define CE_MAX_SG_SEGS            8

/* Task descriptor channel count */

#define CE_NUM_CHANNELS           4

/* Polling timeout (microseconds) */

#define CE_POLL_TIMEOUT_US        1000000     /* 1 second */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* T113 CE Task Descriptor - 44 words (176 bytes)
 *
 * Layout from user manual section 10.1.3.11:
 *   Word 0:     Task ID (channel in bits[3:0])
 *   Word 1:     Common Control
 *   Word 2:     Symmetric Control
 *   Word 3:     Asymmetric Control
 *   Word 4:     Key Address
 *   Word 5:     IV Address
 *   Word 6:     CTR Address
 *   Word 7:     Data Length (words for most, bytes for AES-CTS)
 *   Word 8-15:  Source Address [0..7]
 *   Word 16-23: Source Data Length [0..7]
 *               (words; manual says bytes -- BSP uses /4)
 *   Word 24-31: Destination Address [0..7]
 *   Word 32-39: Destination Data Length [0..7] (words; same caveat)
 *   Word 40:    Next Descriptor Address (0 = last)
 *   Word 41-43: Reserved
 */

/* Scatter-gather entry: address + length pair */

struct t113_ce_scatter_s
{
  uint32_t addr;              /* Physical address (word-aligned) */
  uint32_t len;               /* Length in words. Exception: bytes for AES-CTS.
                               * Confirmed by Linux BSP ce_common.h (CE V3.1).
                               * Note: User Manual says "bytes" but BSP uses /4.
                               */
};

/* T113 CE Task Descriptor - matches hardware layout from Linux BSP.
 *
 * Layout (non-CE_BYTE_ADDR variant):
 *   Word 0:     Task ID (channel in bits[3:0])
 *   Word 1:     Common Control
 *   Word 2:     Symmetric Control
 *   Word 3:     Asymmetric Control
 *   Word 4:     Key Address
 *   Word 5:     IV Address
 *   Word 6:     CTR Address
 *   Word 7:     Data Length (in words, except AES-CTS: bytes)
 *   Words 8-23: Source scatter entries [0..7] (addr+len pairs)
 *   Words 24-39: Destination scatter entries [0..7] (addr+len pairs)
 *   Word 40:    Next Descriptor Address (0 = last)
 *   Words 41-43: Reserved
 */

struct t113_ce_task_s
{
  uint32_t task_id;           /* Word 0:  channel id [3:0] */
  uint32_t common_ctl;        /* Word 1:  alg type, dir, IE, etc */
  uint32_t sym_ctl;           /* Word 2:  key size, mode, etc */
  uint32_t asym_ctl;          /* Word 3:  RSA width/mode */
  uint32_t key_addr;          /* Word 4:  physical addr of key */
  uint32_t iv_addr;           /* Word 5:  physical addr of IV */
  uint32_t ctr_addr;          /* Word 6:  physical addr of CTR out */
  uint32_t data_len;          /* Word 7:  data length (words) */

  /* Words 8-23: source scatter-gather */

  struct t113_ce_scatter_s src[CE_MAX_SG_SEGS];

  /* Words 24-39: destination scatter-gather */

  struct t113_ce_scatter_s dst[CE_MAX_SG_SEGS];
  uint32_t next_desc;         /* Word 40: next descriptor addr */
  uint32_t reserved[3];       /* Words 41-43: padding */
};

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_CE_H */
