/****************************************************************************
 * arch/arm/src/t113/hardware/t113_dmic.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_DMIC_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_DMIC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PDM DMIC controller base address (T113-S3 UM section 8.2.4, p710).
 * Distinct from the audio codec block at 0x02030000.
 */

#define T113_DMIC_BASE              0x02031000

/* Register offsets (UM 8.2.4 register list, p710) */

#define T113_DMIC_EN                (T113_DMIC_BASE + 0x0000)
#define T113_DMIC_SR                (T113_DMIC_BASE + 0x0004)
#define T113_DMIC_CTR               (T113_DMIC_BASE + 0x0008)
#define T113_DMIC_DATA              (T113_DMIC_BASE + 0x0010)
#define T113_DMIC_INTC              (T113_DMIC_BASE + 0x0014)
#define T113_DMIC_INTS              (T113_DMIC_BASE + 0x0018)
#define T113_DMIC_RXFIFO_CTR        (T113_DMIC_BASE + 0x001c)
#define T113_DMIC_RXFIFO_STA        (T113_DMIC_BASE + 0x0020)
#define T113_DMIC_CH_NUM            (T113_DMIC_BASE + 0x0024)
#define T113_DMIC_CH_MAP            (T113_DMIC_BASE + 0x0028)
#define T113_DMIC_CNT               (T113_DMIC_BASE + 0x002c)
#define T113_DMIC_DATA01_VOL        (T113_DMIC_BASE + 0x0030)
#define T113_DMIC_DATA23_VOL        (T113_DMIC_BASE + 0x0034)
#define T113_DMIC_HPF_EN_CTR        (T113_DMIC_BASE + 0x0038)
#define T113_DMIC_HPF_COEF_REG      (T113_DMIC_BASE + 0x003c)
#define T113_DMIC_HPF_GAIN_REG      (T113_DMIC_BASE + 0x0040)

/* DMIC_EN bit fields (UM 8.2.5.1, p710-p712).
 *
 * Bits 7:0 are per-channel enables, two bits per DATA lane (Right=odd,
 * Left=even).  Bit 8 is the global controller enable.  Bit 28 enables
 * the audio-subsys RX synchroniser (off in single-DAI mode).
 */

#define DMIC_EN_RX_SYNC_EN_START    (1u << 29)
#define DMIC_EN_RX_SYNC_EN          (1u << 28)
#define DMIC_EN_GLOBAL              (1u << 8)
#define DMIC_EN_DATA3_CHR           (1u << 7)
#define DMIC_EN_DATA3_CHL           (1u << 6)
#define DMIC_EN_DATA2_CHR           (1u << 5)
#define DMIC_EN_DATA2_CHL           (1u << 4)
#define DMIC_EN_DATA1_CHR           (1u << 3)
#define DMIC_EN_DATA1_CHL           (1u << 2)
#define DMIC_EN_DATA0_CHR           (1u << 1)
#define DMIC_EN_DATA0_CHL           (1u << 0)
#define DMIC_EN_CHAN_MASK           (0xffu << 0)

/* DMIC_SR bit fields (UM 8.2.5.2, p712).
 *
 * Selects the audio sample rate the controller emits into RXFIFO.
 * Combined with DMIC_CTR_OSR (128 or 64 fs) determines the PDM_CLK
 * frequency the controller drives on DMIC-CLK.
 */

#define DMIC_SR_SHIFT               0
#define DMIC_SR_MASK                (0x7u << DMIC_SR_SHIFT)
#define DMIC_SR_48K                 (0x0u << DMIC_SR_SHIFT)
#define DMIC_SR_32K                 (0x1u << DMIC_SR_SHIFT)
#define DMIC_SR_24K                 (0x2u << DMIC_SR_SHIFT)
#define DMIC_SR_16K                 (0x3u << DMIC_SR_SHIFT)
#define DMIC_SR_12K                 (0x4u << DMIC_SR_SHIFT)
#define DMIC_SR_8K                  (0x5u << DMIC_SR_SHIFT)

/* DMIC_CTR bit fields (UM 8.2.5.3, p713).
 *
 * Bit 0 selects oversample rate (PDM_CLK / fs):
 *   0 = 128x  -- 8 kHz .. 24 kHz target rates
 *   1 =  64x  -- 16 kHz .. 48 kHz target rates
 * Bits 7:4 swap the L/R sample inside each DATAn lane (typical PDM
 * pairs use one half per cycle, this lets two mics share one DATA pin
 * without an external multiplexer).
 * Bits 10:8 program a startup delay before RXFIFO sees valid data,
 * giving the digital decimator time to flush the half-band filter
 * residue from the previous power-down (matches sun50i-dmic default
 * of "delay disabled" -- the HPF in HPF_EN_CTR removes the residue
 * digitally instead).
 */

#define DMIC_CTR_DMICFDT_SHIFT      9
#define DMIC_CTR_DMICFDT_MASK       (0x3u << DMIC_CTR_DMICFDT_SHIFT)
#define DMIC_CTR_DMICFDT_5MS        (0x0u << DMIC_CTR_DMICFDT_SHIFT)
#define DMIC_CTR_DMICFDT_10MS       (0x1u << DMIC_CTR_DMICFDT_SHIFT)
#define DMIC_CTR_DMICFDT_20MS       (0x2u << DMIC_CTR_DMICFDT_SHIFT)
#define DMIC_CTR_DMICFDT_30MS       (0x3u << DMIC_CTR_DMICFDT_SHIFT)
#define DMIC_CTR_DMICDFEN           (1u << 8)
#define DMIC_CTR_DATA3_LR_SWAP      (1u << 7)
#define DMIC_CTR_DATA2_LR_SWAP      (1u << 6)
#define DMIC_CTR_DATA1_LR_SWAP      (1u << 5)
#define DMIC_CTR_DATA0_LR_SWAP      (1u << 4)
#define DMIC_CTR_OVERSAMPLE_64X     (1u << 0)

/* DMIC_INTC bit fields (UM 8.2.5.5, p714).
 *
 * RXFIFO_DRQ_EN binds the FIFO trigger level to the system DMA
 * controller via DRQ_DMIC=8.  Driver does not enable the IRQ paths
 * (DATA_IRQ / OVERRUN_IRQ) -- DMA half-buffer / full-buffer events
 * are sufficient for the audio_lowerhalf loop and overrun is silently
 * dropped (matches mainline sun50i-dmic).
 */

#define DMIC_INTC_RXFIFO_DRQ_EN     (1u << 2)
#define DMIC_INTC_RXFIFO_OVERRUN_EN (1u << 1)
#define DMIC_INTC_DATA_IRQ_EN       (1u << 0)

/* DMIC_INTS bit fields (UM 8.2.5.6, p714).
 *
 * R/W1C: write 1 to clear the corresponding pending bit.  Defined
 * for symmetry with INTC; not used by the driver in M3 because IRQs
 * are disabled (DMA cyclic mode handles the data path).
 */

#define DMIC_INTS_RXFIFO_OVERRUN    (1u << 1)
#define DMIC_INTS_RXFIFO_DATA       (1u << 0)

/* DMIC_RXFIFO_CTR bit fields (UM 8.2.5.7, p715).
 *
 * Bit 31 (FLUSH) is W1C self-clearing -- write 1 to drain any stale
 * samples left over from a previous capture session.
 *
 * Bit 9 (RXFIFO_MODE) controls how a 21-bit decimator output (always
 * RXFIFO_O[20:0] internally) maps to the 32-bit DMIC_DATA register.
 * Mode 1 sign-extends MSB so a 16-bit DMA read picks up the high
 * 16 bits of a sign-extended sample, matching audio_lowerhalf S16_LE
 * convention used by nxrecorder.  Mode 0 zero-pads the LSBs and is
 * unused (it would require an arithmetic right shift in software).
 *
 * Bit 8 (Sample_Resolution) toggles the FIFO word width: 16 bit (0)
 * or 24 bit (1).  M3 only validates 16 bit; 24 bit hooks left for
 * future high-resolution capture.
 *
 * Bits 7:0 (TRG_LEVEL) raise DRQ when the FIFO holds more than this
 * many samples.  Default 0x40 (= 64 samples), same as mainline.
 */

#define DMIC_RXFIFO_CTR_FLUSH       (1u << 31)
#define DMIC_RXFIFO_CTR_MODE_LSB    (0u << 9)   /* zero-pad LSB */
#define DMIC_RXFIFO_CTR_MODE_MSB    (1u << 9)   /* sign-extend MSB */
#define DMIC_RXFIFO_CTR_MODE_MASK   (1u << 9)
#define DMIC_RXFIFO_CTR_SAMPLE_16   (0u << 8)
#define DMIC_RXFIFO_CTR_SAMPLE_24   (1u << 8)
#define DMIC_RXFIFO_CTR_SAMPLE_MASK (1u << 8)
#define DMIC_RXFIFO_CTR_TRG_SHIFT   0
#define DMIC_RXFIFO_CTR_TRG_MASK    (0xffu << DMIC_RXFIFO_CTR_TRG_SHIFT)

/* DMIC_RXFIFO_STA bit fields (UM 8.2.5.8, p715-p716).
 *
 * Bits 7:0 hold the current valid-sample word counter.  Read-only
 * for monitoring; not part of the M3 driver path but kept for parity
 * with mainline driver (regmap_read in dmic_runtime_resume).
 */

#define DMIC_RXFIFO_STA_CNT_MASK    (0xffu << 0)

/* DMIC_CH_NUM bit fields (UM 8.2.5.9, p716).
 *
 * Programmed value is "(N - 1)" where N is the active channel count
 * (1..8).  Default 0x1 = 2 channels; we override per-configure.
 */

#define DMIC_CH_NUM_SHIFT           0
#define DMIC_CH_NUM_MASK            (0x7u << DMIC_CH_NUM_SHIFT)

/* DMIC_CH_MAP bit fields (UM 8.2.5.10, p716-p719).
 *
 * Eight 4-bit sub-fields (CH0..CH7) each select which physical input
 * (DATA0L .. DATA3R) routes into that internal channel.  Default is
 * the identity map (0x76543210).  M3 keeps the default because the
 * MQ-R only wires DATA0 and the controller only consumes the first
 * (cur_channels) entries.
 */

#define DMIC_CH_MAP_CH0_SHIFT       0
#define DMIC_CH_MAP_CH1_SHIFT       4
#define DMIC_CH_MAP_CH2_SHIFT       8
#define DMIC_CH_MAP_CH3_SHIFT       12
#define DMIC_CH_MAP_CH4_SHIFT       16
#define DMIC_CH_MAP_CH5_SHIFT       20
#define DMIC_CH_MAP_CH6_SHIFT       24
#define DMIC_CH_MAP_CH7_SHIFT       28
#define DMIC_CH_MAP_DATA0L          0x0u
#define DMIC_CH_MAP_DATA0R          0x1u
#define DMIC_CH_MAP_DATA1L          0x2u
#define DMIC_CH_MAP_DATA1R          0x3u
#define DMIC_CH_MAP_DATA2L          0x4u
#define DMIC_CH_MAP_DATA2R          0x5u
#define DMIC_CH_MAP_DATA3L          0x6u
#define DMIC_CH_MAP_DATA3R          0x7u
#define DMIC_CH_MAP_DEFAULT         0x76543210u

/* DMIC_CNT bit fields (UM 8.2.5.11, p719).
 *
 * Free-running 32-bit counter incremented every sample written to
 * RXFIFO.  Used for A/V sync; M3 zeroes it at start and otherwise
 * leaves it alone.
 */

/* DATA0_DATA1_VOL_CTR / DATA2_DATA3_VOL_CTR bit fields
 * (UM 8.2.5.12 + 8.2.5.13, p720-p722).
 *
 * Four 8-bit sub-fields per register; one per (DATAn, L/R).  Default
 * 0xa0a0a0a0 = 0 dB on every channel.  0x00 = mute, 0xa0 = 0 dB,
 * 0xff = +71.25 dB.  Each step is 0.75 dB.  Symbolic value below
 * gives 0 dB at all four channels of either register.
 */

#define DMIC_VOL_DATA_R_SHIFT       0
#define DMIC_VOL_DATA_L_SHIFT       8
#define DMIC_VOL_DATA_OTHER_R_SHIFT 16
#define DMIC_VOL_DATA_OTHER_L_SHIFT 24
#define DMIC_VOL_VALUE_MASK         0xffu
#define DMIC_VOL_VALUE_MUTE         0x00u
#define DMIC_VOL_VALUE_0DB          0xa0u
#define DMIC_VOL_DEFAULT_0DB_ALL    0xa0a0a0a0u

/* HPF_EN_CTR bit fields (UM 8.2.5.14, p723).
 *
 * Eight per-channel bits; enabling them inserts a digital high-pass
 * filter (DC blocker) in the decimator output path.  PDM mics (e.g.
 * ST MP34DT06J, Knowles SPH0641LU4H-1) have a few-mV DC bias on the
 * digital sample stream that, without HPF, rides on every captured
 * sample as a fixed offset.  Mainline sun50i-dmic enables HPF on
 * every active channel; we mirror that.
 */

#define DMIC_HPF_EN_DATA3_CHR       (1u << 7)
#define DMIC_HPF_EN_DATA3_CHL       (1u << 6)
#define DMIC_HPF_EN_DATA2_CHR       (1u << 5)
#define DMIC_HPF_EN_DATA2_CHL       (1u << 4)
#define DMIC_HPF_EN_DATA1_CHR       (1u << 3)
#define DMIC_HPF_EN_DATA1_CHL       (1u << 2)
#define DMIC_HPF_EN_DATA0_CHR       (1u << 1)
#define DMIC_HPF_EN_DATA0_CHL       (1u << 0)

/* HPF_COEF_REG / HPF_GAIN_REG (UM 8.2.5.15 + 8.2.5.16, p724).
 *
 * Defaults are used; the reset values 0x00ffaa45 / 0x00ffd522 give a
 * cutoff well below the audio band (~3 Hz at 16 kHz) which removes
 * DC + sub-audible noise without affecting voice.  The driver does
 * not need to write either register today; constants are kept for
 * future calibration work.
 */

#define DMIC_HPF_COEF_DEFAULT       0x00ffaa45u
#define DMIC_HPF_GAIN_DEFAULT       0x00ffd522u

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_DMIC_H */
