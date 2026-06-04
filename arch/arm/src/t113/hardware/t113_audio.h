/****************************************************************************
 * arch/arm/src/t113/hardware/t113_audio.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_AUDIO_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_AUDIO_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Internal Audio Codec base address (T113-S3 user manual section 8.4.5).
 * The codec block hosts both the digital ADC/DAC FIFO + control registers
 * and the analog domain (ADC1/2/3, DAC, MICBIAS, BIAS, POWER, HP, ramp).
 */

#define T113_AUDIO_CODEC_BASE       0x02030000

/* Digital domain - DAC path */

#define T113_AC_DAC_DPC             (T113_AUDIO_CODEC_BASE + 0x0000)
#define T113_AC_DAC_VOL_CTRL        (T113_AUDIO_CODEC_BASE + 0x0004)
#define T113_AC_DAC_FIFOC           (T113_AUDIO_CODEC_BASE + 0x0010)
#define T113_AC_DAC_FIFOS           (T113_AUDIO_CODEC_BASE + 0x0014)
#define T113_AC_DAC_TXDATA          (T113_AUDIO_CODEC_BASE + 0x0020)
#define T113_AC_DAC_CNT             (T113_AUDIO_CODEC_BASE + 0x0024)
#define T113_AC_DAC_DG              (T113_AUDIO_CODEC_BASE + 0x0028)

/* Digital domain - ADC path */

#define T113_AC_ADC_FIFOC           (T113_AUDIO_CODEC_BASE + 0x0030)
#define T113_AC_ADC_VOL_CTRL1       (T113_AUDIO_CODEC_BASE + 0x0034)
#define T113_AC_ADC_FIFOS           (T113_AUDIO_CODEC_BASE + 0x0038)
#define T113_AC_ADC_RXDATA          (T113_AUDIO_CODEC_BASE + 0x0040)
#define T113_AC_ADC_CNT             (T113_AUDIO_CODEC_BASE + 0x0044)
#define T113_AC_ADC_DG              (T113_AUDIO_CODEC_BASE + 0x004c)
#define T113_AC_ADC_DIG_CTRL        (T113_AUDIO_CODEC_BASE + 0x0050)
#define T113_AC_VRA1_SPEEDUP_CTRL   (T113_AUDIO_CODEC_BASE + 0x0054)

/* Digital domain - DAP (DRC + HPF) blocks (M2/future use) */

#define T113_AC_DAC_DAP_CTRL        (T113_AUDIO_CODEC_BASE + 0x00f0)
#define T113_AC_ADC_DAP_CTRL        (T113_AUDIO_CODEC_BASE + 0x00f8)

/* Analog domain (codec_state.lock protected) */

#define T113_AC_ADC1_REG            (T113_AUDIO_CODEC_BASE + 0x0300)
#define T113_AC_ADC2_REG            (T113_AUDIO_CODEC_BASE + 0x0304)
#define T113_AC_ADC3_REG            (T113_AUDIO_CODEC_BASE + 0x0308)
#define T113_AC_DAC_REG             (T113_AUDIO_CODEC_BASE + 0x0310)
#define T113_AC_MICBIAS_REG         (T113_AUDIO_CODEC_BASE + 0x0318)
#define T113_AC_RAMP_REG            (T113_AUDIO_CODEC_BASE + 0x031c)
#define T113_AC_BIAS_REG            (T113_AUDIO_CODEC_BASE + 0x0320)
#define T113_AC_HMIC_CTRL           (T113_AUDIO_CODEC_BASE + 0x0328)
#define T113_AC_HMIC_STS            (T113_AUDIO_CODEC_BASE + 0x032c)
#define T113_AC_HP2_REG             (T113_AUDIO_CODEC_BASE + 0x0340)
#define T113_AC_POWER_REG           (T113_AUDIO_CODEC_BASE + 0x0348)
#define T113_AC_ADC_CUR_REG         (T113_AUDIO_CODEC_BASE + 0x034c)

/* AC_DAC_DPC bit fields (offset 0x0000, default 0x00000000) */

#define AC_DAC_DPC_EN_DA            (1u << 31)  /* DAC digital enable */
#define AC_DAC_DPC_HPF_EN           (1u << 18)  /* DAC HPF enable */
#define AC_DAC_DPC_DVOL_SHIFT       12          /* Digital volume bits 17:12 */
#define AC_DAC_DPC_DVOL_MASK        (0x3fu << AC_DAC_DPC_DVOL_SHIFT)
#define AC_DAC_DPC_HUB_EN           (1u << 0)   /* Audio hub enable */

/* AC_DAC_FIFOC bit fields (offset 0x0010, default 0x00004000) */

#define AC_DAC_FIFOC_DAC_FS_SHIFT   29          /* Sample-rate select 31:29 */
#define AC_DAC_FIFOC_DAC_FS_MASK    (0x7u << AC_DAC_FIFOC_DAC_FS_SHIFT)
#define AC_DAC_FIFOC_DAC_FS_48K     (0x0u << AC_DAC_FIFOC_DAC_FS_SHIFT)
#define AC_DAC_FIFOC_DAC_FS_32K     (0x1u << AC_DAC_FIFOC_DAC_FS_SHIFT)
#define AC_DAC_FIFOC_DAC_FS_24K     (0x2u << AC_DAC_FIFOC_DAC_FS_SHIFT)
#define AC_DAC_FIFOC_DAC_FS_16K     (0x3u << AC_DAC_FIFOC_DAC_FS_SHIFT)
#define AC_DAC_FIFOC_DAC_FS_12K     (0x4u << AC_DAC_FIFOC_DAC_FS_SHIFT)
#define AC_DAC_FIFOC_DAC_FS_8K      (0x5u << AC_DAC_FIFOC_DAC_FS_SHIFT)
#define AC_DAC_FIFOC_FIR_VER        (1u << 28)  /* 0=64-tap 1=32-tap */
#define AC_DAC_FIFOC_SEND_LASAT     (1u << 26)  /* 1=last sample on underrun */
#define AC_DAC_FIFOC_FIFO_MODE_SHIFT 24
#define AC_DAC_FIFOC_FIFO_MODE_MASK (0x3u << AC_DAC_FIFOC_FIFO_MODE_SHIFT)

/* TX FIFO_MODE encoding (UM 8.4.6.3, p775).  For 16-bit transmitted samples:
 *   00/10: FIFO_I[19:0] = {TXDATA[31:16], 4'b0}  (sample in upper half)
 *   01/11: FIFO_I[19:0] = {TXDATA[15:0],  4'b0}  (sample in lower half)
 * We use mode 01 so a 16-bit DMA write to TXDATA places the sample in
 * the bottom half of the FIFO word -- mirroring what RX_FIFO_MODE=1
 * does on the ADC side and matching DMA dst_width = DMAC_WIDTH_16BIT.
 */

#define AC_DAC_FIFOC_FIFO_MODE_16   (0x1u << AC_DAC_FIFOC_FIFO_MODE_SHIFT)
#define AC_DAC_FIFOC_DRQ_CLR_SHIFT  21
#define AC_DAC_FIFOC_DRQ_CLR_MASK   (0x3u << AC_DAC_FIFOC_DRQ_CLR_SHIFT)
#define AC_DAC_FIFOC_TX_TRIG_SHIFT  8           /* TX trigger level 14:8 */
#define AC_DAC_FIFOC_TX_TRIG_MASK   (0x7fu << AC_DAC_FIFOC_TX_TRIG_SHIFT)
#define AC_DAC_FIFOC_DAC_MONO_EN    (1u << 6)   /* 1=mono, 128 levels */
#define AC_DAC_FIFOC_TX_BITS_20     (1u << 5)   /* 0=16bit 1=20bit */
#define AC_DAC_FIFOC_DAC_DRQ_EN     (1u << 4)   /* TX DRQ enable */
#define AC_DAC_FIFOC_DAC_IRQ_EN     (1u << 3)   /* TX empty IRQ enable */
#define AC_DAC_FIFOC_FIFO_UR_IRQ_EN (1u << 2)   /* Underrun IRQ */
#define AC_DAC_FIFOC_FIFO_OV_IRQ_EN (1u << 1)   /* Overrun IRQ */
#define AC_DAC_FIFOC_FIFO_FLUSH     (1u << 0)   /* Self-clearing flush */

/* AC_DAC_FIFOS bit fields (offset 0x0014, default 0x00808008) */

#define AC_DAC_FIFOS_TX_EMPTY       (1u << 23)  /* TX FIFO has >=1 free word */
#define AC_DAC_FIFOS_TXE_CNT_SHIFT  8           /* TX empty word count 22:8 */
#define AC_DAC_FIFOS_TXE_CNT_MASK   (0x7fffu << AC_DAC_FIFOS_TXE_CNT_SHIFT)
#define AC_DAC_FIFOS_TXE_INT        (1u << 3)   /* TX empty IRQ pending */
#define AC_DAC_FIFOS_TXU_INT        (1u << 2)   /* TX underrun IRQ pending */
#define AC_DAC_FIFOS_TXO_INT        (1u << 1)   /* TX overrun IRQ pending */

/* AC_DAC_VOL_CTRL bit fields (offset 0x0004, default 0x0000a0a0).  The
 * digital volume scale is 0.75 dB/step from -119.25 dB (0x01) to +71.25 dB
 * (0xff); 0xa0 = 0 dB unity gain, 0x00 = mute.  When DAC_VOL_SEL = 0 the
 * DPC.DVOL field (-1.16 dB/step coarse) drives volume instead, so we set
 * DAC_VOL_SEL = 1 in codec_dac_program_fifo() to use the fine scale.
 */

#define AC_DAC_VOL_SEL              (1u << 16)  /* 1=use DAC_VOL_L/R */
#define AC_DAC_VOL_L_SHIFT          8
#define AC_DAC_VOL_L_MASK           (0xffu << AC_DAC_VOL_L_SHIFT)
#define AC_DAC_VOL_L_0DB            (0xa0u << AC_DAC_VOL_L_SHIFT)
#define AC_DAC_VOL_L_MUTE           (0x00u << AC_DAC_VOL_L_SHIFT)
#define AC_DAC_VOL_R_SHIFT          0
#define AC_DAC_VOL_R_MASK           (0xffu << AC_DAC_VOL_R_SHIFT)
#define AC_DAC_VOL_R_0DB            (0xa0u << AC_DAC_VOL_R_SHIFT)
#define AC_DAC_VOL_R_MUTE           (0x00u << AC_DAC_VOL_R_SHIFT)

/* AC_ADC_FIFOC bit fields (offset 0x0030, default 0x00000400) */

#define AC_ADC_FIFOC_ADFS_SHIFT     29          /* Sample-rate select 31:29 */
#define AC_ADC_FIFOC_ADFS_MASK      (0x7u << AC_ADC_FIFOC_ADFS_SHIFT)
#define AC_ADC_FIFOC_ADFS_48K       (0x0u << AC_ADC_FIFOC_ADFS_SHIFT)
#define AC_ADC_FIFOC_ADFS_32K       (0x1u << AC_ADC_FIFOC_ADFS_SHIFT)
#define AC_ADC_FIFOC_ADFS_24K       (0x2u << AC_ADC_FIFOC_ADFS_SHIFT)
#define AC_ADC_FIFOC_ADFS_16K       (0x3u << AC_ADC_FIFOC_ADFS_SHIFT)
#define AC_ADC_FIFOC_ADFS_12K       (0x4u << AC_ADC_FIFOC_ADFS_SHIFT)
#define AC_ADC_FIFOC_ADFS_8K        (0x5u << AC_ADC_FIFOC_ADFS_SHIFT)
#define AC_ADC_FIFOC_EN_AD          (1u << 28)  /* ADC digital enable */
#define AC_ADC_FIFOC_ADCFDT_SHIFT   26          /* FIFO delay 27:26 */
#define AC_ADC_FIFOC_ADCFDT_MASK    (0x3u << AC_ADC_FIFOC_ADCFDT_SHIFT)
#define AC_ADC_FIFOC_ADCFDT_5MS     (0x0u << AC_ADC_FIFOC_ADCFDT_SHIFT)
#define AC_ADC_FIFOC_ADCFDT_10MS    (0x1u << AC_ADC_FIFOC_ADCFDT_SHIFT)
#define AC_ADC_FIFOC_ADCFDT_20MS    (0x2u << AC_ADC_FIFOC_ADCFDT_SHIFT)
#define AC_ADC_FIFOC_ADCFDT_30MS    (0x3u << AC_ADC_FIFOC_ADCFDT_SHIFT)
#define AC_ADC_FIFOC_ADCDFEN        (1u << 25)  /* FIFO delay enable */
#define AC_ADC_FIFOC_RX_FIFO_MODE   (1u << 24)  /* 0=data in MSB,LSB=0  1=data in LSB,MSB=sign */
#define AC_ADC_FIFOC_RX_SYNC_START  (1u << 21)  /* RX sync start */
#define AC_ADC_FIFOC_RX_SYNC_EN     (1u << 20)  /* RX sync enable */
#define AC_ADC_FIFOC_RX_BITS_20     (1u << 16)  /* 0=16bit 1=20bit */
#define AC_ADC_FIFOC_RX_TRIG_SHIFT  4           /* RX trigger level 11:4 */
#define AC_ADC_FIFOC_RX_TRIG_MASK   (0xffu << AC_ADC_FIFOC_RX_TRIG_SHIFT)
#define AC_ADC_FIFOC_ADC_DRQ_EN     (1u << 3)   /* RX DRQ enable */
#define AC_ADC_FIFOC_ADC_IRQ_EN     (1u << 2)   /* RX available IRQ */
#define AC_ADC_FIFOC_FIFO_OV_IRQ_EN (1u << 1)   /* RX overrun IRQ */
#define AC_ADC_FIFOC_FIFO_FLUSH     (1u << 0)   /* Self-clearing flush */

/* AC_ADC_FIFOS bit fields (offset 0x0038, default 0x00000001) */

#define AC_ADC_FIFOS_RXA            (1u << 23)  /* >=1 sample available */
#define AC_ADC_FIFOS_RXA_CNT_SHIFT  8           /* Sample count 16:8 */
#define AC_ADC_FIFOS_RXA_CNT_MASK   (0x1ffu << AC_ADC_FIFOS_RXA_CNT_SHIFT)
#define AC_ADC_FIFOS_RXA_INT        (1u << 3)   /* Available IRQ pending */
#define AC_ADC_FIFOS_RXO_INT        (1u << 1)   /* Overrun IRQ pending */

/* AC_ADC_DIG_CTRL bit fields (offset 0x0050, default 0x00000000) */

#define AC_ADC_DIG_ADC3_VOL_EN      (1u << 17)  /* ADC3 vol-ctrl enable */
#define AC_ADC_DIG_ADC1_2_VOL_EN    (1u << 16)  /* ADC1/2 vol-ctrl enable */
#define AC_ADC_DIG_CHAN_EN_SHIFT    0           /* Channel enable bits 2:0 */
#define AC_ADC_DIG_CHAN_EN_MASK     (0x7u << AC_ADC_DIG_CHAN_EN_SHIFT)
#define AC_ADC_DIG_CHAN_EN_ADC1     (1u << 0)   /* Enable ADC1 in digital path */
#define AC_ADC_DIG_CHAN_EN_ADC2     (1u << 1)   /* Enable ADC2 in digital path */
#define AC_ADC_DIG_CHAN_EN_ADC3     (1u << 2)   /* Enable ADC3 in digital path */

/* ADC1_REG bit fields (offset 0x0300, default 0x001cc055) */

#define AC_ADC1_REG_ADC1_EN         (1u << 31)  /* ADC1 channel enable */
#define AC_ADC1_REG_LINEINLEN       (1u << 23)  /* LINE-IN L enable */
#define AC_ADC1_REG_LINEINLG        (1u << 22)  /* LINE-IN L 0/6dB gain */
#define AC_ADC1_REG_PGA_GAIN_SHIFT  8           /* PGA gain bits 12:8 */
#define AC_ADC1_REG_PGA_GAIN_MASK   (0x1fu << AC_ADC1_REG_PGA_GAIN_SHIFT)

/* ADC2_REG bit fields (offset 0x0304, default 0x001c0055) */

#define AC_ADC2_REG_ADC2_EN         (1u << 31)  /* ADC2 channel enable */
#define AC_ADC2_REG_LINEINREN       (1u << 23)  /* LINE-IN R enable */
#define AC_ADC2_REG_LINEINRG        (1u << 22)  /* LINE-IN R 0/6dB gain */
#define AC_ADC2_REG_PGA_GAIN_SHIFT  8
#define AC_ADC2_REG_PGA_GAIN_MASK   (0x1fu << AC_ADC2_REG_PGA_GAIN_SHIFT)

/* ADC3_REG bit fields (offset 0x0308, default 0x001c0055)
 * ADC3 is the channel routed to MICIN3P/N differential pair.
 *
 * Bias-current fields IOPAAF / IOPSDM1 / IOPSDM2 / IOPMIC each select a
 * 1.5x..2.25x multiple of the master IOPADC current (UM 8.4.6.114, p843).
 * Higher bias = wider op-amp gain-bandwidth = the analog AAF / SDM
 * integrators can pass higher audio frequencies before rolling off.  All
 * four default to 0x1 (1.75 x) which empirically attenuates above ~5 kHz
 * even at the 16 kHz capture rate.
 */

#define AC_ADC3_REG_ADC3_EN         (1u << 31)  /* ADC3 channel enable */
#define AC_ADC3_REG_MIC3_PGA_EN     (1u << 30)  /* MIC3 PGA enable */
#define AC_ADC3_REG_MIC3_SIN_EN     (1u << 28)  /* MIC3 single-end mode */
#define AC_ADC3_REG_PGA_GAIN_SHIFT  8           /* PGA gain bits 12:8 */
#define AC_ADC3_REG_PGA_GAIN_MASK   (0x1fu << AC_ADC3_REG_PGA_GAIN_SHIFT)
#define AC_ADC3_REG_IOPAAF_SHIFT    6           /* AAF op-amp bias 7:6 */
#define AC_ADC3_REG_IOPAAF_MASK     (0x3u << AC_ADC3_REG_IOPAAF_SHIFT)
#define AC_ADC3_REG_IOPSDM1_SHIFT   4           /* SDM stage 1 bias 5:4 */
#define AC_ADC3_REG_IOPSDM1_MASK    (0x3u << AC_ADC3_REG_IOPSDM1_SHIFT)
#define AC_ADC3_REG_IOPSDM2_SHIFT   2           /* SDM stage 2 bias 3:2 */
#define AC_ADC3_REG_IOPSDM2_MASK    (0x3u << AC_ADC3_REG_IOPSDM2_SHIFT)
#define AC_ADC3_REG_IOPMIC_SHIFT    0           /* MIC pre-amp bias 1:0 */
#define AC_ADC3_REG_IOPMIC_MASK     (0x3u << AC_ADC3_REG_IOPMIC_SHIFT)
#define AC_ADC3_REG_IOP_MAX         0x3u        /* 2.25 x IOPADC */

/* Common PGA gain step encoding (5-bit, 0..31 -> 0..36 dB).
 * 0x0=0dB, 0x1..0x3=6dB, 0x4=9dB, 0x5..0x1F=10..36 dB (1 dB per step
 * starting at 0x6).  See user manual section 8.4.6.114.
 */

#define AC_ADC_PGA_GAIN_0DB         0x00
#define AC_ADC_PGA_GAIN_24DB        0x13
#define AC_ADC_PGA_GAIN_30DB        0x19
#define AC_ADC_PGA_GAIN_36DB        0x1f

/* DAC_REG bit fields (offset 0x0310, default 0x00150000) */

#define AC_DAC_REG_CURRENT_TEST_EN  (1u << 23)  /* MICIN3P current sink test */
#define AC_DAC_REG_DACL_EN          (1u << 15)  /* Analog DAC L enable */
#define AC_DAC_REG_DACR_EN          (1u << 14)  /* Analog DAC R enable */

/* MICBIAS_REG bit fields (offset 0x0318, default 0x40003030) */

#define AC_MICBIAS_HMICBIASEN       (1u << 15)  /* Headphone mic bias enable */
#define AC_MICBIAS_HBIASSEL_SHIFT   13          /* HMIC bias level 14:13 */
#define AC_MICBIAS_HBIASSEL_MASK    (0x3u << AC_MICBIAS_HBIASSEL_SHIFT)
#define AC_MICBIAS_HBIASSEL_2V09    (0x1u << AC_MICBIAS_HBIASSEL_SHIFT)
#define AC_MICBIAS_MMICBIASEN       (1u << 7)   /* Master mic bias enable */
#define AC_MICBIAS_MBIASSEL_SHIFT   5           /* MMIC bias level 6:5 */
#define AC_MICBIAS_MBIASSEL_MASK    (0x3u << AC_MICBIAS_MBIASSEL_SHIFT)
#define AC_MICBIAS_MBIASSEL_2V09    (0x1u << AC_MICBIAS_MBIASSEL_SHIFT)

/* POWER_REG bit fields (offset 0x0348, default 0x80003325).  Note: this
 * register is gated by the system bus reset only and survives the codec
 * BGR reset.  Hardware default already enables ALDO; HPLDO is brought up
 * by the DAC path (M2).
 */

#define AC_POWER_REG_ALDO_EN        (1u << 31)  /* ALDO (analog 1.8V) enable */
#define AC_POWER_REG_HPLDO_EN       (1u << 30)  /* HPLDO (HP 1.8V) enable */
#define AC_POWER_REG_VRA1_FURTHER   (1u << 29)  /* VRA1 manual finish bit */
#define AC_POWER_REG_AVCCPOR        (1u << 16)  /* AVCC POR monitor (RO) */
#define AC_POWER_REG_ALDO_VOL_SHIFT 12          /* ALDO output voltage 14:12 */
#define AC_POWER_REG_ALDO_VOL_MASK  (0x7u << AC_POWER_REG_ALDO_VOL_SHIFT)
#define AC_POWER_REG_ALDO_VOL_180V  (0x3u << AC_POWER_REG_ALDO_VOL_SHIFT)
#define AC_POWER_REG_HPLDO_VOL_SHIFT 8          /* HPLDO output voltage 10:8 */
#define AC_POWER_REG_HPLDO_VOL_MASK (0x7u << AC_POWER_REG_HPLDO_VOL_SHIFT)
#define AC_POWER_REG_HPLDO_VOL_180V (0x3u << AC_POWER_REG_HPLDO_VOL_SHIFT)

/* HP2_REG bit fields (offset 0x0340, default 0x06404000).
 * Used by the DAC path (M2); declared here for register-block coherence.
 */

#define AC_HP2_REG_HPFB_BUF_EN      (1u << 31)  /* HP feedback buffer enable */
#define AC_HP2_REG_HP_GAIN_SHIFT    28          /* Headphone gain 30:28 */
#define AC_HP2_REG_HP_GAIN_MASK     (0x7u << AC_HP2_REG_HP_GAIN_SHIFT)
#define AC_HP2_REG_HP_GAIN_0DB      (0x0u << AC_HP2_REG_HP_GAIN_SHIFT)
#define AC_HP2_REG_HP_DRVEN         (1u << 21)  /* HP driver enable */
#define AC_HP2_REG_HP_DRVOUTEN      (1u << 20)  /* HP driver output enable */
#define AC_HP2_REG_RSWITCH          (1u << 19)  /* 0=HPOUT VCM from RAMP_DAC,
                                                 * 1=HPOUT VCM from VRA1
                                                 */
#define AC_HP2_REG_RAMPEN           (1u << 18)  /* Ramp DAC enable */
#define AC_HP2_REG_HPFB_IN_EN       (1u << 17)  /* HP feedback PAD-in enable */
#define AC_HP2_REG_RAMP_OUT_EN      (1u << 15)  /* Ramp output switch enable */

/* RAMP_REG bit fields (offset 0x031c, default 0x00180000).
 * Used by the DAC pop-suppression ramp (M2).
 */

#define AC_RAMP_REG_HP_PULL_OUT_EN  (1u << 15)  /* Headphone pullout enable */
#define AC_RAMP_REG_RMD_EN          (1u << 3)   /* Ramp manual down enable */
#define AC_RAMP_REG_RMU_EN          (1u << 2)   /* Ramp manual up enable */
#define AC_RAMP_REG_RMC_EN          (1u << 1)   /* Ramp manual control enable */
#define AC_RAMP_REG_RD_EN           (1u << 0)   /* Ramp digital enable */

/* VRA1 speedup-down control (offset 0x0054, default 0x00000000) */

#define AC_VRA1_SPEEDUP_DOWN_STATE  (1u << 4)   /* VRA1 speedup-down active (RO) */
#define AC_VRA1_SPEEDUP_DOWN_CTRL   (1u << 1)   /* Manual speedup-down enable */
#define AC_VRA1_SPEEDUP_RST_CTRL    (1u << 0)   /* Manual speedup-down reset */

/* AC_ADC_DAP_CTR bit fields (offset 0x00F8, default 0x00000000).  The DAP
 * (Digital Audio Processor) hosts the per-channel HPF and DRC blocks.  Two
 * independent DAPs: DAP0 for ADC1/2, DAP1 for ADC3.  HPF cutoff is < 1 Hz
 * (UM section 8.4.6.17, p778 "HPF Function") and is the canonical fix for
 * DC-offset on ADC capture; default-off means the DC bias from the analog
 * front-end passes straight through to the FIFO.
 */

#define AC_ADC_DAP_CTR_DAP0_EN      (1u << 31)  /* DAP for ADC1/2 master enable */
#define AC_ADC_DAP_CTR_DRC0_EN      (1u << 29)  /* ADC1/2 DRC enable */
#define AC_ADC_DAP_CTR_HPF0_EN      (1u << 28)  /* ADC1/2 HPF (DC blocker) */
#define AC_ADC_DAP_CTR_DAP1_EN      (1u << 27)  /* DAP for ADC3 master enable */
#define AC_ADC_DAP_CTR_DRC1_EN      (1u << 25)  /* ADC3 DRC enable */
#define AC_ADC_DAP_CTR_HPF1_EN      (1u << 24)  /* ADC3 HPF (DC blocker) */

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_AUDIO_H */
