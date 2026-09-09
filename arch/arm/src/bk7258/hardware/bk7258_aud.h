/****************************************************************************
 * arch/arm/src/bk7258/hardware/bk7258_aud.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/* BK7258 analog audio (AUD) register definitions.
 *
 * Every offset and bit position below was transcribed field-by-field from
 * the vendor tree shipped in this repository, NOT guessed from a
 * datasheet:
 *
 *   - AUD block layout and bit fields:
 *       bk_avdk_smp/ap/middleware/soc/bk7258_ap/soc/aud_reg.h
 *       bk_avdk_smp/ap/middleware/soc/bk7258_ap/hal/aud_ll_macro_def.h
 *   - Analog (ana_regNN) bit positions:
 *       bk_avdk_smp/ap/middleware/soc/bk7258_ap/soc/sys_struct.h
 *       bk_avdk_smp/ap/middleware/soc/bk7258_ap/hal/sys_ll.h
 *   - SYSCTRL audio clock/interrupt bits:
 *       bk_avdk_smp/ap/middleware/soc/bk7258_ap/soc/sys_struct.h
 *       bk_avdk_smp/ap/middleware/soc/bk7258_ap/hal/sys_hal.c
 *
 * The audio path on this board is: internal mono DAC -> differential
 * AUDLP/AUDLN -> HT6872 class-D amplifier -> speaker, with the amplifier
 * un-muted by GPIO50 (net MUTE).  Capture uses both analog MIC channels:
 * MIC1 (MICP1/MICN1, physical pins 26/27) is the real electret
 * microphone, MIC2 (MICP2/MICN2, pins 28/29) carries the post-amplifier
 * speaker feedback used as the AEC reference (schematic: HT6872 OUT+/OUT-
 * -> R84/R83 39K -> C61/C60 1uF -> MIC2P/MIC2N, annotated
 * "AEC feedback sampling circuit required").
 */

#ifndef __VENDOR_BEKEN_CHIPS_BK7258_HARDWARE_BK7258_AUD_H
#define __VENDOR_BEKEN_CHIPS_BK7258_HARDWARE_BK7258_AUD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bk7258_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AUD block registers.  aud_reg.h names each register by its word index,
 * e.g. "REG_0x07" == AUD_LL_REG_BASE + 0x7*4.
 */

#define BK7258_AUD_DEVICE_ID        (BK7258_AUD_BASE + 0x00u)  /* REG_0x00 */
#define BK7258_AUD_VERSION_ID       (BK7258_AUD_BASE + 0x04u)  /* REG_0x01 */
#define BK7258_AUD_CLK_CONTROL      (BK7258_AUD_BASE + 0x08u)  /* REG_0x02 */
#define BK7258_AUD_GLOBAL_STATUS    (BK7258_AUD_BASE + 0x0cu)  /* REG_0x03 */
#define BK7258_AUD_ADC_CONFIG0      (BK7258_AUD_BASE + 0x10u)  /* REG_0x04 */
#define BK7258_AUD_DAC_CONFIG0      (BK7258_AUD_BASE + 0x1cu)  /* REG_0x07 */
#define BK7258_AUD_FIFO_CONFIG      (BK7258_AUD_BASE + 0x28u)  /* REG_0x0A */
#define BK7258_AUD_FIFO_STATUS      (BK7258_AUD_BASE + 0x38u)  /* REG_0x0E */
#define BK7258_AUD_ADC_FIFO_PORT    (BK7258_AUD_BASE + 0x44u)  /* REG_0x11 */
#define BK7258_AUD_DAC_FIFO_PORT    (BK7258_AUD_BASE + 0x48u)  /* REG_0x12 */
#define BK7258_AUD_EXTEND_CFG       (BK7258_AUD_BASE + 0x60u)  /* REG_0x18 */
#define BK7258_AUD_DAC_FRACMOD      (BK7258_AUD_BASE + 0x64u)  /* REG_0x19 */
#define BK7258_AUD_ADC_FRACMOD      (BK7258_AUD_BASE + 0x68u)  /* REG_0x1A */
#define BK7258_AUD_CONFIG           (BK7258_AUD_BASE + 0xc0u)  /* REG_0x30 */

/* REG_0x02 AUD_CLK_CONTROL */

#define BK7258_AUD_ADC_SOFT_RESET   (1u << 0)
#define BK7258_AUD_ADC_CLK_GATE     (1u << 1)

/* REG_0x04 AUD_ADC_CONFIG0 */

#define BK7258_AUD_ADC_HPF2_BYPASS  (1u << 16)
#define BK7258_AUD_ADC_HPF1_BYPASS  (1u << 17)
#define BK7258_AUD_ADC_GAIN_SHIFT   18
#define BK7258_AUD_ADC_GAIN_MASK    (0x3fu << BK7258_AUD_ADC_GAIN_SHIFT)
#define BK7258_AUD_ADC_SAMPLE_EDGE  (1u << 24)
#define BK7258_AUD_ADC_DIG_MIC_SEL  (1u << 25)

/* REG_0x07 AUD_DAC_CONFIG0.  aud_ll_macro_def.h documents dac_set_gain as
 * bits [23:18], with a -45 dB to +18 dB range and 0x2d representing
 * 0 dB, i.e. a 6-bit digital gain field.
 */

#define BK7258_AUD_DAC_HPF2_BYPASS  (1u << 16)
#define BK7258_AUD_DAC_HPF1_BYPASS  (1u << 17)
#define BK7258_AUD_DAC_GAIN_SHIFT   18
#define BK7258_AUD_DAC_GAIN_MASK    (0x3fu << BK7258_AUD_DAC_GAIN_SHIFT)
#define BK7258_AUD_DAC_GAIN_0DB     0x2du
#define BK7258_AUD_DAC_CLK_INVERT   (1u << 24)

/* REG_0x0A AUD_FIFO_CONFIG */

#define BK7258_AUD_DACR_RD_THRED_SHIFT 0
#define BK7258_AUD_DACR_RD_THRED_MASK  (0x1fu << 0)
#define BK7258_AUD_DACL_RD_THRED_SHIFT 5
#define BK7258_AUD_DACL_RD_THRED_MASK  (0x1fu << 5)
#define BK7258_AUD_ADC_WR_THRED_SHIFT  15
#define BK7258_AUD_ADC_WR_THRED_MASK   (0x1fu << 15)
#define BK7258_AUD_DACR_INT_EN         (1u << 20)
#define BK7258_AUD_DACL_INT_EN         (1u << 21)
#define BK7258_AUD_ADC_INT_EN          (1u << 23)
#define BK7258_AUD_LOOP_ADC2DAC        (1u << 25)

/* REG_0x0E AUD_FIFO_STATUS */

#define BK7258_AUD_DACL_NEAR_FULL   (1u << 1)
#define BK7258_AUD_ADC_NEAR_FULL    (1u << 2)
#define BK7258_AUD_DACL_NEAR_EMPTY  (1u << 5)
#define BK7258_AUD_ADC_NEAR_EMPTY   (1u << 6)
#define BK7258_AUD_DACL_FIFO_FULL   (1u << 9)
#define BK7258_AUD_ADC_FIFO_FULL    (1u << 10)
#define BK7258_AUD_DACL_FIFO_EMPTY  (1u << 13)
#define BK7258_AUD_ADC_FIFO_EMPTY   (1u << 14)
#define BK7258_AUD_DACL_INT_FLAG    (1u << 17)
#define BK7258_AUD_ADC_INT_FLAG     (1u << 18)

/* REG_0x12 AUD_DAC_FIFO_PORT holds the left sample in [15:0] and the
 * right sample in [31:16].  BK7258's DAC is mono (sys_hal_aud_dacr_en()
 * is an explicit "//not support" stub in
 * bk_avdk_smp/ap/middleware/soc/bk7258_ap/hal/sys_hal.c), so only the
 * left half is used here.
 *
 * REG_0x11 AUD_ADC_FIFO_PORT documents the same word layout, but the
 * right datapath is not established on this board. aud_reg.h names the low
 * field
 * (AD_ADC_L_FIFO), but aud_ll_macro_def.h:1758-1767 documents the whole
 * word -- its comment on REG_0x11 states that the high 16 bits are the
 * right channel and the low 16 bits the left channel -- and defines
 * AUD_ADC_FPORT_REG2ADC_R_DI_POS (16). This diagnostic only uses MIC1
 * in the low half; a documented field does not prove a working MIC2 ADC.
 */

#define BK7258_AUD_FIFO_L_SHIFT     0
#define BK7258_AUD_FIFO_R_SHIFT     16
#define BK7258_AUD_FIFO_SAMPLE_MASK 0xffffu

/* REG_0x18 AUD_EXTEND_CFG */

#define BK7258_AUD_DAC_FRACMOD_MANUAL (1u << 0)
#define BK7258_AUD_ADC_FRACMOD_MANUAL (1u << 1)
#define BK7258_AUD_FILT_ENABLE        (1u << 2)

/* REG_0x30 AUD_CONFIG */

#define BK7258_AUD_SAMP_RATE_ADC_SHIFT 0
#define BK7258_AUD_SAMP_RATE_ADC_MASK  (0x3u << 0)
#define BK7258_AUD_DAC_ENABLE          (1u << 2)
#define BK7258_AUD_ADC_ENABLE          (1u << 3)
#define BK7258_AUD_DTMF_ENABLE         (1u << 4)
#define BK7258_AUD_LINEIN_ENABLE       (1u << 5)
#define BK7258_AUD_SAMP_RATE_DAC_SHIFT 6
#define BK7258_AUD_SAMP_RATE_DAC_MASK  (0x3u << 6)
#define BK7258_AUD_APLL_SEL            (1u << 8)
#define BK7258_AUD_DMIC_ENABLE         (1u << 9)

/* Sample-rate field encodings (aud_reg.h SAMPLE_RATE_8K..SAMPLE_RATE_48K).
 * Rates that are not one of these four are reached by enabling the
 * fractional-modulus divider and programming AUD_{DAC,ADC}_FRACMOD, see
 * BK7258_AUD_FRACMOD_* below.
 */

#define BK7258_AUD_SAMP_RATE_8K     0u
#define BK7258_AUD_SAMP_RATE_16K    1u
#define BK7258_AUD_SAMP_RATE_44_1K  2u
#define BK7258_AUD_SAMP_RATE_48K    3u

/* Fractional-modulus base constants, copied verbatim from
 * bk_avdk_smp/ap/middleware/driver/audio/aud/aud_dac_driver.c:44-47 (the
 * ADC driver defines the identical set).  The vendor shifts these before
 * writing: e.g. 11025 Hz uses CONST_DIV_44_1K << 2 and 22050 Hz uses
 * CONST_DIV_44_1K << 1.
 */

#define BK7258_AUD_FRACMOD_16K      0x06590000u
#define BK7258_AUD_FRACMOD_32K      0x01964000u
#define BK7258_AUD_FRACMOD_44_1K    0x049b2368u
#define BK7258_AUD_FRACMOD_48K      0x043b5554u

/* SYSCTRL audio bits.
 *
 *   cksel_aud       sys_struct.h sys_cpu_clk_div_mode1_t bit[25],
 *                   register word 0x8 -> byte offset 0x20, which is the
 *                   same register this repository already knows as
 *                   BK7258_SYS_CLKDIV1.  0 = 26MHz XTAL, 1 = APLL.
 *   aud_cken        sys_cpu_device_clk_enable_t bit[30], register word
 *                   0xc -> byte offset 0x30, i.e. BK7258_SYS_DEVCLK_EN.
 * NOTE the module clock gate (aud_cken) is normally owned by CP1: this AP
 * image asks CP1 for it over the PWC mailbox channel (PM_CLK_ID_AUDIO),
 * exactly like bk7258_pwm.c does for PWM.  The bit is defined here so the
 * driver can *verify* the vote landed rather than to write it blindly.
 *
 * There is deliberately no per-core interrupt-enable bit defined here.
 * The vendor's sys_hal_aud_int_en() writes sys_cpu0_int_0_31_en bit[23],
 * but this OpenVela image runs on CP1/AP, not CPU0: bk7258_irq.c's
 * bk7258_extirq_enable() already routes every external IRQ through
 * BK7258_CPU1_IRQ_EN0/EN1, so up_enable_irq(BK7258_IRQ_AUDIO) is the
 * complete and correct sequence and touching the CPU0 register here
 * would enable the interrupt on the wrong core.
 */

#define BK7258_SYS_CKSEL_AUD        (1u << 25)
#define BK7258_SYS_AUD_CLK_EN       (1u << 30)

/* Analog register block.
 *
 * SYS_ANA_REG0_ADDR is SOC_SYS_REG_BASE + (0x40 << 2)
 * (bk_avdk_smp/ap/middleware/soc/bk7258_ap/soc/sys_reg.h:2066), so
 * ana_regN sits at SYSCTRL_BASE + 0x100 + N*4.
 *
 * These registers are NOT plain memory-mapped registers: the SoC forwards
 * each write to the analog block over an internal SPI link, and software
 * must poll a per-register busy bit afterwards.  sys_ll.h:51-59
 * (sys_ll_set_analog_reg_value) does:
 *
 *     REG_WRITE(addr, value);
 *     while (REG_READ(SYS_ANALOG_REG_SPI_STATE_REG) &
 *            (1 << SYS_ANALOG_REG_SPI_STATE_POS(idx)));
 *
 * with idx == GET_SYS_ANALOG_REG_IDX(addr) == the register number.  This
 * AP config leaves CONFIG_ANA_REG_WRITE_POLL_REG_B unset
 * (bk_avdk_smp/projects/app_ab/ap/config/bk7258_ap/config:1584), which
 * selects the sys_ll.h:40-41 variant: the state register is word 0x3a
 * (byte offset 0xe8) and the wait bit index equals the register number
 * with no +8 offset.  Writing these with a bare putreg32() and no poll
 * lets consecutive writes clobber each other -- the registers read back
 * fine afterwards, so the failure is silent and shows up only as no
 * audio or distorted audio.  See bk7258_aud_ana_write().
 */

#define BK7258_ANA_REG(n)           (BK7258_SYSCTRL_BASE + 0x100u + ((n) * 4u))
#define BK7258_ANA_SPI_STATE        (BK7258_SYSCTRL_BASE + 0xe8u)
#define BK7258_ANA_SPI_BUSY(n)      (1u << (n))

#define BK7258_ANA_REG18            18u   /* bias / vref / micbias trim   */
#define BK7258_ANA_REG19            19u   /* MIC1 analog front-end        */
#define BK7258_ANA_REG20            20u   /* DAC analog path              */
#define BK7258_ANA_REG21            21u   /* DAC bias / current source    */
#define BK7258_ANA_REG27            27u   /* MIC2 analog front-end        */

/* ana_reg18 bits (sys_ana_reg18_t) */

#define BK7258_ANA18_ENAUDBIAS      (1u << 3)
#define BK7258_ANA18_ENADCBIAS      (1u << 4)
#define BK7258_ANA18_ENMICBIAS      (1u << 5)

/* ana_reg19 / ana_reg27 bits.  The two registers are bit-for-bit
 * identical in layout (sys_ana_reg19_t and sys_ana_reg27_t), reg19
 * describing MIC1 and reg27 describing MIC2.
 */

#define BK7258_ANA_MIC_GAIN_SHIFT   15
#define BK7258_ANA_MIC_GAIN_MASK    (0xfu << BK7258_ANA_MIC_GAIN_SHIFT)
#define BK7258_ANA_MIC_EN           (1u << 28)
#define BK7258_ANA_MIC_RST          (1u << 29)

/* ana_reg20 bits (sys_ana_reg20_t) */

#define BK7258_ANA20_DIFFEN         (1u << 13)
#define BK7258_ANA20_LENDCOC        (1u << 16)
#define BK7258_ANA20_DACDRVEN       (1u << 19)
#define BK7258_ANA20_DACLEN         (1u << 21)
#define BK7258_ANA20_DACG_SHIFT     22
#define BK7258_ANA20_DACG_MASK      (0xfu << BK7258_ANA20_DACG_SHIFT)
#define BK7258_ANA20_DACMUTE        (1u << 26)

/* ana_reg21 bits (sys_ana_reg21_t) */

#define BK7258_ANA21_ENIDACL        (1u << 18)
#define BK7258_ANA21_ENBS           (1u << 23)

/* Analog block reset values written by bk_aud_driver_init().
 *
 * The vendor builds each of these at runtime from ~91
 * SYS_ANA_REGnn_<FIELD>_DEFAULT_VAL macros
 * (bk_avdk_smp/ap/middleware/driver/audio/aud/aud_common_driver.c:38-131,
 * combined by ana_regNN_value_cal() at :153-...).  Every one of those
 * macros is a compile-time constant -- there is no efuse read and no
 * per-die calibration -- so the composed values are constants too and are
 * inlined here.
 *
 * Each value below was derived twice, independently, and both derivations
 * agree: (a) by folding the DEFAULT_VAL macros into the bit positions
 * declared by sys_struct.h, and (b) against the literal values the vendor
 * left in a disabled block immediately above the ana_regNN_value_cal()
 * calls (aud_common_driver.c:369-372).
 *
 * Field values worth knowing:
 *   REG18 = 0x00BF8085  iselaud=1 lchckinven1v=1 dacfb2st0v9=1
 *                       micbias_voc=0x10 vrefsel1v=1 capswspi=0x1f
 *                       adref_sel=2; all three bias enables start at 0.
 *   REG19 = 0x81800006  MIC1: isel=2 micirsel1=1 vcmsel=1 dwamode=1
 *                       hcen1stg=1, micen=0 (channel starts disabled).
 *   REG20 = 0xFBC02423  hpdac=1 calcon_sel=1 vcmsel=1 adjdacref=0x10
 *                       diffen=1 dacg=0xf dacdwamode_sel=1 dacsel=0xf;
 *                       daclen/dacdrven/dacmute start at 0.  diffen=1 is
 *                       already the differential mode this board needs.
 *   REG21 = 0x00500000  hc2s=1 vcmsel=1; enidacl/enbs start at 0.
 *   REG27 = 0x91800006  MIC2: identical to REG19 except bit28 micen=1.
 *                       That single bit is how the vendor brings MIC2 up
 *                       -- sys_hal_aud_mic2_en() is a "//not support"
 *                       stub on BK7258, so the only path to MIC2 is this
 *                       constant (or a direct ana_reg27 read/modify).
 */

#define BK7258_ANA18_RESET_VALUE    0x00bf8085u
#define BK7258_ANA19_RESET_VALUE    0x81800006u
#define BK7258_ANA20_RESET_VALUE    0xfbc02423u
#define BK7258_ANA21_RESET_VALUE    0x00500000u
#define BK7258_ANA27_RESET_VALUE    0x91800006u

#endif /* __VENDOR_BEKEN_CHIPS_BK7258_HARDWARE_BK7258_AUD_H */
