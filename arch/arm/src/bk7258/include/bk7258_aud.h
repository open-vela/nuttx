/****************************************************************************
 * arch/arm/src/bk7258/include/bk7258_aud.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/* Chip-level interface to the BK7258 analog audio (AUD) block.
 * Measurement notes are inherited from ref264, not newly verified here.
 *
 * Board wiring this driver assumes (AIDK AI toy development board
 * schematic):
 *
 *   playback  internal mono DAC -> AUDLP/AUDLN (pins 25/24, differential)
 *             -> C48/C46 -> HT6872 IN+/IN- -> speaker.  The amplifier's
 *             CTRL pin is held low by R71 (10K pull-down) so the speaker
 *             is muted until software drives net MUTE (GPIO50) high; the
 *             GPIO itself is owned by the board layer, not by this file.
 *
 *   capture   MIC1 (MICP1/MICN1, pins 26/27) is the electret microphone.
 *             MIC2 (MICP2/MICN2, pins 28/29) is not a microphone at all:
 *             it carries the post-amplifier speaker signal attenuated by
 *             R84/R83 (39K) and DC-blocked by C61/C60 (1uF), i.e. the
 *             hardware echo reference for AEC.
 *
 * Because MIC2 is a *post*-amplifier tap, its signal disappears when the
 * HT6872 is muted.  The reference therefore tracks the amplifier state on
 * its own and callers do not need to gate AEC on the PA GPIO.
 *
 * IMPORTANT, measured on hardware: BK7258's ADC delivers only ONE
 * channel.  AUD_ADC_FIFO_PORT is documented as carrying the left sample in
 * [15:0] and the right in [31:16], and ana_reg27 has a complete micen field
 * for MIC2, so this driver was originally written to capture L=voice /
 * R=reference interleaved -- the layout the vendor's AEC_MODE_HARDWARE path
 * expects (aec_algorithm.c:269-274 de-interleaves mic_addr[i]=lr[2*i],
 * ref_addr[i]=lr[2*i+1]).
 *
 * On this SoC that does not work.  With a tone audibly playing through the
 * HT6872, and MIC1 demonstrably picking it up through the air (RMS rising
 * from 25.7 while silent to 36.4 while playing), MIC2 returned exactly zero
 * -- dc, RMS and peak all 0.0 across 15872 samples.  The vendor HAL agrees:
 * sys_hal_aud_mic2_en() and sys_hal_aud_dacr_en() are both explicit
 * "//not support" stubs for BK7258 (sys_hal.c:1924-1947).  The register
 * fields exist; the second ADC datapath does not.
 *
 * Capture is therefore mono by default (CONFIG_BK7258_AUDIO_AEC_REFERENCE
 * defaults to n) and an AEC stage should build its reference in software
 * from the PCM queued to the DAC rather than from MIC2.
 */

#ifndef __VENDOR_BEKEN_CHIPS_BK7258_INCLUDE_BK7258_AUD_H
#define __VENDOR_BEKEN_CHIPS_BK7258_INCLUDE_BK7258_AUD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Digital gain range for the DAC, from aud_ll_macro_def.h's description of
 * REG_0x07[23:18]: -45 dB .. +18 dB with 0x2d meaning 0 dB.
 */

#define BK7258_AUD_DAC_DIG_GAIN_MAX   0x3fu
#define BK7258_AUD_DAC_DIG_GAIN_0DB   0x2du

/* Analog gain range for the DAC.  ana_reg20.dacg is only 4 bits wide
 * (sys_struct.h sys_ana_reg20_t "dacg : 4, bit[22:25]"), so the usable
 * range is 0..15 even though onboard_speaker_stream.h's ana_gain comment
 * mentions 0x00..0x3f.  The vendor's suggested operating point is 0x0a and
 * the analog block's own reset value is 0x0f.
 */

#define BK7258_AUD_DAC_ANA_GAIN_MAX   0x0fu
#define BK7258_AUD_DAC_ANA_GAIN_DEF   0x0au

/* ADC digital gain is REG_0x04[23:18], 6 bits.  MIC analog gain is
 * ana_reg{19,27}.micgain, 4 bits.
 */

#define BK7258_AUD_ADC_GAIN_MAX       0x3fu
#define BK7258_AUD_ADC_GAIN_0DB       0x2du
#define BK7258_AUD_MIC_GAIN_MAX       0x0fu

/* Analog MIC pre-amplifier gain default.
 *
 * Zero, matching the vendor's SYS_ANA_REG19_MICGAIN_DEFAULT_VAL.  A first
 * bring-up run used half scale (7) and the electret microphone on CN7
 * clipped: MIC1 reported peak 32767 at an RMS of only 2671, i.e. transients
 * were hitting full scale.  Analog gain sits ahead of the ADC, so digital
 * gain cannot undo that clipping -- this is the knob to raise, carefully,
 * if more sensitivity is ever needed.
 */

#define BK7258_AUD_MIC_GAIN_DEF       0x00u

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Audio master clock source.
 *
 * XTAL (26 MHz) is enough for 8k/16k/32k and is what
 * onboard_speaker_stream.h's default configuration uses.  APLL is only
 * required for the 44.1k/48k families and costs an extra calibration
 * sequence (see bk7258_aud.c aud_clk_config()).
 */

enum bk7258_aud_clksrc
{
  BK7258_AUD_CLK_XTAL = 0,
  BK7258_AUD_CLK_APLL
};

struct bk7258_aud_config
{
  uint32_t samplerate;              /* 8000..48000, see aud_set_samp_rate */
  enum bk7258_aud_clksrc clksrc;

  uint8_t dac_dig_gain;             /* 0..0x3f, 0x2d == 0 dB            */
  uint8_t dac_ana_gain;             /* 0..0x0f                          */

  uint8_t adc_gain;                 /* 0..0x3f digital                  */
  uint8_t mic_gain;                 /* 0..0x0f analog, both MIC1/MIC2   */

  /* Run the capture path through the ADC's two high-pass stages instead of
   * bypassing them.
   *
   * The vendor bypasses both, which lets the microphone's DC offset and any
   * low-frequency rumble through untouched.  Neither carries speech, and the
   * offset costs headroom that the quiet electret front end cannot spare, so
   * for voice the filters are worth having.  It is a configuration rather
   * than a fixed choice because the bypassed path is what the vendor
   * validated, and the difference is worth being able to measure.
   */

  bool adc_hpf;

  /* Enable the MIC2 channel so capture yields L=voice / R=echo-reference
   * interleaved pairs.  With this false only MIC1 is enabled and capture
   * is plain mono voice.
   */

  bool capture_reference;
};

/* ISR callbacks.
 *
 * Both run in interrupt context.  tx is invoked when the DAC left FIFO
 * has drained below its read threshold and wants more samples; rx is
 * invoked when the ADC FIFO has risen above its write threshold and has
 * samples to collect.
 */

typedef void (*bk7258_aud_xfer_cb_t)(void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_BK7258_AUDIO

/****************************************************************************
 * Name: bk7258_aud_initialize
 *
 * Description:
 *   Bring the AUD block out of reset: ask CP1 for the AUDP power domain
 *   and the AUD module clock over the PWC mailbox channel, load the
 *   analog block's reset values, enable the audio bias and attach the
 *   AUD interrupt.  Idempotent.
 *
 * Returned Value:
 *   OK on success, a negated errno on failure.  -ENODEV means the AUD
 *   block did not answer with a plausible device/version ID after the
 *   power and clock votes, which normally points at the votes not having
 *   been serviced by CP1.
 *
 ****************************************************************************/

int bk7258_aud_initialize(void);

/* Task-independent ownership across diagnostic calls and PCM sessions. */

int bk7258_aud_acquire(const void *owner);
void bk7258_aud_release(const void *owner);

/****************************************************************************
 * Name: bk7258_aud_read_id
 *
 * Description:
 *   Read AUD_DEVICE_ID.  Intended as the very first bring-up probe: a
 *   non-zero, non-0xffffffff value proves the AUDP power vote, the module
 *   clock vote and the MPU mapping are all in place before any analog
 *   configuration is attempted.
 *
 ****************************************************************************/

uint32_t bk7258_aud_read_id(void);
uint32_t bk7258_aud_read_version(void);

/****************************************************************************
 * Name: bk7258_aud_dac_setup / bk7258_aud_dac_shutdown
 *
 * Description:
 *   Configure (or tear down) the playback path: analog bias, DAC left
 *   channel, differential output mode, gains and sample rate.  setup()
 *   leaves the DAC muted and stopped; call bk7258_aud_dac_start() and
 *   then un-mute once the external amplifier has been enabled.
 *
 ****************************************************************************/

int bk7258_aud_dac_setup(const struct bk7258_aud_config *cfg);
void bk7258_aud_dac_shutdown(void);

void bk7258_aud_dac_start(void);
void bk7258_aud_dac_stop(void);
int bk7258_aud_dac_mute(bool mute);
int bk7258_aud_dac_set_dig_gain(uint8_t gain);

/****************************************************************************
 * Name: bk7258_aud_adc_setup / bk7258_aud_adc_shutdown
 *
 * Description:
 *   Configure (or tear down) the capture path.  When
 *   cfg->capture_reference is true both MIC1 and MIC2 are enabled and
 *   captured samples are L/R interleaved (L = MIC1 voice, R = MIC2 echo
 *   reference); otherwise only MIC1 is enabled.
 *
 ****************************************************************************/

int bk7258_aud_adc_setup(const struct bk7258_aud_config *cfg);
void bk7258_aud_adc_shutdown(void);

void bk7258_aud_adc_start(void);
void bk7258_aud_adc_stop(void);

/****************************************************************************
 * Name: bk7258_aud_set_tx_callback / bk7258_aud_set_rx_callback
 *
 * Description:
 *   Install the FIFO service callback for one direction, with its own
 *   argument.  Pass NULL to leave that direction unserviced.
 *
 *   The two directions are registered separately on purpose: playback and
 *   capture are independent clients with different instance pointers, and
 *   a combined entry point made starting one direction silently tear down
 *   the other.
 *
 ****************************************************************************/

void bk7258_aud_set_tx_callback(bk7258_aud_xfer_cb_t cb, void *arg);
void bk7258_aud_set_rx_callback(bk7258_aud_xfer_cb_t cb, void *arg);

/****************************************************************************
 * Name: bk7258_aud_dac_write
 *
 * Description:
 *   Push up to nsamples mono 16-bit samples into the DAC left FIFO,
 *   stopping early if the FIFO fills.
 *
 * Returned Value:
 *   Number of samples actually written.
 *
 ****************************************************************************/

unsigned int bk7258_aud_dac_write(const int16_t *samples,
                                  unsigned int nsamples);

/****************************************************************************
 * Name: bk7258_aud_adc_read
 *
 * Description:
 *   Drain up to nsamples 16-bit samples from the ADC FIFO, stopping early
 *   when it empties.  With MIC2 enabled the stream is L/R interleaved, so
 *   nsamples counts individual samples and not frames.
 *
 * Returned Value:
 *   Number of samples actually read.
 *
 ****************************************************************************/

unsigned int bk7258_aud_adc_read(int16_t *samples, unsigned int nsamples);

/****************************************************************************
 * Name: bk7258_aud_tx_int_enable / bk7258_aud_rx_int_enable
 *
 * Description:
 *   Gate the DAC-left and ADC FIFO interrupts independently.
 *
 ****************************************************************************/

void bk7258_aud_tx_int_enable(bool enable);
void bk7258_aud_rx_int_enable(bool enable);

uint32_t bk7258_aud_fifo_status(void);
uint32_t bk7258_aud_adc_threshold(void);

#endif /* CONFIG_BK7258_AUDIO */

#endif /* __VENDOR_BEKEN_CHIPS_BK7258_INCLUDE_BK7258_AUD_H */
