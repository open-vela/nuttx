/****************************************************************************
 * arch/arm/src/bk7258/bk7258_aud.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/* BK7258 analog audio (AUD) chip-level driver.
 *
 * Adapted from workspace ref264/board/beken/chips/bk7258. Measurement
 * notes below belong to that reference, not to this firmware's board test.
 * This integration is limited to serialized 16 kHz mono diagnostics.
 *
 * This is a port of the register sequences in
 *
 *   bk_avdk_smp/ap/middleware/driver/audio/aud/aud_common_driver.c
 *   bk_avdk_smp/ap/middleware/driver/audio/aud/aud_dac_driver.c
 *   bk_avdk_smp/ap/middleware/driver/audio/aud/aud_adc_driver.c
 *
 * with the vendor's sys_drv_aud_*() / sys_hal_aud_*() indirection folded
 * into direct register access (see hardware/bk7258_aud.h for the
 * field-by-field provenance of every offset and bit used here).
 *
 * Three things differ from the vendor implementation and are deliberate:
 *
 * 1. Power and module clock come from CP1.  The vendor calls
 *    bk_pm_module_vote_power_ctrl()/bk_pm_clock_ctrl() directly because
 *    its audio driver runs on the core that owns the PMU.  This AP image
 *    does not, so the same two votes are sent over the PWC mailbox
 *    channel, exactly as bk7258_pwm.c does for the PWM module clock.
 *    Watch the polarity: the power command's "on" value is 0
 *    (PM_POWER_MODULE_STATE_ON) while the clock command's "up" value is 1
 *    (PM_CLK_CTRL_PWR_UP).  They are opposite.
 *
 * 2. MIC2 is enabled by writing ana_reg27 directly.  The vendor's
 *    sys_hal_aud_mic2_en() is an explicit "//not support" stub on BK7258
 *    (sys_hal.c:1939-1942); the only reason MIC2 works in the vendor
 *    stack at all is that bk_aud_driver_init() writes the constant
 *    0x91800006 to ana_reg27, whose bit 28 is micen.  This driver keeps
 *    that behaviour but makes it explicit and optional.
 *
 * 3. Analog register writes go through aud_ana_write(), which polls the
 *    analog-SPI busy bit after each write.  A bare putreg32() to these
 *    registers is silently unreliable -- see hardware/bk7258_aud.h.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "irq.h"
#include "bk7258_aud.h"
#include "hardware/bk7258_aud.h"
#include "hardware/bk7258_mbox.h"
#include "hardware/bk7258_sysctrl.h"

#ifdef CONFIG_BK7258_AUDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWC mailbox commands and parameters.
 *
 * Command numbers: bk_avdk_smp/cp/include/driver/pwr_clk.h:25-26.
 * The CP forwards param1/param2 verbatim to bk_pm_module_vote_power_ctrl()
 * and bk_pm_clock_ctrl() (cp/middleware/driver/pwr_clk/pwr_clk.c:206-210
 * -> low_pwr_core.c:222-231).
 *
 * AUDP_AUDIO sub power domain id: POWER_MODULE_NAME_AUDP (6) *
 * PM_MODULE_SUB_POWER_DOMAIN_MAX (20) + 2 == 122, from
 * cp/middleware/soc/bk7258/hal/sys_types.h:326,372-376 and :145.  Passing
 * a *sub* domain to the power command is the vendor's own usage
 * (aud_common_driver.c:359 uses PM_POWER_SUB_MODULE_NAME_AUDP_AUDIO, and
 * i2s_driver.c:146 / psram_hal.c:427 do the same for their sub domains)
 * even though the prototype names the top-level enum.
 *
 * AUD module clock id: PM_CLK_ID_AUDIO == 30, from
 * cp/include/modules/pm.h:285.  Note this is pm_dev_clk_e and NOT the
 * sub-power-domain numbering used by the power command.
 */

#define AUD_PM_POWER_CTRL_CMD      0x1u
#define AUD_PM_CLK_CTRL_CMD        0x2u

#define AUD_PM_SUBDOMAIN_AUDP_AUDIO 122u
#define AUD_PM_CLK_ID_AUDIO         30u

#define AUD_PM_POWER_STATE_ON       0u   /* PM_POWER_MODULE_STATE_ON  */
#define AUD_PM_POWER_STATE_OFF      1u   /* PM_POWER_MODULE_STATE_OFF */
#define AUD_PM_CLK_PWR_DOWN         0u   /* PM_CLK_CTRL_PWR_DOWN      */
#define AUD_PM_CLK_PWR_UP           1u   /* PM_CLK_CTRL_PWR_UP        */

/* PWC only enqueues the request; it does not consume CP1's ACK yet (see
 * the comment in bk7258_motor.c around its own vote).  Until it does, the
 * only way to let CP1 act on the votes is to wait.  bk7258_motor.c uses
 * 20 ms for the PWM clock gate; the audio path additionally has to bring
 * up a power domain, so allow more and then verify by reading the block's
 * device ID.
 */

#define AUD_PM_SETTLE_US            30000

/* Timeout handed to bk7258_mailbox_wait_pwc() after each vote, matching
 * PM_TRANSPORT_TIMEOUT_MS in bk7258_pm_pwc.c.  This bounds the wait for
 * the PWC channel to drain; it is not a wait for CP1 to have acted on the
 * request, which PWC does not yet report.
 */

#define AUD_PM_TRANSPORT_TIMEOUT_MS 500

/* Number of aud_ana_write() busy-poll iterations before giving up.  The
 * analog SPI link runs at a fixed divider off the system clock and the
 * vendor's sys_ll_set_analog_reg_value() spins unbounded; bound it here so
 * a mis-configured clock cannot hang bring-up.
 */

#define AUD_ANA_POLL_LIMIT          100000

/* FIFO service thresholds, in samples.  The DAC left FIFO raises its
 * interrupt once the fill level drops below DACL_RD_THRED, the ADC FIFO
 * once it rises above ADC_WR_THRED.  Both fields are 5 bits wide.
 * v23 observed ADC FIFO_FULL on every IRQ at threshold 16. Do not infer
 * FIFO capacity from the width of the threshold field: request capture
 * service earlier. Leave the hardware-verified playback setting unchanged.
 */

#define AUD_DACL_RD_THRESHOLD       16u
#define AUD_ADC_WR_THRESHOLD        4u

/* A plausible AUD_DEVICE_ID must be neither all-zeroes (block held in
 * reset or unclocked) nor all-ones (bus returning a floating value).
 */

#define AUD_ID_IS_VALID(id)         ((id) != 0u && (id) != 0xffffffffu)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_aud_initialized;
static bool g_aud_dac_ready;
static bool g_aud_adc_ready;
static bool g_aud_irq_attached;
static const void *g_aud_owner;

/* Per-direction callbacks.
 *
 * These must be registered independently, each with its own argument.  An
 * earlier version used a single set_callbacks(tx, rx, arg) entry point with
 * one shared argument, which had two consequences on hardware:
 *
 *   - Starting playback after capture overwrote the rx callback with NULL,
 *     so the ISR found the ADC flag set with nothing to service it and
 *     masked ADC_INT_EN as a safety measure.  Capture then stopped for
 *     good.  Simultaneous play+capture reported zero buffers and zero
 *     samples while capture alone worked.
 *   - Both callbacks were handed the same argument, so with both
 *     directions active one of them would receive the other direction's
 *     instance pointer.
 */

static bk7258_aud_xfer_cb_t g_aud_tx_cb;
static void *g_aud_tx_arg;
static bk7258_aud_xfer_cb_t g_aud_rx_cb;
static void *g_aud_rx_arg;

/* Mirrors of the last programmed capture geometry, needed because the
 * ADC FIFO port always presents both halves and the reader has to know
 * whether the right half is meaningful.
 */

static bool g_aud_capture_reference;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aud_ana_write
 *
 * Description:
 *   Write one analog register and wait for the analog-SPI shadow transfer
 *   to retire, mirroring sys_ll_set_analog_reg_value() in
 *   bk_avdk_smp/ap/middleware/soc/bk7258_ap/hal/sys_ll.h:51-59.
 *
 ****************************************************************************/

static int aud_ana_write(unsigned int reg, uint32_t value)
{
  uint32_t limit = AUD_ANA_POLL_LIMIT;

  putreg32(value, BK7258_ANA_REG(reg));

  while ((getreg32(BK7258_ANA_SPI_STATE) & BK7258_ANA_SPI_BUSY(reg)) != 0)
    {
      if (limit-- == 0)
        {
          return -ETIMEDOUT;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: aud_ana_modify
 *
 * Description:
 *   Read-modify-write one analog register.  The read side is a plain
 *   register read (the shadow is readable directly, see
 *   sys_ll_get_analog_reg_value()); only the write needs the poll.
 *
 ****************************************************************************/

static int aud_ana_modify(unsigned int reg, uint32_t clrbits,
                          uint32_t setbits)
{
  uint32_t value = getreg32(BK7258_ANA_REG(reg));

  value &= ~clrbits;
  value |= setbits;

  return aud_ana_write(reg, value);
}

/****************************************************************************
 * Name: aud_pwc_request
 *
 * Description:
 *   Ask CP1 for the AUDP_AUDIO power domain and the AUD module clock.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: aud_pwc_send
 *
 * Description:
 *   Send one PWC request and wait for the channel to drain.
 *
 *   The wait is mandatory, not an optimisation: the PWC logical channel
 *   holds exactly one in-flight message, and bk7258_mailbox_send_wire()
 *   rejects a second one with -EBUSY while the first is still queued
 *   (bk7258_mailbox_channel.c:931-935).  bk7258_pm_pwc.c pairs every
 *   bk7258_mailbox_send_pwc() with bk7258_mailbox_wait_pwc() for the same
 *   reason; bk7258_pwm.c gets away without it only because the motor needs
 *   a single vote.
 *
 *   The first version of this driver sent the AUDP power vote and the AUD
 *   clock vote back to back and failed on hardware with
 *   "AUD block bring-up failed, error=-16".
 *
 ****************************************************************************/

static int aud_pwc_send(uint8_t command, uint32_t p1, uint32_t p2)
{
  int ret = bk7258_mailbox_send_pwc(command, p1, p2, 0);

  if (ret >= 0)
    {
      ret = bk7258_mailbox_wait_pwc(AUD_PM_TRANSPORT_TIMEOUT_MS);
    }

  return ret;
}

static int aud_pwc_request(bool enable)
{
  int ret;

  if (enable)
    {
      ret = aud_pwc_send(AUD_PM_POWER_CTRL_CMD,
                         AUD_PM_SUBDOMAIN_AUDP_AUDIO,
                         AUD_PM_POWER_STATE_ON);
      if (ret < 0)
        {
          auderr("AUDP power vote failed: %d\n", ret);
          return ret;
        }

      ret = aud_pwc_send(AUD_PM_CLK_CTRL_CMD, AUD_PM_CLK_ID_AUDIO,
                         AUD_PM_CLK_PWR_UP);
      if (ret < 0)
        {
          auderr("AUD clock vote failed: %d\n", ret);
          return ret;
        }
    }
  else
    {
      (void)aud_pwc_send(AUD_PM_CLK_CTRL_CMD, AUD_PM_CLK_ID_AUDIO,
                         AUD_PM_CLK_PWR_DOWN);
      (void)aud_pwc_send(AUD_PM_POWER_CTRL_CMD,
                         AUD_PM_SUBDOMAIN_AUDP_AUDIO,
                         AUD_PM_POWER_STATE_OFF);
      return OK;
    }

  /* PWC does not yet surface CP1's ACK, so fall back to a settle delay
   * followed by a positive readback check.
   */

  nxsig_usleep(AUD_PM_SETTLE_US);
  return OK;
}

/****************************************************************************
 * Name: aud_clk_config
 *
 * Description:
 *   Select the audio master clock, following bk_aud_clk_config() in
 *   aud_common_driver.c:316-342.
 *
 ****************************************************************************/

static int aud_clk_config(enum bk7258_aud_clksrc clksrc)
{
  if (clksrc == BK7258_AUD_CLK_APLL)
    {
      /* The APLL path additionally needs sys_drv_apll_en(),
       * sys_drv_apll_cal_val_set(0x8973CA6F or 0x88AF2EC9 for the 44.1k
       * family), sys_drv_apll_config_set(0xC2A0AE86) and a
       * sys_drv_apll_spi_trigger_set() 1-delay-0 pulse
       * (aud_common_driver.c:319-329 and the per-rate recalibration in
       * aud_dac_driver.c:184-199).  Those registers have not been
       * transcribed into hardware/bk7258_aud.h yet, and guessing them
       * would be exactly the kind of unverified register write this port
       * is trying to avoid.  XTAL covers 8k/16k/32k, which is what the
       * bring-up sequence needs; reject APLL explicitly rather than
       * silently running at the wrong rate.
       */

      return -ENOTSUP;
    }

  /* XTAL: cksel_aud = 0 in SYSCTRL and apll_sel = 0 in AUD_CONFIG. */

  /* CP owns the shared clock register; do not race its updates. */

  if ((getreg32(BK7258_SYS_CLKDIV1) & BK7258_SYS_CKSEL_AUD) != 0)
    {
      return -ENOTSUP;
    }

  modifyreg32(BK7258_AUD_CONFIG, BK7258_AUD_APLL_SEL, 0);

  return OK;
}

/****************************************************************************
 * Name: aud_set_dac_samplerate / aud_set_adc_samplerate
 *
 * Description:
 *   Program the sample-rate field and, for rates that are not one of the
 *   four natively encoded ones, the fractional-modulus divider.
 *
 *   These two functions intentionally do NOT share a table: the vendor
 *   scales the fractional constants differently for the two directions
 *   and the asymmetry is preserved verbatim.  Compare
 *   aud_dac_driver.c:201-227 with aud_adc_driver.c:190-241:
 *
 *     rate    DAC fracmod              ADC fracmod          rate field
 *     11025   CONST_DIV_44_1K << 2     CONST_DIV_44_1K      2 / 2
 *     12000   CONST_DIV_48K   << 2     CONST_DIV_48K        3 / 3
 *     22050   CONST_DIV_44_1K << 1     CONST_DIV_44_1K >> 1 2 / 2
 *     24000   CONST_DIV_48K   << 1     CONST_DIV_48K   >> 1 3 / 3
 *     32000   CONST_DIV_16K   >> 1     CONST_DIV_32K        1 / 3
 *
 ****************************************************************************/

static int aud_set_dac_samplerate(uint32_t samplerate)
{
  uint32_t fracmod = 0;
  uint32_t field;
  bool manual = true;

  switch (samplerate)
    {
      case 8000:
        field = BK7258_AUD_SAMP_RATE_8K;
        manual = false;
        break;

      case 16000:
        field = BK7258_AUD_SAMP_RATE_16K;
        manual = false;
        break;

      case 44100:
        field = BK7258_AUD_SAMP_RATE_44_1K;
        manual = false;
        break;

      case 48000:
        field = BK7258_AUD_SAMP_RATE_48K;
        manual = false;
        break;

      case 11025:
        field = BK7258_AUD_SAMP_RATE_44_1K;
        fracmod = BK7258_AUD_FRACMOD_44_1K << 2;
        break;

      case 12000:
        field = BK7258_AUD_SAMP_RATE_48K;
        fracmod = BK7258_AUD_FRACMOD_48K << 2;
        break;

      case 22050:
        field = BK7258_AUD_SAMP_RATE_44_1K;
        fracmod = BK7258_AUD_FRACMOD_44_1K << 1;
        break;

      case 24000:
        field = BK7258_AUD_SAMP_RATE_48K;
        fracmod = BK7258_AUD_FRACMOD_48K << 1;
        break;

      case 32000:
        field = BK7258_AUD_SAMP_RATE_16K;
        fracmod = BK7258_AUD_FRACMOD_16K >> 1;
        break;

      default:
        return -EINVAL;
    }

  /* Clear the manual bit first so a previous rate's divider cannot leak
   * into a natively encoded rate (aud_dac_driver.c:201 does the same).
   */

  modifyreg32(BK7258_AUD_EXTEND_CFG, BK7258_AUD_DAC_FRACMOD_MANUAL, 0);

  if (manual)
    {
      putreg32(fracmod, BK7258_AUD_DAC_FRACMOD);
      modifyreg32(BK7258_AUD_EXTEND_CFG, 0, BK7258_AUD_DAC_FRACMOD_MANUAL);
    }

  modifyreg32(BK7258_AUD_CONFIG, BK7258_AUD_SAMP_RATE_DAC_MASK,
              field << BK7258_AUD_SAMP_RATE_DAC_SHIFT);

  return OK;
}

static int aud_set_adc_samplerate(uint32_t samplerate)
{
  uint32_t fracmod = 0;
  uint32_t field;
  bool manual = true;

  switch (samplerate)
    {
      case 8000:
        field = BK7258_AUD_SAMP_RATE_8K;
        manual = false;
        break;

      case 16000:
        field = BK7258_AUD_SAMP_RATE_16K;
        manual = false;
        break;

      case 44100:
        field = BK7258_AUD_SAMP_RATE_44_1K;
        manual = false;
        break;

      case 48000:
        field = BK7258_AUD_SAMP_RATE_48K;
        manual = false;
        break;

      case 11025:
        field = BK7258_AUD_SAMP_RATE_44_1K;
        fracmod = BK7258_AUD_FRACMOD_44_1K;
        break;

      case 12000:
        field = BK7258_AUD_SAMP_RATE_48K;
        fracmod = BK7258_AUD_FRACMOD_48K;
        break;

      case 22050:
        field = BK7258_AUD_SAMP_RATE_44_1K;
        fracmod = BK7258_AUD_FRACMOD_44_1K >> 1;
        break;

      case 24000:
        field = BK7258_AUD_SAMP_RATE_48K;
        fracmod = BK7258_AUD_FRACMOD_48K >> 1;
        break;

      case 32000:
        field = BK7258_AUD_SAMP_RATE_48K;
        fracmod = BK7258_AUD_FRACMOD_32K;
        break;

      default:
        return -EINVAL;
    }

  modifyreg32(BK7258_AUD_EXTEND_CFG, BK7258_AUD_ADC_FRACMOD_MANUAL, 0);

  if (manual)
    {
      putreg32(fracmod, BK7258_AUD_ADC_FRACMOD);
      modifyreg32(BK7258_AUD_EXTEND_CFG, 0, BK7258_AUD_ADC_FRACMOD_MANUAL);
    }

  modifyreg32(BK7258_AUD_CONFIG, BK7258_AUD_SAMP_RATE_ADC_MASK,
              field << BK7258_AUD_SAMP_RATE_ADC_SHIFT);

  return OK;
}

/****************************************************************************
 * Name: bk7258_aud_isr
 *
 * Description:
 *   AUD FIFO interrupt.  The block reports DAC-left and ADC conditions in
 *   the same status word (REG_0x0E) and shares one interrupt line, so
 *   dispatch on the individual flags.
 *
 ****************************************************************************/

static int bk7258_aud_isr(int irq, void *context, void *arg)
{
  uint32_t status = getreg32(BK7258_AUD_FIFO_STATUS);

  (void)irq;
  (void)context;
  (void)arg;

  if ((status & BK7258_AUD_DACL_INT_FLAG) != 0 && g_aud_tx_cb != NULL)
    {
      g_aud_tx_cb(g_aud_tx_arg);
    }

  if ((status & BK7258_AUD_ADC_INT_FLAG) != 0 && g_aud_rx_cb != NULL)
    {
      g_aud_rx_cb(g_aud_rx_arg);
    }

  /* The flags are cleared by servicing the corresponding FIFO (draining
   * the ADC below its threshold, refilling the DAC above its threshold).
   * If a callback is missing, mask that direction instead of spinning in
   * the ISR forever.
   */

  if ((status & BK7258_AUD_DACL_INT_FLAG) != 0 && g_aud_tx_cb == NULL)
    {
      modifyreg32(BK7258_AUD_FIFO_CONFIG, BK7258_AUD_DACL_INT_EN, 0);
    }

  if ((status & BK7258_AUD_ADC_INT_FLAG) != 0 && g_aud_rx_cb == NULL)
    {
      modifyreg32(BK7258_AUD_FIFO_CONFIG, BK7258_AUD_ADC_INT_EN, 0);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bk7258_aud_initialize
 ****************************************************************************/

int bk7258_aud_initialize(void)
{
  irqstate_t flags;
  uint32_t id;
  int ret;

  if (g_aud_initialized)
    {
      return OK;
    }

  /* Power domain and module clock first: every register below lives in
   * the AUDP domain or in the analog block it feeds.
   */

  ret = aud_pwc_request(true);
  if (ret < 0)
    {
      auderr("AUDP power/clock vote failed: %d\n", ret);
      return ret;
    }

  id = getreg32(BK7258_AUD_DEVICE_ID);
  if (!AUD_ID_IS_VALID(id))
    {
      auderr("AUD block unreachable, device id=0x%08" PRIx32 "\n", id);
      (void)aud_pwc_request(false);
      return -ENODEV;
    }

  /* Default to the crystal so the block has a valid clock before any
   * analog configuration.  Callers that need 44.1k/48k accuracy select
   * APLL later via *_setup().
   */

  ret = aud_clk_config(BK7258_AUD_CLK_XTAL);
  if (ret < 0)
    {
      (void)aud_pwc_request(false);
      return ret;
    }

  /* AUD_CLK_CONTROL must be left with BOTH bits asserted.
   *
   * This was established by experiment on hardware, because the vendor
   * sources describe bit0 in a way that contradicts how it behaves.
   * aud_ll_macro_def.h:83 calls REG_0x02[0] a soft reset and says
   * "software must clear it", so an earlier version of this driver pulsed
   * it (set, delay, clear).  With that sequence every other register read
   * back correctly and yet the ADC produced nothing at all -- the FIFO
   * stayed in ADC_FIFO_EMPTY and even a polled read returned zero samples.
   *
   * Sweeping all four combinations at 16 kHz stereo, polling for 200 ms
   * each (expected sample count 16000 * 2 * 0.2 = 6400):
   *
   *   clkctl=0  rst=0 gate=0 ->    0 samples
   *   clkctl=1  rst=1 gate=0 -> 4760 samples, mean square 6797105
   *   clkctl=2  rst=0 gate=1 ->    0 samples
   *   clkctl=3  rst=1 gate=1 -> 6400 samples, mean square 9565
   *
   * So bit0 behaves as a release that must stay asserted, not as a pulse:
   * clearing it holds the ADC in reset.  bit1 additionally has to be set
   * for the sample rate to be correct -- with it clear the capture is 26 %
   * short and the mean square is three orders of magnitude higher, i.e.
   * samples are being dropped and corrupted rather than merely attenuated.
   * clkctl=3 hits the expected sample count exactly.
   *
   * The vendor's bk_aud_driver_init() (aud_common_driver.c:375) sets bit0
   * and never clears it, which is consistent with this result.
   */

  putreg32(BK7258_AUD_ADC_SOFT_RESET | BK7258_AUD_ADC_CLK_GATE,
           BK7258_AUD_CLK_CONTROL);
  up_udelay(100);

  /* Load the analog block's reset values, then enable the audio bias.
   * Order matches aud_common_driver.c:377-384.
   */

  ret = aud_ana_write(BK7258_ANA_REG18, BK7258_ANA18_RESET_VALUE);
  if (ret >= 0)
    {
      ret = aud_ana_write(BK7258_ANA_REG19, BK7258_ANA19_RESET_VALUE);
    }

  if (ret >= 0)
    {
      ret = aud_ana_write(BK7258_ANA_REG20, BK7258_ANA20_RESET_VALUE);
    }

  if (ret >= 0)
    {
      ret = aud_ana_write(BK7258_ANA_REG21, BK7258_ANA21_RESET_VALUE);
    }

  if (ret >= 0)
    {
      ret = aud_ana_write(BK7258_ANA_REG27, BK7258_ANA27_RESET_VALUE);
    }

  if (ret >= 0)
    {
      ret = aud_ana_modify(BK7258_ANA_REG18, 0, BK7258_ANA18_ENAUDBIAS);
    }

  if (ret < 0)
    {
      auderr("analog register programming failed: %d\n", ret);
      (void)aud_pwc_request(false);
      return ret;
    }

  /* Start from a quiet block: nothing enabled, no interrupts, no
   * ADC-to-DAC loopback left over from a previous owner.
   */

  flags = enter_critical_section();

  modifyreg32(BK7258_AUD_CONFIG,
              BK7258_AUD_DAC_ENABLE | BK7258_AUD_ADC_ENABLE |
              BK7258_AUD_DTMF_ENABLE | BK7258_AUD_LINEIN_ENABLE |
              BK7258_AUD_DMIC_ENABLE, 0);

  modifyreg32(BK7258_AUD_FIFO_CONFIG,
              BK7258_AUD_DACL_INT_EN | BK7258_AUD_DACR_INT_EN |
              BK7258_AUD_ADC_INT_EN | BK7258_AUD_LOOP_ADC2DAC, 0);

  leave_critical_section(flags);

  if (!g_aud_irq_attached)
    {
      ret = irq_attach(BK7258_IRQ_AUDIO, bk7258_aud_isr, NULL);
      if (ret < 0)
        {
          auderr("irq_attach failed: %d\n", ret);
          (void)aud_pwc_request(false);
          return ret;
        }

      /* bk7258_irq.c routes external interrupts through CPU1's enable
       * registers, which is the core this image runs on, so this single
       * call is the whole story -- no SYSCTRL CPU0 bit to poke.
       */

      up_enable_irq(BK7258_IRQ_AUDIO);
      g_aud_irq_attached = true;
    }

  g_aud_initialized = true;

  audinfo("AUD ready, device id=0x%08" PRIx32 " version=0x%08" PRIx32 "\n",
          id, getreg32(BK7258_AUD_VERSION_ID));

  return OK;
}

int bk7258_aud_acquire(const void *owner)
{
  irqstate_t flags;
  int ret;
  if (owner == NULL) return -EINVAL;
  flags = enter_critical_section();
  ret = g_aud_owner == NULL ? OK : -EBUSY;
  if (ret == OK) g_aud_owner = owner;
  leave_critical_section(flags);
  return ret;
}

void bk7258_aud_release(const void *owner)
{
  irqstate_t flags = enter_critical_section();
  if (owner != NULL && g_aud_owner == owner) g_aud_owner = NULL;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: bk7258_aud_read_id / bk7258_aud_read_version
 ****************************************************************************/

uint32_t bk7258_aud_read_id(void)
{
  return getreg32(BK7258_AUD_DEVICE_ID);
}

uint32_t bk7258_aud_read_version(void)
{
  return getreg32(BK7258_AUD_VERSION_ID);
}

/****************************************************************************
 * Name: bk7258_aud_dac_setup
 ****************************************************************************/

int bk7258_aud_dac_setup(const struct bk7258_aud_config *cfg)
{
  irqstate_t flags;
  uint32_t setbits;
  int ret;

  if (cfg == NULL)
    {
      return -EINVAL;
    }

  if (cfg->dac_dig_gain > BK7258_AUD_DAC_DIG_GAIN_MAX ||
      cfg->dac_ana_gain > BK7258_AUD_DAC_ANA_GAIN_MAX)
    {
      return -EINVAL;
    }

  ret = bk7258_aud_initialize();
  if (ret < 0)
    {
      return ret;
    }

  ret = aud_clk_config(cfg->clksrc);
  if (ret < 0)
    {
      return ret;
    }

  /* Allow shutdown after partially completed setup. */

  g_aud_dac_ready = true;

  /* Analog DAC path.  aud_dac_driver.c:82-118 order: bias, driver, DC
   * offset cancellation, current source, then the channel enable.  The
   * board's AUDLP/AUDLN pair is differential, and the analog reset value
   * already has diffen set, but set it explicitly so the mode does not
   * depend on the reset value staying that way.
   */

  ret = aud_ana_modify(BK7258_ANA_REG21, 0, BK7258_ANA21_ENBS);
  if (ret >= 0)
    {
      ret = aud_ana_modify(BK7258_ANA_REG20, 0, BK7258_ANA20_DACDRVEN);
    }

  if (ret >= 0)
    {
      ret = aud_ana_modify(BK7258_ANA_REG20, 0, BK7258_ANA20_LENDCOC);
    }

  if (ret >= 0)
    {
      ret = aud_ana_modify(BK7258_ANA_REG21, 0, BK7258_ANA21_ENIDACL);
    }

  if (ret >= 0)
    {
      /* Enable the left channel only: BK7258's DAC right channel does not
       * exist (sys_hal_aud_dacr_en() is a "//not support" stub) and the
       * board is mono anyway.  Set differential mode, the analog gain and
       * start muted.
       */

      ret = aud_ana_modify(BK7258_ANA_REG20,
                           BK7258_ANA20_DACG_MASK,
                           BK7258_ANA20_DACLEN | BK7258_ANA20_DIFFEN |
                           BK7258_ANA20_DACMUTE |
                           ((uint32_t)cfg->dac_ana_gain <<
                            BK7258_ANA20_DACG_SHIFT));
    }

  if (ret < 0)
    {
      auderr("DAC analog setup failed: %d\n", ret);
      return ret;
    }

  flags = enter_critical_section();

  /* Digital gain, and bypass both high-pass filters as the vendor does
   * (aud_dac_driver.c:122-124).
   */

  setbits = ((uint32_t)cfg->dac_dig_gain << BK7258_AUD_DAC_GAIN_SHIFT) |
            BK7258_AUD_DAC_HPF1_BYPASS | BK7258_AUD_DAC_HPF2_BYPASS;

  modifyreg32(BK7258_AUD_DAC_CONFIG0,
              BK7258_AUD_DAC_GAIN_MASK | BK7258_AUD_DAC_CLK_INVERT,
              setbits);

  modifyreg32(BK7258_AUD_FIFO_CONFIG, BK7258_AUD_DACL_RD_THRED_MASK,
              AUD_DACL_RD_THRESHOLD << BK7258_AUD_DACL_RD_THRED_SHIFT);

  leave_critical_section(flags);

  ret = aud_set_dac_samplerate(cfg->samplerate);
  if (ret < 0)
    {
      auderr("unsupported DAC sample rate %" PRIu32 "\n", cfg->samplerate);
      return ret;
    }

  g_aud_dac_ready = true;
  return OK;
}

/****************************************************************************
 * Name: bk7258_aud_dac_shutdown
 ****************************************************************************/

void bk7258_aud_dac_shutdown(void)
{
  if (!g_aud_dac_ready)
    {
      return;
    }

  bk7258_aud_dac_stop();
  bk7258_aud_tx_int_enable(false);
  bk7258_aud_dac_mute(true);

  (void)aud_ana_modify(BK7258_ANA_REG20,
                       BK7258_ANA20_DACLEN | BK7258_ANA20_DACDRVEN |
                       BK7258_ANA20_LENDCOC, 0);
  (void)aud_ana_modify(BK7258_ANA_REG21,
                       BK7258_ANA21_ENIDACL | BK7258_ANA21_ENBS, 0);

  g_aud_dac_ready = false;
}

/****************************************************************************
 * Name: bk7258_aud_dac_start / bk7258_aud_dac_stop
 ****************************************************************************/

void bk7258_aud_dac_start(void)
{
  modifyreg32(BK7258_AUD_CONFIG, 0, BK7258_AUD_DAC_ENABLE);
}

void bk7258_aud_dac_stop(void)
{
  modifyreg32(BK7258_AUD_CONFIG, BK7258_AUD_DAC_ENABLE, 0);
}

/****************************************************************************
 * Name: bk7258_aud_dac_mute
 ****************************************************************************/

int bk7258_aud_dac_mute(bool mute)
{
  if (mute)
    {
      return aud_ana_modify(BK7258_ANA_REG20, 0, BK7258_ANA20_DACMUTE);
    }
  else
    {
      return aud_ana_modify(BK7258_ANA_REG20, BK7258_ANA20_DACMUTE, 0);
    }
}

/****************************************************************************
 * Name: bk7258_aud_dac_set_dig_gain
 ****************************************************************************/

int bk7258_aud_dac_set_dig_gain(uint8_t gain)
{
  irqstate_t flags;

  if (gain > BK7258_AUD_DAC_DIG_GAIN_MAX)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  modifyreg32(BK7258_AUD_DAC_CONFIG0, BK7258_AUD_DAC_GAIN_MASK,
              (uint32_t)gain << BK7258_AUD_DAC_GAIN_SHIFT);
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: bk7258_aud_adc_setup
 ****************************************************************************/

int bk7258_aud_adc_setup(const struct bk7258_aud_config *cfg)
{
  irqstate_t flags;
  uint32_t micgain;
  int ret;

  if (cfg == NULL)
    {
      return -EINVAL;
    }

  if (cfg->adc_gain > BK7258_AUD_ADC_GAIN_MAX ||
      cfg->mic_gain > BK7258_AUD_MIC_GAIN_MAX)
    {
      return -EINVAL;
    }

  ret = bk7258_aud_initialize();
  if (ret < 0)
    {
      return ret;
    }

  ret = aud_clk_config(cfg->clksrc);
  if (ret < 0)
    {
      return ret;
    }

  g_aud_adc_ready = true;
  micgain = (uint32_t)cfg->mic_gain << BK7258_ANA_MIC_GAIN_SHIFT;

  /* MIC1 is the real microphone and is always enabled.  MIC2 carries the
   * post-amplifier echo reference; enabling it makes the ADC FIFO word
   * carry a full L/R frame instead of just L.
   */

  ret = aud_ana_modify(BK7258_ANA_REG19, BK7258_ANA_MIC_GAIN_MASK,
                       BK7258_ANA_MIC_EN | micgain);

  if (ret >= 0)
    {
      if (cfg->capture_reference)
        {
          ret = aud_ana_modify(BK7258_ANA_REG27, BK7258_ANA_MIC_GAIN_MASK,
                               BK7258_ANA_MIC_EN | micgain);
        }
      else
        {
          ret = aud_ana_modify(BK7258_ANA_REG27, BK7258_ANA_MIC_EN, 0);
        }
    }

  /* ADC and MIC bias.  micbias is a single global enable shared by both
   * channels (ana_reg18 bit 5), which is harmless here: the schematic
   * feeds MICBIAS to MIC1 only, through R66/R67, while the MIC2 echo tap
   * is DC-blocked by C60/C61 (1uF), so the reference channel's operating
   * point is unaffected.
   */

  if (ret >= 0)
    {
      ret = aud_ana_modify(BK7258_ANA_REG18, 0,
                           BK7258_ANA18_ENAUDBIAS |
                           BK7258_ANA18_ENADCBIAS |
                           BK7258_ANA18_ENMICBIAS);
    }

  if (ret < 0)
    {
      auderr("ADC analog setup failed: %d\n", ret);
      return ret;
    }

  /* Reset the MIC front-end after its parameters are in place, mirroring
   * aud_adc_driver.c:108-111.  Both channels get the pulse so MIC2 comes
   * out of reset together with MIC1 when it is in use.
   */

  ret = aud_ana_modify(BK7258_ANA_REG19, 0, BK7258_ANA_MIC_RST);
  if (ret >= 0 && cfg->capture_reference)
    {
      ret = aud_ana_modify(BK7258_ANA_REG27, 0, BK7258_ANA_MIC_RST);
    }

  if (ret < 0) return ret;

  up_udelay(100);

  ret = aud_ana_modify(BK7258_ANA_REG19, BK7258_ANA_MIC_RST, 0);
  if (ret >= 0 && cfg->capture_reference)
    {
      ret = aud_ana_modify(BK7258_ANA_REG27, BK7258_ANA_MIC_RST, 0);
    }

  if (ret < 0) return ret;

  flags = enter_critical_section();

  /* The two bypass bits join the clear mask so they can be turned off.
   * Previously they were only ever set, which made the high-pass stages
   * unreachable no matter what a caller asked for.
   */

  modifyreg32(BK7258_AUD_ADC_CONFIG0,
              BK7258_AUD_ADC_GAIN_MASK | BK7258_AUD_ADC_SAMPLE_EDGE |
              BK7258_AUD_ADC_DIG_MIC_SEL |
              BK7258_AUD_ADC_HPF1_BYPASS | BK7258_AUD_ADC_HPF2_BYPASS,
              ((uint32_t)cfg->adc_gain << BK7258_AUD_ADC_GAIN_SHIFT) |
              (cfg->adc_hpf ? 0 : (BK7258_AUD_ADC_HPF1_BYPASS |
                                   BK7258_AUD_ADC_HPF2_BYPASS)));

  modifyreg32(BK7258_AUD_FIFO_CONFIG, BK7258_AUD_ADC_WR_THRED_MASK,
              AUD_ADC_WR_THRESHOLD << BK7258_AUD_ADC_WR_THRED_SHIFT);

  leave_critical_section(flags);

  ret = aud_set_adc_samplerate(cfg->samplerate);
  if (ret < 0)
    {
      auderr("unsupported ADC sample rate %" PRIu32 "\n", cfg->samplerate);
      return ret;
    }

  g_aud_capture_reference = cfg->capture_reference;
  g_aud_adc_ready = true;
  return OK;
}

/****************************************************************************
 * Name: bk7258_aud_adc_shutdown
 ****************************************************************************/

void bk7258_aud_adc_shutdown(void)
{
  if (!g_aud_adc_ready)
    {
      return;
    }

  bk7258_aud_adc_stop();
  bk7258_aud_rx_int_enable(false);

  (void)aud_ana_modify(BK7258_ANA_REG19, BK7258_ANA_MIC_EN, 0);
  (void)aud_ana_modify(BK7258_ANA_REG27, BK7258_ANA_MIC_EN, 0);
  (void)aud_ana_modify(BK7258_ANA_REG18,
                       BK7258_ANA18_ENADCBIAS | BK7258_ANA18_ENMICBIAS, 0);

  g_aud_capture_reference = false;
  g_aud_adc_ready = false;
}

/****************************************************************************
 * Name: bk7258_aud_adc_start / bk7258_aud_adc_stop
 ****************************************************************************/

void bk7258_aud_adc_start(void)
{
  modifyreg32(BK7258_AUD_CONFIG, 0, BK7258_AUD_ADC_ENABLE);
}

void bk7258_aud_adc_stop(void)
{
  modifyreg32(BK7258_AUD_CONFIG, BK7258_AUD_ADC_ENABLE, 0);
}

/****************************************************************************
 * Name: bk7258_aud_set_callbacks
 ****************************************************************************/

void bk7258_aud_set_tx_callback(bk7258_aud_xfer_cb_t cb, void *arg)
{
  irqstate_t flags = enter_critical_section();

  g_aud_tx_cb = cb;
  g_aud_tx_arg = arg;

  leave_critical_section(flags);
}

void bk7258_aud_set_rx_callback(bk7258_aud_xfer_cb_t cb, void *arg)
{
  irqstate_t flags = enter_critical_section();

  g_aud_rx_cb = cb;
  g_aud_rx_arg = arg;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: bk7258_aud_dac_write
 ****************************************************************************/

unsigned int bk7258_aud_dac_write(const int16_t *samples,
                                  unsigned int nsamples)
{
  unsigned int written = 0;

  if (samples == NULL)
    {
      return 0;
    }

  while (written < nsamples)
    {
      if ((getreg32(BK7258_AUD_FIFO_STATUS) &
           BK7258_AUD_DACL_FIFO_FULL) != 0)
        {
          break;
        }

      /* Mono: duplicate the sample into both halves so the word is well
       * defined even though the right DAC does not exist on BK7258.
       */

      putreg32(((uint32_t)(uint16_t)samples[written] <<
                BK7258_AUD_FIFO_L_SHIFT) |
               ((uint32_t)(uint16_t)samples[written] <<
                BK7258_AUD_FIFO_R_SHIFT),
               BK7258_AUD_DAC_FIFO_PORT);

      written++;
    }

  return written;
}

/****************************************************************************
 * Name: bk7258_aud_adc_read
 ****************************************************************************/

unsigned int bk7258_aud_adc_read(int16_t *samples, unsigned int nsamples)
{
  unsigned int read = 0;
  unsigned int step;
  uint32_t word;

  if (samples == NULL)
    {
      return 0;
    }

  /* One FIFO word carries L in [15:0] and R in [31:16].  With the echo
   * reference enabled both halves are consumed, so the caller sees
   * interleaved L,R pairs; otherwise only L is stored.
   */

  step = g_aud_capture_reference ? 2 : 1;

  while (read + step <= nsamples)
    {
      if ((getreg32(BK7258_AUD_FIFO_STATUS) &
           BK7258_AUD_ADC_FIFO_EMPTY) != 0)
        {
          break;
        }

      word = getreg32(BK7258_AUD_ADC_FIFO_PORT);

      samples[read++] = (int16_t)((word >> BK7258_AUD_FIFO_L_SHIFT) &
                                 BK7258_AUD_FIFO_SAMPLE_MASK);

      if (step == 2)
        {
          samples[read++] = (int16_t)((word >> BK7258_AUD_FIFO_R_SHIFT) &
                                      BK7258_AUD_FIFO_SAMPLE_MASK);
        }
    }

  return read;
}

/****************************************************************************
 * Name: bk7258_aud_tx_int_enable / bk7258_aud_rx_int_enable
 ****************************************************************************/

void bk7258_aud_tx_int_enable(bool enable)
{
  irqstate_t flags = enter_critical_section();

  if (enable)
    {
      modifyreg32(BK7258_AUD_FIFO_CONFIG, 0, BK7258_AUD_DACL_INT_EN);
    }
  else
    {
      modifyreg32(BK7258_AUD_FIFO_CONFIG, BK7258_AUD_DACL_INT_EN, 0);
    }

  leave_critical_section(flags);
}

void bk7258_aud_rx_int_enable(bool enable)
{
  irqstate_t flags = enter_critical_section();

  if (enable)
    {
      modifyreg32(BK7258_AUD_FIFO_CONFIG, 0, BK7258_AUD_ADC_INT_EN);
    }
  else
    {
      modifyreg32(BK7258_AUD_FIFO_CONFIG, BK7258_AUD_ADC_INT_EN, 0);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: bk7258_aud_fifo_status
 ****************************************************************************/

uint32_t bk7258_aud_fifo_status(void)
{
  return getreg32(BK7258_AUD_FIFO_STATUS);
}

uint32_t bk7258_aud_adc_threshold(void)
{
  return (getreg32(BK7258_AUD_FIFO_CONFIG) &
          BK7258_AUD_ADC_WR_THRED_MASK) >> BK7258_AUD_ADC_WR_THRED_SHIFT;
}

#endif /* CONFIG_BK7258_AUDIO */
