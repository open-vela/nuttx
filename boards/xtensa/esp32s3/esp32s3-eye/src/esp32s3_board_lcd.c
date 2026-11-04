/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-eye/src/esp32s3_board_lcd.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdbool.h>
#include <string.h>       /* [V627] strchr() for the /tmp/net.txt probe */
#include <debug.h>
#include <errno.h>
#include <syslog.h>
#include <fcntl.h>       /* [V554] open() for the /dev/fb0 probe */
#include <sys/ioctl.h>   /* [V554] FBIOGET_* */
#include <sys/mman.h>    /* [V554] mmap() the framebuffer to inspect it */
#include <sys/stat.h>    /* [V554] */

#include <nuttx/arch.h>
#include <nuttx/kthread.h>
#include <nuttx/sched.h>

#include <nuttx/board.h>
#include <nuttx/signal.h>
#include <unistd.h>      /* [V326] usleep */
#include <nuttx/clock.h> /* [V548] up_mdelay */
#include "esp32s3_ledc.h"   /* [V481] LEDC hardware PWM */
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7789.h>
#include <nuttx/kmalloc.h>
#include <nuttx/video/fb.h>   /* [V554] struct fb_videoinfo_s / fb_planeinfo_s */

#include <arch/board/board.h>

#include "esp32s3_gpio.h"
#include "esp32s3_spi.h"
#include "hardware/esp32s3_gpio_sigmap.h"

/* [V546] LEDC backlight helper implemented in arch/xtensa/src/esp32s3/
 * esp32s3_ledc.c.  Declared in esp32s3_ledc.h but that header pulls in
 * arch-private registers, so re-declare it here.
 * [V548] No longer called (the backlight is plain GPIO), but keep the
 * declaration so the LEDC path stays available if we ever revisit it.
 */
extern int esp32s3_ledc_raw_backlight(uint32_t freq, uint32_t duty16)
  __attribute__((unused));


#include "esp32s3-eye.h"

/* [LCDDIAG] Board-level LCD / backlight / SPI pin tracing (black-screen hunt).
 * Prints the physical pin numbers plus their read-back levels so we can tell
 * whether the backlight (GPIO16) is really driven high, whether the shared
 * DC/MISO pin (GPIO14) ended up as a plain GPIO output, and whether the SPI3
 * bus pins were (re)configured successfully.
 */

#define LCDDIAG(fmt, ...) \
  syslog(LOG_ERR, "[LCDDIAG] board: " fmt "\n", ##__VA_ARGS__)

/* [V535] GPIO16 full output chain -- authoritative addresses from headers:
 *
 *   IO_MUX GPIO16           = 0x60009000 + 0x04 + 16*4 = 0x60009044
 *     (PERIPHS_IO_MUX_GPIO0_U = REG_IO_MUX_BASE + 0x04, so configgpio's
 *      (pin+1)*4 formula IS correct.  MCU_SEL=1 = GPIO function.)
 *   GPIO_FUNC16_OUT_SEL_CFG = DR_REG_GPIO_BASE + 0x594 = 0x60004594
 *     (GPIO matrix: WHICH signal drives the pad output.  Reset = 256 =
 *      SIG_GPIO_OUT_IDX so the pad follows GPIO_OUT bit16.  If a peripheral
 *      driver reroutes this, GPIO_OUT toggles but the pin does not!)
 *   GPIO_PIN16_REG          = DR_REG_GPIO_BASE + 0xB4 = 0x600040B4
 *     (bit2 PAD_DRIVER: 1 = open-drain -> HIGH cannot drive the line.)
 *   GPIO_PIN20_REG          = DR_REG_GPIO_BASE + 0xC4 = 0x600040C4
 *     (v534 mistakenly wrote 0x1A00 here believing it was the IO_MUX;
 *      restore it to 0.)
 *
 * The pad only outputs the backlight level when ALL of these agree:
 * MCU_SEL=1 AND OUTSEL=256 AND PAD_DRIVER=0 AND GPIO_OUT.b16 = level. */
static void ai_vox3_bl_dump16(const char *tag)
{
  volatile uint32_t *iomux  = (volatile uint32_t *)0x60009044u;
  volatile uint32_t *outsel = (volatile uint32_t *)0x60004594u;
  volatile uint32_t *pinreg = (volatile uint32_t *)0x600040B4u;
  volatile uint32_t *pin20  = (volatile uint32_t *)0x600040C4u;
  /* [V536] RTC-domain suspects: GPIO16 == XTAL_32K_N pad.
   *   RTCIO_XTAL_32N_PAD_REG  = 0x60008400 + 0xc4 = 0x600084C4
   *     bit19 X32N_MUX_SEL: 1 = pad claimed by the RTC 32K oscillator
   *     (analog domain, overrides the digital IO_MUX entirely).
   *   RTC_CNTL_RTC_CLK_CONF_REG = 0x60008000 + 0x74 = 0x60008074
   *     bit8 DIG_XTAL32K_EN: digital 32K oscillator enable.
   * [V537] Pad HOLD latches live in the RTC power domain and SURVIVE a
   *   USB reset -- a hold bit set by any earlier firmware freezes the pad
   *   forever no matter what every other register says.
   *   RTC_CNTL_RTC_PAD_HOLD_REG = 0x600080D8  (RTC pads = GPIO0..21, bit16)
   *   RTC_CNTL_DIG_PAD_HOLD_REG = 0x600080DC  (digital pads) */
  volatile uint32_t *x32n    = (volatile uint32_t *)0x600084C4u;
  volatile uint32_t *clkcf   = (volatile uint32_t *)0x60008074u;
  volatile uint32_t *rtchold = (volatile uint32_t *)0x600080D8u;
  volatile uint32_t *dighold = (volatile uint32_t *)0x600080DCu;

  LCDDIAG("%s: MUX=0x%08x OUTSEL=0x%08x PIN16=0x%08x X32N=0x%08x "
          "RTCHOLD=0x%08x DIGHOLD=0x%08x",
          tag, (unsigned)*iomux, (unsigned)*outsel, (unsigned)*pinreg,
          (unsigned)*x32n, (unsigned)*rtchold, (unsigned)*dighold);
}

/* [V546] Retained for diagnosis but no longer called: the backlight is now
 * driven by LEDC, and this function's whole purpose was to force GPIO16 to be
 * a plain GPIO output -- which would detach it from the LEDC matrix. */
static void ai_vox3_bl_force_gpio(void);   /* [V847] called again */
static void ai_vox3_bl_force_gpio(void)
{
  volatile uint32_t *iomux    = (volatile uint32_t *)0x60009044u;
  volatile uint32_t *outsel   = (volatile uint32_t *)0x60004594u;
  volatile uint32_t *pinreg   = (volatile uint32_t *)0x600040B4u;
  volatile uint32_t *pin20    = (volatile uint32_t *)0x600040C4u;
  volatile uint32_t *x32n     = (volatile uint32_t *)0x600084C4u;
  volatile uint32_t *clkcf    = (volatile uint32_t *)0x60008074u;
  volatile uint32_t *rtchold  = (volatile uint32_t *)0x600080D8u;
  volatile uint32_t *dighold  = (volatile uint32_t *)0x600080DCu;

  /* RTC domain: give the pad back to the digital IO_MUX and stop the 32K
   * oscillator.  Read-modify-write: these registers hold other fields. */
  *x32n     &= ~(1u << 19);   /* X32N_MUX_SEL = 0: pad follows digital IO_MUX */
  *clkcf    &= ~(1u << 8);    /* DIG_XTAL32K_EN = 0 */
  *rtchold  &= ~(1u << 16);   /* [V537] release GPIO16 RTC-pad hold latch */
  *dighold  &= ~(1u << 16);   /* [V537] same, in case the mapping differs */

  *pin20  = 0;             /* undo v534 accidental write */
  *outsel = 256u;          /* SIG_GPIO_OUT_IDX: pad out <- GPIO_OUT bit16 */
  *pinreg = 0;             /* push-pull, no interrupt */
  *iomux  = (uint32_t)((1u << 12) | (1u << 9) | (2u << 10));
}

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spi_dev_s *g_spidev;
static struct lcd_dev_s *g_lcd;

/****************************************************************************
 * Public Functions
 ****************************************************************************/


/****************************************************************************
 * Name:  lcd_selftest
 *
 * Description:
 *   AI-VOX3 boot self-test (v530).  st7789_lcdinitialize() already issues
 *   SWRESET+SLPOUT+DISPON and fills the panel with 0xffff (white), so a
 *   black screen after boot means the BACKLIGHT is OFF, i.e. the polarity
 *   assumption is wrong.  This thread paints a solid color block via putarea
 *   and BLINKS the backlight (LOW/HIGH each second) so the user can tell us
 *   which backlight state lights the panel -- that disambiguates
 *   active-low vs active-high with a single flash.
 *
 *   [V530] Backlight on AI-VOX3 is ACTIVE-LOW: GPIO16 LOW = on, HIGH = off.
 *   The original openvela board file wrote HIGH, which actually switched the
 *   backlight OFF -- that is why every build looked black even though
 *   st7789_lcdinitialize() returned a valid device and filled white.
 *
 ****************************************************************************/

#define AI_VOX3_LCD_SELFTEST 0   /* [V580] keep the bounded 16 s sweep so we
                                  * still get a hardware proof on every boot,
                                  * but it now EXITS and hands /dev/fb0 to
                                  * info_screen instead of fighting it for
                                  * ever.  ★ Tested with #if below -- never
                                  *   #ifdef (`#define X 0` still counts as
                                  *   defined, which once kept the thread
                                  *   alive after I thought I had removed it). */

/* [V625] Backlight bisection gate.
 *
 * A controlled A/B on real hardware (6 boots each, every run verified by its
 * own ESP-ROM banner) finally separated the screen subsystem from the UI:
 *
 *   v619  screen subsystem OFF              -> 6/6 associations succeeded
 *   v624  panel + fb + SPI3 up, LVGL UI not  -> 0/6 associations succeeded
 *         started (the launcher waited for an address that never came)
 *
 * So the LVGL UI is NOT the breaker - the panel bring-up is.  V625 keeps
 * everything of v624 (SPI3 routed, ST7789 initialised, /dev/fb0 registered)
 * and removes only the two things that touch GPIO16: the [V548] toggle test
 * and the "leave it HIGH" drive, plus the 2 s blkeeper thread.
 *
 * GPIO16 is XTAL_32K_N on the ESP32-S3, and esp32s3_wifi_adapter.c reads
 * RTC_SLOW_CLK_CAL_REG (the slow-clock calibration) when it needs a clock
 * estimate, so a loaded or clamped 32 kHz pad is a plausible mechanism.
 *
 *   guard on  -> backlight never driven, panel stays dark (expect 6/6)
 *   guard off -> v624 behaviour (expect 0/6)
 *
 * Measured, 6 real boots each:
 *   v619  no screen subsystem at all         -> 6/6, no disconnects
 *   v624  panel + fb + SPI3, GPIO16 driven   -> 0/6, reason=2 every time
 *   v625  panel + fb + SPI3, GPIO16 untouched-> 3/6, failures are reason=39
 *
 * Driving GPIO16 is what turns the failure into reason=2 (AUTH_EXPIRE).
 *
 * [V626] THE FIX ATTEMPT: give the pad back to the digital IO_MUX and stop
 * the 32 kHz oscillator BEFORE driving it.  ai_vox3_bl_force_gpio() was
 * written for exactly this and then left uncalled when the backlight moved to
 * plain GPIO - so every build since has driven a pad that the RTC domain
 * still owned.  Re-enable the drive and run the handoff first.
 */

#define V625_NO_BACKLIGHT_DRIVE 0
#define V626_XTAL32K_HANDOFF    0   /* [V627] 0/6 in v626 -> backed off  */
#define V627_DEFER_BACKLIGHT    0   /* [V627] did not help -> see [V628]   */
#define V628_LOG_SLOWCLK        1   /* print the RTC slow-clock source     */
#define V628_FORCE_RC_SLOW      1   /* park the slow clock on RC_SLOW      */
#define V627_WAIT_S             75

/* [V765] GRAM white-shot geometry and one-shot latch.  Declared before
 * v742_panel_waker() so the latch is file-scope rather than block-scope.
 * The shot runs at most once per boot: its purpose is to answer a question
 * about the wake sequence, and repeating it every 2 s would fight the UI
 * for the panel for as long as the watchdog lives. */
#define V765_W                  240
static bool s_v765_whiteshot;  /* max s to wait for /tmp/net.txt      */

/* [V548] Backlight pin mode that allows a TRUE level read-back.
 *
 * esp32s3_configgpio(OUTPUT) alone never sets FUN_IE, so the input buffer is
 * off and esp32s3_gpioread() is hard-wired to 0 for that pad -- every single
 * "BCKL=0" line in the v530..v547 logs was an artifact of that, NOT evidence
 * that the pin was low.  Passing INPUT|OUTPUT sets FUN_IE and keeps the
 * output driver enabled, so gpioread() finally reflects the real pad level.
 */
#define AI_VOX3_BCKL_MODE  (INPUT | OUTPUT)

/* [V818] Backlight as a plain GPIO, in the order v523 proved works.
 *
 * v816 wrote LEDC registers by hand; v817 called the driver's LEDC function.
 * Both left the screen black, and both were reinventing a wheel that the notes
 * already settle.  memory/2026-10-11.md, v517-v523 section, is explicit:
 *
 *   "*** The real cause of the black screen was settled by the v523
 *    measurement: a plain GPIO high is enough, LEDC is wrong."
 *   "v522 polarity demo (five states on boot, confirmed by eye):
 *    backlight lights on a high level (active-high)."
 *   "v481's premise -- that a plain GPIO cannot drive the backlight -- was
 *    refuted by measurement.  v523 went back to plain GPIO high."
 *
 * It also records why the LEDC path was abandoned even though it looked right:
 *
 *   "LEDC failing (all registers correct, no output): CONF0=0x7 (bit2=SIG_OUT_EN),
 *    CONF1=0x80000000 (DUTY_START), DUTY=0x6000 (1536/2048=75%),
 *    func_out_sel=77=LEDC_LS_SIG_OUT4_IDX -- routing, enable, duty and
 *    frequency all correct, and the waveform simply never reached the panel.
 *    Stop going round in circles on LEDC."
 *
 * That is precisely what v816 and v817 produced: correct registers, no light.
 *
 * Two notes from the same entry, both of which v816/v817 violated:
 *
 *   "Order rule: first esp32s3_gpio_matrix_out(pin,0,0,0) to unbind, then
 *    configgpio, then write the level.  In the wrong order the GPIO write does
 *    nothing -- that is the old v9~v17 'black no matter what I write' bug."
 *
 *   A later [V528] comment in this file says the opposite, that calling
 *   gpio_matrix_out(pin,0,0,0) must never be done because signal index 0 is not
 *   a detach.  The two contradict each other.  v523 is the one with an eye-witnessed
 *   result and a recorded symptom for the failure mode, so it is followed here.
 *   If this still comes out dark, that contradiction is the next thing to settle
 *   -- try v528's order (configgpio first, no matrix call) as the single
 *   variable, and nothing else.
 *
 * Active-high, per v522: high = lit.  This also keeps gpioread meaningful,
 * because configgpio() is passed INPUT|OUTPUT so FUN_IE gets set -- without it
 * the input buffer stays off and gpioread returns 0 whatever the pad is doing.
 */
static void ai_vox3_bl_force_gpio(void);

/* [V847] Register forensics for the backlight pad.
 *
 * Offsets, taken from NuttX's own headers rather than guessed:
 *   0x60004004  GPIO_OUT_REG                bit 16
 *   0x60004020  GPIO_ENABLE_REG             bit 16
 *   0x60004594  GPIO_FUNC16_OUT_SEL_CFG     (0x554 + 16 * 4)
 *   0x60009044  IO_MUX GPIO16               (REG_IO_MUX_BASE + (16 + 1) * 4)
 *   0x600084C4  RTC_IO_XTAL_32N_PAD_REG     X32N_MUX_SEL is bit 19
 *   0x60008074  RTC_CNTL_CLK_CONF_REG       DIG_XTAL32K_EN is bit 8
 *   0x600080D8  RTC_CNTL_PAD_HOLD_REG       GPIO16 hold is bit 16
 *
 * The two RTC/clock bits are the point of the whole exercise: while
 * X32N_MUX_SEL is set, the pad belongs to the 32 kHz crystal and none of the
 * GPIO registers below mean anything. */
static void v847_dump16(const char *tag)
{
  volatile uint32_t *out    = (volatile uint32_t *)0x60004004u;
  volatile uint32_t *enable = (volatile uint32_t *)0x60004020u;
  volatile uint32_t *outsel = (volatile uint32_t *)0x60004594u;
  volatile uint32_t *iomux  = (volatile uint32_t *)0x60009044u;
  volatile uint32_t *x32n   = (volatile uint32_t *)0x600084C4u;
  volatile uint32_t *clkcf  = (volatile uint32_t *)0x60008074u;
  volatile uint32_t *hold   = (volatile uint32_t *)0x600080D8u;
  volatile uint32_t *pinreg = (volatile uint32_t *)0x600040B4u;
  volatile uint32_t *inreg  = (volatile uint32_t *)0x6000403Cu;

  syslog(LOG_ERR,
         "[V847] bl16 %s: en=%u out=%u outsel=%u iomux=0x%08x "
         "x32n_mux_sel=%u dig_xtal32k_en=%u pad_hold16=%u "
         "pin16reg=0x%08x (open_drain=%u) pad_reads=%u\n",
         tag,
         (unsigned)((*enable >> 16) & 1u),
         (unsigned)((*out >> 16) & 1u),
         (unsigned)*outsel,
         (unsigned)*iomux,
         (unsigned)((*x32n >> 19) & 1u),
         (unsigned)((*clkcf >> 8) & 1u),
         (unsigned)((*hold >> 16) & 1u),
         (unsigned)*pinreg,
         (unsigned)((*pinreg >> 2) & 1u),
         (unsigned)((*inreg >> 16) & 1u));
}

static void v818_bl_on(void)
{
  const int bl = ESP32S3_EYE_DISPLAY_BCKL;

  /* [V847] *** Take the pad back from the RTC domain FIRST. ***
   *
   * GPIO16 is XTAL_32K_N.  Until X32N_MUX_SEL and the pad-hold
   * latches are cleared, the pad is owned by the 32 kHz crystal and
   * the matrix_out / configgpio / gpiowrite calls below are all
   * no-ops -- while gpioread() keeps reporting a plausible 1 from
   * the floating input buffer.  That is the entire gap between
   * "BL held HIGH pad=1" in the log and a screen with no light.
   *
   * The dumps are one line each: they are the evidence either way. */

  v847_dump16("before");
  ai_vox3_bl_force_gpio();

  /* 1) unbind from whatever signal was driving this pad */
  esp32s3_gpio_matrix_out(bl, SIG_GPIO_OUT_IDX, 0, 0);

  /* 2) plain GPIO, input buffer on so the keeper can read the pad back */
  esp32s3_configgpio(bl, AI_VOX3_BCKL_MODE);

  /* 3) [V836] POLARITY A/B.
   *
   * The "HIGH = lit" rule this file inherited was measured while the pad was
   * written to signal index 0, i.e. disconnected from GPIO_OUT -- so every
   * one of those measurements was taken on a floating pad and proves nothing.
   *
   * v834 was the first build to actually drive the pad, and driving it HIGH
   * turned a working panel black (v831: 3/3 cold boots lit; v835, which
   * inherits v834: 0/3).  That fits an active-low enable with an internal
   * pull-up: floating = on, LOW = on, HIGH = off.
   *
   * So hold LOW for the first 30 s, then switch to HIGH and stay there.
   * Whichever half lights the panel is the real polarity. */

  /* [V837] BACKLIGHT POLARITY SETTLED -- HIGH = lit, held forever.
   *
   * Two independent sources agree on the level:
   *
   *   1. the vendor firmware for this exact board,
   *      examples/ai-vox3/ai-vox3/main.cpp:124
   *          analogWrite(kLcdBacklightPin, 255);  -- GPIO16, 100% duty
   *      analogWrite() maps to LEDC at full duty, i.e. the pad is driven
   *      HIGH.  Nothing in that source ever takes it LOW.
   *
   *   2. v831, the only build confirmed lit on 3/3 cold boots, held this
   *      pad HIGH from this very thread.  v836 turned it LOW for the first
   *      30 s as a polarity A/B and went 0/3 on cold boot.
   *
   * The A/B is over.  There is no branch here on purpose: every write of 0
   * to this pad is "backlight off", and any thread that writes 0 undoes the
   * one that writes 1. */
  /* [V838] FOUR-PHASE BACKLIGHT SWEEP.
   *
   * Every "polarity" conclusion so far was drawn from a build that also
   * changed something else, so none of them can be trusted.  This runs all
   * four candidate states in one boot, 15 s each, forever:
   *
   *     P0 HiZ      -- output disabled, pad floats
   *     P1 DC HIGH  -- push-pull high
   *     P2 DC LOW   -- push-pull low
   *     P3 PWM      -- 1 kHz, 50 % duty (closest thing to the vendor's
   *                    analogWrite() that a plain GPIO can produce)
   *
   * The phase name is logged at every 3 s beat.  Whichever phase lights the
   * screen is the configuration to ship; the others get deleted. */
  /* [V840] Backlight held HIGH, for good.
   *
   * V838 swept this pad through HiZ / DC HIGH / DC LOW / 1 kHz PWM, 15 s
   * each, and the screen stayed black in all four -- so the level here is
   * not what is blocking the picture and there is nothing left to A/B.
   *
   * HIGH is what the vendor firmware for this board does:
   *   examples/ai-vox3/ai-vox3/main.cpp:124  analogWrite(GPIO16, 255)
   * i.e. full duty, never taken low.  Hold it there. */
  {
    static unsigned s_v840_calls;

    s_v840_calls++;

    esp32s3_gpio_matrix_out(bl, SIG_GPIO_OUT_IDX, 0, 0);
    esp32s3_configgpio(bl, AI_VOX3_BCKL_MODE);
    esp32s3_gpiowrite(bl, 1);

    if (s_v840_calls <= 3)
      {
        syslog(LOG_ERR, "[V840] BL held HIGH (level=1) call=%u pad=%d\n",
               s_v840_calls, esp32s3_gpioread(bl));
      }
  }

  v847_dump16("after");

  /* [V848] *** Make the pad move so a meter can settle this. ***
   *
   * The pad measures 3.7 V, which no ESP32-S3 GPIO can produce -- that is
   * the battery rail.  Either the probe is on the wrong node, or the pad is
   * genuinely not being driven despite the registers saying otherwise.
   *
   * Drive it hard HIGH / hard LOW at 1 Hz, six times, on the first call
   * only.  A meter on the real GPIO16 will swing 0 <-> 3.3 V.  A node that
   * sits at 3.7 V through all six transitions is not this pad. */

  {
    static int s_v848_done;

    if (0 && !s_v848_done)   /* [V886] was !s_v848_done -- see note */
      {
        int i;

        s_v848_done = 1;

        for (i = 0; i < 6; i++)
          {
            esp32s3_gpiowrite(bl, 1);
            syslog(LOG_ERR, "[V848] bl16 TOGGLE %d HIGH\n", i);
            up_mdelay(1000);

            esp32s3_gpiowrite(bl, 0);
            syslog(LOG_ERR, "[V848] bl16 TOGGLE %d LOW\n", i);
            up_mdelay(1000);
          }

        esp32s3_gpiowrite(bl, 1);
        syslog(LOG_ERR, "[V848] bl16 toggle done, back to HIGH\n");
      }
  }

  syslog(LOG_ERR,
         "[V818] backlight on as plain GPIO pin %d (active-high, v523 order: "
         "matrix_out(0) -> configgpio(IN|OUT) -> gpiowrite(1)) pad=%d\n",
         bl, esp32s3_gpioread(bl));
}


#if AI_VOX3_LCD_SELFTEST
static void lcd_selftest(void)
{
  struct lcd_planeinfo_s pinfo;
  const int W = 240;
  const int H = 240;
  uint16_t *fb = (uint16_t *)kmm_malloc((size_t)W * H * sizeof(uint16_t));
  const uint16_t colors[4] = { 0xf800, 0x07e0, 0x001f, 0xffff };
  const char *names[4] = { "RED", "GREEN", "BLUE", "WHITE" };
  int phase = 0;
  int i;
  int round;

  if (g_lcd->getplaneinfo(g_lcd, 0, &pinfo) != OK)
    {
      LCDDIAG("selftest: getplaneinfo FAILED");
      return;
    }

  if (fb == NULL)
    {
      LCDDIAG("selftest: kmm_malloc FAILED");
      return;
    }

  /* [V548] Plain-GPIO backlight, ACTIVE-HIGH, held on; panel cycles R/G/B/W
   * exactly like the verified hardware test
   *   C:\Users\xxzx\Desktop\VOX3\backlight_test\backlight_test.ino
   * which does pinMode(BL, OUTPUT); digitalWrite(BL, HIGH); then
   * SWRESET/SLPOUT/COLMOD(0x55)/MADCTL(0x00)/INVON/NORON/DISPON and finally
   * a 4-colour full-screen loop at 2 s per colour.
   *
   * The pin mode here additionally sets INPUT so gpioread() is meaningful. */
  esp32s3_configgpio(ESP32S3_EYE_DISPLAY_BCKL, AI_VOX3_BCKL_MODE);
  esp32s3_gpiowrite(ESP32S3_EYE_DISPLAY_BCKL, 1);   /* ACTIVE-HIGH = on */

  

  for (round = 0; round < 2; round++)
    {
      for (phase = 0; phase < 4; phase++)
        {
          for (i = 0; i < W * H; i++)
            {
              fb[i] = colors[phase];
            }

          pinfo.putarea(g_lcd, 0, H - 1, 0, W - 1,
                        (FAR const uint8_t *)fb,
                        (fb_coord_t)(W * sizeof(uint16_t)));

          /* Re-assert, then read back for real. */
          esp32s3_configgpio(ESP32S3_EYE_DISPLAY_BCKL, AI_VOX3_BCKL_MODE);
          esp32s3_gpiowrite(ESP32S3_EYE_DISPLAY_BCKL, 1);

          

          sleep(2);
        }
    }

  /* [V553] ★ Bounded self-test: 2 rounds x 4 colours x 2 s = 16 s, then EXIT.
   * The old for(;;) loop never returned and fought info_screen (pid=11) for
   * /dev/fb0 forever -- info_screen cleared the panel to black and drew its
   * text once a second while the selftest repainted a solid colour every 3 s,
   * which is exactly the "亮了一下又灭了" flicker the user kept reporting.
   * Now the panel shows a clean R/G/B/W sweep for the first 16 s and then
   * info_screen owns /dev/fb0 exclusively. */

  free(fb);
  
}
#endif

/****************************************************************************
 * Name:  lcd_fb_probe
 *
 * Description:
 *   [V554] Ground-truth probe for the "self-test is perfect but info_screen
 *   shows nothing" question.  The panel proves the SPI3 + software-CS path
 *   works whenever *we* call pinfo.putarea(); what is unproven is whether the
 *   framebuffer device (/dev/fb0, i.e. drivers/lcd/lcd_framebuffer.c) ever
 *   receives the pixels info_screen writes into its mmap.
 *
 *   The self-test writes straight to the LCD, so it can be perfectly correct
 *   while /dev/fb0 is a black hole.  This thread opens /dev/fb0 exactly like
 *   info_screen does, mmaps it, and every 5 s counts how many pixels differ
 *   from the background colour.  Three outcomes are distinguishable:
 *     - fb=<n> px!=bg  with n>0  -> info_screen IS painting; the LCD-side push
 *                                   is what is broken (FBIO_UPDATE path).
 *     - fb=<n> px!=bg  with n==0 -> info_screen never wrote anything (its
 *                                   fb_init failed, or it is stuck).
 *     - open/mmap/ioctl error    -> the framebuffer node itself is the
 *                                   problem (wrong path, not registered).
 *   Runs for 50 s and then stops so it does not spam the log forever.
 *
 ****************************************************************************/

static void lcd_fb_probe(void) __attribute__((unused));
static void lcd_fb_probe(void)
{
  struct fb_videoinfo_s vinfo;
  struct fb_planeinfo_s pinfo;
  uint16_t *mem;
  int fd;
  int i;

  /* [V554] ★ CRITICAL: this thread is started from board_lcd_initialize(),
   * but /dev/fb0 is only created later, by fb_register() ->
   * fb_register_device() -> register_driver("/dev/fb%d").  Opening it too
   * early fails with ENOENT (errno=2) -- which is exactly what the first
   * run of this probe reported.  Wait until well after bringup has
   * registered the node AND after info_screen's own 20 s hold-off, so the
   * first sample reflects what the panel task has actually painted.
   *
   * [V557] Report through syslog() only.  syslog reaches the serial console
   * during boot (proven by every [LCDDIAG] line), whereas the FILE* mirror
   * tried in v556 was never observed and only added risk. */
  sleep(45);

  fd = open("/dev/fb0", O_RDWR);
  if (fd < 0)
    {
      
      return;
    }

  if (ioctl(fd, FBIOGET_VIDEOINFO, (unsigned long)&vinfo) < 0)
    {
      
      close(fd);
      return;
    }

  if (ioctl(fd, FBIOGET_PLANEINFO, (unsigned long)&pinfo) < 0)
    {
      
      close(fd);
      return;
    }

  mem = (uint16_t *)mmap(NULL, pinfo.fblen, PROT_READ | PROT_WRITE,
                         MAP_SHARED | MAP_FILE, fd, 0);
  if (mem == MAP_FAILED)
    {
      
      close(fd);
      return;
    }

  

  for (i = 0; i < 10; i++)   /* 10 x 5 s = 50 s */
    {
      int n = 0;
      unsigned int total;
      unsigned int k;

      sleep(5);

      /* Count pixels that are neither the dark-navy background (0xFFE7) nor
       * pure black -- a non-zero count means somebody drew something. */
      total = (pinfo.stride / 2) * vinfo.yres;
      for (k = 0; k < total; k++)
        {
          uint16_t p = mem[k];
          if (p != 0xFFE7 && p != 0x0000)
            {
              n++;
            }
        }

      
    }

  munmap(mem, pinfo.fblen);
  close(fd);
  
}


/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use, but
 *   with the power setting at 0 (full off).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

/* [V460] 原版的 board_lcd_bckl_keeper 已被删除。
 *
 * 事实：.bak_bl（原始板级文件）里没有 keeper、没有 PWM 脉冲、
 * 没有低->高跳变，GPIO16 一直保持低电平，屏幕是亮的。
 * 所以"保活脉冲"整套东西是多余的，而且是这几轮黑屏的祸根
 * （把它改高 = 关背光）。
 * 真正点亮面板靠 st7789_lcdinitialize() 里的 DISPON(0x29)。
 */

/* [V440] 背光脚是否已经配置过。只用于避免重复 configgpio。 */
static bool s_cfg __attribute__((unused));

void board_lcd_backlight_on(void)
{
  /* [V525] 本函数**故意什么都不做**。
   *
   * 它被 vox3_net_keepalive() 周期性调用。如果在这里 configgpio(GPIO16)
   * 就会和 keeper 线程抢同一个 pad —— v523/v524 的屏幕闪烁正是这种
   * 「多处写同一个引脚、谁最后写谁赢」的竞态造成的。
   *
   * 背光现在由 board_lcd_bl_breath() 独占管理：单点、每2 秒无条件重钉
   * HIGH，不做任何读回判断（gpioread 对 output-only 引脚恒返回 0）。
   */
}

static void board_lcd_bl_breath(void)
{
  /* [V524] 背光 = GPIO16 普通高电平常亮 + 每秒重新钉一次。
   *
   * [V530 更正] 之前写「拉高→亮、拉低→灭（active-high）」是错的。真相：
   * st7789_lcdinitialize() 已经 SWRESET+SLPOUT+DISPON+fill(0xffff)（白屏），
   * 但每版都黑 → 背光其实是 **低电平点亮（active-low）**，写 HIGH=灭。
   * 所以这里改成每 2 秒无条件重钉 **LOW**。
   *
   * ★ 为什么不继续用 LEDC：v481~v522 一直认定必须用 LEDC 硬件 PWM，理由是
   *   "背光 LED 是大电流负载，普通 GPIO 带不动"。v522 把引脚按 plain GPIO
   *   HIGH 驱动时面板正常点亮，实测推翻了该前提——普通 GPIO 完全带得动。
   *
   * ★ LEDC 侧为什么会失败：寄存器读回全部正确，逐一核对无误——
   *     CONF0=0x7  → bit2 = LEDC_SIG_OUT_EN_CH0，输出已使能
   *     CONF1=0x80000000 → LEDC_DUTY_START_CH0
   *     DUTY=0x6000 → 0x600<<4 = 1536/2048 = 75%
   *     func_out_sel=77 = LEDC_LS_SIG_OUT4_IDX → ch4 → GPIO16
   *   路由、使能、占空比、频率全部对，但波形就是没落到面板上。既然 GPIO
   *   能点亮，就用它；不再为一个解释不通的现象继续在 LEDC 上兜圈。
   *
   * ★★★ v525：v524 的「读回判断」是个bug，keeper 一直误判被抢。
   *
   * v524 里keeper 写的是 if (esp32s3_gpioread(bl) != 1) 才重新钉，
   * 结果日志里每秒都打「backlight re-asserted (pin was not HIGH)」。
   * 当时误判成「有别的代码在抢 pad」。真实原因在 esp32s3_configgpio() 里：
   *
   *     if ((attr & INPUT)  != 0) { ...; func |= FUN_IE; }    // 只在 INPUT 分支
   *     if ((attr & OUTPUT) != 0) { ... GPIO_ENABLE_W1TS ... } // 只开出使能
   *
   * 我们只传了 OUTPUT，FUN_IE（输入缓冲使能）从来没被置上。而
   * esp32s3_gpioread() 读的正是 GPIO_IN_REG —— **输入通路**。
   * 输入缓冲关着，gpioread 对这个引脚永远返回 0，跟 pad 实际电平无关。
   *
   * 所以：既不存在抢占者（grep 全树确认 esp32s3_ledc_raw_backlight()
   * **只有定义、没有任何调用者**，LEDC 根本没被启动过），也读不出真实
   * 电平。唯一的闪烁源就是 v523 那次性动作退出后没人再管这个pad。
   *
   * 修法：不再读回判断，**无条件重钉**。每 2 秒两步写一次
   * （configgpio(OUTPUT) + gpiowrite(1)），开销可忽略，且不管未来谁碰
   * 这个 pad 都能恢复。
   *
   * ★★★ v528 关键修正：绝对**不要**调 esp32s3_gpio_matrix_out(bl, 0, 0, 0)。
   *   信号索引 0 在 ESP32-S3 上不是"解绑"，而是把 pad 输出源接到信号 0，
   *   等于把 GPIO_OUT_REG 和引脚断开，gpiowrite 失效。原始 v460 板级代码
   *   只做 configgpio+gpiowrite 就点亮了，从没碰过矩阵输出选择器。
   *   之前那句"先解绑再写电平"的"铁律"是建立在错误前提（LEDC 抢 pin）
   * 上的，纯属多此一举还帮倒忙，已删除。
   */
  const int bl = ESP32S3_EYE_DISPLAY_BCKL;
  int beat = 0;


  /* [V767] 背光 A/B 相位。0 = LOW 相, 1 = HIGH 相。
   * 非 volatile 也可以（单线程持有），但用 bool 让语义直白。 */
  bool s_v767_phase = true;

  

  for (;;)
    {
      /* [V550] Plain GPIO, ACTIVE-HIGH, re-asserted with INPUT|OUTPUT so the
       * read-back is real (see AI_VOX3_BCKL_MODE).  This thread touches ONLY
       * the backlight pad -- never the panel data -- so /dev/fb0 is the sole
       * owner of the SPI3 bus. */
      /* [V767] ★★★ 背光极性 A/B 判定 —— 本轮唯一变量。
       *
       * 面板侧三条对照假设已被 v765/v766 全部证伪（NORON / fill 顺序 /
       * COLMOD 缺失），而"背光是亮的"这个前提**从未被实测过**：
       * 本文件注释里极性出现过三种说法（v524 active-high / v530
       * active-low / v550 又回到 active-high），能一次定论的 v530
       * 自检闪烁线程被 AI_VOX3_LCD_SELFTEST=0 关掉后再没跑过。
       *
       * 若是极性猜错，面板其实一直正常工作，只是没人给它光 —— 那屏
       * 就永远是黑的，与 GRAM / DISPON / 顺序全都无关。
       *
       * 因此这里让 keeper 以 3 s 为周期在 LOW / HIGH 之间交替，并在
       * 日志里打出当前相位：哪一相亮，极性就是哪一相。面板数据流一行
       * 不动，唯一变量就是这一个引脚的电平。
       *
       * 判读表：
       *   「LOW 相亮、HIGH 相灭」  => ACTIVE-LOW，把 1 改成 0 收工
       *   「HIGH 相亮、LOW 相灭」  => ACTIVE-HIGH（现状即正确）
       *   两相都不亮              => 背光与面板同时另有问题（换口径）
       *   两相都亮 / 相位看不出来  => 背光不是当前阻塞点，回到面板输出
       */
      /* [V768] A/B 对照：背光固定 HIGH。v767 已证明相位能在 pad 上翻转，
       * 但两种电平下屏都是黑的 => 背光极性不是阻塞点，本轮拉平它，
       * 让唯一的变量变成「面板是否还在被周期性重初始化」。 */
      /* [V816] LEDC hardware PWM -- a plain GPIO cannot supply the LED
       * current, which is the whole reason this screen never lit. */
      v818_bl_on();

      /* [V767] 相位标记。用 syslog 而不是 LCDDIAG：keeper 每 3 s 一拍，
       * 日志里成对出现 [V767] bl phase=HIGH / LOW，肉眼即可对齐时间轴。
       * 两相各自都打寄存器快照，免得只看电平不够用。 */
      syslog(LOG_INFO, "[V768] bl held HIGH (v767 A/B proved polarity is"
             " not the blocker)\n");

      /* [V678] Read the pad back for real (the mode carries INPUT since
       * V550, so gpioread is meaningful).  The panel went dark after hours
       * with no display-off in the log -- prove the keeper is alive and the
       * pad is still HIGH. */
      if (0)   /* [V838] disabled for the sweep: it forces the pad
                  HIGH whenever it reads 0, which would cancel
                  the HiZ and DC-LOW phases. */
        {
          /* [V802] The keeper used to only complain here, which made a LOW
           * pad permanent: it re-pinned and wrote 1 every beat, but nothing
           * ever changed, and `beat` (the heartbeat counter) only advances in
           * the HIGH branch, so the thread silently stopped reporting.
           *
           * A normal/black capture pair taken from the same build showed
           * exactly that: the healthy boot printed "bl pin HIGH, keeper alive"
           * twelve times, the black one never printed it once, while every
           * other indicator was identical.  The panel was rendering into GRAM
           * the whole time -- it simply had no light.
           *
           * So: actually repair.  Re-assert the pad, then read the OUTPUT
           * enable register back.  If the enable bit was cleared, that names
           * the mechanism (see esp32s3_configgpio(), which clears the bit with
           * W1TC when INPUT is set and sets it with W1TS when OUTPUT is set --
           * an INPUT-only re-pin by another thread leaves it cleared), and the
           * repair puts it back.  If the enable bit is still set and the pad
           * still reads LOW, something is holding the line electrically and
           * that is worth saying plainly rather than retrying forever.
           */
          /* [V803b] Read GPIO_ENABLE_REG directly.  This file already pokes
           * the register block through a plain pointer (see the 0x60004000
           * base used further down), and DR_REG_GPIO_BASE is not visible here.
           * The enable register is base + 0x20. */
          volatile uint32_t *v802_gpio = (volatile uint32_t *)0x60004000u;
          uint32_t en = v802_gpio[0x20 / 4];

          esp32s3_configgpio(bl, AI_VOX3_BCKL_MODE);
          esp32s3_gpiowrite(bl, 1);   /* [V837] HIGH = lit */
          up_udelay(200);

          if (esp32s3_gpioread(bl) == 0)
            {
              /* Still low.  Say which mechanism, once, not every beat. */
              static int reported;

              if (reported < 4)
                {
                  reported++;
                  syslog(LOG_ERR,
                         "[V802] bl still LOW after repair; "
                         "GPIO_ENABLE_REG=0x%08x out_en=%u "
                         "(out_en=0 => another thread re-pinned INPUT-only; "
                         "out_en=1 => line held low externally)\n",
                         (unsigned)en,
                         (unsigned)((en >> bl) & 1));
                  ai_vox3_bl_dump16("v802low");
                }
            }
          else
            {
              /* Repaired.  Do not wait for the 15-beat heartbeat to say so:
                 the recovery is the thing worth reporting. */
              static int recovered;

              if (recovered < 8)
                {
                  recovered++;
                  syslog(LOG_ERR,
                         "[V802] bl RECOVERED to HIGH after re-assert "
                         "(was LOW; GPIO_ENABLE_REG was 0x%08x)\n",
                         (unsigned)en);
                }

              beat++;
            }
        }
      else if (++beat >= 15)
        {
          beat = 0;
          syslog(LOG_INFO, "[V679] bl pin HIGH, keeper alive\n");
          /* [V679] Register snapshot on every heartbeat.  Anything that
           * reprograms GPIO16 through direct register writes (bypassing
           * esp32s3_configgpio, e.g. RTC / WiFi-domain code) is invisible
           * to the gpio.c audit -- this dump is not. */
          ai_vox3_bl_dump16("beat16");
        }

      /* Heartbeat so we can tell from the log that the keeper is alive and
       * that the pad still reads back HIGH. */
      

      /* [V767] keeper 相位周期 3 s（原来 2 s）—— 让闪烁肉眼可辨。 */
      sleep(3);
    }
}

/* [V627] Is /tmp/net.txt carrying a real address yet?
 *
 * netinit_publish_net_state() is the sole writer; line 1 is either the IPv4
 * address or the literal "NO NET", so a '.' in line 1 is a reliable
 * "DHCP finished" signal - the same contract the V624 UI launcher uses.
 */

static bool v627_net_has_ip(void)
{
  char buf[48];
  bool ok = false;
  FILE *f = fopen("/tmp/net.txt", "r");

  if (f != NULL)
    {
      if (fgets(buf, sizeof(buf), f) != NULL)
        {
          ok = (strchr(buf, '.') != NULL);
        }

      fclose(f);
    }

  return ok;
}

/* [V627] Deferred backlight bring-up.  THE EXPERIMENT.
 *
 * v619 (no screen subsystem) associated 6/6; v624/v626 (panel up AND GPIO16
 * driven during bring-up) associated 0/6 with reason=2; v625 (panel up,
 * GPIO16 never driven) managed 3/6.  The pad being driven is what kills the
 * first association - and on this board the first association is the only
 * one that counts.
 *
 * This thread leaves GPIO16 in its reset state until the link is already up,
 * then drives it HIGH and falls through into the 2 s keeper.  If association
 * returns to 6/6 the breaker is the TIMING of the drive (the pad is clamped
 * while auth / the 4-way handshake runs) rather than the pad state itself.
 */

static void v627_bckl_waiter(void)
{
  const int bl = ESP32S3_EYE_DISPLAY_BCKL;
  int waited = 0;

  

  while (waited < V627_WAIT_S && !v627_net_has_ip())
    {
      sleep(1);
      waited++;
    }

  

  esp32s3_configgpio(bl, AI_VOX3_BCKL_MODE);
  esp32s3_gpiowrite(bl, 1);   /* [V837] HIGH = lit */
  
  ai_vox3_bl_dump16("v627post");

  board_lcd_bl_breath();       /* never returns; keeps the pad pinned HIGH */
}



/* [V628] The RTC slow clock: who is feeding it?
 *
 *   RTC_CNTL_RTC_CLK_CONF_REG = DR_REG_RTCCNTL_BASE + 0x74 = 0x60008074
 *     bits[31:30] ANA_CLK_RTC_SEL : 0 = RC_SLOW (internal ~136 kHz),
 *                                   1 = XTAL32K (external 32.768 kHz),
 *                                   2 = 8 MHz / 256
 *     bit[8]      DIG_XTAL32K_EN  : digital 32 kHz oscillator enable
 *
 * The xtensa ESP32-S3 port in openvela NEVER programs this field (only the
 * risc-v ESP32-C3 legacy port has rtc_clk_slow_freq_set()), so whatever the
 * ROM bootloader selected stays in force for the whole session.
 *
 * Why it matters here: the panel shares BOTH 32 kHz crystal pads --
 *   XTAL_32K_P = GPIO15  (SPI3 chip-select)
 *   XTAL_32K_N = GPIO16  (backlight)
 * If the ROM parked the slow clock on that crystal, then the SPI3 CS edges
 * on GPIO15 and the parked-HIGH backlight on GPIO16 both corrupt the WiFi
 * blob's slow-clock estimate: r39 while only CS toggles, r2 once the
 * backlight pad is held HIGH.
 *
 * V628 reads the field, then hands the slow clock to RC_SLOW *before* any
 * board code touches GPIO15 or GPIO16.
 */

static unsigned v628_rtc_clk_conf(void)
{
  volatile uint32_t *clkcf = (volatile uint32_t *)0x60008074u;
  return (unsigned)*clkcf;
}

static unsigned v628_slow_sel(void)
{
  return (v628_rtc_clk_conf() >> 30) & 0x3u;
}

/* [V742] *** Panel wake watchdog ***
 *
 * Symptom: after a real power-on reset (unplug / replug USB) the panel stays
 * black while the firmware is completely healthy - backlight keeper alive
 * ("bl pin HIGH, keeper alive"), the framebuffer refresh loop alive
 * ("fb beat=N row120=2336 row210=2336", i.e. non-black pixels), IP obtained,
 * panic 0.  The same firmware is fine after an esptool hard reset, so the
 * difference is the power-rail sequence: on a real POR our one-shot
 * SWRESET/SLPOUT/DISPON sequence can reach a panel that is not powered yet,
 * and a ST7789 that missed it sits in sleep-in / display-off and ignores
 * every subsequent GRAM write forever.
 *
 * The user also reports that with the xiaozhi firmware the panel comes back
 * when the power key is pressed.  The key is not a GPIO (a full unplug +
 * key-press scan showed no edge on GPIO0, and GPIO45/46 only move together,
 * i.e. hand noise), so it is a hardware rail latch: the press powers the
 * panel, which then needs a fresh SLPOUT + DISPON to start showing anything.
 *
 * So: re-assert "display on" periodically.  On an already-lit panel this is
 * a no-op (no reset, no blank, no flicker).  On a panel that just got power
 * it is exactly the missing wake-up.  Follow it with a full-screen
 * FBIO_UPDATE so the panel receives a complete frame instead of just the
 * dirty region (otherwise the parts of GRAM the UI has not touched since
 * power-up stay as power-on garbage). */

/* [V769] 传输层取证转储，实现在 esp32s3_spi.c */
extern void esp32s3_v769_spi_dump(uint32_t id);

/* [V770] GPIO matrix 路由取证 —— 「三根线到底有没有接到 SPI3」的直接证据。
 *
 * esp32s3_gpio_matrix_out() 的实现就三行：
 *     regaddr = GPIO_FUNC0_OUT_SEL_CFG_REG + (pin * 4);
 *     putreg32(signal_idx << 0, regaddr);
 * 也就是每个 pad 有一个 9 bit 的 OUT_SEL 字段（FUNCn_OUT_SEL_CFG）。
 * 若 cs/clk/mosi 三个 pad 的 OUT_SEL 不是 SPI3 的信号索引，
 * 说明路由被后来某段代码改掉了（configgpio / gpiowrite 之类都不会动它，
 * 但 RTC pad hold、WiFi 复用、以及任何直接写 IO_MUX 的代码会）。
 *
 * 期望值（本板 .config）：CS=15 CLK=17 MOSI=21。SPI3 的 *_OUT_IDX 由
 * esp32s3_spi_config_s 给出，数值在运行时从寄存器读回，不在这里写死。
 */


/* ---------------------------------------------------------------------------
 * [V771] RAW bit-bang ST7789 bring-up.
 *
 * This is the first test in this project that touches the panel *electrically*
 * instead of probing software state.  It drives CS/DC/CLK/MOSI with nothing but
 * esp32s3_gpiowrite(); the SPI peripheral, the GPIO matrix routing, NuttX'
 * spibus, and the whole st7789 driver are bypassed.
 *
 * Verdict table:
 *   panel lights  -> NuttX SPI path is the bug  (keep digging in software)
 *   panel black   -> panel/ribbon/backlight/RST=NC is the bug (stop guessing)
 * ---------------------------------------------------------------------------
 */

#define V771_PIN_CS   15
#define V771_PIN_DC   14
#define V771_PIN_CLK  17
#define V771_PIN_MOSI 21
#define V771_PIN_BL   16

static inline void v771_clk_low(void)
{
  esp32s3_gpiowrite(V771_PIN_CLK, false);
}

static inline void v771_clk_high(void)
{
  esp32s3_gpiowrite(V771_PIN_CLK, true);
}

/* Mode 0: idle low, data latched on the rising edge. */
static void v771_putbit(bool bit)
{
  v771_clk_low();
  esp32s3_gpiowrite(V771_PIN_MOSI, bit);
  up_udelay(2);
  v771_clk_high();
  up_udelay(2);
}

static void v771_putbyte(uint8_t val)
{
  int i;

  for (i = 7; i >= 0; i--)
    {
      v771_putbit((val & (1u << i)) != 0);
    }
}

static void v771_cmd(uint8_t c)
{
  esp32s3_gpiowrite(V771_PIN_DC, false);   /* DC low = command */
  v771_putbyte(c);
}

static void v771_data8(uint8_t d)
{
  esp32s3_gpiowrite(V771_PIN_DC, true);    /* DC high = data */
  v771_putbyte(d);
}

/* [V812] How many rounds of the raw bit-bang pass to allow before giving the
 * SPI pads back regardless of fb0.  Five rounds is about ten seconds, which
 * covers the cold-boot window the pass was written for; see the comment at the
 * break below for why waiting for fb0 is not a usable exit condition. */
#define V812_RAW_ROUNDS 5

static void v771_fill16(uint16_t px)
{
  int i;

  esp32s3_gpiowrite(V771_PIN_DC, true);
  for (i = 0; i < 240 * 240; i++)
    {
      v771_putbyte((uint8_t)(px >> 8));
      v771_putbyte((uint8_t)(px & 0xff));
    }
}

/* [V772] v769 word counter, so the hold loop can prove the fb path
 * is actually moving pixels through the restored route. */
extern uint32_t g_v769_spi_words;

static int v771_raw_panel(int argc, char **argv)
{
syslog(LOG_ERR, "[V771] raw bit-bang panel probe armed\n");
  volatile uint32_t *base = (volatile uint32_t *)0x60004000u;
  int attempt;


  /* [V784] Hand the panel over once the framebuffer is up.
   *
   * The raw pass exists only to LIGHT this panel.  But every round ends in
   * v771_fill16(0xffff) -- a full white frame -- and v783 ran it for 40
   * rounds x 2 s, so it kept repainting GRAM white while and after the UI
   * drew.  That is the white screen: the capture shows the raw pass at
   * rounds #1..#11 while g_v769_spi_words stayed pinned at 57600 (exactly
   * one frame) and XZUI was already "page built, entering loop".
   *
   * So: light the panel until /dev/fb0 exists, then stop for good.  One
   * access() per round, no new state, and if the access fails we keep
   * lighting -- which is the desired behaviour on a cold boot. */
  /* [V828] One round only, then it hands the pads back.
   *
   * v826 disabled this thread because it was interleaving with the UI, and the
   * screen went black again -- so it is doing something the driver does not, and
   * disabling it was the wrong conclusion.  The difference is the ordering:
   *
   *   rawpanel:    RAMWR -> fill(0xffff) -> DISPON     contents in place first
   *   st7789:      fill(0xffff) -> DISPON, and the real UI frames only reach
   *                GRAM afterwards through FBIO_UPDATE
   *
   * So with rawpanel gone the panel is switched on with a placeholder in GRAM
   * and the UI has to repaint it.  That was fine before, and the difference was
   * the flicker, not the black screen.
   *
   * Both problems have the same fix: run the pass once and then get out of the
   * way.  Five rounds over 80 s (the v767 runaway) is what fought the UI; one
   * round takes the pads for about a second, fills GRAM, switches the panel on
   * with something in it, and returns the pads to the SPI peripheral.
   */
  for (attempt = 1; attempt <= 1; attempt++)
    {
      /* [V784] fb0 up => the UI owns the panel now; stop filling it
       * with white.  Until then, keep going: on a cold boot the panel
       * needs these bytes precisely because the UI cannot write yet. */
      if (attempt > 1 && access("/dev/fb0", F_OK) == 0)
        {
          syslog(LOG_ERR, "[V784] fb0 is up at round %d -- raw pass done, panel handed to the UI (no more white fills)\n", attempt);
          break;
        }

      /* [V812] Give the pads back after a bounded number of rounds, whether or
       * not fb0 ever appears.
       *
       * This loop takes CS/CLK/MOSI/DC away from the GPIO matrix and drives them
       * as plain GPIO, so while it runs the SPI peripheral cannot reach the
       * panel at all -- its own comment says so.  The UI pushes frames through
       * that peripheral, so for as long as this loop owns the pads, `fb beat`
       * stays at zero and nothing is ever drawn.
       *
       * The only exit was fb0 appearing, and a capture of the black screen shows
       * why that is not enough: 40 rounds of "fb words so far=0" then the thread
       * ending.  fb0 is registered late by design (the comment in v739 says the
       * framebuffer waits for the network), so on a cold boot this thread runs
       * its full 80 seconds -- covering the entire window in which the UI is
       * trying to come up.  Everything downstream reported success the whole
       * time: page built, display ok, [V788] done.  The frames simply had
       * nowhere to go.
       *
       * Five rounds is enough to cover the cold-boot window the pass exists for
       * (the panel rail comes up in the first seconds, and the waker thread
       * owns waking it from that point on).  After that the pads must go back,
       * because a panel that is being driven by nobody is strictly worse than
       * one being driven slightly late.
       *
       * Nothing else about the pass changes: it still detaches the pads, still
       * sends the same sequence, and still stops as soon as fb0 shows up. */
      if (attempt > V812_RAW_ROUNDS)
        {
          syslog(LOG_ERR,
                 "[V812] %d rounds done and fb0 still absent -- handing CS/"
                 "CLK/MOSI/DC back to the SPI peripheral now, rather than "
                 "holding them for another %d s\n",
                 V812_RAW_ROUNDS, (40 - V812_RAW_ROUNDS) * 2);
          break;
        }


      /* Detach all four pads from the GPIO matrix and hand them to plain GPIO.
       * After this the SPI peripheral cannot reach them either way, so what
       * the panel sees is entirely what this loop writes. */

      esp32s3_gpio_matrix_out(V771_PIN_CS,   SIG_GPIO_OUT_IDX, 0, 0);
      esp32s3_gpio_matrix_out(V771_PIN_CLK,  SIG_GPIO_OUT_IDX, 0, 0);
      esp32s3_gpio_matrix_out(V771_PIN_MOSI, SIG_GPIO_OUT_IDX, 0, 0);
      esp32s3_gpio_matrix_out(V771_PIN_DC,   SIG_GPIO_OUT_IDX, 0, 0);

      esp32s3_configgpio(V771_PIN_CS,   OUTPUT);
      esp32s3_configgpio(V771_PIN_CLK,  OUTPUT);
      esp32s3_configgpio(V771_PIN_MOSI, OUTPUT);
      esp32s3_configgpio(V771_PIN_DC,   OUTPUT);

      /* Idle levels: CS high, CLK low, MOSI low, DC high. */

      esp32s3_gpiowrite(V771_PIN_CS, true);
      esp32s3_gpiowrite(V771_PIN_CLK, false);
      esp32s3_gpiowrite(V771_PIN_MOSI, false);
      esp32s3_gpiowrite(V771_PIN_DC, true);
      up_udelay(10000);

      /* Backlight asserted (active-high per v766/xiaozhi). */

      /* [V803] Leave the backlight alone.
       *
       * The backlight belongs to the blkeeper thread, and the two of us have
       * been fighting over GPIO16 for as long as this loop runs.  It exits
       * when /dev/fb0 appears, which on a cold boot is roughly 30 seconds in,
       * so about fifteen rounds of this loop re-pinned GPIO16 every two seconds
       * while blkeeper re-pinned it the same way -- alternating OUTPUT here and
       * INPUT|OUTPUT there.  Whichever ran last won, and that was not always
       * the one turning the backlight on.
       *
       * A normal/black capture pair from the same build showed it directly:
       * every indicator matched except "bl pin HIGH, keeper alive", which
       * appeared twelve times in the healthy boot and never in the black one.
       * The panel was being driven correctly throughout; it had no light.
       *
       * This raw pass exists to put bytes into GRAM before the UI can, and it
       * needs CS/CLK/MOSI/DC for that.  The backlight is not part of the job.
       */

      syslog(LOG_ERR,
             "[V775] raw bring-up (no SWRESET) #%d/40 armed: "
             "CS/DC/CLK/MOSI on plain GPIO, BL(16)=1\n", attempt);

      esp32s3_gpiowrite(V771_PIN_CS, false);
      up_udelay(1000);


      /* [V774] NO SWRESET here -- v773 proved it turns this panel black.
       * This sequence, byte for byte, is the only known-good bring-up:
       * it lights the panel with zero help from the SPI peripheral. */
      v771_cmd(0x11);                 /* SLPOUT  */
      up_udelay(150000);
      v771_cmd(0x3a); v771_data8(0x55);   /* COLMOD 16bpp */
      v771_cmd(0x36); v771_data8(0x00);   /* MADCTL: no swap */
      v771_cmd(0x21);                 /* INVON   */
      v771_cmd(0x13);                 /* NORON   */
      up_udelay(10000);
      v771_cmd(0x2c);                 /* RAMWR   */
      /* [V885] black, not white.  This frame is bit-banged once per
       * raw bring-up round and the round ran seven times, so a white
       * fill here is the slow white flashing at boot.  It still has
       * to paint something - with no fill the panel shows power-up
       * garbage until the UI takes over. */
      v771_fill16(0x0000);            /* solid black */
      up_udelay(20000);
      v771_cmd(0x29);                 /* DISPON  */
      up_udelay(100000);

      esp32s3_gpiowrite(V771_PIN_CS, true);
      up_udelay(1000);

      syslog(LOG_ERR,
             "[V775] raw bring-up #%d/40 done (no SWRESET) "
             "(SWRESET-less: SLPOUT/COLMOD/MADCTL/INVON/NORON/RAMWR/DISPON)"
             " | OUT_SEL CS=%u CLK=%u MOSI=%u DC=%u"
             " | GPIO_OUT 0x%08x CS=%u DC=%u CLK=%u MOSI=%u\n",
             attempt,
             (unsigned)(base[0x554 / 4 + V771_PIN_CS] & 0x1ff),
             (unsigned)(base[0x554 / 4 + V771_PIN_CLK] & 0x1ff),
             (unsigned)(base[0x554 / 4 + V771_PIN_MOSI] & 0x1ff),
             (unsigned)(base[0x554 / 4 + V771_PIN_DC] & 0x1ff),
             (unsigned)base[0x04 / 4],
             (unsigned)((base[0x04 / 4] >> V771_PIN_CS) & 1),
             (unsigned)((base[0x04 / 4] >> V771_PIN_DC) & 1),
             (unsigned)((base[0x04 / 4] >> V771_PIN_CLK) & 1),
             (unsigned)((base[0x04 / 4] >> V771_PIN_MOSI) & 1));

      /* [V775] Hand CLK/MOSI back to SPI3 after every round so /dev/fb0 can
       * push real pixels.  The panel is held on plain GPIO only for the short
       * window while this round is bit-banged. */
      esp32s3_gpio_matrix_out(V771_PIN_CLK,  SPI3_CLK_OUT_IDX, 0, 0);
      esp32s3_gpio_matrix_out(V771_PIN_MOSI, SPI3_D_OUT_IDX,   0, 0);
      esp32s3_gpio_matrix_out(V771_PIN_CS,   SIG_GPIO_OUT_IDX, 0, 0);
      esp32s3_gpio_matrix_out(V771_PIN_DC,   SIG_GPIO_OUT_IDX, 0, 0);
      esp32s3_configgpio(V771_PIN_CLK,  OUTPUT);
      esp32s3_configgpio(V771_PIN_MOSI, OUTPUT);
      esp32s3_gpiowrite(V771_PIN_CS, true);
      esp32s3_gpiowrite(V771_PIN_CLK, false);

      /* [V775] Let the panel power-on reset finish before the next replay. */
      syslog(LOG_ERR,
             "[V775] round %d/40 | route live CLK=%u MOSI=%u"
             " | fb words so far=%u\n",
             attempt,
             (unsigned)(base[0x554 / 4 + V771_PIN_CLK] & 0x1ff),
             (unsigned)(base[0x554 / 4 + V771_PIN_MOSI] & 0x1ff),
             (unsigned)g_v769_spi_words);

      sleep(3);
    }

  /* ---------------------------------------------------------------------
   * [V772] GIVE THE SPI ROUTE BACK.
   *
   * v771 detached CLK/MOSI from the SPI3 signals so it could bit-bang them,
   * and never restored them: the panel stayed white because every
   * st7789_wrram() byte after that point left the chip through a pad that no
   * longer routed to SPI3 (log proof: [V770] beat CLK(17)=256 MOSI(21)=256,
   * where 66/68 is the correct reading).
   *
   * Put the route back exactly as esp32s3_spi_init() had it:
   *   CLK  = SPI3_CLK_OUT_IDX (66)
   *   MOSI = SPI3_D_OUT_IDX   (68)
   *   CS   = SIG_GPIO_OUT_IDX -- software chip select, correct because
   *         CONFIG_ESP32S3_SPI_SWCS=y and SPI_UDCS is off, so esp32s3_spi3
   *         select() drives CS(15) with a plain gpiowrite().
   *   DC   = SIG_GPIO_OUT_IDX -- plain GPIO, driven by esp32s3_spi3_cmddata().
   * ------------------------------------------------------------------- */

  esp32s3_gpio_matrix_out(V771_PIN_CLK,  SPI3_CLK_OUT_IDX, 0, 0);
  esp32s3_gpio_matrix_out(V771_PIN_MOSI, SPI3_D_OUT_IDX,   0, 0);
  esp32s3_gpio_matrix_out(V771_PIN_CS,   SIG_GPIO_OUT_IDX, 0, 0);
  esp32s3_gpio_matrix_out(V771_PIN_DC,   SIG_GPIO_OUT_IDX, 0, 0);

  /* Idle levels the panel must be left in: CS high, CLK low, DC high. */

  esp32s3_configgpio(V771_PIN_CLK,  OUTPUT);
  esp32s3_configgpio(V771_PIN_MOSI, OUTPUT);
  esp32s3_configgpio(V771_PIN_CS,   OUTPUT);
  esp32s3_configgpio(V771_PIN_DC,   OUTPUT);
  esp32s3_gpiowrite(V771_PIN_CS, true);
  esp32s3_gpiowrite(V771_PIN_CLK, false);
  esp32s3_gpiowrite(V771_PIN_DC, true);

  /* Independent read-back: the log now states whether the restore took.
   * Expected: CLK=66 MOSI=68 CS=256 DC=256. */

  syslog(LOG_ERR,
         /* [V783] The SPI route restore below still runs after the raw
          * rounds (it must, or /dev/fb0 can never push a frame).  v783
          * only adds the missing setpower(1) to the wake loop. */
         "[V775] raw bring-up finished, SPI route left live: "
         "CS=%u CLK=%u MOSI=%u DC=%u\n",
         (unsigned)(base[0x554 / 4 + V771_PIN_CS] & 0x1ff),
         (unsigned)(base[0x554 / 4 + V771_PIN_CLK] & 0x1ff),
         (unsigned)(base[0x554 / 4 + V771_PIN_MOSI] & 0x1ff),
         (unsigned)(base[0x554 / 4 + V771_PIN_DC] & 0x1ff));


  /* [V772] Stay alive briefly so /dev/fb0 -- which is deferred until the
   * network is up -- can come up and push a real frame through the restored
   * route.  Do NOT touch the pads again after this point. */

  /* [V775] The 8 x 1 s hold from v772 is gone: v775 already hands the route
   * back inside every round and replays continuously. */
  for (attempt = 0; attempt < 0; attempt++)
    {
      sleep(1);
      syslog(LOG_ERR,
             "[V775] final | route CLK=%u MOSI=%u CS=%u DC=%u"
             " | V769 words=%u\n",
             attempt + 1,
             (unsigned)(base[0x554 / 4 + V771_PIN_CLK] & 0x1ff),
             (unsigned)(base[0x554 / 4 + V771_PIN_MOSI] & 0x1ff),
             (unsigned)(base[0x554 / 4 + V771_PIN_CS] & 0x1ff),
             (unsigned)(base[0x554 / 4 + V771_PIN_DC] & 0x1ff),
             (unsigned)g_v769_spi_words);
    }

  syslog(LOG_ERR, "[V775] raw panel thread done, SPI route left live\n");
  /* [V829] Hand the pads back to SPI3 before exiting.  This was never done.
   *
   * v771_raw_panel() takes CS/CLK/MOSI/DC off the GPIO matrix and drives them in
   * software:
   *
   *     esp32s3_gpio_matrix_out(V771_PIN_CLK,  SIG_GPIO_OUT_IDX, 0, 0);
   *     esp32s3_gpio_matrix_out(V771_PIN_MOSI, SIG_GPIO_OUT_IDX, 0, 0);
   *          ... and likewise CS and DC
   *
   * and then simply returned.  Nothing ever put them back, so from that moment on
   * every st7789_wrram() transfer went out through the pads' own latches instead
   * of the SPI peripheral.  The driver's own diagnostic says what that looks
   * like -- "Expected: CLK=66 MOSI=68" -- and the observed value is 256, which is
   * SIG_GPIO_OUT_IDX, i.e. still the bit-bang route.
   *
   * The consequence is that once this thread has run, the panel can never receive
   * another frame: GRAM stays as it was and the backlight is on, which is exactly
   * "backlight, no picture".  It also explains the whole sequence of attempts
   * today: v825 lit because this thread had only just run and the UI frames
   * followed quickly enough; v826 disabled the thread and the panel was then lit
   * with nothing in GRAM; v828 re-enabled it for one round and the same thing
   * happened again.
   *
   * The signal indices are the ones the driver uses on this board:
   *   SPI3_CLK_OUT_IDX = 66, SPI3_D_OUT_IDX = 68
   * [V831] CS and DC must go back to SIG_GPIO_OUT_IDX (256), NOT to 0.
   *
   * v829 wrote 0 for them and that is what turned a lit panel black: with
   * CONFIG_ESP32S3_SPI_SWCS=y and SPI_UDCS off, esp32s3_spi3 select() drives
   * CS(15) with a plain gpiowrite() and esp32s3_spi3_cmddata() drives DC(14)
   * the same way.  Both therefore need the pad routed to GPIO_OUT (signal
   * index 256), exactly as the V772/V775 restore above already does.
   * Signal 0 disconnects the pad, so the panel then sees clock and data with
   * no chip-select and no command/data strobe -- it latches nothing, which
   * reads on the bench as "backlight on, picture gone".
   *
   * Proof, same probe both times:
   *   v825 (panel lit) : [V770] CS(15)=256 CLK(17)=66 MOSI(21)=68 DC(14)=256
   *   v830 (panel dead): [V770] CS(15)=0   CLK(17)=66 MOSI(21)=68 DC(14)=0
   */
  esp32s3_gpio_matrix_out(V771_PIN_CLK,  66 /* SPI3_CLK_OUT_IDX */, 0, 0);
  esp32s3_gpio_matrix_out(V771_PIN_MOSI, 68 /* SPI3_D_OUT_IDX   */, 0, 0);
  esp32s3_gpio_matrix_out(V771_PIN_CS,   SIG_GPIO_OUT_IDX, 0, 0);
  esp32s3_gpio_matrix_out(V771_PIN_DC,   SIG_GPIO_OUT_IDX, 0, 0);

  {
    volatile uint32_t *b = (volatile uint32_t *)0x60004000u;

    syslog(LOG_ERR,
           "[V831] SPI pads handed back: CLK=%u (want 66) MOSI=%u (want 68) "
           "CS=%u (want 256) DC=%u (want 256)\n",
           (unsigned)(b[0x554 / 4 + V771_PIN_CLK] & 0x1ff),
           (unsigned)(b[0x554 / 4 + V771_PIN_MOSI] & 0x1ff),
           (unsigned)(b[0x554 / 4 + V771_PIN_CS] & 0x1ff),
           (unsigned)(b[0x554 / 4 + V771_PIN_DC] & 0x1ff));
  }

  return 0;
}



/* ---------------------------------------------------------------------------
 * [V777] Backlight pad probe -- CORRECTED addressing.
 *
 * V776's register readouts were garbage, for two independent reasons, both of
 * which are the same class of mistake it was supposed to be checking:
 *
 *   1. IO_MUX base.  DR_REG_IO_MUX_BASE is 0x60009000, NOT 0x60004000 (that
 *      is the GPIO base).  v776 read 0x60004000 + (16+1)*4, which lands in the
 *      GPIO register block and has nothing to do with IO_MUX.  Hence the
 *      uniform "IO_MUX=0x00000000" -- it was never reading the register.
 *
 *   2. FUN_IE bit.  On ESP32-S3 FUN_IE is BIT(9) (hardware/esp32s3_iomux.h:94),
 *      not bit 7 as v776 assumed.
 *
 * So v776 proved nothing about the backlight pad.  Its "FUN_IE is always 0,
 * therefore every gpioread(16)=1 is just the output latch" conclusion is
 * WITHDRAWN -- it rested on reading the wrong address.
 *
 * Correct sources (verified against the headers):
 *   GPIO_OUT_REG    = 0x60004000 + 0x04    bit16 = output latch level
 *   GPIO_ENABLE_REG = 0x60004000 + 0x20    bit16 = output driver enable
 *   IO_MUX(pin 16)  = 0x60009000 + (16+1)*4
 *                     bit9  = FUN_IE (input buffer)
 *                     bit10-12 = FUN_DRV
 *                     bit8-11 = MCU_SEL (overlaps, printed raw)
 *   GPIO_FUNCn_OUT_SEL_CFG = 0x60004000 + 0x554 + 16*4   bits8:0
 *
 * What this version does differently: it prints the whole IO_MUX register so
 * the value can be checked against what esp32s3_configgpio() should have
 * written, instead of trusting a hand-picked bit.
 * ---------------------------------------------------------------------------
 */

#define V777_PIN_BL 16
#define V777_GPIO_BASE 0x60004000u
#define V777_IOMUX_BASE 0x60009000u

static void v777_show(const char *tag)
{
  volatile uint32_t *gpio = (volatile uint32_t *)V777_GPIO_BASE;
  volatile uint32_t *mux  = (volatile uint32_t *)V777_IOMUX_BASE;
  uint32_t out  = gpio[0x04 / 4];
  uint32_t en   = gpio[0x20 / 4];
  uint32_t mreg = mux[((V777_PIN_BL + 1) * 4) / 4];
  uint32_t osel = gpio[0x554 / 4 + V777_PIN_BL] & 0x1ff;

  syslog(LOG_ERR,
         "[V777] %-28s gpioread=%d OUT.bit16=%u EN.bit16=%u"
         " | IO_MUX=0x%08x FUN_IE(bit9)=%u DRV(bits10-12)=%u MCU_SEL(bits8-11)=0x%x"
         " | OUT_SEL=%u\n",
         tag,
         (int)esp32s3_gpioread(V777_PIN_BL),
         (unsigned)((out >> V777_PIN_BL) & 1),
         (unsigned)((en >> V777_PIN_BL) & 1),
         (unsigned)mreg,
         (unsigned)((mreg >> 9) & 1),
         (unsigned)((mreg >> 10) & 7),
         (unsigned)((mreg >> 8) & 0xf),
         (unsigned)osel);
}

static int v777_bl_probe(int argc, char **argv)
{
  sleep(4);

  syslog(LOG_ERR, "[V777] ===== backlight probe (corrected IO_MUX base"
         " 0x60009000, FUN_IE=bit9) =====\n");

  v777_show("Q0 as-booted");

  /* Q1: pure input, output driver off, input buffer explicitly ON.
   * Whatever the pad reads here is the panel circuit acting on its own.
   * This is the reading v776 could not take. */

  esp32s3_configgpio(V777_PIN_BL, INPUT);
  up_udelay(300000);
  v777_show("Q1 INPUT (driver off)");

  /* Q2: drive HIGH, input buffer still on (INPUT|OUTPUT). */

  esp32s3_configgpio(V777_PIN_BL, INPUT | OUTPUT);
  esp32s3_gpiowrite(V777_PIN_BL, true);
  up_udelay(300000);
  v777_show("Q2 IN|OUT HIGH");

  /* Q3: drive LOW, input buffer still on. */

  esp32s3_gpiowrite(V777_PIN_BL, false);
  up_udelay(300000);
  v777_show("Q3 IN|OUT LOW");

  /* Q4: release completely.  If the pad reads HIGH here, the panel circuit is
   * sourcing current on BL and our OUTPUT driver has been fighting it. */

  esp32s3_configgpio(V777_PIN_BL, INPUT);
  up_udelay(800000);
  v777_show("Q4 released (INPUT only)");

  /* Q5: what the raw bit-bang thread actually does -- plain OUTPUT, no
   * input buffer.  This is the state the panel sees today. */

  esp32s3_configgpio(V777_PIN_BL, OUTPUT);
  esp32s3_gpiowrite(V777_PIN_BL, true);
  up_udelay(300000);
  v777_show("Q5 OUTPUT HIGH (v771 style)");

  /* Restore the long-lived state used by every other thread. */

  esp32s3_configgpio(V777_PIN_BL, AI_VOX3_BCKL_MODE);
  esp32s3_gpiowrite(V777_PIN_BL, true);
  up_udelay(100000);
  v777_show("Q6 restored");

  syslog(LOG_ERR, "[V777] ===== probe end =====\n");
  return 0;
}



static void v770_dump_route(const char *tag)
{
  /* GPIO_FUNCn_OUT_SEL_CFG_REG = DR_REG_GPIO_BASE + 0x554 + n * 4
   * DR_REG_GPIO_BASE = 0x60004000
   */
  volatile uint32_t *base = (volatile uint32_t *)0x60004000u;
  unsigned cs  = base[0x554 / 4 + 15];   /* pin 15 = CS   */
  unsigned clk = base[0x554 / 4 + 17];   /* pin 17 = CLK  */
  unsigned mosi = base[0x554 / 4 + 21];  /* pin 21 = MOSI */
  unsigned dc  = base[0x554 / 4 + 14];   /* pin 14 = DC, 软件 GPIO */

  syslog(LOG_ERR,
         "[V770] %s gpio-matrix OUT_SEL (bits8:0) "
         "CS(15)=%u CLK(17)=%u MOSI(21)=%u DC(14)=%u "
         "(0x1ff = SIG_GPIO_OUT_IDX = pad follows GPIO_OUT; "
         "SPI3 sig ids are small numbers) "
         "IO_MUX CS=0x%08x CLK=0x%08x MOSI=0x%08x\n",
         tag, cs & 0x1ff, clk & 0x1ff, mosi & 0x1ff, dc & 0x1ff,
         (unsigned)base[0x044 / 4 + 15],
         (unsigned)base[0x044 / 4 + 17],
         (unsigned)base[0x044 / 4 + 21]);
}

static int s_v742_waker_started;

/* [V822] Backlight alone: the elimination build.
 *
 * Everything else here has been checked line by line and found sound.  The UI
 * maps the framebuffer, draws into it and pushes frames through FBIO_UPDATE;
 * the driver writes GRAM with the right window and no errors -- [V801]
 * fb_updates=2 fb_err=0 spi_bytes=460800, which is exactly two 240x240x2-byte
 * frames.  The wake sequence is re-issued five times across the first thirty
 * seconds.  The screen is still black.
 *
 * So stop adding and remove instead.  This function touches one pad and nothing
 * else -- no SPI, no LVGL, no WiFi, no framebuffer -- and it never writes the
 * pad again, so nothing else can take it back:
 *
 *   glows     -> the pad, the LED and its driver are fine; the fault is
 *                downstream of here, i.e. the panel, and the next question is
 *                whether it ever leaves Sleep-In.
 *   still dark -> no software change can produce light here.  The pad can be
 *                correct at the SoC and still reach nothing: the driver
 *                transistor, the resistor, the LED and the FPC are all outside
 *                anything this firmware can observe.
 *
 * That second outcome is worth stating plainly, because it ends the search:
 * every measurement available from software has been taken, and all of them
 * say the system is working.
 *
 * v522 already asked this exact question and a person answered it -- backlight
 * on a high level, plain GPIO, no PWM (memory/2026-10-11.md, v517-v523).  That
 * is the configuration below, unchanged.
 */
static void v822_backlight_only(void)
{
  const int bl = ESP32S3_EYE_DISPLAY_BCKL;

  /* v523 order: unbind, configure INPUT|OUTPUT so the input buffer is on,
   * then drive high (active-high, per the v522 eye-witnessed demo). */
  esp32s3_gpio_matrix_out(bl, SIG_GPIO_OUT_IDX, 0, 0);
  esp32s3_configgpio(bl, AI_VOX3_BCKL_MODE);
  esp32s3_gpiowrite(bl, 1);   /* [V837] HIGH = lit */

  syslog(LOG_ERR,
         "[V822] GPIO%d HIGH and held.  Nothing else runs: no SPI, no LVGL, no "
         "WiFi, no fb.  No glow here means no software change can make one.\n",
         bl);

  for (;;)
    {
      sleep(3600);
    }
}


/* [V946] Cold-boot progress bar.
 *
 * On a cold power-on /dev/fb0 is not registered until the network is up, so
 * for the first ~40 s the UI has no canvas: the panel shows only the 0xFFFF
 * fill left by st7789_lcdinitialize(), and CONFIG_LCD_ST7789_INVCOLOR turns
 * that into pure black.  A user watching the device cannot tell a booting
 * board from a dead one.
 *
 * This paints a progress bar straight into GRAM through putarea(), the same
 * path the V765 white-shot probe used.  putarea() needs only g_lcd, which is
 * valid from "panel lit early", so it works in exactly the window where
 * nothing else can draw.  The call site is gated on fd < 0 and therefore
 * stops for good the moment /dev/fb0 opens -- from then on the UI owns the
 * panel and there is no third writer to contend with.
 *
 * Colour note: INVON inverts everything, so 0x0000 is WHITE here and 0xFFFF
 * is black.  The bar grows one cell per beat, and a beat is 2 s for the first
 * 60 beats, so it sweeps once in about 42 s -- the length of the window it
 * covers.  beat advances at the bottom of the waker loop, so the value seen
 * here is the previous beat; that only shifts the phase by one cell.
 */

#define V946_BAR_CELLS 20
#define V946_CELL_W    10
#define V946_BAR_H     12
#define V946_BAR_X     20
#define V946_BAR_Y     200
#define V946_BAR_W     (V946_BAR_CELLS * V946_CELL_W)

static int v946_boot_indicator(int beat)
{
  static uint16_t *s_bar;
  struct lcd_planeinfo_s p;
  int cells;
  int x;
  int y;

  if (g_lcd == 0 || g_lcd->getplaneinfo == 0)
    {
      return -1;
    }

  if (s_bar == 0)
    {
      s_bar = (uint16_t *)malloc(V946_BAR_W * V946_BAR_H *
                                 sizeof(uint16_t));

      if (s_bar == 0)
        {
          syslog(LOG_ERR, "[V946] boot bar: no memory, indicator off\n");
          return -1;
        }
    }

  if (g_lcd->getplaneinfo(g_lcd, 0, &p) != OK || p.putarea == 0)
    {
      return -1;
    }

  cells = beat % (V946_BAR_CELLS + 1);

  for (y = 0; y < V946_BAR_H; y++)
    {
      for (x = 0; x < V946_BAR_W; x++)
        {
          s_bar[y * V946_BAR_W + x] =
            ((x / V946_CELL_W) < cells) ? 0x0000 : 0xffff;
        }
    }

  return p.putarea(g_lcd, V946_BAR_Y, V946_BAR_Y + V946_BAR_H - 1,
                   V946_BAR_X, V946_BAR_X + V946_BAR_W - 1,
                   (FAR const uint8_t *)s_bar,
                   (fb_coord_t)(V946_BAR_W * sizeof(uint16_t)));
}


/* [V948-A] Panel / backlight blink self-test.
 *
 * Every software-side check is already green on this board:
 *   [V819]  SLPOUT+NORON+DISPON x5
 *   [V769]  SPI3 poll_exchange words=2400 stall=0   (the V946 bar really left)
 *   [V847]  bl16 out=1 outsel=256 pad_reads=1       (GPIO16 reads back HIGH)
 *   display(on=1) x55
 * and the screen is still pure black.  Pure black -- not dark grey -- also
 * means the backlight is not lit, so "panel not showing" and "backlight dead"
 * are indistinguishable from the log alone.
 *
 * Six cycles of 300 ms on / 300 ms off, a 200x40 block, so it reads as a
 * deliberate probe and not as flicker.  Answer it with your eyes:
 *   blinks + white block -> panel and backlight are both alive
 *   blinks, no white     -> backlight alive, panel not showing
 *   nothing at all       -> the panel rail is not powered; no firmware
 *                           change can light this panel
 *
 * putarea() only emits CASET/RASET/RAMWR, never a protocol command, so this
 * cannot push the panel out of whatever state the waker left it in.  The
 * backlight is left ON at the end and blkeeper re-asserts it every 3 s.
 */

#define V948_BLK_W   200
#define V948_BLK_H   40
#define V948_BLK_X   20
#define V948_BLK_Y   100
#define V948_CYCLES  6
#define V948_HOLD_US 300000

static int v948_panel_blink(void)
{
  static uint16_t *s_blk;
  static int s_done;
  struct lcd_planeinfo_s p;
  unsigned int i;
  unsigned int n;

  if (s_done)
    {
      return 0;
    }

  s_done = 1;

  if (g_lcd == 0 || g_lcd->getplaneinfo == 0)
    {
      syslog(LOG_ERR, "[V948] blink: no lcd device\n");
      return -1;
    }

  if (g_lcd->getplaneinfo(g_lcd, 0, &p) != OK || p.putarea == 0)
    {
      syslog(LOG_ERR, "[V948] blink: no putarea\n");
      return -1;
    }

  s_blk = (uint16_t *)malloc(V948_BLK_W * V948_BLK_H * sizeof(uint16_t));

  if (s_blk == 0)
    {
      syslog(LOG_ERR, "[V948] blink: no memory\n");
      return -1;
    }

  syslog(LOG_ERR, "[V948] blink test start: %d x (%d ms on / %d ms off)\n",
         V948_CYCLES, V948_HOLD_US / 1000, V948_HOLD_US / 1000);

  for (i = 0; i < V948_CYCLES; i++)
    {
      /* ON: white block (INVON renders 0x0000 as white) + backlight high */

      n = V948_BLK_W * V948_BLK_H;
      while (n > 0)
        {
          s_blk[--n] = 0x0000;
        }

      (void)p.putarea(g_lcd, V948_BLK_Y, V948_BLK_Y + V948_BLK_H - 1,
                      V948_BLK_X, V948_BLK_X + V948_BLK_W - 1,
                      (FAR const uint8_t *)s_blk,
                      (fb_coord_t)(V948_BLK_W * sizeof(uint16_t)));

      esp32s3_gpiowrite(ESP32S3_EYE_DISPLAY_BCKL, 1);
      usleep(V948_HOLD_US);

      /* OFF: black block + backlight low */

      n = V948_BLK_W * V948_BLK_H;
      while (n > 0)
        {
          s_blk[--n] = 0xffff;
        }

      (void)p.putarea(g_lcd, V948_BLK_Y, V948_BLK_Y + V948_BLK_H - 1,
                      V948_BLK_X, V948_BLK_X + V948_BLK_W - 1,
                      (FAR const uint8_t *)s_blk,
                      (fb_coord_t)(V948_BLK_W * sizeof(uint16_t)));

      esp32s3_gpiowrite(ESP32S3_EYE_DISPLAY_BCKL, 0);
      usleep(V948_HOLD_US);

      syslog(LOG_ERR, "[V948] blink %u/%d done\n", i + 1, V948_CYCLES);
    }

  esp32s3_gpiowrite(ESP32S3_EYE_DISPLAY_BCKL, 1);

  syslog(LOG_ERR, "[V948] blink test finished, backlight left ON\n");
  return 0;
}


/* [V949] LCD pad IO_MUX repair.
 *
 * IO_MUX base on ESP32-S3 is 0x60009000 and GPIOn sits at base + 0x04 + n*4.
 * bits[14:12] = MCU_SEL (0 = GPIO matrix, 1 = alt function F1).
 *
 * The log shows 0x1a00 for GPIO16, i.e. MCU_SEL=1, i.e. the backlight pad is
 * owned by U0RXD.  A pin on an alternate function ignores both GPIO_OUT and
 * the GPIO-matrix OUT_SEL route, so the backlight can never light and a SPI3
 * route on such a pad would never carry a byte.  Clearing MCU_SEL is the one
 * thing that makes a pad a GPIO again.
 *
 * Called every beat.  Idempotent: once MCU_SEL is 1 the loop only reads.
 *
 * [V955] CORRECTION -- the premise above was wrong, so this function did
 * the exact opposite of what its own comment claims.
 *
 *   nuttx/arch/xtensa/src/esp32s3/hardware/esp32s3_iomux.h:170
 *       #define PIN_FUNC_GPIO 1
 *
 * On ESP32-S3, MCU_SEL = 1 IS the GPIO function; 0 selects peripheral
 * function F0.  So the original 0x1a00 reading did not mean "the pad is
 * owned by U0RXD", it meant "the pad is already a clean GPIO", and forcing
 * it to 0 is what actually disconnected these five pads.
 *
 * esp32s3_configgpio() agrees: its no-FUNCTION branch writes
 * PIN_FUNC_GPIO, i.e. 1.  That is why v818_bl_on() kept putting 1 back
 * every 3 s and looked like an attacker in the logs -- it was undoing
 * this function's damage, and the beat-by-beat repair was the thing
 * breaking the pads.
 */

#define V949_IOMUX_BASE 0x60009000u

static void v949_pad_fix(void)
{
  static const unsigned pins[5] = { 15, 14, 17, 21, 16 };
  static int s_announced;
  unsigned int i;
  int fixed = 0;

  for (i = 0; i < 5; i++)
    {
      volatile uint32_t *reg =
        (volatile uint32_t *)(V949_IOMUX_BASE + 0x04u + (pins[i] * 4u));
      uint32_t v = *reg;
      unsigned sel = (unsigned)((v >> 12) & 0x7u);

      if (sel != 1)   /* [V955] 1 == PIN_FUNC_GPIO == GPIO on ESP32-S3 */
        {
          /* MCU_SEL -> 1 (GPIO).  Also drop WPD/WPU/FUN_IE so the pad is a
           * clean push-pull output; the LCD only ever drives these pins. */

          uint32_t nv = (v & ~(0x7u << 12)) | (1u << 12);  /* [V955] GPIO */
          nv &= ~(0x7u << 8);
          *reg = nv;
          fixed++;

          if (!s_announced)
            {
              syslog(LOG_ERR, "[V955] pad %u MCU_SEL was %u -> 1 (GPIO) "
                     "(iomux 0x%08x -> 0x%08x)\n",
                     pins[i], sel, (unsigned)v, (unsigned)nv);
            }
        }
    }

  if (fixed && !s_announced)
    {
      s_announced = 1;
      syslog(LOG_ERR, "[V955] %d LCD pad(s) restored to GPIO "
             "(MCU_SEL=1=PIN_FUNC_GPIO); re-asserting backlight\n", fixed);
    }

  if (fixed)
    {
      esp32s3_gpiowrite(ESP32S3_EYE_DISPLAY_BCKL, 1);
    }
}

static void v742_panel_waker(void)
{
  int fd = -1;
  int beat = 0;

    /* [V819] Bounded wake retries -- see the comment at the setpower() call.
     * Five attempts with a growing delay cover roughly 25 s, which spans the
     * window the panel rail needs on a cold power-on. */
    int wake_attempts = 0;
  #define V819_WAKE_ATTEMPTS     5
  #define V819_WAKE_FIRST_DELAY_S 2
  /* [V800] The panel must be woken, then LEFT ALONE.
   *
   * The wake itself is not the problem; repeating it every 2 s is.  A cold
   * capture of a black-screen boot showed the waker issuing the whole power-up
   * sequence 51 times in one boot:
   *
   *   SLPOUT(0x11) x51   NORON(0x13) x51   DISPON(0x29) x51   INVON(0x21) x51
   *   while the GRAM fill happened only twice
   *
   * Every pass ends with the panel just coming out of Sleep-In, and the ST7789
   * needs 120 ms after SLPOUT before the next command means anything.  Replaying
   * the sequence faster than the panel can settle holds it in that transient
   * permanently -- which looks exactly like a black screen.  And because the
   * wake nominally keeps happening, every software-side check agrees that
   * everything is fine: display ok, page built, spi_bytes far over budget.
   * That combination is what made this look impossible for so long.
   *
   * So: wake a few times to cover the cold-boot window, then stop touching the
   * protocol.  fb0 stays open and FBIO_UPDATE keeps running, because that is
   * what feeds GRAM and it does not disturb the panel state. */
  int v800_settle_left = 0;   /* [V827] unused; kept to avoid churn */

  /* [V760] Let the boot settle, then re-assert the panel wake-up forever.
   *
   * Two things were wrong here and between them they are exactly the
   * cold-boot black screen plus "the power key does nothing".
   *
   * 1) The old code opened /dev/fb0 exactly once, 5 s after this thread was
   *    created.  board_lcd_initialize() creates us at "panel lit early"
   *    time, but the framebuffer is deferred until the network is up -- the
   *    V735 wait alone is 3 s, then the framebuffer allocation, then the UI
   *    start.  So that one open() always raced ahead of fb_register() and
   *    returned -1, and the FBIO_UPDATE below never ran even once.  It is
   *    the other half of the fix: the wake-up handshake alone still leaves
   *    the panel holding power-on garbage, it needs a complete frame pushed
   *    to it.  The board log shows both halves at once:
   *      "[V742] panel wake re-assert #1 ..., fb=-1"
   *    while "ls /dev/fb0" later in the very same session lists the device.
   *
   * 2) setpower(1) used to reach the panel only as DISPON(0x29) + INVON,
   *    with no SLPOUT(0x11).  A panel that just got its rail back sits in
   *    Sleep-In and ignores DISPON entirely, so pressing the power key
   *    powered the panel but nothing ever told it to wake up.  That is the
   *    "power key does nothing" half; V760 fixes it inside
   *    st7789_setpower(), which now emits SLPOUT before DISPON.
   *
   * Retry the open from inside the loop instead of pinning it once. */

  sleep(2);

  for (;;)
    {
      /* [V949] Make the LCD pads real GPIOs before anything uses
       * them.  Log shows MCU_SEL=1 on GPIO16, i.e. the backlight pad
       * is owned by U0RXD: gpiowrite sets the register but the pin
       * never moves, and pad_reads=1 is the UART idle level, not the
       * backlight.  Same for any SPI3 route on such a pad. */
      v949_pad_fix();

      /* [V784] This has printed fb=-1 on every beat since v768, which
       * is why the FBIO_UPDATE half of this wake never ran: the panel
       * was being woken, but no frame was ever pushed through it.  The
       * retry itself is right (fb registers late, behind DHCP), so it
       * stays -- it is now the ONLY thing that has to succeed for the
       * UI to appear, hence the V784 errno reporting added next. */
      if (fd < 0)
        {
          fd = open("/dev/fb0", O_RDWR);

          /* [V784] Say what the failure was, once every 15 beats.  A
           * persistent ENOENT means the frame path is still dead and the
           * panel will keep showing whatever the last fill left there. */
          if (fd < 0 && (beat % 15) == 0)
            {
              syslog(LOG_ERR, "[V784] open(/dev/fb0) still failing beat=%d errno=%d\n", beat, errno);
            }
        }

      /* [V948-A] One-shot blink probe, only while there is no
       * framebuffer.  Judge it with your eyes, not with the log. */
      if (fd < 0)
        {
          (void)v948_panel_blink();
        }

      /* [V946] Keep the cold-boot window visible.  Strictly gated on
       * fd < 0: putarea() writes GRAM directly and needs only g_lcd,
       * so it works before /dev/fb0 exists.  The moment the node
       * opens, this stops for good and the UI takes the panel. */
      if (fd < 0)
        {
          (void)v946_boot_indicator(beat);

          if (beat > 0 && (beat % 10) == 0)
            {
              syslog(LOG_ERR, "[V946] boot bar cell=%d (fb0 not up "
                     "yet, beat=%d)\n", beat % (V946_BAR_CELLS + 1),
                     beat);
            }
        }

      /* [V765] GRAM 直刷自检：把整帧填成纯白，绕开 LVGL 与 fb 驱动，
       * 直接走 st7789_putarea()。
       *
       * 目的只有一个 —— 把「数据没进 GRAM」和「进了 GRAM 面板也不显示」
       * 分开。到 v764 为止这两件事在日志上完全同形：软件侧全绿
       * （display ok / page built / fb=3），屏幕全黑；面板无 MISO
       * （miso=-1）所以无法回读，任何「面板收到了什么」的推断都不可验证。
       * 唯一的办法是改变输出、看现象。
       *
       * 判读表（冷上电后看这一段）：
       *   刷成纯白后亮      => GRAM 窗口/偏移正确，问题在唤醒协议
       *   刷成纯白后仍黑    => 数据根本没进 GRAM，SPI3 传输层问题
       *   花屏 / 出现条纹   => 进了 GRAM 但 MADCTL 或 offset 错
       *
       * 只做一次，且必须在 fb 打开成功之后：fb 还没注册时 open 返回 -1，
       * 此时刷不了任何东西，硬刷只会拿到 ENOENT 而不是有效结论。
       * 一次性是为了不和 UI 长期抢面板 —— 这正是 v553 修过的 flicker。 */

      /* [V884] Disabled.  This was the four-band colour-bar probe
       * (0xf800 / 0x07e0 / 0x001f / 0xffff) followed by sleep(5).
       * On a board that halts instead of resetting, whatever it
       * painted stays on the panel for good -- that is the
       * "stuck on colour bars" symptom.  Restore the original
       * condition to bring the probe back. */
      if (0 && fd >= 0 && !s_v765_whiteshot)
        {
          uint16_t *wbuf;
          int nfilled = -1;

          s_v765_whiteshot = 1;

          wbuf = (uint16_t *)malloc(V765_W * V765_W * sizeof(uint16_t));

          if (wbuf == NULL)
            {
              syslog(LOG_ERR, "[V765] GRAM white-shot: malloc FAILED\n");
            }
          else
            {
              int i;

              /* [V842] Four bands, not one white field.
               *
               * CONFIG_LCD_ST7789_INVCOLOR=y, so st7789_display() sends
               * INVON(0x21) with DISPON and the panel inverts everything it
               * shows.  A frame of 0xFFFF therefore comes out BLACK -- the
               * probe was indistinguishable from the fault it exists to
               * rule out, and every "white-shot, still black" reading since
               * v765 has meant nothing.
               *
               * Bands survive inversion: 0xF800/0x07E0/0x001F/0xFFFF come
               * out cyan / magenta / yellow / black.  If any band is
               * visible then GRAM is being written AND displayed, and the
               * remaining fault is in the UI or the framebuffer path, not
               * in the panel or the transport. */
              {
                int y;
                int x;

                for (y = 0; y < V765_W; y++)
                  {
                    uint16_t c;

                    if (y < V765_W / 4)
                      {
                        c = 0xf800;          /* red   -> cyan  under INVON */
                      }
                    else if (y < V765_W / 2)
                      {
                        c = 0x07e0;          /* green -> magenta */
                      }
                    else if (y < (V765_W * 3) / 4)
                      {
                        c = 0x001f;          /* blue  -> yellow */
                      }
                    else
                      {
                        c = 0xffff;          /* white -> black  */
                      }

                    for (x = 0; x < V765_W; x++)
                      {
                        wbuf[y * V765_W + x] = c;
                      }
                  }
              }

              if (g_lcd != NULL && g_lcd->getplaneinfo != NULL)
                {
                  struct lcd_planeinfo_s p;

                  if (g_lcd->getplaneinfo(g_lcd, 0, &p) == OK &&
                      p.putarea != NULL)
                    {
                      nfilled = p.putarea(g_lcd, 0, V765_W - 1, 0, V765_W - 1,
                                          (FAR const uint8_t *)wbuf,
                                          (fb_coord_t)(V765_W *
                                                       sizeof(uint16_t)));

                      /* [V842] Hold it.  The UI starts painting within a
                       * second or two and would otherwise cover the probe
                       * before anyone could look at it. */
                      sleep(5);
                    }
                }

              /* [V798] Read the SPI3 counters that st7789.c now keeps.
               *
               * putarea=0 cannot distinguish "sent" from "accepted": the
               * driver returns OK unconditionally.  The byte count is the only
               * figure here that reflects the transport, and for a 240x240
               * white frame it must be at least 240*240*2 bytes across the
               * row burst.  Far fewer means the bytes never left the SoC. */
              {
                extern void st7789_v797_spi_stats(uint32_t *, uint32_t *);
                uint32_t sc = 0;
                uint32_t sb = 0;

                st7789_v797_spi_stats(&sc, &sb);
                syslog(LOG_ERR,
                       "[V798] white-shot spi_calls=%u spi_bytes=%u "
                       "expected_min=%u (bytes<expected_min = never left "
                       "the SoC)\n",
                       (unsigned)sc, (unsigned)sb,
                       (unsigned)(V765_W * V765_W * 2));
              }

              /* [V801] Cross-check the two counters.
               *
               * fb_updates comes from drivers/video/fb.c on the success path of
               * the ioctl the UI calls; spi_bytes comes from st7789.c at the SPI
               * bursts.  They answer different questions:
               *
               *   fb_updates rising, spi_bytes flat  -> the frame never left
               *                                                 the SoC
               *   both rising, panel still white      -> GRAM is not being
               *                                                 displayed; window
               *                                                 or offset
               *   fb_updates flat                    -> the UI is not asking
               *                                                 for a push at all
               */
              {
                extern void fb_v801_stats(uint32_t *, uint32_t *, uint32_t *);
                extern void st7789_v797_spi_stats(uint32_t *, uint32_t *);
                uint32_t fu = 0;
                uint32_t fe = 0;
                uint32_t fl = 0;
                uint32_t sc = 0;
                uint32_t sb = 0;

                fb_v801_stats(&fu, &fe, &fl);
                st7789_v797_spi_stats(&sc, &sb);
                syslog(LOG_ERR,
                       "[V801] fb_updates=%u fb_err=%u last_errno=%u "
                       "spi_calls=%u spi_bytes=%u\n",
                       (unsigned)fu, (unsigned)fe, (unsigned)fl,
                       (unsigned)sc, (unsigned)sb);
              }

              syslog(LOG_ERR,
                     "[V765] GRAM white-shot %dx%d putarea=%d "
                     "(neg = data never reached GRAM; >=0 but still black = "
                     "panel protocol/state, not the frame path)\n",
                     V765_W, V765_W, nfilled);
              free(wbuf);
            }
        }

      /* [V768] ★ 本轮唯一的变量：停止周期重初始化面板。
       *
       * v767 之前，面板协议态每 2~6 s 就被重写一次（setpower(1) =>
       * SLPOUT + NORON + DISPON + INVON，75 s 内共 30+ 次，见日志）。
       * 只要这个循环在跑，「面板能不能亮」这个问题就没有答案可言 ——
       * 面板永远处于被复位的中间态。
       *
       * v768 让 waker 只保留 /dev/fb0 的 open 重试（面板数据由此进 GRAM），
       * 不再调 setpower。面板协议态此后只由 st7789_lcdinitialize() 设置一次，
       * 之后由 UI 持续写 GRAM。若这样就亮 => 根因是周期重初始化本身；
       * 若仍然黑 => 面板从冷上电起就没被真正唤醒，问题在初始化那一刻。
       *
       * v769 实测：静态面板仍然黑 => 该分支排除。换成传输层取证，每拍打
       * 一次 SPI3 的 poll_exchange 账 + 硬件寄存器快照 —— 这是唯一能回答
       * 「字节到底有没有出了芯片」的独立口径。
       */

      /* [V819] Re-issue the wake sequence -- bounded, and only while it is
       * still plausibly needed.
       *
       * Stopping here was the mistake that made the screen black.  v768 removed
       * setpower() from this loop on the reasoning that a panel which is already
       * up should be left alone, and v769 then concluded from "static panel still
       * dark" that periodic re-initialisation was not the issue.  Both steps are
       * wrong, and st7789.c:1018 says why in as many words:
       *
       *   "The ST7789 powers up in Sleep-In.  While it is asleep DISPON(0x29) is
       *    silently ignored -- the charge pump is off and the panel shows
       *    nothing.  The one-shot SWRESET + SLPOUT + DISPON in
       *    st7789_lcdinitialize() is clocked out before the panel rail is up,
       *    those bytes are lost, and nothing ever re-issues SLPOUT.  A warm (EN)
       *    reset works because the rail never dropped."
       *
       * So the single boot-time attempt is the one most likely to be lost, and
       * "leave a working panel alone" is the right instinct applied to the wrong
       * moment: it is exactly the retry that recovers from the lost attempt.
       *
       * Bounded, because v767 was right that an unbounded loop is wrong -- 51
       * re-issues in 80 s leaves the panel permanently in a half-reset state.
       * The difference is that the loop has to end once the panel has had a
       * reasonable opportunity to be awake:
       *
       *   - attempts are capped at V819_WAKE_ATTEMPTS
       *   - the delay grows between them, so the retries spread across the whole
       *     window in which the rail can come up rather than bunching at one end
       *
       * No "is it lit yet" test is attempted: the ST7789 has no ACK and MISO is
       * not connected on this board, so there is no way to ask it (see the
       * existing note about that).  Counting attempts and growing the interval
       * is the honest alternative to a check that cannot exist.
       */
      if (wake_attempts < V819_WAKE_ATTEMPTS)   /* [V886] restored; was `wake_attempts < V819_WAKE_ATTEMPTS`
                  -- see the note above this block */
        {
          /* [V840] Growing delay for the first five attempts so
           * they spread across the boot window (~2/6/12/20/30 s),
           * then a slow heartbeat that never stops. */
          int delay_s = (wake_attempts < V819_WAKE_ATTEMPTS)
                        ? (V819_WAKE_FIRST_DELAY_S
                           + (wake_attempts * 2))
                        : 8;

          wake_attempts++;

          g_lcd->setpower(g_lcd, 1);

          /* [V840] A panel whose rail dropped comes back on its
           * power-on defaults: it has forgotten COLMOD, MADCTL and
           * RAMCTL, so the RGB565 stream is decoded with the wrong
           * pixel format and the screen stays black even though
           * every byte arrives.  Replay those every 6th attempt.
           * (No ACK and MISO is not connected on this board, so
           * there is no way to ask the panel whether it needs it.) */
          if ((wake_attempts % 6) == 0)
            {
              extern int st7789_reconfigure(FAR struct lcd_dev_s *dev);

              st7789_reconfigure(g_lcd);
            }

          syslog(LOG_ERR,
                 "[V819] wake re-assert %d/%d (SLPOUT+NORON+DISPON), "
                 "next in %d s -- the boot-time sequence is lost whenever the "
                 "panel rail is not up yet\n",
                 wake_attempts, V819_WAKE_ATTEMPTS, delay_s);

          sleep(delay_s);
        }
      else if (wake_attempts == V819_WAKE_ATTEMPTS)
        {
          wake_attempts++;   /* announce the stop exactly once */

          syslog(LOG_ERR,
                 "[V819] %d wake attempts done; leaving the panel alone from "
                 "here (the UI writes GRAM now)\n", V819_WAKE_ATTEMPTS);

          /* [V892] Announce the switch of strategy exactly once: from here
           * on the loop must keep the panel on WITHOUT ever re-issuing
           * SLPOUT, because SLPOUT's 120 ms settle is a 120 ms blank frame,
           * and that is the visible flicker V886 was chasing.  V886 solved
           * it by stopping entirely after five attempts, which is what
           * leaves a cold boot (rail coming up after the window) black for
           * good.  Keeping DISPON alive and dropping SLPOUT fixes both.
           */

          syslog(LOG_ERR,
                 "[V892] wake window closed; DISPON-only keep-alive from now on\n");

        }

      /* [V948-B] Periodic FULL wake after the fast window.
       *
       * [V892] leaves only DISPON running from here, and DISPON is
       * silently ignored by a panel that is still in Sleep-In
       * (st7789.c:1039).  The five fast attempts above land at
       * 7..40 s, so a panel rail that comes up later -- the normal
       * case when the power key is pressed after the board has
       * already booted -- is never sent SLPOUT again and stays dark
       * for good.
       *
       * 8 beats is ~48 s of steady state: slow enough that it cannot
       * reproduce the V767 half-reset (that was a 2 s period, 51
       * SLPOUTs in 80 s), frequent enough that the panel is woken
       * within a minute of the rail coming up.  120 ms of SLPOUT
       * settle per 48 s is not visible as flicker. */

      /* [V951] Only when V950 is not already doing it, and 4x more
       * often: fd >= 0 is the state where nothing else wakes the
       * panel, and 8 beats there is 48 s -- long enough to miss a
       * power-key press entirely. */

      if (wake_attempts > V819_WAKE_ATTEMPTS && fd >= 0)
        {
          static int s_v948_idle;

          if (++s_v948_idle >= 2)
            {
              s_v948_idle = 0;

              if (g_lcd != NULL)
                {
                  g_lcd->setpower(g_lcd, 1);
                  syslog(LOG_ERR, "[V948] periodic full wake "
                         "(SLPOUT+NORON+DISPON) re-issued\n");
                }
            }
        }

      esp32s3_v769_spi_dump(3);
      v770_dump_route("beat");

#ifdef CONFIG_FB_UPDATE
      if (fd >= 0)
        {
          struct fb_area_s area;

          area.x = 0;
          area.y = 0;
          area.w = 240;
          area.h = 240;

          (void)ioctl(fd, FBIO_UPDATE, (unsigned long)&area);
        }
#endif

      /* [V783] *** RESTORED: the actual panel wake. ***
       *
       * v768 removed the st7789_setpower(1) call from this loop to run a
       * "static panel" A/B, and never put it back.  Everything below this
       * point only ever did open("/dev/fb0") + FBIO_UPDATE, i.e. it wrote
       * GRAM but never touched the panel PROTOCOL state.  Since the
       * panel powers up in Sleep-In + display-off and silently ignores
       * GRAM writes in that state, and the one-shot init in
       * st7789_lcdinitialize() lands before the rail is up on a cold
       * boot, the panel has not been woken since power-on.
       *
       * Order matters: SLPOUT (with its 120 ms settle) -> NORON -> DISPON
       * -> then push a full frame, so the first thing GRAM receives is a
       * complete image rather than power-on garbage. */
      /* [V800] Five passes at the initial 2 s period is 10 s, comfortably past
       * the 120 ms SLPOUT settle plus NORON and DISPON, and past the window in
       * which the panel rail comes up on a cold boot. */
      /* [V827] The v800 block that used to sit here is gone.
       *
       * There were two independent wake sequences running at once, both of them
       * re-issuing SLPOUT + NORON + DISPON during the first seconds after boot:
       *
       *   v800_settle_left = 5, decrementing every 2 s
       *   v819 wake_attempts, five tries with a growing delay
       *
       * v819's bound never covered v800's counter, so removing one of them left
       * the other running.  A capture of the black screen shows the result --
       * twenty-two setpower() calls and not one "[V819] 5 wake attempts done",
       * i.e. the bounded sequence never reached its own stop condition while the
       * v800 one kept firing underneath it.
       *
       * That is the flicker and it is worse than the original black screen:
       * DISPON resets the panel's display state, so re-issuing the sequence
       * every couple of seconds discards whatever the UI has drawn.  The panel
       * never settles long enough to show a frame.
       *
       * v819's sequence replaces it: same intent (cover the window in which the
       * panel rail comes up), a bound that is actually reached, and delays that
       * grow instead of a fixed 2 s so the retries spread across the window
       * rather than bunching at its start.
       */
      v800_settle_left = 0;

      /* [V892] DISPON-only keep-alive.
       *
       * Placement: this sits in the tail of the while(1) loop, which runs to
       * completion on EVERY iteration.  The `if (wake_attempts < ...)`
       * branch above sleeps and then falls through to here, and the trailing
       * sleep(beat < 60 ? 2 : 6) is still below, so this neither skips a
       * beat nor shortens one.
       *
       * It therefore fires once per beat for the whole life of the task:
       * during the first V819_WAKE_ATTEMPTS beats right after
       * g_lcd->setpower(g_lcd, 1) (redundant but harmless - DISPON is
       * idempotent), and after the window as the ONLY thing the loop puts on
       * the wire.  No SLPOUT anywhere in this call, so no 120 ms blank and
       * no flicker; DISPON is still re-asserted, so the panel cannot drift
       * back to display-off.
       */

      if (g_lcd != NULL)
        {
          extern int st7789_display_on_only(FAR struct lcd_dev_s *dev);

          /* [V950] Split on /dev/fb0, not on a timer.
           *
           * Before fb0 exists the panel may come up at any moment
           * (the power key is a hardware rail latch), and a panel that
           * just came up is in SLEEP-IN -- where DISPON is silently
           * ignored.  Only SLPOUT gets it out of there, so in this
           * window every beat issues the full setpower(1).
           *
           * After fb0 exists the UI is painting and the panel is
           * demonstrably awake, so DISPON alone is enough and there is
           * no 120 ms blank to flicker through. */

          if (fd < 0)
            {
              static int s_v950_n;

              g_lcd->setpower(g_lcd, 1);

              if ((++s_v950_n % 6) == 0)
                {
                  extern int st7789_reconfigure(
                                         FAR struct lcd_dev_s *dev);

                  /* A panel whose rail dropped forgot COLMOD, MADCTL
                   * and RAMCTL.  Replay them with the wake. */

                  st7789_reconfigure(g_lcd);

                  syslog(LOG_ERR, "[V950] full wake x%d while fb0 is "
                         "absent (SLPOUT+NORON+DISPON+reconfigure)\n",
                         s_v950_n);
                }
            }
          else
            {
              st7789_display_on_only(g_lcd);
            }
        }


      if ((++beat % 15) == 1)
        {
          /* [V765] NORON joins the wake sequence: SLPOUT alone leaves the
           * ST7789 in inversion/partial mode, where DISPON is accepted and
           * GRAM is writable but nothing is displayed.  Executed inside
           * st7789_setpower() (patched in st7789.c); named here so the log
           * line matches what is actually put on the wire. */
          syslog(LOG_ERR, "[V783] panel wake re-assert #%d "
                   "(setpower: SLPOUT+120ms+NORON+DISPON, then FBIO_UPDATE), "
                   "fb=%d\n", beat, fd);
        }

      /* [V760] 2 s for the first ~2 minutes so a power-key press lights the
       * panel quickly, then back off to 6 s.  Every beat pays the 120 ms
       * SLPOUT settle inside st7789_setpower(), so do not go below 2 s. */

      sleep(beat < 60 ? 2 : 6);
    }
}

int board_lcd_initialize(void)
{
  int i;
  /* [V839] ★★★ THE FUNCTION MUST RUN EXACTLY ONCE.
   *
   * Measured on v838: one boot produced THREE "raw bring-up #1/40" and
   * THREE "blkeeper create" lines.  board_lcd_initialize() is called from
   * esp32s3_bringup.c, from graphics/nxmu/nxmu_start.c:95 and from
   * drivers/lcd/lcd_framebuffer.c:690 (through fb_register), and they do
   * not run in sequence.
   *
   * The V550 guard further down -- `if (g_lcd != NULL) return OK;` -- is
   * too late to stop them: g_lcd is assigned at the end of this function,
   * after the SPI setup, so every caller that arrives while the first one
   * is still working sees NULL and falls through anyway.
   *
   * Each of those three passes then started its OWN v771_raw_panel() and
   * its OWN board_lcd_bl_breath().  v771_raw_panel() bit-bangs CS/CLK/
   * MOSI/DC and hands them back to SPI3 afterwards; three of them running
   * on independent schedules means the routing of those four pads at any
   * given moment is a race -- the panel either ends up on the SPI route
   * (picture) or on the GPIO latches (black).  That is the intermittency.
   *
   * So the flag is set on ENTRY, and late callers wait for the first pass
   * to finish rather than duplicating it. */
  static bool s_v839_started;
  static bool s_v839_done;

  if (s_v839_started)
    {
      int w;

      for (w = 0; w < 200 && !s_v839_done; w++)
        {
          up_mdelay(100);                /* up to 20 s */
        }

      syslog(LOG_ERR, "[V839] re-entry suppressed (first pass done=%d); "
             "NOT creating a second rawpanel/blkeeper\n", (int)s_v839_done);

      return s_v839_done ? OK : -EAGAIN;
    }

  s_v839_started = true;
  syslog(LOG_ERR, "[V839] board_lcd_initialize(): first pass, "
         "this thread owns the LCD threads\n");

  /* [V821] Pin the backlight HIGH as the very first thing in this function.
   *
   * Everything about this pad has been argued over for a day -- active-high vs
   * active-low, GPIO vs LEDC, matrix_out before or after configgpio -- while the
   * observation that actually settles it is one line long: v522 ran a five-state
   * demo on boot and a person confirmed the backlight lights on a high level
   * (memory/2026-10-11.md, v517-v523).  v523 then went back to a plain GPIO high
   * and that is the configuration being restored here.
   *
   * Doing it first is the point.  GPIO16 is re-pinned from eight other places
   * (board_lcd.c:354, 373, 672, 787, 1784, 1845, 1855 and the blkeeper loop), and
   * each of them does configgpio() on its own schedule.  Whichever runs last
   * decides what the pad actually does, which makes every other observation
   * conditional on thread timing.
   *
   * The three steps are the v523 order verbatim: unbind from whatever signal was
   * driving the pad, configure it with INPUT|OUTPUT so the input buffer is on
   * (without FUN_IE, gpioread returns 0 regardless of the pad -- see v525), then
   * write the level.
   */
  /* [V825] Release GPIO16 from the RTC domain FIRST, then drive it.
   *
   * This is the one thing v821/v823 missed.  GPIO16 on the ESP32-S3 is
   * XTAL_32K_N, the 32 kHz crystal's negative terminal, and while the RTC pad
   * hold is engaged the pad does not follow the digital IO_MUX -- writing the
   * output latch has no effect on the pin.  The symptom looked like a dead
   * backlight: gpio readback stayed 0 no matter what was written, and the v822
   * elimination build, which consisted of nothing but three lines driving this
   * pad, still reported pad_reads=0.
   *
   * ai_vox3_bl_force_gpio() already does exactly this and its comment names the
   * trap: "GPIO16 stays in the RTC domain and shut the oscillator down before
   * the pad is driven as a push-pull output" -- the 32 kHz pad has to be handed
   * back to the digital side before it can be used at all.  It was called a few
   * hundred lines further down ([V626]), which is after everything above had
   * already tried to light the backlight, so it ran too late to help them.
   *
   * Calling it here puts it ahead of every other attempt.
   */
  ai_vox3_bl_force_gpio();

  esp32s3_gpio_matrix_out(ESP32S3_EYE_DISPLAY_BCKL,
                          SIG_GPIO_OUT_IDX, 0, 0);
  esp32s3_configgpio(ESP32S3_EYE_DISPLAY_BCKL, AI_VOX3_BCKL_MODE);
  esp32s3_gpiowrite(ESP32S3_EYE_DISPLAY_BCKL, 1);

  syslog(LOG_ERR,
         "[V825] backlight after RTC release: wrote 1, pad reads %d "
         "(0 here means the pad is still held and the pin is not the backlight "
         "yet)\n",
         esp32s3_gpioread(ESP32S3_EYE_DISPLAY_BCKL));

  /* [V822] Elimination build: stop here.  No SPI, no panel sequence, no
   * framebuffer, no UI, no WiFi.  One pad, driven high, held.
   *
   * Returning without initialising the panel is safe for this experiment --
   * fb_register() calls in, gets a non-NULL g_lcd (set just below by the
   * guard's own assignment path) and the rest of the stack proceeds harmlessly;
   * nothing will draw, which is the point.  If the backlight glows, every
   * measurement available from software has already passed and the fault is in
   * the panel or the wiring, not in this firmware.
   *
   * This replaces, rather than adds to, the previous behaviour -- v819's bounded
   * wake retries and v820's decoupling from WiFi both stay in the tree but are
   * never reached. */
  /* [V823] Full initialisation resumes here.
   *
   * v822 returned straight after pinning the pad, and it answered its question:
   * the backlight follows the USB power rail, not the firmware.  Pulling USB and
   * powering up again leaves the panel unpowered, which the code's own note
   * explains -- the power key is a hardware rail latch, the press powers the
   * panel, and the panel then needs a fresh SLPOUT + DISPON to show anything.
   *
   * So the diagnostic build is over.  What is worth keeping from it is the
   * ordering it established: the backlight is pinned first, before SPI3 and
   * before the panel sequence, so nothing later can quietly re-pin the pad.
   *
   * The wake retries added by v819 are what make a key-press recovery work --
   * five attempts spread over roughly 30 s, which covers the moment the rail
   * comes up.  Without the panel being powered there is nothing for any of this
   * to do, which is a hardware fact rather than a firmware one.
   */
  syslog(LOG_ERR,
         "[V823] full build: backlight pinned (pad readback=%d), continuing "
         "with SPI3 and the panel sequence\n",
         esp32s3_gpioread(ESP32S3_EYE_DISPLAY_BCKL));

  /* [V550] ★★★ IDEMPOTENCY GUARD -- this is the fix for "亮了一下又灭了".
   *
   * board_lcd_initialize() is reached through TWO independent paths:
   *   1) esp32s3_bringup.c  -> board_lcd_initialize()
   *   2) fb_register()      -> drivers/lcd/lcd_framebuffer.c:689
   *                           -> board_lcd_initialize()   (framework call)
   * Both run on every boot, so the whole body executed twice.  Each run
   * created its own "lcdselftest" kthread (pid 9 AND pid 10), and both
   * threads then wrote full 240x240 frames to the SAME SPI3 bus while
   * re-asserting the backlight -- two writers on one bus = the panel
   * visibly lights up, blanks, lights up, blanks.  The second run also
   * re-issued SWRESET(0x01) to an already-initialised panel.
   *
   * Make the function idempotent: the first call does the real work, any
   * later call returns OK immediately. */
  if (g_lcd != NULL)
    {
      
      return OK;
    }

  /* Initialize non-SPI GPIOs */

  /* [V628] RTC slow-clock audit + fix.  MUST run before the first touch of
   * GPIO15 (SPI3 CS) or GPIO16 (backlight) - both are 32 kHz crystal pads. */
#if V628_LOG_SLOWCLK
  LCDDIAG("[V628] RTC_CLK_CONF=0x%08x ANA_CLK_RTC_SEL=%u "
          "(0=RC_SLOW 1=XTAL32K 2=8M/256) DIG_XTAL32K_EN=%u",
          v628_rtc_clk_conf(), v628_slow_sel(),
          (unsigned)((v628_rtc_clk_conf() >> 8) & 0x1u));
#endif

#if V628_FORCE_RC_SLOW
  {
    volatile uint32_t *clkcf = (volatile uint32_t *)0x60008074u;
    unsigned prev = v628_slow_sel();

    if (prev != 0u)
      {
        /* ANA_CLK_RTC_SEL = 0 (RC_SLOW) and stop the digital 32 kHz
         * oscillator, exactly as IDF's rtc_clk_slow_freq_set() +
         * rtc_clk_xtal32k_disable() pair does. */
        *clkcf = (*clkcf & ~(0x3u << 30)) & ~(1u << 8);
        
      }
    else
      {
        
      }
  }
#endif

  /* [V548] Print the REAL display-bus pins (SPI3, per the vendor example).
   * The old line printed SPI3 too but those values were still the Kconfig
   * defaults (cs=10 clk=6 mosi=7 miso=2) because the defconfig never set
   * them.  Now they really are cs=15 clk=17 mosi=21 miso=-1. */
  LCDDIAG("pins: DC=%d BCKL=%d SPI=%d | SPI3 cs=%d clk=%d mosi=%d miso=%d",
          ESP32S3_EYE_DISPLAY_DC, ESP32S3_EYE_DISPLAY_BCKL,
          ESP32S3_EYE_DISPLAY_SPI,
          CONFIG_ESP32S3_SPI3_CSPIN, CONFIG_ESP32S3_SPI3_CLKPIN,
          CONFIG_ESP32S3_SPI3_MOSIPIN, CONFIG_ESP32S3_SPI3_MISOPIN);

  esp32s3_configgpio(ESP32S3_EYE_DISPLAY_DC, OUTPUT);

  /* [V766] ★★ 背光先拉高，再初始化面板 —— 对齐小智 InitDisplay 的第一行。
   *
   * 小智 VOX3/ai_vox/ai_vox-main/examples/ai-vox3/ai-vox3/main.cpp:123-124：
   *
   *     pinMode(kLcdBacklightPin, OUTPUT);
   *     analogWrite(kLcdBacklightPin, 255);
   *
   * 然后才 spi_bus_initialize / esp_lcd_new_panel_io_spi /
   * esp_lcd_panel_init。也就是说面板是在背光已建立的状态下接受
   * 初始化命令的。
   *
   * 本板原来相反：先 configgpio(SPI 引脚) + st7789_lcdinitialize()，
   * 背光交给后面的 blkeeper 线程拉高。面板的 charge pump 与
   * backlight 的关系随面板批次而异，但既然同一块板子上的小智固件
   * 用这个顺序工作，就没有理由不照抄。
   *
   * 只写一次电平，不改 keeper 的 2 s 重钉逻辑 —— keeper 仍然每 2 s
   * 重新 configgpio + write(1)，覆盖这里的初值。 */

  esp32s3_configgpio(ESP32S3_EYE_DISPLAY_BCKL, AI_VOX3_BCKL_MODE);
  esp32s3_gpiowrite(ESP32S3_EYE_DISPLAY_BCKL, 1);
  LCDDIAG("[V766] backlight HIGH BEFORE panel init (matches xiaozhi"
          " InitDisplay order)");

  /* [V767] 背光极性 A/B 已启用：keeper 每 3 s 翻转一次，日志 [V767] bl
   * phase=HIGH / LOW 成对出现。这一版面板初始化序列与 v766 完全相同，
   * 唯一区别就是这个引脚的电平在 LOW/HIGH 之间交替。
   *
   * 判读表：
   *   「LOW 相亮、HIGH 相灭」  => ACTIVE-LOW，把 1 改成 0 收工
   *   「HIGH 相亮、LOW 相灭」  => ACTIVE-HIGH（现状即正确）
   *   两相都不亮              => 背光与面板同时另有问题（换口径）
   *   两相都亮 / 看不出相位    => 背光不是阻塞点，回到面板输出
   *
   * 注：v679 的 "bl pin reads LOW" ERR 在 LOW 相是预期中的假警，不影响
   * 判读，故本轮不碰它 —— 一个变量一轮，不夹带无关改动。 */
  v770_dump_route("post-init");

  LCDDIAG("[V770] gpio-matrix route dumped (CS/CLK/MOSI OUT_SEL read back)"
          " -- panel init identical to v768");
  LCDDIAG("[V768] STATIC PANEL A/B: panel wake watchdog disabled"
          " (no more per-2s setpower), backlight fixed HIGH, init once");

  /* [V546] Snapshot the GPIO16 chain BEFORE) any board code touches it --
   * this reveals which peripheral driver (if any) has rerouted the pad. */
  ai_vox3_bl_dump16("pre16");

  /* [V548] BACKLIGHT: plain GPIO output, driven HIGH, then a slow 3x toggle
   * with a REAL read-back at each step.
   *
   * AUTHORITATIVE EVIDENCE -- the one build verified on real hardware:
   *   C:\Users\xxzx\Desktop\VOX3\backlight_test\backlight_test.ino
   *       pinMode(PIN_LCD_BL, OUTPUT);
   *       digitalWrite(PIN_LCD_BL, HIGH);      // plain GPIO, HIGH = ON
   *
   * The pin mode used here is INPUT|OUTPUT rather than bare OUTPUT: only the
   * INPUT flag makes esp32s3_configgpio() set FUN_IE, and without FUN_IE
   * esp32s3_gpioread() always returns 0 regardless of the actual pad level.
   * That is why every previous "BCKL=0" log line was meaningless.
   *
   * The toggle loop below is the decisive test: if the read-back follows the
   * write (1,0,1,0,...), the pad is genuinely driving the backlight line.
   */
  if (!V625_NO_BACKLIGHT_DRIVE && !V627_DEFER_BACKLIGHT)
    {
      if (V626_XTAL32K_HANDOFF)
        {
          /* [V626] Take GPIO16 out of the RTC 32 kHz domain and shut the
           * oscillator down before the pad is driven as a push-pull output.
           * XTAL_32K_N is GPIO16 on the ESP32-S3, and the WiFi port reads the
           * RTC slow-clock calibration (esp32s3_wifi_adapter.c,
           * RTC_SLOW_CLK_CAL_REG), so a loaded crystal pad shows up as failed
           * authentication timing. */
          ai_vox3_bl_force_gpio();
          ai_vox3_bl_dump16("v626post-force");
        }

      
      /* [V826] The three-round 1/0 blink test is gone.  It was a diagnostic for
       * "does the backlight respond at all", and it runs three times at boot --
       * which on a board where the backlight now works reads as a flicker. */
      for (i = 0; i < 0; i++)
        {
          esp32s3_configgpio(ESP32S3_EYE_DISPLAY_BCKL, AI_VOX3_BCKL_MODE);
          esp32s3_gpiowrite(ESP32S3_EYE_DISPLAY_BCKL, 1);
          up_mdelay(50);
          esp32s3_gpiowrite(ESP32S3_EYE_DISPLAY_BCKL, 1);
          up_mdelay(50);
        }

      /* Leave it ON (ACTIVE-HIGH). */
      esp32s3_configgpio(ESP32S3_EYE_DISPLAY_BCKL, AI_VOX3_BCKL_MODE);
      esp32s3_gpiowrite(ESP32S3_EYE_DISPLAY_BCKL, 1);
      
      ai_vox3_bl_dump16("post16");
    }
  else if (V627_DEFER_BACKLIGHT)
    {
      /* [V627] GPIO16 stays in its reset state for now; the [V548] drive is
       * handed to v627_bckl_waiter(), which asserts the backlight only once
       * /tmp/net.txt carries a real address - i.e. after association and
       * DHCP have both finished. */
      
    }
  else
    {
      /* [V625] GPIO16 is left completely untouched (reset state = input, no
       * drive).  The panel is initialised but dark. */
      
    }

  LCDDIAG("DC(%d) level=%d, BCKL(%d) level=%d",
          ESP32S3_EYE_DISPLAY_DC,
          (int)esp32s3_gpioread(ESP32S3_EYE_DISPLAY_DC),
          ESP32S3_EYE_DISPLAY_BCKL,
          (int)esp32s3_gpioread(ESP32S3_EYE_DISPLAY_BCKL));

  g_spidev = esp32s3_spibus_initialize(ESP32S3_EYE_DISPLAY_SPI);
  if (!g_spidev)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n",
             ESP32S3_EYE_DISPLAY_SPI);
      return -ENODEV;
    }

  LCDDIAG("spibus_initialize(%d)=%p", ESP32S3_EYE_DISPLAY_SPI,
          (FAR void *)g_spidev);

  /* Re-claim DC: SPI bus init reconfigures GPIO14 as MISO (LCD needs DC). */
  esp32s3_configgpio(ESP32S3_EYE_DISPLAY_DC, OUTPUT);

  LCDDIAG("after re-claim DC: DC(%d) level=%d", ESP32S3_EYE_DISPLAY_DC,
          (int)esp32s3_gpioread(ESP32S3_EYE_DISPLAY_DC));

  /* [V552] ★★★ SOFTWARE CS -- the real reason the panel stayed blank even
   * with the backlight proven on.
   *
   * esp32s3_spi_select() body is:
   *     #if SPI_HAVE_SWCS
   *       if (cs_pin >= 0) esp32s3_gpiowrite(cs_pin, !selected);
   *     #endif
   * and SPI_HAVE_SWCS is 1 only when CONFIG_ESP32S3_SPI_SWCS is set.  It was
   * NOT set, so the whole function compiled to a no-op: the ST7789 driver's
   * SPI_SELECT() calls did nothing, CS stayed under the SPI3 hardware's
   * automatic control (which releases CS after each byte group), and every
   * command/data pair was mis-framed.  Backlight on, panel blank -- exactly
   * the symptom.
   *
   * With CONFIG_ESP32S3_SPI_SWCS=y the driver owns CS as a plain GPIO:
   *   esp32s3_spi_init() does configgpio(cs, OUTPUT_FUNCTION_1) +
   *   gpio_matrix_out(cs, SIG_GPIO_OUT_IDX, 0, 0), and
   *   esp32s3_spi_select() then simply gpiowrite()s it.
   *
   * Therefore we must NOT attach SPI3_CS0_OUT_IDX to the CS pin here -- that
   * would steal it back from the GPIO path and break software CS.  Only the
   * CLK and MOSI lines stay on the SPI3 hardware signals.
   *
   * SPI3 signal indices (hardware/esp32s3_gpio_sigmap.h):
   *   SPI3_CLK_OUT_IDX = 66, SPI3_D_OUT_IDX = 68.
   */
  esp32s3_configgpio(CONFIG_ESP32S3_SPI3_CLKPIN, OUTPUT);
  esp32s3_configgpio(CONFIG_ESP32S3_SPI3_MOSIPIN, OUTPUT);
  esp32s3_gpio_matrix_out(CONFIG_ESP32S3_SPI3_CLKPIN, SPI3_CLK_OUT_IDX, 0, 0);
  esp32s3_gpio_matrix_out(CONFIG_ESP32S3_SPI3_MOSIPIN, SPI3_D_OUT_IDX, 0, 0);

  /* CS: driven by software (SPI_SELECT -> gpiowrite).  A plain GPIO output,
   * idle HIGH (deselected).  The SPI driver already routed it; re-assert the
   * GPIO function and the idle level so the first transfer starts clean. */
  esp32s3_configgpio(CONFIG_ESP32S3_SPI3_CSPIN, OUTPUT);
  esp32s3_gpio_matrix_out(CONFIG_ESP32S3_SPI3_CSPIN, SIG_GPIO_OUT_IDX, 0, 0);
  esp32s3_gpiowrite(CONFIG_ESP32S3_SPI3_CSPIN, 1);

  esp32s3_configgpio(ESP32S3_EYE_DISPLAY_DC, OUTPUT);

  

  g_lcd = st7789_lcdinitialize(g_spidev);
  if (!g_lcd)
    {
      lcderr("ERROR: st7789_lcdinitialize() failed\n");
      return -ENODEV;
    }

  LCDDIAG("st7789_lcdinitialize()=%p", (FAR void *)g_lcd);

  /* [V771] Electrical A/B: bypass the whole NuttX SPI stack and bit-bang the
   * panel directly.  If this lights it, the fault is in software; if it does
   * not, no further driver work can help. */
  {
    int rid;

    /* [V826] rawpanel is no longer started.
     *
     * It detaches CS/CLK/MOSI/DC from the GPIO matrix and drives them in
     * software to fill GRAM with white.  That was written when the UI could not
     * reach the panel at all.  Now that it can -- v824 registered the LVGL
     * display driver and fb_updates climbs past 2 -- the same thread fights the
     * UI for one SPI bus, and the two interleavings are what the backlight
     * flicker on screen is: white fill, UI frame, wake re-assert, repeat.
     *
     * The v812 bound is left in place for the case where the UI genuinely cannot
     * get a framebuffer, but the thread is not created.
     */
    if (1)
      {
        rid = kthread_create("rawpanel", 100, 8192,
                             (main_t)v771_raw_panel, NULL);
      }
    syslog(LOG_ERR, "[V780] raw bring-up thread pid=%d (1 round, no SWRESET, panelwake is the real waker, "
           "then SPI route restored for /dev/fb0)\n", rid);
  }

  /* [V776] Measure the backlight pad instead of assuming it.  Runs once at
   * t+4 s, restores the usual OUTPUT HIGH before returning, and never
   * touches the panel. */
  {
    int bid;

  if (0) {
  /* [V778] blprobe DISABLED: it drove GPIO16 every 300 ms and
   * polluted the very pad it measured -- v777 Q4 read HIGH
   * only because the probe itself was re-asserting it. */
    bid = kthread_create("blprobe", 110, 3072, (main_t)v777_bl_probe, NULL);
  }
    syslog(LOG_ERR, "[V777] backlight probe (corrected addr) pid=%d\n", bid);
  }

  

#if AI_VOX3_LCD_SELFTEST
  {
    int spid = kthread_create("lcdselftest", 120, 4096,
                              (main_t)lcd_selftest, NULL);
    
  }
#else
  if (V627_DEFER_BACKLIGHT)
    {
      int dpid;

      /* [V627] The deferred-backlight waiter replaces the keeper: it waits
       * for an address, drives GPIO16, then falls through into
       * board_lcd_bl_breath() and keeps the pad pinned. */
      dpid = kthread_create("bcklwait", 100, 3072,
                            (main_t)v627_bckl_waiter, NULL);
      
    }
  else if (!V625_NO_BACKLIGHT_DRIVE)
    {
      int bpid = -1;

      /* [V550] Backlight keeper: plain GPIO, ACTIVE-HIGH, re-asserted every
       * 2 s.  It does NOT touch the panel data, so /dev/fb0 owns the display
       * alone.  [V625] not started while the gate is on. */
  if (1) { /* [V785] RE-ENABLED */
    /* [V785] The backlight keeper is back, and the v778 note below
     * is superseded.  v778 turned this off with the reason "it put
     * CLK/MOSI back on the SPI3 route every 3 s" -- read the current
     * board_lcd_bl_breath() and that is simply not what it does:
     * the loop body writes only `bl` (GPIO16) and reads it back.
     * No gpio_matrix_out, no CLK, no MOSI.
     *
     * Leaving it off meant GPIO16 was written once by the raw pass
     * round #1 and never re-pinned.  v784 proved the panel side is
     * healthy (white-shot putarea=0, fb=3, the UI ran to "listening")
     * and was still intermittently black on the next cold boot --
     * which is what an unpinned backlight pad looks like.
     * v760, the last confirmed-lit version, ran this thread. */
  /* [V778] blkeeper DISABLED: it re-configured GPIO16 AND put
   * CLK/MOSI back on the SPI3 route every 3 s -- i.e. it was
   * fighting the bit-bang pass it was meant to protect. */
    bpid = kthread_create("blkeeper", 120, 2048,
    (main_t)board_lcd_bl_breath, NULL);
  }
      /* [V785] board_lcd_bl_breath() must reappear in the ELF symbol
       * table.  If `nm nuttx | grep board_lcd_bl_breath` is empty after
       * this build, the gate did not take and the backlight is still
       * unpinned -- do not ship. */
      syslog(LOG_INFO, "[V678] blkeeper create pid=%d\n", bpid);
      
    }
#endif

  /* [V742] Start the panel wake watchdog exactly once.  board_lcd_initialize()
   * is idempotent (see the V550 guard at the top) but it is still reached
   * from two paths, so the flag matters. */

  if (!s_v742_waker_started)
    {
      int wpid;

      s_v742_waker_started = 1;
  if (1) { /* [V780] RE-ENABLED */
      /* [V778] panelwake DISABLED: it issued SLPOUT+NORON+DISPON
       * on its own schedule, so the panel had two independent
       * init authorities racing. */
    wpid = kthread_create("panelwake", 130, 3072,
    (main_t)v742_panel_waker, NULL);
  }
      /* [V780] v778 disabled this thread and concluded "contention is
       * not the cause".  That conclusion was void: v760 PASSED cold boot
       * (2026-10-15.md:291 -- SLPOUT x296, fb=-1 x0) and 2026-10-14.md:292
       * explains why -- the power key is a hardware latch that only
       * restores the rail, and the panel then sits in Sleep-In ignoring
       * everything until one more SLPOUT+DISPON.  THIS THREAD IS THE
       * ONLY THING THAT LIGHTS THE PANEL ON COLD BOOT.  Never disable it
       * without putting an equivalent wake path in place. */
      /* [V783] The wake loop now really does SLPOUT+NORON+DISPON.  Until
       * v783 it had not done so since v768, which is why every version
       * from v768 onward was black on cold boot while the log cheerfully
       * claimed "SLPOUT + NORON + DISPON" on every 2 s beat. */
      syslog(LOG_ERR, "[V760] panel wake watchdog pid=%d "
             "(SLPOUT before DISPON + fb open retry)\n", wpid);
    }

  /* [V839] first pass complete -- release anyone waiting in the
   * entry guard above. */
  s_v839_done = true;

  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 * Input Parameters:
 *   devno - LCD device nmber
 *
 * Returned Value:
 *   LCD device pointer if success or NULL if failed.
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int devno)
{
  if (!g_lcd)
    {
      lcderr("ERROR: Failed to bind SPI port %d to LCD %d\n",
             ESP32S3_EYE_DISPLAY_SPI, devno);
    }
  else
    {
      lcdinfo("SPI port %d bound to LCD %d\n",
              ESP32S3_EYE_DISPLAY_SPI,
              devno);
      return g_lcd;
    }

  return NULL;
}

/****************************************************************************
 * Name:  board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  /* Turn the display off */

  g_lcd->setpower(g_lcd, 0);
}
