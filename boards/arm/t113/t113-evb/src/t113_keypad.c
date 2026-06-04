/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113_keypad.c
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
 * Board-level LRADC resistor-ladder keypad driver for the t113-evb.
 *
 * The board schematic (doc/schematic.txt lines 2210-2213) wires five
 * momentary buttons K3..K7 in a voltage-ladder on the LRADC input:
 *
 *   LRADC -- 51k -- AVCC(1.8V)
 *         +-+-- Rkey -- GND  (one Rkey per button)
 *
 *   K3 VOL+   Rkey=6k8   V ~ 0.21 V   raw ~ 7
 *   K4 VOL-   Rkey=8k2   V ~ 0.41 V   raw ~ 14
 *   K5 MENU   Rkey=10k   V ~ 0.59 V   raw ~ 20
 *   K6 ENTER  Rkey=11k   V ~ 0.75 V   raw ~ 27
 *   K7 HOME   Rkey=13k   V ~ 0.88 V   raw ~ 31
 *
 * Idle (no key pressed) reads ~1.8 V -> raw 63 (LRADC 6-bit full scale).
 * Match tolerance is +/- 2 LSB on each band.  The raw->keycode table is
 * cited in comments against the schematic line numbers above.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/input/keyboard.h>
#include <nuttx/input/virtio-input-event-codes.h>

#include "t113_lradc.h"

#ifdef CONFIG_T113_LRADC_KEYPAD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LRADC_IDLE_THRESHOLD  55   /* raw >= this -> no key pressed */
#define LRADC_NO_KEY          0xffffu

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lradc_key_s
{
  uint8_t  raw_min;
  uint8_t  raw_max;
  uint16_t keycode;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* raw-value bands -> Linux keycodes from nuttx/input/virtio-input-event-
 * codes.h.  Band half-width = 2 LSB (schematic doc/schematic.txt:2210).
 */

static const struct lradc_key_s g_keymap[] =
{
  {  5,  9, KEY_VOLUMEUP   },  /* K3: 6k8  -> raw 7  */
  { 12, 16, KEY_VOLUMEDOWN },  /* K4: 8k2  -> raw 14 */
  { 18, 22, KEY_MENU       },  /* K5: 10k  -> raw 20 */
  { 25, 28, KEY_ENTER      },  /* K6: 11k  -> raw 27 */
  { 29, 33, KEY_HOME       },  /* K7: 13k  -> raw 31 */
};

static struct keyboard_lowerhalf_s g_kbd_lower;
static uint16_t                    g_current_key = LRADC_NO_KEY;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_keypad_classify
 *
 * Description:
 *   Return the keycode matching raw, or LRADC_NO_KEY if raw falls outside
 *   any band.  Idle (raw >= threshold) always returns LRADC_NO_KEY.
 *
 ****************************************************************************/

static uint16_t t113_keypad_classify(uint32_t raw)
{
  unsigned int i;

  if (raw >= LRADC_IDLE_THRESHOLD)
    {
      return LRADC_NO_KEY;
    }

  for (i = 0; i < sizeof(g_keymap) / sizeof(g_keymap[0]); i++)
    {
      if (raw >= g_keymap[i].raw_min && raw <= g_keymap[i].raw_max)
        {
          return g_keymap[i].keycode;
        }
    }

  return LRADC_NO_KEY;
}

/****************************************************************************
 * Name: t113_keypad_isr_hook
 *
 * Description:
 *   LRADC raw-sample hook invoked in interrupt context.  Emits PRESS on
 *   rising edge of a valid keycode and RELEASE on the falling edge.
 *
 ****************************************************************************/

static void t113_keypad_isr_hook(uint32_t status, uint32_t raw,
                                 FAR void *priv)
{
  uint16_t keycode = t113_keypad_classify(raw);

  UNUSED(status);
  UNUSED(priv);

  if (keycode == g_current_key)
    {
      return;
    }

  if (g_current_key != LRADC_NO_KEY)
    {
      keyboard_event(&g_kbd_lower, g_current_key, KEYBOARD_RELEASE);
    }

  if (keycode != LRADC_NO_KEY)
    {
      keyboard_event(&g_kbd_lower, keycode, KEYBOARD_PRESS);
    }

  g_current_key = keycode;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_keypad_initialize
 *
 * Description:
 *   Register /dev/kbd0 and install the LRADC raw-sample hook.  Must be
 *   called after t113_lradc_initialize and after the application has
 *   opened /dev/lradc0 once to arm ao_rxint (or the board may open and
 *   hold the device; see configs/keypad/defconfig).
 *
 ****************************************************************************/

void t113_keypad_initialize(void)
{
  int fd;
  int ret;

  ret = keyboard_register(&g_kbd_lower, "/dev/kbd0", 16);
  if (ret < 0)
    {
      ierr("ERROR: keyboard_register /dev/kbd0 failed: %d\n", ret);
      return;
    }

  t113_lradc_register_hook(t113_keypad_isr_hook, NULL);

  /* Open /dev/lradc0 once and keep the file descriptor open for the
   * lifetime of the system.  The NuttX ADC upper half calls ao_setup
   * and ao_rxint(true) from the first open, which attaches the ISR
   * and unmasks LRADC interrupts.  Without this the keypad hook would
   * never see any samples.
   */

  fd = open("/dev/lradc0", O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      ierr("ERROR: open /dev/lradc0 failed: %d\n", errno);
    }
}

#endif /* CONFIG_T113_LRADC_KEYPAD */
