/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_sdmmc.c
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

#include <stdint.h>
#include <syslog.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/sdio.h>

#include "espressif/esp_gpio.h"

#include "esp32p4-function-ev-board.h"

/* Entry point of the arch-level ESP32-P4 SDMMC/SDIO host driver.  Declared
 * here rather than pulling in the register-heavy chip header, which is not
 * on the board build's include path.
 */

struct sdio_dev_s *esp32p4_sdmmc_sdio_initialize(int slotno);

#ifdef CONFIG_ESP32P4_SDMMC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CCCR / CIS registers (SDIO function 0) used by the Stage-1 probe */

#define SDIO_CCCR_REV            0x00  /* CCCR/SDIO revision */
#define SDIO_CCCR_CISPTR0        0x09  /* Common CIS pointer [7:0] */
#define SDIO_CCCR_CISPTR1        0x0a  /* Common CIS pointer [15:8] */
#define SDIO_CCCR_CISPTR2        0x0b  /* Common CIS pointer [23:16] */

#define CISTPL_END               0xff  /* End-of-chain tuple */
#define CISTPL_MANFID            0x20  /* Manufacturer identification */

#define CIS_WALK_MAX_TUPLES      32    /* Bound the CIS walk */

/* ESP32-C6 boot time after reset is released (esp-hosted slave firmware) */

#define C6_RESET_ASSERT_MS       10
#define C6_BOOT_DELAY_MS         500

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_c6_reset
 *
 * Description:
 *   Reset-cycle the on-board ESP32-C6.  The reset line drives the C6 EN /
 *   CHIP_PU and is ACTIVE-LOW: driving it low holds the C6 in reset and
 *   driving it high releases it to boot.  So we pulse low-then-high and then
 *   wait for the C6 to boot its esp-hosted slave firmware.  (esp-hosted's own
 *   host driver drives this line the same way.)
 *
 ****************************************************************************/

static void board_c6_reset(void)
{
  esp_configgpio(BOARD_C6_RESET_GPIO, OUTPUT);

  /* Assert reset (drive low) */

  esp_gpiowrite(BOARD_C6_RESET_GPIO, false);
  up_mdelay(C6_RESET_ASSERT_MS);

  /* Release reset (drive high) and wait for the C6 to boot */

  esp_gpiowrite(BOARD_C6_RESET_GPIO, true);
  up_mdelay(C6_BOOT_DELAY_MS);
}

/****************************************************************************
 * Name: board_c6_read_cis_manfid
 *
 * Description:
 *   Walk the SDIO common CIS to find the CISTPL_MANFID tuple and read the
 *   16-bit manufacturer (vendor) code and the 16-bit card (device) code.
 *
 * Returned Value:
 *   OK on success (vendor/device populated); a negated errno on failure.
 *
 ****************************************************************************/

static int board_c6_read_cis_manfid(struct sdio_dev_s *dev,
                                     uint16_t *vendor, uint16_t *device)
{
  uint32_t cisptr = 0;
  uint8_t value;
  int ret;
  int i;

  /* Read the 3-byte common CIS pointer from the CCCR */

  ret = sdio_io_rw_direct(dev, false, 0, SDIO_CCCR_CISPTR0, 0, &value);
  if (ret < 0)
    {
      return ret;
    }

  cisptr = value;
  ret = sdio_io_rw_direct(dev, false, 0, SDIO_CCCR_CISPTR1, 0, &value);
  if (ret < 0)
    {
      return ret;
    }

  cisptr |= (uint32_t)value << 8;
  ret = sdio_io_rw_direct(dev, false, 0, SDIO_CCCR_CISPTR2, 0, &value);
  if (ret < 0)
    {
      return ret;
    }

  cisptr |= (uint32_t)value << 16;

  /* Walk the tuple chain looking for CISTPL_MANFID */

  for (i = 0; i < CIS_WALK_MAX_TUPLES; i++)
    {
      uint8_t code;
      uint8_t link;

      ret = sdio_io_rw_direct(dev, false, 0, cisptr, 0, &code);
      if (ret < 0)
        {
          return ret;
        }

      if (code == CISTPL_END)
        {
          return -ENODATA;
        }

      /* Next byte is the link (tuple body length) */

      ret = sdio_io_rw_direct(dev, false, 0, cisptr + 1, 0, &link);
      if (ret < 0)
        {
          return ret;
        }

      if (code == CISTPL_MANFID && link >= 4)
        {
          uint8_t b[4];
          int j;

          for (j = 0; j < 4; j++)
            {
              ret = sdio_io_rw_direct(dev, false, 0, cisptr + 2 + j, 0,
                                      &b[j]);
              if (ret < 0)
                {
                  return ret;
                }
            }

          *vendor = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
          *device = (uint16_t)b[2] | ((uint16_t)b[3] << 8);
          return OK;
        }

      /* Advance to the next tuple: code + link + body */

      cisptr += 2 + link;
    }

  return -ENODATA;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_sdmmc_init
 *
 * Description:
 *   Stage-1 esp-hosted bring-up hook: reset the on-board ESP32-C6, bring up
 *   the ESP32-P4 SDMMC host on slot 1 as an SDIO host, start clocking in
 *   1-bit / 400 kHz ID mode, probe the C6 as an SDIO card, and log the
 *   CCCR revision and the CIS manufacturer (vendor) ID over syslog.
 *
 *   This does NOT register a block device (the C6 is an SDIO function
 *   device, not an SD memory card); it only proves the SDIO link.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_sdmmc_init(void)
{
  struct sdio_dev_s *dev;
  uint16_t vendor = 0;
  uint16_t device = 0;
  uint8_t cccr_rev = 0;
  int ret;

  /* Bring up the SDMMC host (slot 1 -> C6 over the GPIO matrix) */

  dev = esp32p4_sdmmc_sdio_initialize(BOARD_SDMMC_SLOT);
  if (dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: esp32p4_sdmmc_sdio_initialize failed\n");
      return -ENODEV;
    }

  /* Attach the SDIO interrupt and start ID-mode clocking (1-bit/400 kHz) */

  ret = SDIO_ATTACH(dev);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: SDIO_ATTACH failed: %d\n", ret);
      return ret;
    }

  SDIO_CLOCK(dev, CLOCK_IDMODE);

  /* Reset-cycle the C6 so it (re)boots the esp-hosted slave firmware while
   * the host clock is already running.
   */

  board_c6_reset();

  /* Probe the C6 as an SDIO card (CMD5 / CMD3 / CMD7 / CCCR read) */

  ret = sdio_probe(dev);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sdio_probe failed: %d\n", ret);
      return ret;
    }

  /* CMD52: read the CCCR/SDIO revision at address 0x00 */

  ret = sdio_io_rw_direct(dev, false, 0, SDIO_CCCR_REV, 0, &cccr_rev);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: CMD52 CCCR read failed: %d\n", ret);
      return ret;
    }

  /* CMD52 chain: read the CIS manufacturer (vendor) ID */

  ret = board_c6_read_cis_manfid(dev, &vendor, &device);
  if (ret < 0)
    {
      syslog(LOG_WARNING,
             "SDIO probe OK, CCCR rev=0x%02x, CIS MANFID unavailable: %d\n",
             cccr_rev, ret);
      return OK;
    }

  syslog(LOG_INFO,
         "esp-hosted SDIO link up: CCCR rev=0x%02x vendor=0x%04x "
         "device=0x%04x\n", cccr_rev, vendor, device);

  return OK;
}

#endif /* CONFIG_ESP32P4_SDMMC */
