/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-korvo-2/src/esp32s3_board_lcd.c
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

#include <debug.h>
#include <errno.h>

#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7789.h>
#include <nuttx/spi/spi.h>

#include "esp32s3_gpio.h"
#include "esp32s3_spi.h"

#include "esp32s3-korvo-2.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spi_dev_s *g_spidev;
static struct lcd_dev_s *g_lcd;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 *
 * Description:
 *   Initialize the ST7789V display connected to SPI2.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  esp32s3_configgpio(KORVO_2_DISPLAY_DC, OUTPUT);
  esp32s3_configgpio(KORVO_2_DISPLAY_BACKLIGHT, OUTPUT);
  esp32s3_gpiowrite(KORVO_2_DISPLAY_BACKLIGHT, true);

  g_spidev = esp32s3_spibus_initialize(KORVO_2_DISPLAY_SPI);
  if (g_spidev == NULL)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n",
             KORVO_2_DISPLAY_SPI);
      return -ENODEV;
    }

  g_lcd = st7789_lcdinitialize(g_spidev);
  if (g_lcd == NULL)
    {
      lcderr("ERROR: st7789_lcdinitialize() failed\n");
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: board_lcd_getdev
 *
 * Description:
 *   Return the LCD object for the requested display number.
 *
 * Input Parameters:
 *   devno - Display number.
 *
 * Returned Value:
 *   LCD device pointer on success; NULL on failure.
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int devno)
{
  if (g_lcd == NULL)
    {
      lcderr("ERROR: Failed to bind SPI port %d to LCD %d\n",
             KORVO_2_DISPLAY_SPI, devno);
      return NULL;
    }

  lcdinfo("SPI port %d bound to LCD %d\n", KORVO_2_DISPLAY_SPI, devno);
  return g_lcd;
}

/****************************************************************************
 * Name: board_lcd_uninitialize
 *
 * Description:
 *   Turn off the LCD.
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  if (g_lcd != NULL)
    {
      g_lcd->setpower(g_lcd, 0);
    }
}
