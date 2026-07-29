/****************************************************************************
 * boards/arm/mcx-nxxx/frdm-mcxn947/src/n94x_spi.c
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
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#ifdef CONFIG_SPI_DRIVER
#  include <nuttx/spi/spi_transfer.h>
#endif

#include "nxxx_gpio.h"
#include "n947_lpspi.h"
#include "nxxx_port.h"

#include "frdm-mcxn947.h"

#include <arch/board/board.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_N947_LPSPI0
void n947_lpspi0select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  (void)dev;
  (void)devid;

#ifdef GPIO_LPSPI0_CS
  nxxx_gpio_write(GPIO_LPSPI0_CS, !selected);
#else
  (void)selected;
#endif
}

uint8_t n947_lpspi0status(struct spi_dev_s *dev, uint32_t devid)
{
  (void)dev;
  (void)devid;
  return SPI_STATUS_PRESENT;
}
#endif

/****************************************************************************
 * Name: n94x_spidev_initialize
 *
 * Description:
 *   Initialize the board-visible SPI bus and, when CONFIG_SPI_DRIVER is
 *   enabled, register /dev/spi0 for generic bus testing.
 *
 ****************************************************************************/

int n94x_spidev_initialize(void)
{
#ifdef CONFIG_N947_LPSPI0
  struct spi_dev_s *spi;

#ifdef PORT_LPSPI0_CS
  nxxx_port_configure(PORT_LPSPI0_CS);
#endif

#ifdef GPIO_LPSPI0_CS
  nxxx_config_gpio(GPIO_LPSPI0_CS);
#endif

  spi = n947_lpspibus_initialize(0);
  if (spi == NULL)
    {
      return -ENODEV;
    }

#ifdef CONFIG_SPI_DRIVER
  return spi_register(spi, 0);
#else
  return OK;
#endif

#else
  return -ENODEV;
#endif
}
