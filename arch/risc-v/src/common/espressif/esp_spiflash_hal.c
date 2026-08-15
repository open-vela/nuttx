/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_spiflash_hal.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>

#include <debug.h>

#include "esp_flash.h"
#include "esp_private/esp_flash_internal.h"
#include "esp_spiflash.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int esp_spiflash_result(int ret)
{
  if (ret != ESP_OK)
    {
      ferr("ERROR: ESP Flash HAL operation failed: %d\n", ret);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int spi_flash_read(uint32_t address, void *buffer, uint32_t length)
{
  return esp_spiflash_result(esp_flash_read(NULL, buffer, address, length));
}

int spi_flash_erase_range(uint32_t address, uint32_t length)
{
  return esp_spiflash_result(esp_flash_erase_region(NULL, address, length));
}

int spi_flash_write(uint32_t address, const void *buffer, uint32_t length)
{
  return esp_spiflash_result(esp_flash_write(NULL, buffer, address, length));
}

int esp_spiflash_init(void)
{
  int ret;

  if (esp_flash_default_chip == NULL ||
      !esp_flash_chip_driver_initialized(esp_flash_default_chip))
    {
      ret = esp_flash_app_init();
      if (ret != ESP_OK)
        {
          ferr("ERROR: failed to initialize ESP Flash OS hooks: %d\n", ret);
          return -EIO;
        }

      ret = esp_flash_init_default_chip();
      if (ret != ESP_OK)
        {
          ferr("ERROR: failed to initialize default ESP Flash: %d\n", ret);
          return -ENODEV;
        }
    }

  return OK;
}
