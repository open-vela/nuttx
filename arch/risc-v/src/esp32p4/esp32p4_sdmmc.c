/****************************************************************************
 * arch/risc-v/src/esp32p4/esp32p4_sdmmc.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/mmcsd.h>

#include "esp_gpio.h"
#include "esp32p4_sdmmc.h"

#include "hal/sdmmc_ll.h"
#include "hal/sdmmc_hal.h"
#include "soc/hp_sys_clkrst_struct.h"
#include "soc/sdmmc_struct.h"
#include "soc/sdmmc_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32P4_SDMMC_SLOT0_CLK  43
#define ESP32P4_SDMMC_SLOT0_CMD  44
#define ESP32P4_SDMMC_SLOT0_D0   39
#define ESP32P4_SDMMC_SLOT0_D1   40
#define ESP32P4_SDMMC_SLOT0_D2   41
#define ESP32P4_SDMMC_SLOT0_D3   42

#define SDMMC_CMD_TIMEOUT_US     500000
#define SDMMC_DATA_TIMEOUT_US    1000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32p4_sdmmc_dev_s
{
  struct sdio_dev_s    sdio;
  int                  slotno;
  uint32_t             clock;
  bool                 widebus;
  uint16_t             blocklen;
  uint32_t             nblocks;
  uint32_t             response[4];
  uint32_t             events;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32p4_sdmmc_dev_s g_sdmmc_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void esp32p4_sdmmc_reset(FAR struct sdio_dev_s *dev)
{
  sdmmc_ll_reset_controller(&SDMMC);
  sdmmc_ll_reset_dma(&SDMMC);
  sdmmc_ll_reset_fifo(&SDMMC);
}

static sdio_capset_t esp32p4_sdmmc_capabilities(FAR struct sdio_dev_s *dev)
{
  return SDIO_CAPS_4BIT;
}

static sdio_statset_t esp32p4_sdmmc_status(FAR struct sdio_dev_s *dev)
{
  return SDIO_STATUS_PRESENT;
}

static void esp32p4_sdmmc_widebus(FAR struct sdio_dev_s *dev, bool enable)
{
  FAR struct esp32p4_sdmmc_dev_s *priv =
    (FAR struct esp32p4_sdmmc_dev_s *)dev;

  priv->widebus = enable;
  sdmmc_ll_set_card_width(&SDMMC, priv->slotno,
                          enable ? SD_BUS_WIDTH_4_BIT :
                                   SD_BUS_WIDTH_1_BIT);
}

static void esp32p4_sdmmc_clock(FAR struct sdio_dev_s *dev,
                                enum sdio_clock_e rate)
{
  FAR struct esp32p4_sdmmc_dev_s *priv =
    (FAR struct esp32p4_sdmmc_dev_s *)dev;
  uint32_t div;

  switch (rate)
    {
      case CLOCK_SDIO_DISABLED:
        sdmmc_ll_enable_card_clock(&SDMMC, priv->slotno, false);
        return;

      case CLOCK_IDMODE:
        div = 100;
        break;

      case CLOCK_MMC_TRANSFER:
      case CLOCK_SD_TRANSFER_1BIT:
      case CLOCK_SD_TRANSFER_4BIT:
        div = 2;
        break;

      default:
        div = 100;
        break;
    }

  sdmmc_ll_set_card_clock_div(&SDMMC, priv->slotno, div);
  sdmmc_ll_enable_card_clock(&SDMMC, priv->slotno, true);
}

static int esp32p4_sdmmc_attach(FAR struct sdio_dev_s *dev)
{
  return OK;
}

static int esp32p4_sdmmc_sendcmd(FAR struct sdio_dev_s *dev,
                                 uint32_t cmd, uint32_t arg)
{
  FAR struct esp32p4_sdmmc_dev_s *priv =
    (FAR struct esp32p4_sdmmc_dev_s *)dev;
  sdmmc_hw_cmd_t hw_cmd;
  uint32_t cmdidx;
  uint32_t rsptype;
  uint32_t data_flags;

  cmdidx     = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  rsptype    = cmd & MMCSD_RESPONSE_MASK;
  data_flags = cmd & MMCSD_DATAXFR_MASK;

  memset(&hw_cmd, 0, sizeof(hw_cmd));
  hw_cmd.card_num = priv->slotno;
  hw_cmd.cmd_index = cmdidx;
  hw_cmd.start_command = 1;

  if (rsptype != MMCSD_NO_RESPONSE)
    {
      hw_cmd.response_expect = 1;
      if (rsptype == MMCSD_R2_RESPONSE)
        {
          hw_cmd.response_long = 1;
        }

      if (rsptype == MMCSD_R1B_RESPONSE)
        {
          hw_cmd.check_response_crc = 1;
        }
    }

  if (data_flags == MMCSD_RDDATAXFR || data_flags == MMCSD_WRDATAXFR)
    {
      hw_cmd.data_expected = 1;
      hw_cmd.rw = (data_flags == MMCSD_WRDATAXFR) ? 1 : 0;
    }

  sdmmc_ll_set_command_arg(&SDMMC, arg);
  sdmmc_ll_set_command(&SDMMC, hw_cmd);

  return OK;
}

#ifdef CONFIG_SDIO_BLOCKSETUP
static void esp32p4_sdmmc_blocksetup(FAR struct sdio_dev_s *dev,
                                     unsigned int blocklen,
                                     unsigned int nblocks)
{
  FAR struct esp32p4_sdmmc_dev_s *priv =
    (FAR struct esp32p4_sdmmc_dev_s *)dev;

  priv->blocklen = (uint16_t)blocklen;
  priv->nblocks  = (uint32_t)nblocks;

  sdmmc_ll_set_block_size(&SDMMC, blocklen);
  sdmmc_ll_set_data_transfer_len(&SDMMC, blocklen * nblocks);
}
#endif

static int esp32p4_sdmmc_recvsetup(FAR struct sdio_dev_s *dev,
                                   FAR uint8_t *buffer,
                                   size_t nbytes)
{
  FAR uint32_t *p = (FAR uint32_t *)buffer;
  size_t words = nbytes / 4;
  size_t i;

  for (i = 0; i < words; i++)
    {
      p[i] = SDMMC.buffifo.val;
    }

  return OK;
}

static int esp32p4_sdmmc_sendsetup(FAR struct sdio_dev_s *dev,
                                   FAR const uint8_t *buffer,
                                   size_t nbytes)
{
  FAR const uint32_t *p = (FAR const uint32_t *)buffer;
  size_t words = nbytes / 4;
  size_t i;

  for (i = 0; i < words; i++)
    {
      SDMMC.buffifo.val = p[i];
    }

  return OK;
}

static int esp32p4_sdmmc_cancel(FAR struct sdio_dev_s *dev)
{
  sdmmc_hw_cmd_t stop_cmd;
  memset(&stop_cmd, 0, sizeof(stop_cmd));
  stop_cmd.stop_abort_cmd = 1;
  stop_cmd.start_command  = 1;
  sdmmc_ll_set_command(&SDMMC, stop_cmd);
  return OK;
}

static int esp32p4_sdmmc_waitresponse(FAR struct sdio_dev_s *dev,
                                      uint32_t cmd)
{
  FAR struct esp32p4_sdmmc_dev_s *priv =
    (FAR struct esp32p4_sdmmc_dev_s *)dev;
  uint32_t status;
  int timeout = SDMMC_CMD_TIMEOUT_US;

  while (timeout > 0)
    {
      status = sdmmc_ll_get_interrupt_raw(&SDMMC);
      if (status & SDMMC_LL_EVENT_CMD_DONE)
        {
          sdmmc_ll_clear_interrupt(&SDMMC, SDMMC_LL_EVENT_CMD_DONE);
          priv->response[0] = SDMMC.resp[0];
          priv->response[1] = SDMMC.resp[1];
          priv->response[2] = SDMMC.resp[2];
          priv->response[3] = SDMMC.resp[3];
          return OK;
        }

      if (status & (SDMMC_LL_EVENT_RTO | SDMMC_LL_EVENT_RESP_ERR))
        {
          sdmmc_ll_clear_interrupt(&SDMMC, status);
          return -EIO;
        }

      up_udelay(10);
      timeout -= 10;
    }

  return -ETIMEDOUT;
}

static int esp32p4_sdmmc_recv_r1(FAR struct sdio_dev_s *dev,
                                 uint32_t cmd,
                                 FAR uint32_t *r1)
{
  FAR struct esp32p4_sdmmc_dev_s *priv =
    (FAR struct esp32p4_sdmmc_dev_s *)dev;

  if (r1 != NULL)
    {
      *r1 = priv->response[0];
    }

  return OK;
}

static int esp32p4_sdmmc_recv_r2(FAR struct sdio_dev_s *dev,
                                 uint32_t cmd,
                                 FAR uint32_t r2[4])
{
  FAR struct esp32p4_sdmmc_dev_s *priv =
    (FAR struct esp32p4_sdmmc_dev_s *)dev;

  if (r2 != NULL)
    {
      r2[0] = priv->response[3];
      r2[1] = priv->response[2];
      r2[2] = priv->response[1];
      r2[3] = priv->response[0];
    }

  return OK;
}

static int esp32p4_sdmmc_recv_r3(FAR struct sdio_dev_s *dev,
                                 uint32_t cmd,
                                 FAR uint32_t *r3)
{
  FAR struct esp32p4_sdmmc_dev_s *priv =
    (FAR struct esp32p4_sdmmc_dev_s *)dev;

  if (r3 != NULL)
    {
      *r3 = priv->response[0];
    }

  return OK;
}

static int esp32p4_sdmmc_recv_r4(FAR struct sdio_dev_s *dev,
                                 uint32_t cmd,
                                 FAR uint32_t *r4)
{
  FAR struct esp32p4_sdmmc_dev_s *priv =
    (FAR struct esp32p4_sdmmc_dev_s *)dev;

  if (r4 != NULL)
    {
      *r4 = priv->response[0];
    }

  return OK;
}

static int esp32p4_sdmmc_recv_r5(FAR struct sdio_dev_s *dev,
                                 uint32_t cmd,
                                 FAR uint32_t *r5)
{
  FAR struct esp32p4_sdmmc_dev_s *priv =
    (FAR struct esp32p4_sdmmc_dev_s *)dev;

  if (r5 != NULL)
    {
      *r5 = priv->response[0];
    }

  return OK;
}

static int esp32p4_sdmmc_recv_r6(FAR struct sdio_dev_s *dev,
                                 uint32_t cmd,
                                 FAR uint32_t *r6)
{
  FAR struct esp32p4_sdmmc_dev_s *priv =
    (FAR struct esp32p4_sdmmc_dev_s *)dev;

  if (r6 != NULL)
    {
      *r6 = priv->response[0];
    }

  return OK;
}

static int esp32p4_sdmmc_recv_r7(FAR struct sdio_dev_s *dev,
                                 uint32_t cmd,
                                 FAR uint32_t *r7)
{
  FAR struct esp32p4_sdmmc_dev_s *priv =
    (FAR struct esp32p4_sdmmc_dev_s *)dev;

  if (r7 != NULL)
    {
      *r7 = priv->response[0];
    }

  return OK;
}

static void esp32p4_sdmmc_waitenable(FAR struct sdio_dev_s *dev,
                                     sdio_eventset_t eventset,
                                     uint32_t timeout)
{
  FAR struct esp32p4_sdmmc_dev_s *priv =
    (FAR struct esp32p4_sdmmc_dev_s *)dev;

  priv->events = eventset;
}

static sdio_eventset_t esp32p4_sdmmc_eventwait(FAR struct sdio_dev_s *dev)
{
  return SDIOWAIT_TRANSFERDONE;
}

static void esp32p4_sdmmc_callbackenable(FAR struct sdio_dev_s *dev,
                                         sdio_eventset_t eventset)
{
}

static int esp32p4_sdmmc_registercallback(FAR struct sdio_dev_s *dev,
                                          worker_t callback,
                                          FAR void *arg)
{
  return OK;
}

#ifdef CONFIG_SDIO_DMA
static bool esp32p4_sdmmc_dmasupported(FAR struct sdio_dev_s *dev)
{
  return false;
}

static int esp32p4_sdmmc_dmarecvsetup(FAR struct sdio_dev_s *dev,
                                      FAR uint8_t *buffer,
                                      size_t buflen)
{
  return -ENOSYS;
}

static int esp32p4_sdmmc_dmasendsetup(FAR struct sdio_dev_s *dev,
                                      FAR const uint8_t *buffer,
                                      size_t buflen)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct sdio_dev_s *esp32p4_sdmmc_init(int slotno)
{
  FAR struct esp32p4_sdmmc_dev_s *priv = &g_sdmmc_dev;

  memset(priv, 0, sizeof(struct esp32p4_sdmmc_dev_s));
  priv->slotno = slotno;
  nxmutex_init(&priv->sdio.mutex);

  /* Assign SDIO methods */

  priv->sdio.reset            = esp32p4_sdmmc_reset;
  priv->sdio.capabilities     = esp32p4_sdmmc_capabilities;
  priv->sdio.status           = esp32p4_sdmmc_status;
  priv->sdio.widebus          = esp32p4_sdmmc_widebus;
  priv->sdio.clock            = esp32p4_sdmmc_clock;
  priv->sdio.attach           = esp32p4_sdmmc_attach;
  priv->sdio.sendcmd          = esp32p4_sdmmc_sendcmd;
#ifdef CONFIG_SDIO_BLOCKSETUP
  priv->sdio.blocksetup       = esp32p4_sdmmc_blocksetup;
#endif
  priv->sdio.recvsetup        = esp32p4_sdmmc_recvsetup;
  priv->sdio.sendsetup        = esp32p4_sdmmc_sendsetup;
  priv->sdio.cancel           = esp32p4_sdmmc_cancel;
  priv->sdio.waitresponse     = esp32p4_sdmmc_waitresponse;
  priv->sdio.recv_r1          = esp32p4_sdmmc_recv_r1;
  priv->sdio.recv_r2          = esp32p4_sdmmc_recv_r2;
  priv->sdio.recv_r3          = esp32p4_sdmmc_recv_r3;
  priv->sdio.recv_r4          = esp32p4_sdmmc_recv_r4;
  priv->sdio.recv_r5          = esp32p4_sdmmc_recv_r5;
  priv->sdio.recv_r6          = esp32p4_sdmmc_recv_r6;
  priv->sdio.recv_r7          = esp32p4_sdmmc_recv_r7;
  priv->sdio.waitenable       = esp32p4_sdmmc_waitenable;
  priv->sdio.eventwait        = esp32p4_sdmmc_eventwait;
  priv->sdio.callbackenable   = esp32p4_sdmmc_callbackenable;
  priv->sdio.registercallback = esp32p4_sdmmc_registercallback;
#ifdef CONFIG_SDIO_DMA
  priv->sdio.dmasupported     = esp32p4_sdmmc_dmasupported;
  priv->sdio.dmarecvsetup     = esp32p4_sdmmc_dmarecvsetup;
  priv->sdio.dmasendsetup     = esp32p4_sdmmc_dmasendsetup;
#endif

  /* 1. Enable SDMMC Core Clock & Bus Clock */

  HP_SYS_CLKRST.soc_clk_ctrl1.reg_sdmmc_sys_clk_en = 1;
  HP_SYS_CLKRST.peri_clk_ctrl01.reg_sdio_ls_clk_en = 1;

  /* 2. Configure IOMUX Pins for Slot 0 */

  esp_configgpio(ESP32P4_SDMMC_SLOT0_CLK, OUTPUT | PULLUP);
  esp_configgpio(ESP32P4_SDMMC_SLOT0_CMD, INPUT | OUTPUT | PULLUP);
  esp_configgpio(ESP32P4_SDMMC_SLOT0_D0,  INPUT | OUTPUT | PULLUP);
  esp_configgpio(ESP32P4_SDMMC_SLOT0_D1,  INPUT | OUTPUT | PULLUP);
  esp_configgpio(ESP32P4_SDMMC_SLOT0_D2,  INPUT | OUTPUT | PULLUP);
  esp_configgpio(ESP32P4_SDMMC_SLOT0_D3,  INPUT | OUTPUT | PULLUP);

  /* 3. Reset SDMMC Controller */

  esp32p4_sdmmc_reset(&priv->sdio);
  esp32p4_sdmmc_clock(&priv->sdio, CLOCK_IDMODE);

  mcinfo("ESP32-P4 SDMMC Slot %d initialized successfully\n", slotno);
  return &priv->sdio;
}

FAR struct sdio_dev_s *sdio_initialize(int slotno)
{
  return esp32p4_sdmmc_init(slotno);
}
