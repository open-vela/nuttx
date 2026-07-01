/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_sdmmc.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "arm64_internal.h"
#include "hardware/rk3576_sdmmc.h"
#include "rk3576_sdmmc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SDMMC timeout (microseconds) */

#define SDMMC_TIMEOUT_US           1000000
#define SDMMC_CMD_TIMEOUT_US       100000
#define SDMMC_DATA_TIMEOUT_US      500000

/* Source clock (from CRU) */

#define SDMMC_SRC_CLK_HZ          24000000

/* FIFO depth (in bytes) */

#define SDMMC_FIFO_DEPTH           512

/* Max blocks per transfer */

#define SDMMC_MAX_BLOCKS           255

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SDMMC controller base addresses */

const uint32_t g_sdmmc_base[RK3576_SDMMC_COUNT] =
{
  RK3576_SDMMC0_ADDR,
  RK3576_SDMMC1_ADDR,
  RK3576_EMMC_ADDR,
};

/* Per-controller state */

struct rk3576_sdmmc_dev_s
{
  int card_type;                /* Card type (none/mmc/sd/sdio) */
  bool initialized;             /* Initialized flag */
  uint32_t clock;               /* Current clock frequency */
  uint32_t block_size;          /* Block size */
};

static struct rk3576_sdmmc_dev_s g_sdmmc_dev[RK3576_SDMMC_COUNT];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_sdmmc_validate
 *
 * Description:
 *   Validate controller index.
 *
 ****************************************************************************/

static inline int rk3576_sdmmc_validate(int ctrl)
{
  if (ctrl < 0 || ctrl >= RK3576_SDMMC_COUNT)
    {
      mcerr("SDMMC: invalid controller %d\n", ctrl);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: rk3576_sdmmc_wait_cmd_done
 *
 * Description:
 *   Wait for command to complete.
 *
 ****************************************************************************/

static int rk3576_sdmmc_wait_cmd_done(int ctrl)
{
  uint32_t base = g_sdmmc_base[ctrl];
  int timeout = SDMMC_CMD_TIMEOUT_US;

  while (timeout--)
    {
      uint32_t status = getreg32(base + SDMMC_RINTSTS);

      if (status & SDMMC_INT_CMD_DONE)
        {
          putreg32(SDMMC_INT_CMD_DONE, base + SDMMC_RINTSTS);
          return OK;
        }

      if (status & SDMMC_INT_RTO)
        {
          putreg32(SDMMC_INT_RTO, base + SDMMC_RINTSTS);
          mcerr("SDMMC%d: command timeout\n", ctrl);
          return -ETIMEDOUT;
        }

      if (status & SDMMC_INT_RE)
        {
          putreg32(SDMMC_INT_RE, base + SDMMC_RINTSTS);
          mcerr("SDMMC%d: response error\n", ctrl);
          return -EIO;
        }

      if (status & SDMMC_INT_HLE)
        {
          putreg32(SDMMC_INT_HLE, base + SDMMC_RINTSTS);
          mcerr("SDMMC%d: hardware lock error\n", ctrl);
          return -EBUSY;
        }

      up_udelay(1);
    }

  mcerr("SDMMC%d: wait cmd done timeout\n", ctrl);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_sdmmc_wait_data_done
 *
 * Description:
 *   Wait for data transfer to complete.
 *
 ****************************************************************************/

static int rk3576_sdmmc_wait_data_done(int ctrl)
{
  uint32_t base = g_sdmmc_base[ctrl];
  int timeout = SDMMC_DATA_TIMEOUT_US;

  while (timeout--)
    {
      uint32_t status = getreg32(base + SDMMC_RINTSTS);

      if (status & SDMMC_INT_DATA_OVER)
        {
          putreg32(SDMMC_INT_DATA_OVER, base + SDMMC_RINTSTS);
          return OK;
        }

      if (status & SDMMC_INT_DCRC)
        {
          putreg32(SDMMC_INT_DCRC, base + SDMMC_RINTSTS);
          mcerr("SDMMC%d: data CRC error\n", ctrl);
          return -EIO;
        }

      if (status & SDMMC_INT_DRTO)
        {
          putreg32(SDMMC_INT_DRTO, base + SDMMC_RINTSTS);
          mcerr("SDMMC%d: data timeout\n", ctrl);
          return -ETIMEDOUT;
        }

      if (status & SDMMC_INT_FRUN)
        {
          putreg32(SDMMC_INT_FRUN, base + SDMMC_RINTSTS);
          mcerr("SDMMC%d: FIFO underrun/overrun\n", ctrl);
          return -EIO;
        }

      up_udelay(1);
    }

  mcerr("SDMMC%d: wait data done timeout\n", ctrl);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_sdmmc_send_cmd
 *
 * Description:
 *   Send a command to the SDMMC controller.
 *
 ****************************************************************************/

static int rk3576_sdmmc_send_cmd(int ctrl, uint32_t cmd, uint32_t arg)
{
  uint32_t base = g_sdmmc_base[ctrl];
  int ret;

  /* Wait for command interface to be idle */

  {
    int timeout = 1000;
    while (timeout--)
      {
        if (!(getreg32(base + SDMMC_CMD) & SDMMC_CMD_START))
          {
            break;
          }

        up_udelay(1);
      }
  }

  /* Clear pending interrupts */

  putreg32(SDMMC_INT_ALL, base + SDMMC_RINTSTS);

  /* Set command argument */

  putreg32(arg, base + SDMMC_CMDARG);

  /* Send command */

  putreg32(cmd | SDMMC_CMD_START, base + SDMMC_CMD);

  /* Wait for command done */

  ret = rk3576_sdmmc_wait_cmd_done(ctrl);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: rk3576_sdmmc_send_cmd_no_resp
 *
 * Description:
 *   Send a command without expecting a response.
 *
 ****************************************************************************/

static int rk3576_sdmmc_send_cmd_no_resp(int ctrl, uint32_t cmd, uint32_t arg)
{
  return rk3576_sdmmc_send_cmd(ctrl, cmd, arg);
}

/****************************************************************************
 * Name: rk3576_sdmmc_send_cmd_r1
 *
 * Description:
 *   Send a command expecting R1 response.
 *
 ****************************************************************************/

static int rk3576_sdmmc_send_cmd_r1(int ctrl, uint32_t cmd, uint32_t arg,
                                     uint32_t *resp)
{
  int ret;

  cmd |= SDMMC_CMD_RESP_EXP | SDMMC_CMD_CHECK_RESP_CRC;
  ret = rk3576_sdmmc_send_cmd(ctrl, cmd, arg);
  if (ret < 0)
    {
      return ret;
    }

  if (resp)
    {
      *resp = getreg32(g_sdmmc_base[ctrl] + SDMMC_RESP0);
    }

  return OK;
}

/****************************************************************************
 * Name: rk3576_sdmmc_send_cmd_r2
 *
 * Description:
 *   Send a command expecting R2 (long) response.
 *
 ****************************************************************************/

static int rk3576_sdmmc_send_cmd_r2(int ctrl, uint32_t cmd, uint32_t arg,
                                     uint32_t resp[4])
{
  int ret;

  cmd |= SDMMC_CMD_LONG_RESP | SDMMC_CMD_RESP_EXP |
         SDMMC_CMD_CHECK_RESP_CRC;
  ret = rk3576_sdmmc_send_cmd(ctrl, cmd, arg);
  if (ret < 0)
    {
      return ret;
    }

  if (resp)
    {
      resp[0] = getreg32(g_sdmmc_base[ctrl] + SDMMC_RESP0);
      resp[1] = getreg32(g_sdmmc_base[ctrl] + SDMMC_RESP1);
      resp[2] = getreg32(g_sdmmc_base[ctrl] + SDMMC_RESP2);
      resp[3] = getreg32(g_sdmmc_base[ctrl] + SDMMC_RESP3);
    }

  return OK;
}

/****************************************************************************
 * Name: rk3576_sdmmc_send_cmd_r7
 *
 * Description:
 *   Send a command expecting R7 (short) response.
 *
 ****************************************************************************/

static int rk3576_sdmmc_send_cmd_r7(int ctrl, uint32_t cmd, uint32_t arg,
                                     uint32_t *resp)
{
  int ret;

  cmd |= SDMMC_CMD_RESP_EXP;
  ret = rk3576_sdmmc_send_cmd(ctrl, cmd, arg);
  if (ret < 0)
    {
      return ret;
    }

  if (resp)
    {
      *resp = getreg32(g_sdmmc_base[ctrl] + SDMMC_RESP0);
    }

  return OK;
}

/****************************************************************************
 * Name: rk3576_sdmmc_reset_controller
 *
 * Description:
 *   Reset the SDMMC controller.
 *
 ****************************************************************************/

static int rk3576_sdmmc_reset_controller(int ctrl)
{
  uint32_t base = g_sdmmc_base[ctrl];
  int timeout = 1000;

  /* Assert controller reset */

  putreg32(SDMMC_CTRL_CONTROLLER_RESET, base + SDMMC_CTRL);

  /* Wait for reset to complete */

  while (timeout--)
    {
      if (!(getreg32(base + SDMMC_CTRL) & SDMMC_CTRL_CONTROLLER_RESET))
        {
          return OK;
        }

      up_udelay(10);
    }

  mcerr("SDMMC%d: controller reset timeout\n", ctrl);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_sdmmc_init_card
 *
 * Description:
 *   Initialize and identify the card.
 *
 ****************************************************************************/

static int rk3576_sdmmc_init_card(int ctrl)
{
  uint32_t base = g_sdmmc_base[ctrl];
  uint32_t resp;
  int ret;

  /* Send CMD0 (GO_IDLE_STATE) */

  ret = rk3576_sdmmc_send_cmd_no_resp(ctrl,
          SDMMC_CMD_START | SDMMC_CMD_SEND_INIT | SDMMC_CMD_SEND_STOP, 0);
  if (ret < 0)
    {
      return ret;
    }

  /* Send CMD8 (SEND_IF_COND) for SD v2 */

  ret = rk3576_sdmmc_send_cmd_r7(ctrl,
          SDMMC_CMD_START, 0x000001AA, NULL);
  if (ret == OK)
    {
      /* SD v2 card detected */

      mcinfo("SDMMC%d: SD v2 card detected\n", ctrl);
      g_sdmmc_dev[ctrl].card_type = CARD_TYPE_SD;

      /* Send ACMD41 (SD_SEND_OP_COND) */

      for (int i = 0; i < 100; i++)
        {
          /* Send CMD55 (APP_CMD) */

          ret = rk3576_sdmmc_send_cmd_r1(ctrl,
                  SDMMC_CMD_START, 0x00000000, NULL);
          if (ret < 0)
            {
              continue;
            }

          /* Send ACMD41 */

          ret = rk3576_sdmmc_send_cmd_r7(ctrl,
                  SDMMC_CMD_START, 0x40FF8000, NULL);
          if (ret < 0)
            {
              continue;
            }

          resp = getreg32(base + SDMMC_RESP0);
          if (resp & 0x80000000)
            {
              /* Card ready */

              mcinfo("SDMMC%d: SD card ready, OCR=0x%08x\n", ctrl, resp);
              break;
            }

          usleep(10000);
        }
    }
  else
    {
      /* Try MMC or SD v1 */

      mcinfo("SDMMC%d: trying MMC/SD v1\n", ctrl);

      /* Send CMD0 again */

      rk3576_sdmmc_send_cmd_no_resp(ctrl,
          SDMMC_CMD_START | SDMMC_CMD_SEND_INIT | SDMMC_CMD_SEND_STOP, 0);

      /* Send CMD1 (SEND_OP_COND) for MMC */

      for (int i = 0; i < 100; i++)
        {
          ret = rk3576_sdmmc_send_cmd_r1(ctrl,
                  SDMMC_CMD_START, 0x40FF8000, NULL);
          if (ret < 0)
            {
              continue;
            }

          resp = getreg32(base + SDMMC_RESP0);
          if (resp & 0x80000000)
            {
              mcinfo("SDMMC%d: MMC card ready\n", ctrl);
              g_sdmmc_dev[ctrl].card_type = CARD_TYPE_MMC;
              break;
            }

          usleep(10000);
        }
    }

  /* Send CMD2 (ALL_SEND_CID) */

  uint32_t cid[4];
  ret = rk3576_sdmmc_send_cmd_r2(ctrl,
          SDMMC_CMD_START, 0x00000000, cid);
  if (ret < 0)
    {
      mcerr("SDMMC%d: CMD2 failed\n", ctrl);
      return ret;
    }

  mcinfo("SDMMC%d: CID = %08x %08x %08x %08x\n",
          ctrl, cid[0], cid[1], cid[2], cid[3]);

  /* Send CMD3 (SET_RELATIVE_ADDR) */

  ret = rk3576_sdmmc_send_cmd_r1(ctrl,
          SDMMC_CMD_START, 0x00000000, NULL);
  if (ret < 0)
    {
      mcerr("SDMMC%d: CMD3 failed\n", ctrl);
      return ret;
    }

  /* Send CMD9 (SEND_CSD) */

  uint32_t csd[4];
  ret = rk3576_sdmmc_send_cmd_r2(ctrl,
          SDMMC_CMD_START, 0x00000000, csd);
  if (ret >= 0)
    {
      mcinfo("SDMMC%d: CSD = %08x %08x %08x %08x\n",
              ctrl, csd[0], csd[1], csd[2], csd[3]);
    }

  /* Send CMD7 (SELECT_CARD) */

  ret = rk3576_sdmmc_send_cmd_r1(ctrl,
          SDMMC_CMD_START, 0x00000000, NULL);
  if (ret < 0)
    {
      mcwarn("SDMMC%d: CMD7 failed (non-critical)\n", ctrl);
    }

  /* Set block size to 512 bytes */

  ret = rk3576_sdmmc_send_cmd_r1(ctrl,
          SDMMC_CMD_START | SDMMC_CMD_DATA_EXPECT, 0x200, NULL);
  if (ret < 0)
    {
      mcwarn("SDMMC%d: SET_BLOCKLEN failed\n", ctrl);
    }

  g_sdmmc_dev[ctrl].block_size = 512;

  mcinfo("SDMMC%d: card initialized, type=%d\n",
          ctrl, g_sdmmc_dev[ctrl].card_type);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_sdmmc_init
 *
 * Description:
 *   Initialize SDMMC controller and detect card.
 *
 ****************************************************************************/

int rk3576_sdmmc_init(int ctrl)
{
  uint32_t base;
  int ret;

  ret = rk3576_sdmmc_validate(ctrl);
  if (ret < 0)
    {
      return ret;
    }

  base = g_sdmmc_base[ctrl];

  /* Initialize state */

  memset(&g_sdmmc_dev[ctrl], 0, sizeof(struct rk3576_sdmmc_dev_s));
  g_sdmmc_dev[ctrl].card_type = CARD_TYPE_NONE;
  g_sdmmc_dev[ctrl].block_size = 512;

  /* Reset controller */

  ret = rk3576_sdmmc_reset_controller(ctrl);
  if (ret < 0)
    {
      return ret;
    }

  /* Reset FIFO */

  putreg32(SDMMC_CTRL_FIFO_RESET, base + SDMMC_CTRL);
  usleep(100);

  /* Reset DMA */

  putreg32(SDMMC_CTRL_DMA_RESET, base + SDMMC_CTRL);
  usleep(100);

  /* Enable power */

  putreg32(1, base + SDMMC_PWREN);

  /* Set clock to 400kHz for card identification */

  rk3576_sdmmc_set_clock(ctrl, SDMMC_CLK_400KHZ);

  /* Enable clock */

  putreg32(SDMMC_CLKENA_CCLK_EN, base + SDMMC_CLKENA);

  /* Set FIFO threshold */

  putreg32((0x3 << SDMMC_FIFOTH_DWIDTH_SHIFT) |
           (0x3 << 16) |    /* TX trigger level */
           (0x7 << 0),      /* RX trigger level */
           base + SDMMC_FIFOTH);

  /* Enable all interrupts */

  putreg32(SDMMC_INT_ALL, base + SDMMC_INTMASK);

  /* Detect and initialize card */

  ret = rk3576_sdmmc_init_card(ctrl);
  if (ret < 0)
    {
      mcwarn("SDMMC%d: no card detected or init failed\n", ctrl);
    }

  g_sdmmc_dev[ctrl].initialized = true;

  mcinfo("SDMMC%d: initialized\n", ctrl);
  return OK;
}

/****************************************************************************
 * Name: rk3576_sdmmc_card_detect
 *
 * Description:
 *   Check if a card is present.
 *
 ****************************************************************************/

int rk3576_sdmmc_card_detect(int ctrl)
{
  uint32_t base;

  if (rk3576_sdmmc_validate(ctrl) < 0)
    {
      return 0;
    }

  base = g_sdmmc_base[ctrl];

  return (getreg32(base + SDMMC_CARDDET) & SDMMC_CARDDET_CARD) ? 1 : 0;
}

/****************************************************************************
 * Name: rk3576_sdmmc_card_type
 *
 * Description:
 *   Get the type of card detected.
 *
 ****************************************************************************/

int rk3576_sdmmc_card_type(int ctrl)
{
  if (rk3576_sdmmc_validate(ctrl) < 0)
    {
      return CARD_TYPE_NONE;
    }

  return g_sdmmc_dev[ctrl].card_type;
}

/****************************************************************************
 * Name: rk3576_sdmmc_set_clock
 *
 * Description:
 *   Set SDMMC clock frequency.
 *
 ****************************************************************************/

void rk3576_sdmmc_set_clock(int ctrl, uint32_t freq)
{
  uint32_t base;
  uint32_t div;

  if (rk3576_sdmmc_validate(ctrl) < 0)
    {
      return;
    }

  base = g_sdmmc_base[ctrl];

  /* Calculate divider:
   * CCLK = SDCLK / (2 * (CLK_DIV + 1))
   * CLK_DIV = (SDCLK / (2 * CCLK)) - 1
   */

  div = (SDMMC_SRC_CLK_HZ / (2 * freq)) - 1;
  if (div > 0xff)
    {
      div = 0xff;
    }

  /* Disable clock during change */

  putreg32(0, base + SDMMC_CLKENA);

  /* Update clock divider */

  putreg32(div, base + SDMMC_CLKDIV);

  /* Send update clock command */

  putreg32(SDMMC_CMD_UPDATE_CLK | SDMMC_CMD_START, base + SDMMC_CMD);

  /* Wait for update to complete */

  {
    int timeout = 1000;
    while (timeout--)
      {
        if (!(getreg32(base + SDMMC_CMD) & SDMMC_CMD_START))
          {
            break;
          }

        up_udelay(1);
      }
  }

  /* Enable clock */

  putreg32(SDMMC_CLKENA_CCLK_EN, base + SDMMC_CLKENA);

  /* Send update clock command again */

  putreg32(SDMMC_CMD_UPDATE_CLK | SDMMC_CMD_START, base + SDMMC_CMD);

  {
    int timeout = 1000;
    while (timeout--)
      {
        if (!(getreg32(base + SDMMC_CMD) & SDMMC_CMD_START))
          {
            break;
          }

        up_udelay(1);
      }
  }

  g_sdmmc_dev[ctrl].clock = freq;

  mcinfo("SDMMC%d: clock set to %u Hz (div=%u)\n", ctrl, freq, div);
}

/****************************************************************************
 * Name: rk3576_sdmmc_read
 *
 * Description:
 *   Read data from the card.
 *
 ****************************************************************************/

int rk3576_sdmmc_read(int ctrl, uint32_t sector,
                       uint32_t count, uint8_t *buf)
{
  uint32_t base;
  int ret;

  ret = rk3576_sdmmc_validate(ctrl);
  if (ret < 0)
    {
      return ret;
    }

  if (buf == NULL || count == 0)
    {
      return -EINVAL;
    }

  base = g_sdmmc_base[ctrl];

  /* Send CMD17 (READ_SINGLE_BLOCK) or CMD18 (READ_MULTIPLE_BLOCK) */

  uint32_t cmd;
  if (count == 1)
    {
      cmd = SDMMC_CMD_START | SDMMC_CMD_RESP_EXP |
            SDMMC_CMD_CHECK_RESP_CRC | SDMMC_CMD_DATA_EXPECT |
            SDMMC_CMD_WAIT_PRVDATA | 17;
    }
  else
    {
      cmd = SDMMC_CMD_START | SDMMC_CMD_RESP_EXP |
            SDMMC_CMD_CHECK_RESP_CRC | SDMMC_CMD_DATA_EXPECT |
            SDMMC_CMD_WAIT_PRVDATA | 18;
    }

  ret = rk3576_sdmmc_send_cmd_r1(ctrl, cmd, sector, NULL);
  if (ret < 0)
    {
      return ret;
    }

  /* Read data from FIFO */

  uint32_t total = count * g_sdmmc_dev[ctrl].block_size;
  uint32_t offset = 0;

  while (offset < total)
    {
      /* Wait for RX data request */

      int timeout = SDMMC_DATA_TIMEOUT_US;
      while (timeout--)
        {
          uint32_t status = getreg32(base + SDMMC_RINTSTS);
          if (status & SDMMC_INT_RXDR)
            {
              break;
            }

          if (status & SDMMC_INT_DATA_OVER)
            {
              break;
            }

          up_udelay(1);
        }

      /* Read data from FIFO (4 bytes at a time) */

      uint32_t words = (total - offset) / 4;
      if (words > SDMMC_FIFO_DEPTH / 4)
        {
          words = SDMMC_FIFO_DEPTH / 4;
        }

      for (uint32_t i = 0; i < words; i++)
        {
          uint32_t data = getreg32(base + SDMMC_DATA);
          buf[offset + 0] = (uint8_t)(data & 0xff);
          buf[offset + 1] = (uint8_t)((data >> 8) & 0xff);
          buf[offset + 2] = (uint8_t)((data >> 16) & 0xff);
          buf[offset + 3] = (uint8_t)((data >> 24) & 0xff);
          offset += 4;
        }
    }

  /* Wait for data transfer to complete */

  ret = rk3576_sdmmc_wait_data_done(ctrl);

  /* Send CMD12 (STOP_TRANSMISSION) for multi-block read */

  if (count > 1 && ret == OK)
    {
      rk3576_sdmmc_send_cmd_r1(ctrl,
          SDMMC_CMD_START | SDMMC_CMD_RESP_EXP |
          SDMMC_CMD_CHECK_RESP_CRC | SDMMC_CMD_STOP_ABORT, 0, NULL);
    }

  return (ret == OK) ? (int)total : ret;
}

/****************************************************************************
 * Name: rk3576_sdmmc_write
 *
 * Description:
 *   Write data to the card.
 *
 ****************************************************************************/

int rk3576_sdmmc_write(int ctrl, uint32_t sector,
                        uint32_t count, const uint8_t *buf)
{
  uint32_t base;
  int ret;

  ret = rk3576_sdmmc_validate(ctrl);
  if (ret < 0)
    {
      return ret;
    }

  if (buf == NULL || count == 0)
    {
      return -EINVAL;
    }

  base = g_sdmmc_base[ctrl];

  /* Send CMD24 (WRITE_SINGLE_BLOCK) or CMD25 (WRITE_MULTIPLE_BLOCK) */

  uint32_t cmd;
  if (count == 1)
    {
      cmd = SDMMC_CMD_START | SDMMC_CMD_RESP_EXP |
            SDMMC_CMD_CHECK_RESP_CRC | SDMMC_CMD_DATA_EXPECT |
            SDMMC_CMD_WAIT_PRVDATA | 24;
    }
  else
    {
      cmd = SDMMC_CMD_START | SDMMC_CMD_RESP_EXP |
            SDMMC_CMD_CHECK_RESP_CRC | SDMMC_CMD_DATA_EXPECT |
            SDMMC_CMD_WAIT_PRVDATA | 25;
    }

  ret = rk3576_sdmmc_send_cmd_r1(ctrl, cmd, sector, NULL);
  if (ret < 0)
    {
      return ret;
    }

  /* Write data to FIFO */

  uint32_t total = count * g_sdmmc_dev[ctrl].block_size;
  uint32_t offset = 0;

  while (offset < total)
    {
      /* Wait for TX data request */

      int timeout = SDMMC_DATA_TIMEOUT_US;
      while (timeout--)
        {
          uint32_t status = getreg32(base + SDMMC_RINTSTS);
          if (status & SDMMC_INT_TXDR)
            {
              break;
            }

          up_udelay(1);
        }

      /* Write data to FIFO (4 bytes at a time) */

      uint32_t words = (total - offset) / 4;
      if (words > SDMMC_FIFO_DEPTH / 4)
        {
          words = SDMMC_FIFO_DEPTH / 4;
        }

      for (uint32_t i = 0; i < words; i++)
        {
          uint32_t data = (uint32_t)buf[offset + 0] |
                         ((uint32_t)buf[offset + 1] << 8) |
                         ((uint32_t)buf[offset + 2] << 16) |
                         ((uint32_t)buf[offset + 3] << 24);
          putreg32(data, base + SDMMC_DATA);
          offset += 4;
        }
    }

  /* Wait for data transfer to complete */

  ret = rk3576_sdmmc_wait_data_done(ctrl);

  /* Send CMD12 (STOP_TRANSMISSION) for multi-block write */

  if (count > 1 && ret == OK)
    {
      rk3576_sdmmc_send_cmd_r1(ctrl,
          SDMMC_CMD_START | SDMMC_CMD_RESP_EXP |
          SDMMC_CMD_CHECK_RESP_CRC | SDMMC_CMD_STOP_ABORT, 0, NULL);
    }

  return (ret == OK) ? (int)total : ret;
}

/****************************************************************************
 * Name: rk3576_sdmmc_reset
 *
 * Description:
 *   Reset SDMMC controller.
 *
 ****************************************************************************/

int rk3576_sdmmc_reset(int ctrl)
{
  int ret;

  ret = rk3576_sdmmc_validate(ctrl);
  if (ret < 0)
    {
      return ret;
    }

  /* Reset controller */

  ret = rk3576_sdmmc_reset_controller(ctrl);
  if (ret < 0)
    {
      return ret;
    }

  /* Re-initialize */

  return rk3576_sdmmc_init(ctrl);
}
