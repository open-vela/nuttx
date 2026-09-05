#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/mmcsd.h>
#include <nuttx/sdio.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

#include "esp_sdmmc.h"
#include "esp_gpio.h"
#include "esp_irq.h"

#ifdef CONFIG_ESP32P4_SDMMC_LDO
#  include "esp_ldo.h"
#endif

#include "hal/sdmmc_ll.h"
#include "hal/sd_types.h"
#include "soc/sdmmc_struct.h"
#include "soc/sdmmc_reg.h"
#include "soc/interrupts.h"
#include "esp_cache.h"

/* ========================================================================
 * 全局状态
 * ======================================================================== */

#define __DECLARE_RCC_ATOMIC_ENV  0

static struct esp32p4_sdmmc_state_s g_sdmmc_state[2];

/* ========================================================================
 * 辅助宏
 * ======================================================================== */

#define SDMMC_SRC_CLK_KHZ       160000  /* PLL160M */

/* ========================================================================
 * 调试宏 - 使用 printf 输出调试信息
 * ======================================================================== */

#define SDMMC_DEBUG 1

#if SDMMC_DEBUG
#  define sdmmc_dbg(fmt, ...) \
  printf("[SDMMC] " fmt "\n", ##__VA_ARGS__)
#  define sdmmc_err(fmt, ...) \
  printf("[SDMMC ERROR] %s:%d " fmt "\n", __func__, __LINE__, ##__VA_ARGS__)
#else
#  define sdmmc_dbg(fmt, ...)  do { } while (0)
#  define sdmmc_err(fmt, ...)  do { } while (0)
#endif

/* ========================================================================
 * LDO
 * ======================================================================== */

#ifdef CONFIG_ESP32P4_SDMMC_LDO

static int esp32p4_sdmmc_ldo_init(struct esp32p4_sdmmc_state_s *state)
{
  struct esp_ldo_config_t *ldo;

  if (state->slot != 0)
    {
      return OK;
    }

  ldo = kmm_malloc(sizeof(struct esp_ldo_config_t));
  if (ldo == NULL)
    {
      sdmmc_err("LDO malloc failed");
      return -ENOMEM;
    }

  memset(ldo, 0, sizeof(*ldo));
  ldo->chan_id    = CONFIG_ESP32P4_SDMMC_LDO_CHAN;
  ldo->voltage_mv = CONFIG_ESP32P4_SDMMC_LDO_VOLTAGE;

  sdmmc_dbg("LDO init: chan=%d voltage=%dmV", ldo->chan_id, ldo->voltage_mv);

  int ret = esp_ldo_channel_acquire(ldo);
  if (ret < 0)
    {
      kmm_free(ldo);
      sdmmc_err("LDO acquire failed: %d", ret);
      return ret;
    }

  state->ldo_config = ldo;
  sdmmc_dbg("LDO OK: chan=%d @ %dmV", ldo->chan_id, ldo->voltage_mv);
  return OK;
}

static void esp32p4_sdmmc_ldo_deinit(struct esp32p4_sdmmc_state_s *state)
{
  struct esp_ldo_config_t *ldo = state->ldo_config;

  if (ldo != NULL)
    {
      esp_ldo_channel_release(ldo);
      kmm_free(ldo);
      state->ldo_config = NULL;
    }
}

#else

static int esp32p4_sdmmc_ldo_init(struct esp32p4_sdmmc_state_s *state)
{
  sdmmc_dbg("LDO disabled in config");
  return OK;
}

static void esp32p4_sdmmc_ldo_deinit(struct esp32p4_sdmmc_state_s *state)
{
}

#endif

/* ========================================================================
 * GPIO - 使用 IOMUX function0（与 HAL 一致）
 * ======================================================================== */

static int esp32p4_sdmmc_gpio_init(struct esp32p4_sdmmc_state_s *state)
{
  const int *pins;
  int npins;
  int i;

  sdmmc_dbg("GPIO init slot=%d", state->slot);

  if (state->slot == 0)
    {
      static const int slot0_pins[] =
      {
        ESP32P4_SDMMC0_CLK, ESP32P4_SDMMC0_CMD,
        ESP32P4_SDMMC0_D0,  ESP32P4_SDMMC0_D1,
        ESP32P4_SDMMC0_D2,  ESP32P4_SDMMC0_D3,
      };
      pins  = slot0_pins;
      npins = 6;

      sdmmc_dbg("Slot0 pins: CLK=%d CMD=%d D0=%d D1=%d D2=%d D3=%d",
                ESP32P4_SDMMC0_CLK, ESP32P4_SDMMC0_CMD,
                ESP32P4_SDMMC0_D0, ESP32P4_SDMMC0_D1,
                ESP32P4_SDMMC0_D2, ESP32P4_SDMMC0_D3);
    }
  else
    {
      sdmmc_err("Slot1 not enabled");
      return -EINVAL;
    }

  for (i = 0; i < npins; i++)
    {
      gpio_pinattr_t attr;
      bool is_clk = false;

      if ((state->slot == 0 && pins[i] == ESP32P4_SDMMC0_CLK) ||
          (state->slot == 1
#ifdef CONFIG_ESP32P4_SDMMC_SLOT1
           //&& pins[i] == CONFIG_ESP32P4_SDMMC_SLOT1_CLK
#endif
          ))
        {
          is_clk = true;
        }

      if (is_clk)
        {
          attr = OUTPUT_FUNCTION_1 | PULLUP;
        }
      else
        {
          attr = INPUT_FUNCTION_1 | PULLUP;
        }

      sdmmc_dbg("  pin %d: attr=0x%x mcu_sel=0 (IOMUX)%s",
                pins[i], attr, is_clk ? " (CLK)" : "");
      esp_configgpio(pins[i], attr);
    }



  sdmmc_dbg("Slot %d GPIO OK", state->slot);
  return OK;
}

/* ========================================================================
 * 辅助函数
 * ======================================================================== */

static struct esp32p4_sdmmc_state_s *esp32p4_sdmmc_get_state(struct sdio_dev_s *dev)
{
  return (struct esp32p4_sdmmc_state_s *)dev;
}

/* ========================================================================
 * 时钟控制 - 使用 HAL 的两阶段分频
 * ======================================================================== */

static void esp32p4_sdmmc_set_clock(struct esp32p4_sdmmc_state_s *state,
                                    uint32_t freq_khz)
{
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);
  int host_div;
  int card_div;

  sdmmc_dbg("set_clock: target=%lukHz", (unsigned long)freq_khz);

  if (freq_khz >= 40000)
    {
      host_div = 4;
      card_div = 0;
    }
  else if (freq_khz >= 20000)
    {
      host_div = 8;
      card_div = 0;
    }
  else if (freq_khz >= 400)
    {
      host_div = 10;
      card_div = 20;
    }
  else
    {
      host_div = 2;
      card_div = (SDMMC_SRC_CLK_KHZ * 1000 / 2) / (2 * freq_khz * 1000);
    }

  sdmmc_ll_enable_card_clock(hw, state->slot, false);
  sdmmc_ll_set_clock_div(hw, host_div);
  sdmmc_ll_set_card_clock_div(hw, state->slot, card_div);

  {
    sdmmc_hw_cmd_t cmd = {0};
    cmd.update_clk_reg = 1;
    cmd.wait_complete = 1;
    cmd.use_hold_reg = 1;
    cmd.card_num = state->slot;
    cmd.start_command = 1;

    sdmmc_ll_set_command_arg(hw, 0);
    sdmmc_ll_set_command(hw, cmd);

    uint32_t timeout = 10000;
    while (!sdmmc_ll_is_command_taken(hw))
      {
        if (--timeout == 0)
          {
            sdmmc_err("update_clk timeout!");
            break;
          }
        up_udelay(1);
      }
  }

  sdmmc_ll_enable_card_clock(hw, state->slot, true);
  sdmmc_ll_enable_card_clock_low_power(hw, state->slot, true);

  {
    sdmmc_hw_cmd_t cmd = {0};
    cmd.update_clk_reg = 1;
    cmd.wait_complete = 1;
    cmd.use_hold_reg = 1;
    cmd.card_num = state->slot;
    cmd.start_command = 1;

    sdmmc_ll_set_command_arg(hw, 0);
    sdmmc_ll_set_command(hw, cmd);

    uint32_t timeout = 10000;
    while (!sdmmc_ll_is_command_taken(hw))
      {
        if (--timeout == 0)
          {
            sdmmc_err("update_clk2 timeout!");
            break;
          }
        up_udelay(1);
      }
  }

  if (card_div == 0)
    {
      state->freq_khz = SDMMC_SRC_CLK_KHZ / host_div;
    }
  else
    {
      state->freq_khz = SDMMC_SRC_CLK_KHZ / host_div / (card_div * 2);
    }

  sdmmc_dbg("set_clock: host_div=%d card_div=%d actual=%lukHz",
            host_div, card_div, (unsigned long)state->freq_khz);
}

/* ========================================================================
 * 控制器复位（使用 LL 层）
 * ======================================================================== */

static void esp32p4_sdmmc_reset_controller(void)
{
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);
  uint32_t timeout;

  sdmmc_dbg("reset_controller");

  sdmmc_ll_reset_dma(hw);
  timeout = 10000;
  while (!sdmmc_ll_is_dma_reset_done(hw))
    {
      if (--timeout == 0)
        {
          sdmmc_err("DMA reset timeout!");
          break;
        }
      up_udelay(10);
    }

  sdmmc_ll_reset_fifo(hw);
  timeout = 10000;
  while (!sdmmc_ll_is_fifo_reset_done(hw))
    {
      if (--timeout == 0)
        {
          sdmmc_err("FIFO reset timeout!");
          break;
        }
      up_udelay(10);
    }

  sdmmc_ll_reset_controller(hw);
  timeout = 10000;
  while (!sdmmc_ll_is_controller_reset_done(hw))
    {
      if (--timeout == 0)
        {
          sdmmc_err("Controller reset timeout!");
          break;
        }
      up_udelay(10);
    }

  sdmmc_ll_clear_interrupt(hw, 0xFFFFFFFF);
  sdmmc_ll_clear_idsts_interrupt(hw, 0xFFFFFFFF);

  sdmmc_dbg("reset_controller done");
}

/* ========================================================================
 * 等待数据总线空闲
 * ======================================================================== */

static void esp32p4_sdmmc_wait_idle(void)
{
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);
  uint32_t timeout = 100000;

  while (sdmmc_ll_is_card_data_busy(hw) && --timeout)
    {
      up_udelay(1);
    }

  if (timeout == 0)
    {
      sdmmc_err("wait_idle timeout!");
    }
}

/* ========================================================================
 * DMA 描述符设置 - 使用 HAL 兼容的 sdmmc_desc_t 格式
 * ======================================================================== */

static void esp32p4_sdmmc_setup_dma(struct esp32p4_sdmmc_state_s *state,
                                    uint8_t *buffer, size_t nbytes, bool is_tx)
{
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);
  size_t desc_idx = 0;
  size_t remaining = nbytes;
  uint8_t *buf_ptr = buffer;

  sdmmc_dbg("setup_dma: buf=%p size=%zu %s", buffer, nbytes,
            is_tx ? "TX" : "RX");

  memset(state->dma_desc, 0, sizeof(state->dma_desc));

  while (remaining > 0 && desc_idx < SDMMC_DMA_DESC_NUM)
    {
      size_t chunk = remaining;
      if (chunk > 4096) chunk = 4096;

      struct esp32p4_sdmmc_dmadesc_s *desc = &state->dma_desc[desc_idx];

      bool is_last = (remaining <= 4096) ||
                     (desc_idx == SDMMC_DMA_DESC_NUM - 1);

      desc->ctrl = (1U << 3) |   /* first_descriptor */
                   (1U << 4) |   /* second_address_chained */
                   (1U << 31);   /* owned_by_idmac */

      if (is_last)
        {
          desc->ctrl |= (1U << 2);   /* last_descriptor */
          desc->ctrl &= ~(1U << 4);  /* clear chained */
          desc->next_desc = NULL;
        }
      else
        {
          desc->next_desc = (void *)&state->dma_desc[desc_idx + 1];
        }

      desc->size = (chunk + 3) & (~3);
      desc->buf_addr = (void *)buf_ptr;

      buf_ptr += chunk;
      remaining -= chunk;
      desc_idx++;
    }

  esp_cache_msync(state->dma_desc,
                  sizeof(state->dma_desc),
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  if (is_tx)
    {
      esp_cache_msync((void *)buffer, nbytes,
                      ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    }

  sdmmc_ll_set_desc_addr(hw, (uint32_t)&state->dma_desc[0]);
  sdmmc_dbg("DMA desc addr=0x%08lx",
            (unsigned long)&state->dma_desc[0]);

  sdmmc_ll_enable_dma(hw, true);
  sdmmc_ll_poll_demand(hw);
}

/* ========================================================================
 * SDIO 接口函数
 * ======================================================================== */

#ifdef CONFIG_SDIO_MUXBUS
static int esp32p4_sdmmc_lock(struct sdio_dev_s *dev, bool lock)
{
  return OK;
}
#endif

static void esp32p4_sdmmc_reset(struct sdio_dev_s *dev)
{
  sdmmc_dbg("reset");
  esp32p4_sdmmc_reset_controller();
}

static sdio_capset_t esp32p4_sdmmc_capabilities(struct sdio_dev_s *dev)
{
  sdmmc_dbg("capabilities: 4BIT | DMA | DMABEFOREWRITE");
  return SDIO_CAPS_4BIT | SDIO_CAPS_DMASUPPORTED | SDIO_CAPS_DMABEFOREWRITE;
}

static sdio_statset_t esp32p4_sdmmc_status(struct sdio_dev_s *dev)
{
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);
  bool detected = sdmmc_ll_is_card_detected(hw, 0);
  sdmmc_dbg("status: detected=%d", detected);
  if (detected)
    {
      return SDIO_STATUS_PRESENT;
    }
  return 0;
}

static void esp32p4_sdmmc_widebus(struct sdio_dev_s *dev, bool wide)
{
  struct esp32p4_sdmmc_state_s *state = esp32p4_sdmmc_get_state(dev);
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);

  sdmmc_dbg("widebus: slot=%d wide=%d", state->slot, wide);

  sdmmc_ll_set_card_width(hw, state->slot,
                          wide ? SD_BUS_WIDTH_4_BIT : SD_BUS_WIDTH_1_BIT);
  state->widebus = wide;
}

static void esp32p4_sdmmc_clock(struct sdio_dev_s *dev,
                                enum sdio_clock_e rate)
{
  struct esp32p4_sdmmc_state_s *state = esp32p4_sdmmc_get_state(dev);
  uint32_t freq;

  switch (rate)
    {
      case CLOCK_IDMODE:
        freq = 400;
        break;
      case CLOCK_MMC_TRANSFER:
      case CLOCK_SD_TRANSFER_1BIT:
        freq = 20000;
        break;
      case CLOCK_SD_TRANSFER_4BIT:
        freq = 40000;
        break;
      default:
        freq = 20000;
        break;
    }

  sdmmc_dbg("clock: rate=%d -> freq=%lukHz", rate, (unsigned long)freq);
  esp32p4_sdmmc_set_clock(state, freq);
}

static int esp32p4_sdmmc_attach(struct sdio_dev_s *dev)
{
  sdmmc_dbg("attach");
  return OK;
}

/* ========================================================================
 * 命令发送（使用 sdmmc_hw_cmd_t）
 * ======================================================================== */

static int esp32p4_sdmmc_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                                 uint32_t arg)
{
  struct esp32p4_sdmmc_state_s *state = esp32p4_sdmmc_get_state(dev);
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);
  uint32_t cmdidx = cmd & MMCSD_CMDIDX_MASK;
  sdmmc_hw_cmd_t s_cmd;
  uint32_t timeout;

  sdmmc_dbg("sendcmd: CMD%lu arg=0x%08lx",
            (unsigned long)cmdidx, (unsigned long)arg);

  esp32p4_sdmmc_wait_idle();

  timeout = 10000;
  while (!sdmmc_ll_is_command_taken(hw))
    {
      if (--timeout == 0)
        {
          sdmmc_err("CMD%lu pre-wait timeout", (unsigned long)cmdidx);
          return -ETIMEDOUT;
        }
      up_udelay(1);
    }

  sdmmc_ll_clear_interrupt(hw, SDMMC_LL_EVENT_DEFAULT);

  sdmmc_ll_set_command_arg(hw, arg);

  memset(&s_cmd, 0, sizeof(s_cmd));
  s_cmd.cmd_index = cmdidx;
  s_cmd.wait_complete = 1;
  s_cmd.use_hold_reg = 1;
  s_cmd.card_num = state->slot;

  if (cmdidx == 0)
    {
      s_cmd.send_init = 1;
    }

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_NO_RESPONSE)
    {
      s_cmd.response_expect = 1;

      if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE)
        {
          s_cmd.check_response_crc = 1;
        }

      if ((cmd & MMCSD_RESPONSE_MASK) == MMCSD_R2_RESPONSE)
        {
          s_cmd.response_long = 1;
        }
    }

  if (cmd & MMCSD_R1B_RESPONSE)
    {
      s_cmd.stop_abort_cmd = 1;
    }

  if (cmd & MMCSD_DATAXFR)
    {
      s_cmd.data_expected = 1;
      if (cmd & MMCSD_WRXFR)
        {
          s_cmd.rw = 1;
        }
    }

  s_cmd.start_command = 1;

  sdmmc_ll_set_command(hw, s_cmd);

  return OK;
}

static int esp32p4_sdmmc_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                                   uint32_t *R)
{
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);
  uint32_t cmdidx = cmd & MMCSD_CMDIDX_MASK;
  uint32_t timeout = 100000;
  uint32_t status;
  bool is_r3 = ((cmd & MMCSD_RESPONSE_MASK) == MMCSD_R3_RESPONSE);

  sdmmc_dbg("recvshort: CMD%lu", (unsigned long)cmdidx);

  while (timeout--)
    {
      status = sdmmc_ll_get_interrupt_raw(hw);

      uint32_t err_mask;
      if (is_r3)
        {
          err_mask = SDMMC_LL_EVENT_RTO | SDMMC_LL_EVENT_HLE;
        }
      else
        {
          err_mask = SDMMC_LL_EVENT_RTO | SDMMC_LL_EVENT_RESP_ERR |
                     SDMMC_LL_EVENT_RCRC | SDMMC_LL_EVENT_HLE |
                     SDMMC_LL_EVENT_SBE | SDMMC_LL_EVENT_EBE;
        }

      if (status & err_mask)
        {
          sdmmc_err("CMD%lu error status=0x%08lx",
                    (unsigned long)cmdidx, (unsigned long)status);
          sdmmc_ll_clear_interrupt(hw, status);
          return -EIO;
        }

      if (status & SDMMC_LL_EVENT_CMD_DONE)
        {
          break;
        }
      up_udelay(1);
    }

  if ((status & SDMMC_LL_EVENT_CMD_DONE) == 0)
    {
      sdmmc_err("CMD%lu timeout status=0x%08lx",
                (unsigned long)cmdidx,
                (unsigned long)sdmmc_ll_get_interrupt_raw(hw));
      return -ETIMEDOUT;
    }

  sdmmc_ll_clear_interrupt(hw, SDMMC_LL_EVENT_CMD_DONE);

  *R = hw->resp[0];

  sdmmc_dbg("recvshort: CMD%lu resp=0x%08lx",
            (unsigned long)cmdidx, (unsigned long)*R);
  return OK;
}

static int esp32p4_sdmmc_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                                  uint32_t R[4])
{
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);
  uint32_t cmdidx = cmd & MMCSD_CMDIDX_MASK;
  uint32_t timeout = 100000;
  uint32_t status;

  sdmmc_dbg("recvlong: CMD%lu", (unsigned long)cmdidx);

  while (timeout--)
    {
      status = sdmmc_ll_get_interrupt_raw(hw);

      if (status & (SDMMC_LL_EVENT_RTO | SDMMC_LL_EVENT_RESP_ERR |
                    SDMMC_LL_EVENT_RCRC | SDMMC_LL_EVENT_HLE |
                    SDMMC_LL_EVENT_SBE | SDMMC_LL_EVENT_EBE))
        {
          sdmmc_err("CMD%lu long error status=0x%08lx",
                    (unsigned long)cmdidx, (unsigned long)status);
          sdmmc_ll_clear_interrupt(hw, status);
          return -EIO;
        }

      if (status & SDMMC_LL_EVENT_CMD_DONE)
        {
          break;
        }
      up_udelay(1);
    }

  if ((status & SDMMC_LL_EVENT_CMD_DONE) == 0)
    {
      sdmmc_err("CMD%lu long timeout", (unsigned long)cmdidx);
      return -ETIMEDOUT;
    }

  sdmmc_ll_clear_interrupt(hw, SDMMC_LL_EVENT_CMD_DONE);

  R[0] = hw->resp[0];
  R[1] = hw->resp[1];
  R[2] = hw->resp[2];
  R[3] = hw->resp[3];

  sdmmc_dbg("recvlong: CMD%lu resp=%08lx %08lx %08lx %08lx",
            (unsigned long)cmdidx,
            (unsigned long)R[3], (unsigned long)R[2],
            (unsigned long)R[1], (unsigned long)R[0]);
  return OK;
}

/* ========================================================================
 * 数据接收（使用 LL 层 DMA）- 纯轮询，无中断
 * ======================================================================== */

static int esp32p4_sdmmc_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                                   size_t nbytes)
{
  struct esp32p4_sdmmc_state_s *state = esp32p4_sdmmc_get_state(dev);
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);
  uint32_t timeout;

  sdmmc_dbg("recvsetup: buf=%p nbytes=%zu", buffer, nbytes);

  state->dma_buffer = buffer;
  state->dma_nbytes = nbytes;
  state->dma_is_tx = false;

  if (((uintptr_t)buffer & 63) != 0 || (nbytes & 63) != 0)
    {
      if (nbytes > sizeof(state->bounce_buf))
        {
          sdmmc_err("recv: nbytes %zu > bounce_buf %zu", nbytes,
                    sizeof(state->bounce_buf));
          return -EINVAL;
        }
      state->dma_use_bounce = true;
      buffer = state->bounce_buf;
      sdmmc_dbg("recvsetup: using bounce_buf=%p", buffer);
    }
  else
    {
      state->dma_use_bounce = false;
    }

  esp32p4_sdmmc_wait_idle();

  sdmmc_ll_reset_fifo(hw);
  timeout = 10000;
  while (!sdmmc_ll_is_fifo_reset_done(hw))
    {
      if (--timeout == 0)
        {
          sdmmc_err("FIFO reset timeout");
          return -ETIMEDOUT;
        }
      up_udelay(10);
    }

  sdmmc_ll_reset_dma(hw);
  timeout = 10000;
  while (!sdmmc_ll_is_dma_reset_done(hw))
    {
      if (--timeout == 0)
        {
          sdmmc_err("DMA reset timeout");
          return -ETIMEDOUT;
        }
      up_udelay(10);
    }

  sdmmc_ll_set_data_transfer_len(hw, nbytes);
  sdmmc_ll_set_block_size(hw, nbytes);

  esp32p4_sdmmc_setup_dma(state, buffer, nbytes, false);

  sdmmc_ll_clear_interrupt(hw, 0xFFFFFFFF);
  sdmmc_ll_clear_idsts_interrupt(hw, 0xFFFFFFFF);

  sdmmc_dbg("recvsetup OK (DMA configured, polling mode)");
  return OK;
}

/* ========================================================================
 * 数据发送（使用 LL 层 DMA）- 纯轮询，无中断
 * ======================================================================== */

static int esp32p4_sdmmc_sendsetup(struct sdio_dev_s *dev,
                                   const uint8_t *buffer, size_t nbytes)
{
  struct esp32p4_sdmmc_state_s *state = esp32p4_sdmmc_get_state(dev);
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);
  uint32_t timeout;

  sdmmc_dbg("sendsetup: buf=%p nbytes=%zu", buffer, nbytes);

  state->dma_buffer = (uint8_t *)buffer;
  state->dma_nbytes = nbytes;
  state->dma_is_tx = true;

  if (((uintptr_t)buffer & 63) != 0 || (nbytes & 63) != 0)
    {
      if (nbytes > sizeof(state->bounce_buf))
        {
          sdmmc_err("send: nbytes %zu > bounce_buf %zu", nbytes,
                    sizeof(state->bounce_buf));
          return -EINVAL;
        }
      state->dma_use_bounce = true;
      memcpy(state->bounce_buf, buffer, nbytes);
      buffer = state->bounce_buf;
      sdmmc_dbg("sendsetup: using bounce_buf=%p", buffer);
    }
  else
    {
      state->dma_use_bounce = false;
    }

  esp32p4_sdmmc_wait_idle();

  sdmmc_ll_reset_fifo(hw);
  timeout = 10000;
  while (!sdmmc_ll_is_fifo_reset_done(hw))
    {
      if (--timeout == 0)
        {
          sdmmc_err("FIFO reset timeout");
          return -ETIMEDOUT;
        }
      up_udelay(10);
    }

  sdmmc_ll_reset_dma(hw);
  timeout = 10000;
  while (!sdmmc_ll_is_dma_reset_done(hw))
    {
      if (--timeout == 0)
        {
          sdmmc_err("DMA reset timeout");
          return -ETIMEDOUT;
        }
      up_udelay(10);
    }

  sdmmc_ll_set_data_transfer_len(hw, nbytes);
  sdmmc_ll_set_block_size(hw, nbytes);

  esp32p4_sdmmc_setup_dma(state, (uint8_t *)buffer, nbytes, true);

  sdmmc_ll_clear_interrupt(hw, 0xFFFFFFFF);
  sdmmc_ll_clear_idsts_interrupt(hw, 0xFFFFFFFF);

  sdmmc_dbg("sendsetup OK (DMA configured, polling mode)");
  return OK;
}

static int esp32p4_sdmmc_cancel(struct sdio_dev_s *dev)
{
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);

  sdmmc_dbg("cancel");
  sdmmc_ll_stop_dma(hw);

  sdmmc_hw_cmd_t cmd = {0};
  cmd.stop_abort_cmd = 1;
  cmd.wait_complete = 1;
  cmd.start_command = 1;
  sdmmc_ll_set_command(hw, cmd);

  return OK;
}

static int esp32p4_sdmmc_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  return OK;
}

static void esp32p4_sdmmc_waitenable(struct sdio_dev_s *dev,
                                     sdio_eventset_t eventset,
                                     uint32_t timeout)
{
  struct esp32p4_sdmmc_state_s *state = esp32p4_sdmmc_get_state(dev);
  state->waitevents = eventset;
  (void)timeout;
}

/* ========================================================================
 * 事件等待 - 纯轮询 DMA 完成状态，不依赖中断
 * ======================================================================== */

static sdio_eventset_t esp32p4_sdmmc_eventwait(struct sdio_dev_s *dev)
{
  struct esp32p4_sdmmc_state_s *state = esp32p4_sdmmc_get_state(dev);
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);
  uint32_t timeout;
  uint32_t status;
  uint32_t idsts;

  if (state->dma_buffer == NULL || state->dma_nbytes == 0)
    {
      return SDIOWAIT_CMDDONE;
    }

  timeout = 10000;  /* 10000 * 100us = 1秒 */
  bool data_done = false;
  bool dma_done = false;

  sdmmc_dbg("eventwait: polling DMA, nbytes=%zu", state->dma_nbytes);

  while (timeout--)
    {
      status = sdmmc_ll_get_interrupt_raw(hw);
      idsts  = sdmmc_ll_get_idsts_interrupt_raw(hw);

      if (status & (SDMMC_LL_EVENT_DCRC | SDMMC_LL_EVENT_FRUN |
                    SDMMC_LL_EVENT_HTO | SDMMC_LL_EVENT_EBE |
                    SDMMC_LL_EVENT_SBE | SDMMC_LL_EVENT_DTO))
        {
          sdmmc_err("DMA data error status=0x%08lx idsts=0x%08lx",
                    (unsigned long)status, (unsigned long)idsts);
          sdmmc_ll_clear_interrupt(hw, status);
          sdmmc_ll_clear_idsts_interrupt(hw, idsts);
          state->dma_buffer = NULL;
          state->dma_nbytes = 0;
          return SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR;
        }

      if (idsts & (1 << 2))  /* FBE */
        {
          sdmmc_err("DMA FBE error idsts=0x%08lx", (unsigned long)idsts);
          sdmmc_ll_clear_idsts_interrupt(hw, idsts);
          state->dma_buffer = NULL;
          state->dma_nbytes = 0;
          return SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR;
        }

      if (status & SDMMC_LL_EVENT_DATA_OVER)
        {
          data_done = true;
          sdmmc_ll_clear_interrupt(hw, SDMMC_LL_EVENT_DATA_OVER);
        }

      if (state->dma_is_tx)
        {
          if (idsts & SDMMC_LL_EVENT_DMA_TI)
            {
              dma_done = true;
              sdmmc_ll_clear_idsts_interrupt(hw, SDMMC_LL_EVENT_DMA_TI);
            }
        }
      else
        {
          if (idsts & SDMMC_LL_EVENT_DMA_RI)
            {
              dma_done = true;
              sdmmc_ll_clear_idsts_interrupt(hw, SDMMC_LL_EVENT_DMA_RI);
            }
        }

      if (data_done && dma_done)
        {
          break;
        }

      up_udelay(100);
    }

  if (!data_done || !dma_done)
    {
      sdmmc_err("DMA timeout: data_done=%d dma_done=%d status=0x%08lx idsts=0x%08lx",
                data_done, dma_done,
                (unsigned long)sdmmc_ll_get_interrupt_raw(hw),
                (unsigned long)sdmmc_ll_get_idsts_interrupt_raw(hw));
      state->dma_buffer = NULL;
      state->dma_nbytes = 0;
      return SDIOWAIT_TIMEOUT;
    }

  /* RX 完成后：invalidate cache，拷贝回用户 buffer */
  if (!state->dma_is_tx)
    {
      uint8_t *inv_buf = state->dma_use_bounce
                         ? state->bounce_buf
                         : state->dma_buffer;
      size_t inv_len = (state->dma_nbytes + 63) & ~63;

      esp_cache_msync(inv_buf, inv_len, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

      if (state->dma_use_bounce)
        {
          memcpy(state->dma_buffer, state->bounce_buf, state->dma_nbytes);
          sdmmc_dbg("eventwait: copied %zu bytes from bounce_buf", state->dma_nbytes);
        }
    }

  sdmmc_dbg("eventwait: DMA complete");

  sdmmc_ll_stop_dma(hw);
  sdmmc_ll_enable_dma(hw, false);

  state->dma_buffer = NULL;
  state->dma_nbytes = 0;

  return SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE;
}

static void esp32p4_sdmmc_callbackenable(struct sdio_dev_s *dev,
                                          sdio_eventset_t eventset)
{
}

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
static int esp32p4_sdmmc_registercallback(struct sdio_dev_s *dev,
                                           worker_t callback, void *arg)
{
  struct esp32p4_sdmmc_state_s *state = esp32p4_sdmmc_get_state(dev);

  state->callback    = callback;
  state->callbackarg = arg;

  return OK;
}
#endif

static void esp32p4_sdmmc_gotextcsd(struct sdio_dev_s *dev,
                                    const uint8_t *buffer)
{
}

/* ========================================================================
 * 初始化 - 纯轮询 DMA 模式，不注册中断
 * ======================================================================== */

int esp32p4_sdmmc_initialize(int slot, int minor)
{
  struct esp32p4_sdmmc_state_s *state = &g_sdmmc_state[slot];
  struct sdio_dev_s *dev;
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);
  int ret;

  sdmmc_dbg("===== esp32p4_sdmmc_initialize slot=%d minor=%d =====",
         slot, minor);

  if (state->initialized)
    {
      sdmmc_dbg("Already initialized");
      return OK;
    }

  memset(state, 0, sizeof(*state));
  state->slot = slot;

  nxsem_init(&state->sem, 0, 0);

  /* 1. LDO（Slot0 需要 1.8V/3.3V 电源） */
  sdmmc_dbg("Step 1: LDO init");
  if (slot == 0)
    {
      ret = esp32p4_sdmmc_ldo_init(state);
      if (ret < 0)
        {
          sdmmc_err("LDO init failed: %d", ret);
          goto err_ldo;
        }
    }

  /* 2. GPIO */
  sdmmc_dbg("Step 2: GPIO init");
  ret = esp32p4_sdmmc_gpio_init(state);
  if (ret < 0)
    {
      sdmmc_err("GPIO init failed: %d", ret);
      goto err_gpio;
    }

  /* 3. 使能 SDMMC 总线时钟 */
  sdmmc_dbg("Step 3: Enable bus clock");
  sdmmc_ll_enable_bus_clock(0, true);

  /* 4. 复位 SDMMC 模块 */
  sdmmc_dbg("Step 4: Reset register");
  sdmmc_ll_reset_register(0);

  /* 5. 选择时钟源并设置分频 */
  sdmmc_dbg("Step 5: Clock source setup (host_div=2)");
  sdmmc_ll_select_clk_source(hw, SDMMC_CLK_SRC_PLL160M);
  sdmmc_ll_set_clock_div(hw, 2);  /* 160MHz / 2 = 80MHz */
  sdmmc_ll_init_phase_delay(hw);

  /* 6. 控制器内部复位 */
  sdmmc_dbg("Step 6: Controller reset");
  esp32p4_sdmmc_reset_controller();

  /* 7. 超时配置 */
  sdmmc_dbg("Step 7: Timeout config");
  sdmmc_ll_set_data_timeout(hw, 0xFFFFFF);
  sdmmc_ll_set_response_timeout(hw, 0xFF);

  /* 8. 块大小 */
  sdmmc_dbg("Step 8: Block size");
  sdmmc_ll_set_block_size(hw, 512);

  /* 9. 初始化 DMA */
  sdmmc_dbg("Step 9: DMA init");
  sdmmc_ll_init_dma(hw);
  sdmmc_ll_enable_dma(hw, true);

  /* 10. 中断 - 不注册！纯轮询模式 */
  sdmmc_dbg("Step 10: Polling mode, skip IRQ registration");
  state->cpuint = -1;
  sdmmc_ll_enable_global_interrupt(hw, false);
  sdmmc_ll_clear_interrupt(hw, 0xFFFFFFFF);

  /* 11. 设置初始时钟 400kHz */
  sdmmc_dbg("Step 11: Set init clock 400kHz");
  esp32p4_sdmmc_set_clock(state, 400);

  /* 12. 检测卡是否存在 + 寄存器 dump */
  sdmmc_dbg("Step 12: Card detect + register dump");
  {
    bool detected = sdmmc_ll_is_card_detected(hw, slot);
    sdmmc_dbg("Card detected: %s", detected ? "YES" : "NO");

    sdmmc_dbg("Status:  0x%08lx", (unsigned long)hw->status.val);
    sdmmc_dbg("Rintsts: 0x%08lx", (unsigned long)sdmmc_ll_get_interrupt_raw(hw));

    /* 手动 CMD0 测试 */
    sdmmc_ll_clear_interrupt(hw, 0xFFFFFFFF);
    sdmmc_ll_set_command_arg(hw, 0);
    sdmmc_hw_cmd_t test_cmd = {0};
    test_cmd.cmd_index = 0;
    test_cmd.send_init = 1;
    test_cmd.start_command = 1;
    test_cmd.use_hold_reg = 1;
    sdmmc_ll_set_command(hw, test_cmd);
    up_udelay(10000);
    sdmmc_dbg("CMD0 Rintsts: 0x%08lx",
           (unsigned long)sdmmc_ll_get_interrupt_raw(hw));
  }

  /* 13. 填充 sdio_dev_s */
  sdmmc_dbg("Step 13: Fill sdio_dev_s");
  dev = &state->dev;

  nxmutex_init(&dev->mutex);

#ifdef CONFIG_SDIO_MUXBUS
  dev->lock             = esp32p4_sdmmc_lock;
#endif
  dev->reset            = esp32p4_sdmmc_reset;
  dev->capabilities     = esp32p4_sdmmc_capabilities;
  dev->status           = esp32p4_sdmmc_status;
  dev->widebus          = esp32p4_sdmmc_widebus;
  dev->clock            = esp32p4_sdmmc_clock;
  dev->attach           = esp32p4_sdmmc_attach;
  dev->sendcmd          = esp32p4_sdmmc_sendcmd;
  dev->recvsetup        = esp32p4_sdmmc_recvsetup;
  dev->sendsetup        = esp32p4_sdmmc_sendsetup;
  dev->cancel           = esp32p4_sdmmc_cancel;
  dev->waitresponse     = esp32p4_sdmmc_waitresponse;
  dev->recv_r1          = esp32p4_sdmmc_recvshort;
  dev->recv_r2          = esp32p4_sdmmc_recvlong;
  dev->recv_r3          = esp32p4_sdmmc_recvshort;
  dev->recv_r4          = esp32p4_sdmmc_recvshort;
  dev->recv_r5          = esp32p4_sdmmc_recvshort;
  dev->recv_r6          = esp32p4_sdmmc_recvshort;
  dev->recv_r7          = esp32p4_sdmmc_recvshort;
  dev->waitenable       = esp32p4_sdmmc_waitenable;
  dev->eventwait        = esp32p4_sdmmc_eventwait;
  dev->callbackenable   = esp32p4_sdmmc_callbackenable;
#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
  dev->registercallback = esp32p4_sdmmc_registercallback;
#endif
  dev->gotextcsd        = esp32p4_sdmmc_gotextcsd;
  if(slot==0){
  /* 14. 注册到 mmcsd 子系统 */
  sdmmc_dbg("Step 14: Register to mmcsd");
  ret = mmcsd_slotinitialize(minor, dev);
  if (ret < 0)
    {
      sdmmc_err("mmcsd_slotinitialize failed: %d", ret);
      goto err_mmcsd;
    }
   }
  state->initialized = true;
  sdmmc_dbg("===== Slot %d -> /dev/mmcsd%d initialized OK =====",
         slot, minor);

  return OK;

err_mmcsd:
err_gpio:
  esp32p4_sdmmc_ldo_deinit(state);
err_ldo:
  nxsem_destroy(&state->sem);
  sdmmc_err("Init failed, ret=%d", ret);
  return ret;
}
/* 在 esp32p4_sdmmc.c 末尾添加 */
struct sdio_dev_s *esp32p4_sdmmc_getdev(int slot)
{
    if (slot < 0 || slot >= 2) return NULL;
    if (!g_sdmmc_state[slot].initialized) return NULL;
    return &g_sdmmc_state[slot].dev;
}
