/****************************************************************************
 * arch/risc-v/src/esp32p4/esp32p4_i2s.c
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

#include <sys/param.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/spinlock.h>
#include <nuttx/queue.h>
#include <nuttx/wqueue.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>

#include "riscv_internal.h"
#include "esp_gpio_p4.h"
#include "esp_irq_p4.h"
#include "esp32p4_i2s.h"

#include "soc/interrupts.h"
#include "soc/hp_sys_clkrst_struct.h"
#include "soc/lp_clkrst_struct.h"
#include "soc/ahb_dma_struct.h"
#include "soc/ahb_dma_reg.h"
#include "soc/gpio_sig_map.h"
#include "hal/i2s_ll.h"
#include "hal/i2s_hal.h"
#include "hal/ahb_dma_ll.h"
#include "hal/gdma_ll.h"
#include "hal/dma_types.h"
#include "hal/i2s_periph.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ESP32P4_I2S_MAXINFLIGHT
#  define CONFIG_ESP32P4_I2S_MAXINFLIGHT 4
#endif

#define I2S_SRC_CLK_FREQ 160000000UL /* PLL_160M clock source */

#define I2S_ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Audio buffer tracking container */

struct esp32p4_i2s_buf_s
{
  struct esp32p4_i2s_buf_s *flink;
  dma_descriptor_t dma_desc;
  i2s_callback_t callback;
  FAR void *arg;
  FAR struct ap_buffer_s *apb;
  int result;
};

/* Stream state for TX or RX */

struct esp32p4_i2s_stream_s
{
  sq_queue_t pend;
  sq_queue_t act;
  sq_queue_t done;
  struct work_s work;
  int cpuint;
  bool running;
};

/* I2S controller private structure */

struct esp32p4_i2s_priv_s
{
  struct i2s_dev_s dev;
  int port;
  int dma_channel;
  mutex_t lock;
  spinlock_t slock;
  uint32_t rate;
  uint32_t mclk;
  uint8_t data_width;
  uint8_t channels;
  struct esp32p4_i2s_stream_s tx;
  struct esp32p4_i2s_stream_s rx;
  struct esp32p4_i2s_buf_s tx_bufs[CONFIG_ESP32P4_I2S_MAXINFLIGHT];
  struct esp32p4_i2s_buf_s rx_bufs[CONFIG_ESP32P4_I2S_MAXINFLIGHT];
  sq_queue_t tx_free;
  sq_queue_t rx_free;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP32P4_I2S0_TX
static int      i2s_txchannels(FAR struct i2s_dev_s *dev, uint8_t channels);
static uint32_t i2s_txsamplerate(FAR struct i2s_dev_s *dev, uint32_t rate);
static uint32_t i2s_txdatawidth(FAR struct i2s_dev_s *dev, int bits);
static int      i2s_send(FAR struct i2s_dev_s *dev,
                         FAR struct ap_buffer_s *apb,
                         i2s_callback_t callback, FAR void *arg,
                         uint32_t timeout);
static int      i2s_tx_interrupt(int irq, FAR void *context, FAR void *arg);
static void     i2s_tx_worker(FAR void *arg);
#endif

#ifdef CONFIG_ESP32P4_I2S0_RX
static int      i2s_rxchannels(FAR struct i2s_dev_s *dev, uint8_t channels);
static uint32_t i2s_rxsamplerate(FAR struct i2s_dev_s *dev, uint32_t rate);
static uint32_t i2s_rxdatawidth(FAR struct i2s_dev_s *dev, int bits);
static int      i2s_receive(FAR struct i2s_dev_s *dev,
                            FAR struct ap_buffer_s *apb,
                            i2s_callback_t callback, FAR void *arg,
                            uint32_t timeout);
static int      i2s_rx_interrupt(int irq, FAR void *context, FAR void *arg);
static void     i2s_rx_worker(FAR void *arg);
#endif

static int      i2s_ioctl(FAR struct i2s_dev_s *dev, int cmd,
                          unsigned long arg);
static uint32_t i2s_getmclkfrequency(FAR struct i2s_dev_s *dev);
static uint32_t i2s_setmclkfrequency(FAR struct i2s_dev_s *dev,
                                     uint32_t frequency);
static void     i2s_set_clock(FAR struct esp32p4_i2s_priv_s *priv);
static void     i2s_hw_init(FAR struct esp32p4_i2s_priv_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2s_ops_s g_i2sops =
{
#ifdef CONFIG_ESP32P4_I2S0_TX
  .i2s_txchannels        = i2s_txchannels,
  .i2s_txsamplerate      = i2s_txsamplerate,
  .i2s_txdatawidth       = i2s_txdatawidth,
  .i2s_send              = i2s_send,
#endif
#ifdef CONFIG_ESP32P4_I2S0_RX
  .i2s_rxchannels        = i2s_rxchannels,
  .i2s_rxsamplerate      = i2s_rxsamplerate,
  .i2s_rxdatawidth       = i2s_rxdatawidth,
  .i2s_receive           = i2s_receive,
#endif
  .i2s_ioctl             = i2s_ioctl,
  .i2s_getmclkfrequency  = i2s_getmclkfrequency,
  .i2s_setmclkfrequency  = i2s_setmclkfrequency,
};

static struct esp32p4_i2s_priv_s g_i2s0_priv =
{
  .dev.ops     = &g_i2sops,
  .port        = 0,
  .dma_channel = 0,
  .lock        = NXMUTEX_INITIALIZER,
  .slock       = SP_UNLOCKED,
  .rate        = CONFIG_ESP32P4_I2S0_SAMPLE_RATE,
  .mclk        = CONFIG_ESP32P4_I2S0_SAMPLE_RATE * 256,
  .data_width  = CONFIG_ESP32P4_I2S0_DATA_BIT_WIDTH,
  .channels    = 2,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2s_set_clock
 *
 * Description:
 *   Calculate and apply MCLK and BCLK dividers for the given sample rate.
 ****************************************************************************/

static void i2s_set_clock(FAR struct esp32p4_i2s_priv_s *priv)
{
  i2s_dev_t *hw = I2S_LL_GET_HW(priv->port);
  hal_utils_clk_div_t mclk_div;
  uint32_t bclk_div;
  uint32_t bclk;

  if (priv->rate == 0)
    {
      return;
    }

  if (priv->mclk == 0)
    {
      priv->mclk = priv->rate * 256;
    }

  i2s_hal_calc_mclk_precise_division(I2S_SRC_CLK_FREQ,
                                     priv->mclk, &mclk_div);

  /* BCLK = rate * channels * slot_bits (32 bits slot) */

  bclk = priv->rate * priv->channels * 32;
  bclk_div = (bclk > 0) ? (priv->mclk / bclk) : 8;
  if (bclk_div == 0)
    {
      bclk_div = 1;
    }

#ifdef CONFIG_ESP32P4_I2S0_TX
  HP_SYS_CLKRST.peri_clk_ctrl13.reg_i2s0_tx_clk_en = 1;
  _i2s_ll_tx_clk_set_src(hw, I2S_CLK_SRC_PLL_160M);
  _i2s_ll_tx_set_mclk(hw, &mclk_div);
  i2s_ll_tx_set_bck_div_num(hw, bclk_div);
#endif

#ifdef CONFIG_ESP32P4_I2S0_RX
  HP_SYS_CLKRST.peri_clk_ctrl11.reg_i2s0_rx_clk_en = 1;
  _i2s_ll_rx_clk_set_src(hw, I2S_CLK_SRC_PLL_160M);
  _i2s_ll_rx_set_mclk(hw, &mclk_div);
  i2s_ll_rx_set_bck_div_num(hw, bclk_div);
#endif

  i2sinfo("I2S%d clk: rate=%" PRIu32 ", mclk=%" PRIu32
          ", bclk_div=%" PRIu32 "\n",
          priv->port, priv->rate, priv->mclk, bclk_div);
}

/****************************************************************************
 * Name: i2s_hw_init
 *
 * Description:
 *   Initialize the I2S and AHB-DMA hardware controllers.
 ****************************************************************************/

static void i2s_hw_init(FAR struct esp32p4_i2s_priv_s *priv)
{
  i2s_dev_t *hw = I2S_LL_GET_HW(priv->port);
  int i;

  /* 1. Enable AHB-DMA bus clock and reset */

  HP_SYS_CLKRST.soc_clk_ctrl1.reg_ahb_pdma_sys_clk_en = 1;
  HP_SYS_CLKRST.hp_rst_en1.reg_rst_en_ahb_pdma = 0;
  ahb_dma_ll_force_enable_reg_clock(&AHB_DMA, true);
  ahb_dma_ll_set_default_memory_range(&AHB_DMA);

  /* 2. Enable I2S bus clock and reset */

  HP_SYS_CLKRST.soc_clk_ctrl2.reg_i2s0_apb_clk_en = 1;
  LP_AON_CLKRST.hp_clk_ctrl.hp_pad_i2s0_mclk_en = 1;
  HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_i2s0_apb = 1;
  HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_i2s0_apb = 0;

#ifdef CONFIG_ESP32P4_I2S0_TX
  HP_SYS_CLKRST.peri_clk_ctrl13.reg_i2s0_tx_clk_en = 1;
  _i2s_ll_mclk_bind_to_tx_clk(hw);
#endif

#ifdef CONFIG_ESP32P4_I2S0_RX
  HP_SYS_CLKRST.peri_clk_ctrl11.reg_i2s0_rx_clk_en = 1;
  _i2s_ll_mclk_bind_to_rx_clk(hw);
#endif

  /* 3. Configure GPIOs via Matrix MUX */

#if defined(CONFIG_ESP32P4_I2S0_MCLKPIN) && (CONFIG_ESP32P4_I2S0_MCLKPIN >= 0)
  esp_configgpio(CONFIG_ESP32P4_I2S0_MCLKPIN, OUTPUT);
  esp_gpio_matrix_out(CONFIG_ESP32P4_I2S0_MCLKPIN,
                      I2S0_MCLK_PAD_OUT_IDX, false, false);
#endif

#ifdef CONFIG_ESP32P4_I2S0_BCLKPIN
  esp_configgpio(CONFIG_ESP32P4_I2S0_BCLKPIN, OUTPUT);
  esp_gpio_matrix_out(CONFIG_ESP32P4_I2S0_BCLKPIN,
                      I2S0_O_BCK_PAD_OUT_IDX, false, false);
#endif

#ifdef CONFIG_ESP32P4_I2S0_WSPIN
  esp_configgpio(CONFIG_ESP32P4_I2S0_WSPIN, OUTPUT);
  esp_gpio_matrix_out(CONFIG_ESP32P4_I2S0_WSPIN,
                      I2S0_O_WS_PAD_OUT_IDX, false, false);
#endif

#ifdef CONFIG_ESP32P4_I2S0_DOUTPIN
  esp_configgpio(CONFIG_ESP32P4_I2S0_DOUTPIN, OUTPUT);
  esp_gpio_matrix_out(CONFIG_ESP32P4_I2S0_DOUTPIN,
                      I2S0_O_SD_PAD_OUT_IDX, false, false);
#endif

#ifdef CONFIG_ESP32P4_I2S0_DINPIN
  esp_configgpio(CONFIG_ESP32P4_I2S0_DINPIN, INPUT | PULLUP);
  esp_gpio_matrix_in(CONFIG_ESP32P4_I2S0_DINPIN,
                     I2S0_I_SD_PAD_IN_IDX, false);
#endif

  /* 4. Configure I2S TX & RX Slot, Standard Philips format */

#ifdef CONFIG_ESP32P4_I2S0_TX
  i2s_ll_tx_reset(hw);
  i2s_ll_tx_reset_fifo(hw);
  i2s_ll_tx_set_slave_mod(hw, false);
  i2s_ll_tx_enable_msb_shift(hw, true);
  i2s_ll_tx_set_ws_width(hw, 32);
  i2s_ll_tx_set_sample_bit(hw, 32, priv->data_width);
  i2s_ll_tx_set_half_sample_bit(hw, 32);
  i2s_ll_tx_set_chan_num(hw, 2);
  i2s_ll_tx_set_active_chan_mask(hw, 0x03);

  /* AHB-DMA TX Channel setup */

  ahb_dma_ll_tx_connect_to_periph(&AHB_DMA, priv->dma_channel,
                                  SOC_GDMA_TRIG_PERIPH_I2S0);
  ahb_dma_ll_tx_set_priority(&AHB_DMA, priv->dma_channel, 1);
  ahb_dma_ll_tx_enable_data_burst(&AHB_DMA, priv->dma_channel, true);
  ahb_dma_ll_tx_enable_descriptor_burst(&AHB_DMA, priv->dma_channel, true);
  ahb_dma_ll_tx_reset_channel(&AHB_DMA, priv->dma_channel);
#endif

#ifdef CONFIG_ESP32P4_I2S0_RX
  i2s_ll_rx_reset(hw);
  i2s_ll_rx_reset_fifo(hw);
  i2s_ll_rx_set_slave_mod(hw, true); /* RX slave shares TX clock */
  i2s_ll_rx_enable_msb_shift(hw, true);
  i2s_ll_rx_set_ws_width(hw, 32);
  i2s_ll_rx_set_sample_bit(hw, 32, priv->data_width);
  i2s_ll_rx_set_half_sample_bit(hw, 32);
  i2s_ll_rx_set_chan_num(hw, 2);
  i2s_ll_rx_set_active_chan_mask(hw, 0x03);

  /* AHB-DMA RX Channel setup */

  ahb_dma_ll_rx_connect_to_periph(&AHB_DMA, priv->dma_channel,
                                  SOC_GDMA_TRIG_PERIPH_I2S0);
  ahb_dma_ll_rx_set_priority(&AHB_DMA, priv->dma_channel, 1);
  ahb_dma_ll_rx_enable_data_burst(&AHB_DMA, priv->dma_channel, true);
  ahb_dma_ll_rx_enable_descriptor_burst(&AHB_DMA, priv->dma_channel, true);
  ahb_dma_ll_rx_reset_channel(&AHB_DMA, priv->dma_channel);
#endif

  /* 5. Initialize buffer queues */

  sq_init(&priv->tx.pend);
  sq_init(&priv->tx.act);
  sq_init(&priv->tx.done);
  sq_init(&priv->tx_free);

  sq_init(&priv->rx.pend);
  sq_init(&priv->rx.act);
  sq_init(&priv->rx.done);
  sq_init(&priv->rx_free);

  for (i = 0; i < CONFIG_ESP32P4_I2S_MAXINFLIGHT; i++)
    {
      sq_addlast((FAR sq_entry_t *)&priv->tx_bufs[i], &priv->tx_free);
      sq_addlast((FAR sq_entry_t *)&priv->rx_bufs[i], &priv->rx_free);
    }

  /* 6. Configure clock rates */

  i2s_set_clock(priv);

  /* 7. Setup DMA Interrupts */

#ifdef CONFIG_ESP32P4_I2S0_TX
  priv->tx.cpuint = esp_setup_irq(ETS_AHB_PDMA_OUT_CH0_INTR_SOURCE, 1,
                                  ESP_IRQ_TRIGGER_LEVEL,
                                  i2s_tx_interrupt, priv);
  DEBUGASSERT(priv->tx.cpuint >= 0);
  up_enable_irq(priv->tx.cpuint);
  ahb_dma_ll_tx_enable_interrupt(&AHB_DMA, priv->dma_channel,
                                 AHB_DMA_OUT_TOTAL_EOF_CH0_INT_ENA, true);
#endif

#ifdef CONFIG_ESP32P4_I2S0_RX
  priv->rx.cpuint = esp_setup_irq(ETS_AHB_PDMA_IN_CH0_INTR_SOURCE, 1,
                                  ESP_IRQ_TRIGGER_LEVEL,
                                  i2s_rx_interrupt, priv);
  DEBUGASSERT(priv->rx.cpuint >= 0);
  up_enable_irq(priv->rx.cpuint);
  ahb_dma_ll_rx_enable_interrupt(&AHB_DMA, priv->dma_channel,
                                 AHB_DMA_IN_SUC_EOF_CH0_INT_ENA, true);
#endif
}

#ifdef CONFIG_ESP32P4_I2S0_TX

/****************************************************************************
 * Name: i2s_tx_worker
 ****************************************************************************/

static void i2s_tx_worker(FAR void *arg)
{
  FAR struct esp32p4_i2s_priv_s *priv = (FAR struct esp32p4_i2s_priv_s *)arg;
  FAR struct esp32p4_i2s_buf_s *buf;
  irqstate_t flags;

  for (; ; )
    {
      flags = spin_lock_irqsave(&priv->slock);
      buf = (FAR struct esp32p4_i2s_buf_s *)sq_remfirst(&priv->tx.done);
      spin_unlock_irqrestore(&priv->slock, flags);

      if (buf == NULL)
        {
          break;
        }

      if (buf->callback != NULL)
        {
          buf->callback(&priv->dev, buf->apb, buf->arg, buf->result);
        }

      flags = spin_lock_irqsave(&priv->slock);
      sq_addlast((FAR sq_entry_t *)buf, &priv->tx_free);
      spin_unlock_irqrestore(&priv->slock, flags);
    }
}

/****************************************************************************
 * Name: i2s_tx_interrupt
 ****************************************************************************/

static int i2s_tx_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct esp32p4_i2s_priv_s *priv = (FAR struct esp32p4_i2s_priv_s *)arg;
  uint32_t st = ahb_dma_ll_tx_get_interrupt_status(&AHB_DMA,
                                                  priv->dma_channel, false);
  FAR struct esp32p4_i2s_buf_s *buf;
  FAR struct esp32p4_i2s_buf_s *next;

  ahb_dma_ll_tx_clear_interrupt_status(&AHB_DMA, priv->dma_channel, st);

  if ((st & AHB_DMA_OUT_TOTAL_EOF_CH0_INT_ST) != 0)
    {
      buf = (FAR struct esp32p4_i2s_buf_s *)sq_remfirst(&priv->tx.act);
      if (buf != NULL)
        {
          buf->result = OK;
          sq_addlast((FAR sq_entry_t *)buf, &priv->tx.done);
        }

      next = (FAR struct esp32p4_i2s_buf_s *)sq_remfirst(&priv->tx.pend);
      if (next != NULL)
        {
          sq_addlast((FAR sq_entry_t *)next, &priv->tx.act);
          ahb_dma_ll_tx_reset_channel(&AHB_DMA, priv->dma_channel);
          ahb_dma_ll_tx_set_desc_addr(&AHB_DMA, priv->dma_channel,
                                      (uint32_t)&next->dma_desc);
          ahb_dma_ll_tx_start(&AHB_DMA, priv->dma_channel);
          i2s_ll_tx_start(I2S_LL_GET_HW(priv->port));
        }
      else
        {
          priv->tx.running = false;
        }

      work_queue(HPWORK, &priv->tx.work, i2s_tx_worker, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: i2s_txchannels
 ****************************************************************************/

static int i2s_txchannels(FAR struct i2s_dev_s *dev, uint8_t channels)
{
  FAR struct esp32p4_i2s_priv_s *priv = (FAR struct esp32p4_i2s_priv_s *)dev;
  if (channels != 1 && channels != 2)
    {
      return -EINVAL;
    }

  priv->channels = channels;
  i2s_set_clock(priv);
  return OK;
}

/****************************************************************************
 * Name: i2s_txsamplerate
 ****************************************************************************/

static uint32_t i2s_txsamplerate(FAR struct i2s_dev_s *dev, uint32_t rate)
{
  FAR struct esp32p4_i2s_priv_s *priv = (FAR struct esp32p4_i2s_priv_s *)dev;
  priv->rate = rate;
  priv->mclk = rate * 256;
  i2s_set_clock(priv);
  return priv->rate;
}

/****************************************************************************
 * Name: i2s_txdatawidth
 ****************************************************************************/

static uint32_t i2s_txdatawidth(FAR struct i2s_dev_s *dev, int bits)
{
  FAR struct esp32p4_i2s_priv_s *priv = (FAR struct esp32p4_i2s_priv_s *)dev;
  i2s_dev_t *hw = I2S_LL_GET_HW(priv->port);

  if (bits != 16 && bits != 24 && bits != 32)
    {
      return -EINVAL;
    }

  priv->data_width = bits;
  i2s_ll_tx_set_sample_bit(hw, 32, priv->data_width);
  return priv->data_width;
}

/****************************************************************************
 * Name: i2s_send
 ****************************************************************************/

static int i2s_send(FAR struct i2s_dev_s *dev,
                    FAR struct ap_buffer_s *apb,
                    i2s_callback_t callback, FAR void *arg,
                    uint32_t timeout)
{
  FAR struct esp32p4_i2s_priv_s *priv = (FAR struct esp32p4_i2s_priv_s *)dev;
  FAR struct esp32p4_i2s_buf_s *buf;
  irqstate_t flags;

  if (apb == NULL || apb->nbytes == 0)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&priv->slock);
  buf = (FAR struct esp32p4_i2s_buf_s *)sq_remfirst(&priv->tx_free);
  if (buf == NULL)
    {
      spin_unlock_irqrestore(&priv->slock, flags);
      return -ENOMEM;
    }

  buf->apb = apb;
  buf->callback = callback;
  buf->arg = arg;
  buf->result = OK;

  buf->dma_desc.dw0.size = I2S_ALIGN_UP(apb->nbytes, 4);
  buf->dma_desc.dw0.length = apb->nbytes;
  buf->dma_desc.dw0.owner = 1;
  buf->dma_desc.dw0.suc_eof = 1;
  buf->dma_desc.buffer = apb->samp;
  buf->dma_desc.next = NULL;

  if (!priv->tx.running)
    {
      priv->tx.running = true;
      sq_addlast((FAR sq_entry_t *)buf, &priv->tx.act);
      ahb_dma_ll_tx_reset_channel(&AHB_DMA, priv->dma_channel);
      ahb_dma_ll_tx_set_desc_addr(&AHB_DMA, priv->dma_channel,
                                  (uint32_t)&buf->dma_desc);
      ahb_dma_ll_tx_start(&AHB_DMA, priv->dma_channel);
      i2s_ll_tx_start(I2S_LL_GET_HW(priv->port));
    }
  else
    {
      sq_addlast((FAR sq_entry_t *)buf, &priv->tx.pend);
    }

  spin_unlock_irqrestore(&priv->slock, flags);
  return OK;
}

#endif /* CONFIG_ESP32P4_I2S0_TX */

#ifdef CONFIG_ESP32P4_I2S0_RX

/****************************************************************************
 * Name: i2s_rx_worker
 ****************************************************************************/

static void i2s_rx_worker(FAR void *arg)
{
  FAR struct esp32p4_i2s_priv_s *priv = (FAR struct esp32p4_i2s_priv_s *)arg;
  FAR struct esp32p4_i2s_buf_s *buf;
  irqstate_t flags;

  for (; ; )
    {
      flags = spin_lock_irqsave(&priv->slock);
      buf = (FAR struct esp32p4_i2s_buf_s *)sq_remfirst(&priv->rx.done);
      spin_unlock_irqrestore(&priv->slock, flags);

      if (buf == NULL)
        {
          break;
        }

      if (buf->callback != NULL)
        {
          buf->callback(&priv->dev, buf->apb, buf->arg, buf->result);
        }

      flags = spin_lock_irqsave(&priv->slock);
      sq_addlast((FAR sq_entry_t *)buf, &priv->rx_free);
      spin_unlock_irqrestore(&priv->slock, flags);
    }
}

/****************************************************************************
 * Name: i2s_rx_interrupt
 ****************************************************************************/

static int i2s_rx_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct esp32p4_i2s_priv_s *priv = (FAR struct esp32p4_i2s_priv_s *)arg;
  uint32_t st = ahb_dma_ll_rx_get_interrupt_status(&AHB_DMA,
                                                  priv->dma_channel, false);
  FAR struct esp32p4_i2s_buf_s *buf;
  FAR struct esp32p4_i2s_buf_s *next;

  ahb_dma_ll_rx_clear_interrupt_status(&AHB_DMA, priv->dma_channel, st);

  if ((st & AHB_DMA_IN_SUC_EOF_CH0_INT_ST) != 0)
    {
      buf = (FAR struct esp32p4_i2s_buf_s *)sq_remfirst(&priv->rx.act);
      if (buf != NULL)
        {
          buf->apb->nbytes = buf->apb->nmaxbytes;
          buf->result = OK;
          sq_addlast((FAR sq_entry_t *)buf, &priv->rx.done);
        }

      next = (FAR struct esp32p4_i2s_buf_s *)sq_remfirst(&priv->rx.pend);
      if (next != NULL)
        {
          sq_addlast((FAR sq_entry_t *)next, &priv->rx.act);
          i2s_ll_rx_set_eof_num(I2S_LL_GET_HW(priv->port),
                                next->apb->nmaxbytes);
          ahb_dma_ll_rx_reset_channel(&AHB_DMA, priv->dma_channel);
          ahb_dma_ll_rx_set_desc_addr(&AHB_DMA, priv->dma_channel,
                                      (uint32_t)&next->dma_desc);
          ahb_dma_ll_rx_start(&AHB_DMA, priv->dma_channel);
          i2s_ll_rx_start(I2S_LL_GET_HW(priv->port));
        }
      else
        {
          priv->rx.running = false;
        }

      work_queue(HPWORK, &priv->rx.work, i2s_rx_worker, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: i2s_rxchannels
 ****************************************************************************/

static int i2s_rxchannels(FAR struct i2s_dev_s *dev, uint8_t channels)
{
  FAR struct esp32p4_i2s_priv_s *priv = (FAR struct esp32p4_i2s_priv_s *)dev;
  if (channels != 1 && channels != 2)
    {
      return -EINVAL;
    }

  priv->channels = channels;
  i2s_set_clock(priv);
  return OK;
}

/****************************************************************************
 * Name: i2s_rxsamplerate
 ****************************************************************************/

static uint32_t i2s_rxsamplerate(FAR struct i2s_dev_s *dev, uint32_t rate)
{
  FAR struct esp32p4_i2s_priv_s *priv = (FAR struct esp32p4_i2s_priv_s *)dev;
  priv->rate = rate;
  priv->mclk = rate * 256;
  i2s_set_clock(priv);
  return priv->rate;
}

/****************************************************************************
 * Name: i2s_rxdatawidth
 ****************************************************************************/

static uint32_t i2s_rxdatawidth(FAR struct i2s_dev_s *dev, int bits)
{
  FAR struct esp32p4_i2s_priv_s *priv = (FAR struct esp32p4_i2s_priv_s *)dev;
  i2s_dev_t *hw = I2S_LL_GET_HW(priv->port);

  if (bits != 16 && bits != 24 && bits != 32)
    {
      return -EINVAL;
    }

  priv->data_width = bits;
  i2s_ll_rx_set_sample_bit(hw, 32, priv->data_width);
  return priv->data_width;
}

/****************************************************************************
 * Name: i2s_receive
 ****************************************************************************/

static int i2s_receive(FAR struct i2s_dev_s *dev,
                       FAR struct ap_buffer_s *apb,
                       i2s_callback_t callback, FAR void *arg,
                       uint32_t timeout)
{
  FAR struct esp32p4_i2s_priv_s *priv = (FAR struct esp32p4_i2s_priv_s *)dev;
  FAR struct esp32p4_i2s_buf_s *buf;
  irqstate_t flags;

  if (apb == NULL || apb->nmaxbytes == 0)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&priv->slock);
  buf = (FAR struct esp32p4_i2s_buf_s *)sq_remfirst(&priv->rx_free);
  if (buf == NULL)
    {
      spin_unlock_irqrestore(&priv->slock, flags);
      return -ENOMEM;
    }

  buf->apb = apb;
  buf->callback = callback;
  buf->arg = arg;
  buf->result = OK;

  buf->dma_desc.dw0.size = I2S_ALIGN_UP(apb->nmaxbytes, 4);
  buf->dma_desc.dw0.length = 0;
  buf->dma_desc.dw0.owner = 1;
  buf->dma_desc.dw0.suc_eof = 1;
  buf->dma_desc.buffer = apb->samp;
  buf->dma_desc.next = NULL;

  if (!priv->rx.running)
    {
      priv->rx.running = true;
      sq_addlast((FAR sq_entry_t *)buf, &priv->rx.act);
      i2s_ll_rx_set_eof_num(I2S_LL_GET_HW(priv->port), apb->nmaxbytes);
      ahb_dma_ll_rx_reset_channel(&AHB_DMA, priv->dma_channel);
      ahb_dma_ll_rx_set_desc_addr(&AHB_DMA, priv->dma_channel,
                                  (uint32_t)&buf->dma_desc);
      ahb_dma_ll_rx_start(&AHB_DMA, priv->dma_channel);
      i2s_ll_rx_start(I2S_LL_GET_HW(priv->port));
    }
  else
    {
      sq_addlast((FAR sq_entry_t *)buf, &priv->rx.pend);
    }

  spin_unlock_irqrestore(&priv->slock, flags);
  return OK;
}

#endif /* CONFIG_ESP32P4_I2S0_RX */

/****************************************************************************
 * Name: i2s_ioctl
 ****************************************************************************/

static int i2s_ioctl(FAR struct i2s_dev_s *dev, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: i2s_getmclkfrequency
 ****************************************************************************/

static uint32_t i2s_getmclkfrequency(FAR struct i2s_dev_s *dev)
{
  FAR struct esp32p4_i2s_priv_s *priv = (FAR struct esp32p4_i2s_priv_s *)dev;
  return priv->mclk;
}

/****************************************************************************
 * Name: i2s_setmclkfrequency
 ****************************************************************************/

static uint32_t i2s_setmclkfrequency(FAR struct i2s_dev_s *dev,
                                     uint32_t frequency)
{
  FAR struct esp32p4_i2s_priv_s *priv = (FAR struct esp32p4_i2s_priv_s *)dev;
  priv->mclk = frequency;
  i2s_set_clock(priv);
  return priv->mclk;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32p4_i2sbus_initialize
 ****************************************************************************/

FAR struct i2s_dev_s *esp32p4_i2sbus_initialize(int port)
{
  FAR struct esp32p4_i2s_priv_s *priv;

  if (port == 0)
    {
      priv = &g_i2s0_priv;
    }
  else
    {
      return NULL;
    }

  nxmutex_lock(&priv->lock);
  i2s_hw_init(priv);
  nxmutex_unlock(&priv->lock);

  return &priv->dev;
}
