/****************************************************************************
 * arch/arm/src/bk7258/bk7258_serial.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/bk7258_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Console = UART0 (on-board CH340).  NuttX runs as the CPU0 main core
 * (replacing the ARMINO CP app); CPU0 owns UART0 and the bootloader has
 * already set up its clock/pins.  115200 8N1, IRQ = EXTINT + 4.
 */

#define CONSOLE_BASE   BK7258_UART0_BASE
#define CONSOLE_IRQ    BK7258_IRQ_UART0
#define CONSOLE_BAUD   115200
#define BK7258_UARTCLK 26000000u

#ifndef CONFIG_UART0_RXBUFSIZE
#  define CONFIG_UART0_RXBUFSIZE 256
#endif
#ifndef CONFIG_UART0_TXBUFSIZE
#  define CONFIG_UART0_TXBUFSIZE 256
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bk7258_uart_s
{
  uint32_t uartbase;
  uint32_t baud;
  uint8_t  irq;
  uint8_t  ie;      /* Shadow of enabled interrupt bits */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  bk7258_setup(struct uart_dev_s *dev);
static void bk7258_shutdown(struct uart_dev_s *dev);
static int  bk7258_attach(struct uart_dev_s *dev);
static void bk7258_detach(struct uart_dev_s *dev);
static int  bk7258_interrupt(int irq, void *context, void *arg);
static int  bk7258_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  bk7258_receive(struct uart_dev_s *dev, unsigned int *status);
static void bk7258_rxint(struct uart_dev_s *dev, bool enable);
static bool bk7258_rxavailable(struct uart_dev_s *dev);
static void bk7258_send(struct uart_dev_s *dev, int ch);
static void bk7258_txint(struct uart_dev_s *dev, bool enable);
static bool bk7258_txready(struct uart_dev_s *dev);
static bool bk7258_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup       = bk7258_setup,
  .shutdown    = bk7258_shutdown,
  .attach      = bk7258_attach,
  .detach      = bk7258_detach,
  .ioctl       = bk7258_ioctl,
  .receive     = bk7258_receive,
  .rxint       = bk7258_rxint,
  .rxavailable = bk7258_rxavailable,
  .send        = bk7258_send,
  .txint       = bk7258_txint,
  .txready     = bk7258_txready,
  .txempty     = bk7258_txempty,
};

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

static struct bk7258_uart_s g_uart0priv =
{
  .uartbase = CONSOLE_BASE,
  .baud     = CONSOLE_BAUD,
  .irq      = CONSOLE_IRQ,
  .ie       = 0,
};

static struct uart_dev_s g_uart0port =
{
  .isconsole = true,
  .recv      =
  {
    .size    = CONFIG_UART0_RXBUFSIZE,
    .buffer  = g_uart0rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART0_TXBUFSIZE,
    .buffer  = g_uart0txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart0priv,
};

#define CONSOLE_DEV g_uart0port

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void bk7258_restoreint(struct bk7258_uart_s *priv)
{
  putreg32(priv->ie, BK7258_UART_INT_ENABLE(priv->uartbase));
}

/****************************************************************************
 * Name: bk7258_setup
 ****************************************************************************/

static int bk7258_setup(struct uart_dev_s *dev)
{
  struct bk7258_uart_s *priv = dev->priv;
  uint32_t div = (BK7258_UARTCLK + (priv->baud / 2)) / priv->baud - 1;
  uint32_t cfg;
  uint32_t fc;

  /* Do not soft reset (it would break the bootloader's working UART0 TX
   * state).  Only set 8N1 / baud / TX+RX enable on top of the existing
   * config, keeping the bootloader's clock/FIFO state.
   */

  cfg  = getreg32(BK7258_UART_CONFIG(priv->uartbase));
  cfg &= ~(UART_CFG_DATA_BITS_MASK | UART_CFG_CLK_DIV_MASK);
  cfg |= UART_CFG_DATA_BITS_8;
  cfg |= (div << UART_CFG_CLK_DIV_SHIFT) & UART_CFG_CLK_DIV_MASK;
  cfg |= UART_CFG_TX_ENABLE | UART_CFG_RX_ENABLE;
  putreg32(cfg, BK7258_UART_CONFIG(priv->uartbase));

  /* FIFO config: set RX threshold to 1 and enable RX stop detection so a
   * single keystroke triggers the RX interrupt.  (Without the soft reset,
   * fifo_config keeps the bootloader's high RX threshold, so interactive
   * input never triggers an interrupt and received characters pile up in
   * the RX FIFO -- SWD measured rx_fifo_count=60.)
   * fifo_config (0x14): tx_threshold[7:0] | rx_threshold[15:8] |
   * rx_stop_detect[17:16].
   */

  fc  = getreg32(BK7258_UART_FIFO_CFG(priv->uartbase));
  fc &= ~((0xffu << 8) | (0x3u << 16));
  fc |= (0x01u << 8) | (0x02u << 16);
  putreg32(fc, BK7258_UART_FIFO_CFG(priv->uartbase));

  /* All interrupts disabled for now */

  priv->ie = 0;
  bk7258_restoreint(priv);
  return OK;
}

static void bk7258_shutdown(struct uart_dev_s *dev)
{
  struct bk7258_uart_s *priv = dev->priv;
  priv->ie = 0;
  bk7258_restoreint(priv);
}

static int bk7258_attach(struct uart_dev_s *dev)
{
  struct bk7258_uart_s *priv = dev->priv;
  int ret = irq_attach(priv->irq, bk7258_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

static void bk7258_detach(struct uart_dev_s *dev)
{
  struct bk7258_uart_s *priv = dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

static int bk7258_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct bk7258_uart_s *priv = dev->priv;
  uint32_t status;

  status = getreg32(BK7258_UART_INT_STATUS(priv->uartbase));

  /* Write 1 to clear the asserted status bits */

  putreg32(status, BK7258_UART_INT_STATUS(priv->uartbase));

  if (status & (UART_INT_RX_NEED_READ | UART_INT_RX_FINISH))
    {
      uart_recvchars(dev);
    }

  if (status & UART_INT_TX_NEED_WRITE)
    {
      uart_xmitchars(dev);
    }

  return OK;
}

static int bk7258_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  UNUSED(filep);
  UNUSED(cmd);
  UNUSED(arg);
  return -ENOTTY;
}

static int bk7258_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct bk7258_uart_s *priv = dev->priv;
  uint32_t rbr = getreg32(BK7258_UART_FIFO_PORT(priv->uartbase));

  *status = 0;
  return (int)((rbr >> UART_FIFO_RX_DATA_SHIFT) & 0xff);
}

static void bk7258_rxint(struct uart_dev_s *dev, bool enable)
{
  struct bk7258_uart_s *priv = dev->priv;
  irqstate_t flags = enter_critical_section();

  if (enable)
    {
      priv->ie |= UART_INT_RX_NEED_READ | UART_INT_RX_FINISH;
    }
  else
    {
      priv->ie &= ~(UART_INT_RX_NEED_READ | UART_INT_RX_FINISH);
    }

  bk7258_restoreint(priv);
  leave_critical_section(flags);
}

static bool bk7258_rxavailable(struct uart_dev_s *dev)
{
  struct bk7258_uart_s *priv = dev->priv;
  return (getreg32(BK7258_UART_FIFO_STATUS(priv->uartbase)) &
          UART_FIFO_RD_READY) != 0;
}

static void bk7258_send(struct uart_dev_s *dev, int ch)
{
  struct bk7258_uart_s *priv = dev->priv;
  putreg32((uint32_t)(uint8_t)ch, BK7258_UART_FIFO_PORT(priv->uartbase));
}

static void bk7258_txint(struct uart_dev_s *dev, bool enable)
{
  struct bk7258_uart_s *priv = dev->priv;
  irqstate_t flags = enter_critical_section();

  if (enable)
    {
      priv->ie |= UART_INT_TX_NEED_WRITE;
      bk7258_restoreint(priv);

      /* Fake a TX interrupt to prime the pump */

      uart_xmitchars(dev);
    }
  else
    {
      priv->ie &= ~UART_INT_TX_NEED_WRITE;
      bk7258_restoreint(priv);
    }

  leave_critical_section(flags);
}

static bool bk7258_txready(struct uart_dev_s *dev)
{
  struct bk7258_uart_s *priv = dev->priv;
  return (getreg32(BK7258_UART_FIFO_STATUS(priv->uartbase)) &
          UART_FIFO_WR_READY) != 0;
}

static bool bk7258_txempty(struct uart_dev_s *dev)
{
  struct bk7258_uart_s *priv = dev->priv;
  return (getreg32(BK7258_UART_FIFO_STATUS(priv->uartbase)) &
          UART_FIFO_TX_EMPTY) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm_earlyserialinit(void)
{
  /* The console low level is already set up by bk7258_lowsetup(); make sure
   * RX is enabled here as well.
   */

  CONSOLE_DEV.isconsole = true;
  bk7258_setup(&CONSOLE_DEV);
}

void arm_serialinit(void)
{
  uart_register("/dev/console", &g_uart0port);
  uart_register("/dev/ttyS0", &g_uart0port);
}

void up_putc(int ch)
{
  arm_lowputc((char)ch);
}
