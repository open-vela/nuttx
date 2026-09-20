/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_serial.c
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
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/spinlock.h>

#include <arch/board/board.h>

#include "loongarch_internal.h"
#include "ls2k0300_config.h"
#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_uart.h"
#include "ls2k0300.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef USE_SERIALDRIVER

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0port
#    define TTYS0_DEV       g_uart0port
#    define SERIAL_CONSOLE  1
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart1port
#    define TTYS0_DEV       g_uart1port
#    define SERIAL_CONSOLE  1
#  else
#    error "No valid console UART defined"
#  endif
#else
#  undef  CONSOLE_DEV
#  if defined(CONFIG_LS2K0300_UART0)
#    define TTYS0_DEV       g_uart0port
#    define SERIAL_CONSOLE  1
#  endif
#endif

#ifdef HAVE_UART_DEVICE

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uintptr_t uartbase;
  uint32_t  baud;
  uint8_t   irq;
  uint8_t   im;
  spinlock_t lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint32_t up_serialin(struct up_dev_s *priv, int offset);
static void up_serialout(struct up_dev_s *priv, int offset, uint32_t value);
static void up_restoreuartint(struct up_dev_s *priv, uint8_t im);
static void up_disableuartint(struct up_dev_s *priv, uint8_t *im);

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

#ifdef CONFIG_LS2K0300_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

static struct up_dev_s g_uart0priv =
{
  .uartbase  = LS2K0300_UART0_BASE,
  .baud      = CONFIG_UART0_BAUD,
  .irq       = LS2K0300_IRQ_UART0,
  .lock      = SP_UNLOCKED,
};

static uart_dev_t g_uart0port =
{
#if SERIAL_CONSOLE == 1
  .isconsole = 1,
#endif
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
#endif

#ifdef CONFIG_LS2K0300_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

static struct up_dev_s g_uart1priv =
{
  .uartbase  = LS2K0300_UART1_BASE,
  .baud      = CONFIG_UART1_BAUD,
  .irq       = LS2K0300_IRQ_UART1,
  .lock      = SP_UNLOCKED,
};

static uart_dev_t g_uart1port =
{
  .isconsole = 0,
  .recv      =
  {
    .size    = CONFIG_UART1_RXBUFSIZE,
    .buffer  = g_uart1rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART1_TXBUFSIZE,
    .buffer  = g_uart1txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart1priv,
};
#endif

#ifdef CONFIG_LS2K0300_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];

static struct up_dev_s g_uart2priv =
{
  .uartbase  = LS2K0300_UART2_BASE,
  .baud      = CONFIG_UART2_BAUD,
  .irq       = LS2K0300_IRQ_UART2,
  .lock      = SP_UNLOCKED,
};

static uart_dev_t g_uart2port =
{
  .isconsole = 0,
  .recv      =
  {
    .size    = CONFIG_UART2_RXBUFSIZE,
    .buffer  = g_uart2rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART2_TXBUFSIZE,
    .buffer  = g_uart2txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart2priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg8(priv->uartbase + offset);
}

static void up_serialout(struct up_dev_s *priv, int offset, uint32_t value)
{
  putreg8((uint8_t)value, priv->uartbase + offset);
}

static void up_restoreuartint(struct up_dev_s *priv, uint8_t im)
{
  irqstate_t flags = spin_lock_irqsave(&priv->lock);

  priv->im = im;
  up_serialout(priv, UART_IER_OFFSET, im);

  spin_unlock_irqrestore(&priv->lock, flags);
}

static void up_disableuartint(struct up_dev_s *priv, uint8_t *im)
{
  irqstate_t flags = spin_lock_irqsave(&priv->lock);

  if (im)
    {
      *im = priv->im;
    }

  priv->im = 0;
  up_serialout(priv, UART_IER_OFFSET, 0);
  spin_unlock_irqrestore(&priv->lock, flags);
}

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t uart_ref_clk = LS2K0300_APB_FREQ * 1000000;
  uint32_t div_val = (uart_ref_clk + (priv->baud * 8)) / (priv->baud * 16);

  up_serialout(priv, UART_LCR_OFFSET, UART_LCR_DLAB);
  up_serialout(priv, UART_DLL_OFFSET, div_val & 0xff);
  up_serialout(priv, UART_DLH_OFFSET, (div_val >> 8) & 0xff);
  up_serialout(priv, UART_LCR_OFFSET, UART_LCR_WLEN8);
  up_serialout(priv, UART_FCR_OFFSET,
               UART_FCR_FIFOEN | UART_FCR_RXRST | UART_FCR_TXRST |
               UART_FCR_RXTRG_1);

  return OK;
}

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_disableuartint(priv, NULL);
}

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  up_serialout(priv, UART_IER_OFFSET,
               UART_IER_ERBFI | UART_IER_ETBEI);

  ret = irq_attach(priv->irq, up_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv = (struct up_dev_s *)dev->priv;
  uint32_t           status;
  int                passes;

  for (passes = 0; passes < 256; passes++)
    {
      status = up_serialin(priv, UART_IIR_OFFSET);

      if ((status & UART_IIR_NOINT) || status == 0)
        {
          break;
        }

      if ((status & 0x0e) == UART_IIR_RDI ||
          (status & 0x0e) == UART_IIR_CTI)
        {
          uart_recvchars(dev);
        }

      if ((status & 0x0e) == UART_IIR_THRI)
        {
          uart_xmitchars(dev);
        }
    }

  return OK;
}

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  if (status)
    {
      *status = 0;
    }

  return up_serialin(priv, UART_RBR_OFFSET) & 0xff;
}

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= UART_IER_ERBFI;
#endif
    }
  else
    {
      priv->im &= ~UART_IER_ERBFI;
    }

  up_serialout(priv, UART_IER_OFFSET, priv->im);
  leave_critical_section(flags);
}

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return !!(up_serialin(priv, UART_LSR_OFFSET) & UART_LSR_DR);
}

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_serialout(priv, UART_THR_OFFSET, (uint32_t)ch);
}

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= UART_IER_ETBEI;
      up_serialout(priv, UART_IER_OFFSET, priv->im);
      uart_xmitchars(dev);
#endif
    }
  else
    {
      priv->im &= ~UART_IER_ETBEI;
      up_serialout(priv, UART_IER_OFFSET, priv->im);
    }

  leave_critical_section(flags);
}

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return !!(up_serialin(priv, UART_LSR_OFFSET) & UART_LSR_THRE);
}

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return !!(up_serialin(priv, UART_LSR_OFFSET) & UART_LSR_TEMT);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

void loongarch_earlyserialinit(void)
{
#ifdef CONFIG_LS2K0300_UART0
  up_disableuartint(TTYS0_DEV.priv, NULL);
#endif

#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}

#endif

void loongarch_serialinit(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

#ifdef CONFIG_LS2K0300_UART0
  uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif

#ifdef CONFIG_LS2K0300_UART2
  uart_register("/dev/ttyS2", &g_uart2port);
#endif
}

void up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  uint8_t imr;

  up_disableuartint(priv, &imr);
  loongarch_lowputc(ch);
  up_restoreuartint(priv, imr);
#endif
}

#else /* HAVE_UART_DEVICE */

void loongarch_earlyserialinit(void)
{
}

void loongarch_serialinit(void)
{
}

void up_putc(int ch)
{
}

#endif /* HAVE_UART_DEVICE */
#else /* USE_SERIALDRIVER */

void up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  loongarch_lowputc(ch);
#endif
}

#endif /* USE_SERIALDRIVER */

#if 0

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdarg.h>

static char log_buffer[512];

void uart_printf(const char *format, ...)
{
  va_list args;
  char *str;
  int len;

  va_start(args, format);
  vsnprintf(log_buffer, sizeof(log_buffer), format, args);
  va_end(args);

  /* up_puts(log_buffer); */

  str = (char *)log_buffer;
  len = strlen(str);

  while (len-- > 0 && *str)
    {
      if (*str == '\n')
        {
          loongarch_lowputc('\r');
        }

      loongarch_lowputc(*str++);
    }
}
#endif
