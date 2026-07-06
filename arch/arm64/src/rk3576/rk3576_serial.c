/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_serial.c
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
 ***************************************************************************/

/* Reference:
 *
 * Rockchip RK3576 TRM — Chapter 10: UART
 * Rockchip RK3576 Datasheet — UART peripheral section
 * DesignWare APB UART (DW_apb_uart) programming guide
 *
 * RK3576 has 12 UART ports (UART0–UART11), all DesignWare APB UART
 * compatible with 64-byte TX/RX FIFOs, up to 8 Mbps.
 */

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/init.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/serial.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "rk3576_serial.h"
#include "arm64_arch_timer.h"
#include "rk3576_boot.h"
#include "arm64_gic.h"
#include "hardware/rk3576_memorymap.h"

#ifdef USE_SERIALDRIVER

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* RK3576 UART Register Access Macros
 *
 * RK3576 uses DesignWare APB UART. Registers are 32-bit aligned but
 * some sub-registers are 8-bit (RBR/THR/DLL/DLH/IER/LSR/USR).
 * Use getreg32/putreg32 for 32-bit access and getreg8/putreg8 for 8-bit.
 */

#define RK3576_UART_REG(base, off) ((base) + (off))

#define rk3576_uart_rbr(b)  getreg32(RK3576_UART_REG(b, RK3576_UART_RBR_OFFSET))
#define rk3576_uart_thr(b, v) putreg32(v, RK3576_UART_REG(b, RK3576_UART_THR_OFFSET))
#define rk3576_uart_dll(b)  getreg32(RK3576_UART_REG(b, RK3576_UART_DLL_OFFSET))
#define rk3576_uart_dlh(b)  getreg32(RK3576_UART_REG(b, RK3576_UART_DLH_OFFSET))
#define rk3576_uart_ier(b)  getreg32(RK3576_UART_REG(b, RK3576_UART_IER_OFFSET))
#define rk3576_uart_iir(b)  getreg32(RK3576_UART_REG(b, RK3576_UART_IIR_OFFSET))
#define rk3576_uart_fcr(b, v) putreg32(v, RK3576_UART_REG(b, RK3576_UART_FCR_OFFSET))
#define rk3576_uart_lcr(b)  getreg32(RK3576_UART_REG(b, RK3576_UART_LCR_OFFSET))
#define rk3576_uart_lcr_set(b, v) putreg32(v, RK3576_UART_REG(b, RK3576_UART_LCR_OFFSET))
#define rk3576_uart_mcr(b)  getreg32(RK3576_UART_REG(b, RK3576_UART_MCR_OFFSET))
#define rk3576_uart_lsr(b)  getreg32(RK3576_UART_REG(b, RK3576_UART_LSR_OFFSET))
#define rk3576_uart_msr(b)  getreg32(RK3576_UART_REG(b, RK3576_UART_MSR_OFFSET))
#define rk3576_uart_usr(b)  getreg32(RK3576_UART_REG(b, RK3576_UART_USR_OFFSET))

/* UART0 Settings should be same as U-Boot Bootloader */

#ifndef CONFIG_UART0_BAUD
#  define CONFIG_UART0_BAUD 115200
#endif

#ifndef CONFIG_UART0_BITS
#  define CONFIG_UART0_BITS 8
#endif

#ifndef CONFIG_UART0_PARITY
#  define CONFIG_UART0_PARITY 0
#endif

#ifndef CONFIG_UART0_2STOP
#  define CONFIG_UART0_2STOP 0
#endif

#ifndef CONFIG_UART0_RXBUFSIZE
#  define CONFIG_UART0_RXBUFSIZE 256
#endif

#ifndef CONFIG_UART0_TXBUFSIZE
#  define CONFIG_UART0_TXBUFSIZE 256
#endif

/* Default settings for other UARTs */

#ifndef CONFIG_UART1_BAUD
#  define CONFIG_UART1_BAUD 115200
#endif
#ifndef CONFIG_UART1_BITS
#  define CONFIG_UART1_BITS 8
#endif
#ifndef CONFIG_UART1_PARITY
#  define CONFIG_UART1_PARITY 0
#endif
#ifndef CONFIG_UART1_2STOP
#  define CONFIG_UART1_2STOP 0
#endif
#ifndef CONFIG_UART1_RXBUFSIZE
#  define CONFIG_UART1_RXBUFSIZE 256
#endif
#ifndef CONFIG_UART1_TXBUFSIZE
#  define CONFIG_UART1_TXBUFSIZE 256
#endif

#ifndef CONFIG_UART2_BAUD
#  define CONFIG_UART2_BAUD 115200
#endif
#ifndef CONFIG_UART2_BITS
#  define CONFIG_UART2_BITS 8
#endif
#ifndef CONFIG_UART2_PARITY
#  define CONFIG_UART2_PARITY 0
#endif
#ifndef CONFIG_UART2_2STOP
#  define CONFIG_UART2_2STOP 0
#endif
#ifndef CONFIG_UART2_RXBUFSIZE
#  define CONFIG_UART2_RXBUFSIZE 256
#endif
#ifndef CONFIG_UART2_TXBUFSIZE
#  define CONFIG_UART2_TXBUFSIZE 256
#endif

#ifndef CONFIG_UART3_BAUD
#  define CONFIG_UART3_BAUD 115200
#endif
#ifndef CONFIG_UART3_BITS
#  define CONFIG_UART3_BITS 8
#endif
#ifndef CONFIG_UART3_PARITY
#  define CONFIG_UART3_PARITY 0
#endif
#ifndef CONFIG_UART3_2STOP
#  define CONFIG_UART3_2STOP 0
#endif
#ifndef CONFIG_UART3_RXBUFSIZE
#  define CONFIG_UART3_RXBUFSIZE 256
#endif
#ifndef CONFIG_UART3_TXBUFSIZE
#  define CONFIG_UART3_TXBUFSIZE 256
#endif

#ifndef CONFIG_UART4_BAUD
#  define CONFIG_UART4_BAUD 115200
#endif
#ifndef CONFIG_UART4_BITS
#  define CONFIG_UART4_BITS 8
#endif
#ifndef CONFIG_UART4_PARITY
#  define CONFIG_UART4_PARITY 0
#endif
#ifndef CONFIG_UART4_2STOP
#  define CONFIG_UART4_2STOP 0
#endif
#ifndef CONFIG_UART4_RXBUFSIZE
#  define CONFIG_UART4_RXBUFSIZE 256
#endif
#ifndef CONFIG_UART4_TXBUFSIZE
#  define CONFIG_UART4_TXBUFSIZE 256
#endif

/* Console UART — RK3576 uses UART2 as debug console (same as U-Boot) */

#define CONSOLE_DEV     g_uart2port         /* UART2 is console */
#define TTYS0_DEV       g_uart2port         /* UART2 is ttyS0 */
#define UART2_ASSIGNED  1

/***************************************************************************
 * Private Types
 ***************************************************************************/

/* RK3576 UART Configuration */

struct rk3576_uart_config
{
  unsigned long uart;  /* UART Base Address */
};

/* RK3576 UART Device Data */

struct rk3576_uart_data
{
  uint32_t baud_rate;  /* UART Baud Rate */
  uint32_t ier;        /* Saved IER value */
  uint8_t  parity;     /* 0=none, 1=odd, 2=even */
  uint8_t  bits;       /* Number of bits (5, 6, 7, or 8) */
  bool     stopbits2;  /* true: 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_RS485
  bool     rs485;      /* true: RS-485 mode enabled */
  bool     rs485_dir;  /* RS-485 direction: true=TX, false=RX */
#endif
};

/* RK3576 UART Port */

struct rk3576_uart_port_s
{
  struct rk3576_uart_data data;     /* UART Device Data */
  struct rk3576_uart_config config; /* UART Configuration */
  unsigned int irq_num;             /* UART IRQ Number */
  spinlock_t lock;                  /* Protect IER and MCR modifications */
#ifdef CONFIG_SERIAL_RS485
  struct serial_rs485 rs485_cfg;    /* RS-485 configuration */
#endif
};

/***************************************************************************
 * Private Function Prototypes
 ***************************************************************************/

static void rk3576_uart_rxint(struct uart_dev_s *dev, bool enable);
static void rk3576_uart_txint(struct uart_dev_s *dev, bool enable);

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/***************************************************************************
 * Name: rk3576_uart_divisor
 *
 * Description:
 *   Select a divisor to produce the BAUD from the UART SCLK.
 *
 *     BAUD = SCLK / (16 * DL), or
 *     DL   = SCLK / BAUD / 16
 *
 * Returned Value:
 *   UART Divisor
 *
 ***************************************************************************/

static uint32_t rk3576_uart_divisor(uint32_t baud)
{
  DEBUGASSERT(baud != 0);
  return RK3576_UART_SCLK / (baud << 4);
}

/***************************************************************************
 * Name: rk3576_uart_wait
 *
 * Description:
 *   Wait for UART to be non-busy (USR_BUSY cleared).
 *
 * Returned Value:
 *   Zero (OK) on success; ERROR if timeout.
 *
 ***************************************************************************/

static int rk3576_uart_wait(struct uart_dev_s *dev)
{
  struct rk3576_uart_port_s *port = (struct rk3576_uart_port_s *)dev->priv;
  const struct rk3576_uart_config *config = &port->config;
  int i;

  for (i = 0; i < RK3576_UART_TIMEOUT_MS; i++)
    {
      uint32_t status = rk3576_uart_usr(config->uart);

      if ((status & RK3576_UART_USR_BUSY) == 0)
        {
          return OK;
        }

      up_mdelay(1);
    }

  _err("UART timeout\n");
  return ERROR;
}

/***************************************************************************
 * Name: rk3576_uart_irq_handler
 *
 * Description:
 *   Common UART interrupt handler. Calls uart_xmitchars or uart_recvchars
 *   to perform the appropriate data transfers.
 *
 ***************************************************************************/

static int rk3576_uart_irq_handler(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  const struct rk3576_uart_port_s *port =
    (struct rk3576_uart_port_s *)dev->priv;
  const struct rk3576_uart_config *config = &port->config;
  uint32_t status;
  int passes;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);

  for (passes = 0; passes < 256; passes++)
    {
      status = rk3576_uart_iir(config->uart);

      switch (status & RK3576_UART_IIR_IID_MASK)
        {
          case RK3576_UART_IIR_IID_RECV:
          case RK3576_UART_IIR_IID_TIMEOUT:
            {
              uart_recvchars(dev);
              break;
            }

          case RK3576_UART_IIR_IID_TXEMPTY:
            {
              uart_xmitchars(dev);
              break;
            }

          case RK3576_UART_IIR_IID_MODEM:
            {
              status = rk3576_uart_msr(config->uart);
              break;
            }

          case RK3576_UART_IIR_IID_LINESTATUS:
            {
              status = rk3576_uart_lsr(config->uart);
              break;
            }

          case RK3576_UART_IIR_IID_BUSY:
            {
              status = rk3576_uart_usr(config->uart);
              break;
            }

          case RK3576_UART_IIR_IID_NONE:
            {
              return OK;
            }

          default:
            {
              _err("ERROR: Unexpected IIR: %02" PRIx32 "\n", status);
              break;
            }
        }
    }

  return OK;
}

/***************************************************************************
 * Name: rk3576_uart_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, FIFOs, etc. Called the first
 *   time that the serial port is opened.
 *
 ***************************************************************************/

static int rk3576_uart_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct rk3576_uart_port_s *port = (struct rk3576_uart_port_s *)dev->priv;
  const struct rk3576_uart_config *config = &port->config;
  struct rk3576_uart_data *data = &port->data;
  uint32_t dl;
  uint32_t lcr;
  int ret;

  DEBUGASSERT(data != NULL);

  /* Clear and reset FIFOs, then enable with trigger levels in one write */

  {
    uint32_t fcr = RK3576_UART_FCR_RFIFOR | RK3576_UART_FCR_XFIFOR |
                   RK3576_UART_FCR_FIFOE | RK3576_UART_FCR_RT_HALF |
                   RK3576_UART_FCR_TFT_HALF;

#ifdef CONFIG_UART1_DMA
    /* Enable DMA handshake mode when DMA is configured.
     * In this mode the UART generates DMA request signals
     * instead of interrupts for data transfer.
     */

    if (config->uart != RK3576_UART0_ADDR)
      {
        fcr |= RK3576_UART_FCR_DMAM;
      }
#endif

    rk3576_uart_fcr(config->uart, fcr);
  }

  /* Save current IER */

  data->ier = rk3576_uart_ier(config->uart);

  /* Build LCR value */

  lcr = 0;

  switch (data->bits)
    {
    case 5:
      lcr |= RK3576_UART_LCR_DLS_5BITS;
      break;

    case 6:
      lcr |= RK3576_UART_LCR_DLS_6BITS;
      break;

    case 7:
      lcr |= RK3576_UART_LCR_DLS_7BITS;
      break;

    case 8:
    default:
      lcr |= RK3576_UART_LCR_DLS_8BITS;
      break;
    }

  if (data->stopbits2)
    {
      lcr |= RK3576_UART_LCR_STOP;
    }

  if (data->parity == 1)
    {
      lcr |= RK3576_UART_LCR_PEN;
    }
  else if (data->parity == 2)
    {
      lcr |= (RK3576_UART_LCR_PEN | RK3576_UART_LCR_EPS);
    }

  /* Set DLAB to access divisor latch when UART is not busy */

  ret = rk3576_uart_wait(dev);
  if (ret < 0)
    {
      _err("UART wait failed, ret=%d\n", ret);
      return ret;
    }

  rk3576_uart_lcr_set(config->uart, lcr | RK3576_UART_LCR_DLAB);

  ret = rk3576_uart_wait(dev);
  if (ret < 0)
    {
      _err("UART wait failed, ret=%d\n", ret);
      return ret;
    }

  /* Set the BAUD divisor */

  dl = rk3576_uart_divisor(data->baud_rate);
  putreg32(0, config->uart + RK3576_UART_DLH_OFFSET);
  putreg32(dl & 0xffff,
           config->uart + RK3576_UART_DLL_OFFSET);

  /* Check the BAUD divisor */

  if (getreg32(config->uart + RK3576_UART_DLL_OFFSET) != (dl & 0xffff))
    {
      _err("UART BAUD divisor failed\n");
      return ERROR;
    }

  /* Clear DLAB */

  rk3576_uart_lcr_set(config->uart, lcr);

  /* Enable Auto Flow Control in the Modem Control Register */

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
  {
    irqstate_t flags = spin_lock_irqsave(&port->lock);
    uint32_t mcr = rk3576_uart_mcr(config->uart);
    mcr |= RK3576_UART_MCR_AFCE;
    putreg32(mcr, config->uart + RK3576_UART_MCR_OFFSET);
    spin_unlock_irqrestore(&port->lock, flags);
  }
#endif

#endif /* CONFIG_SUPPRESS_UART_CONFIG */
  return OK;
}

/***************************************************************************
 * Name: rk3576_uart_shutdown
 *
 * Description:
 *   Disable the UART Port. Called when the serial port is closed.
 *
 ***************************************************************************/

static void rk3576_uart_shutdown(struct uart_dev_s *dev)
{
  /* Disable the Receive and Transmit Interrupts */

  rk3576_uart_rxint(dev, false);
  rk3576_uart_txint(dev, false);
}

/***************************************************************************
 * Name: rk3576_uart_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.
 *   This method is called when the serial port is opened.
 *
 ***************************************************************************/

static int rk3576_uart_attach(struct uart_dev_s *dev)
{
  int ret;
  const struct rk3576_uart_port_s *port =
    (struct rk3576_uart_port_s *)dev->priv;

  DEBUGASSERT(port != NULL);

  /* Attach UART Interrupt Handler */

  ret = irq_attach(port->irq_num, rk3576_uart_irq_handler, dev);

  /* Set Interrupt Priority in Generic Interrupt Controller v3 */

#ifdef CONFIG_ARCH_IRQPRIO
  up_prioritize_irq(port->irq_num, 0);
#endif
  up_set_irq_type(port->irq_num, IRQ_RISING_EDGE);

  /* Enable UART Interrupt */

  if (ret == OK)
    {
      up_enable_irq(port->irq_num);
    }
  else
    {
      _err("IRQ attach failed, ret=%d\n", ret);
    }

  return ret;
}

/***************************************************************************
 * Name: rk3576_uart_detach
 *
 * Description:
 *   Detach UART interrupts. Called when the serial port is closed normally
 *   just before the shutdown method is called.
 *
 ***************************************************************************/

static void rk3576_uart_detach(struct uart_dev_s *dev)
{
  const struct rk3576_uart_port_s *port =
    (struct rk3576_uart_port_s *)dev->priv;

  DEBUGASSERT(port != NULL);

  /* Disable UART Interrupt */

  up_disable_irq(port->irq_num);

  /* Detach UART Interrupt Handler */

  irq_detach(port->irq_num);
}

/***************************************************************************
 * Name: rk3576_uart_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 ***************************************************************************/

static int rk3576_uart_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      case TIOCSBRK:
        {
          struct inode *inode = filep->f_inode;
          struct uart_dev_s *dev = inode->i_private;
          struct rk3576_uart_port_s *port =
            (struct rk3576_uart_port_s *)dev->priv;
          uint32_t lcr = rk3576_uart_lcr(port->config.uart);
          rk3576_uart_lcr_set(port->config.uart, lcr | RK3576_UART_LCR_BC);
          break;
        }

      case TIOCCBRK:
        {
          struct inode *inode = filep->f_inode;
          struct uart_dev_s *dev = inode->i_private;
          struct rk3576_uart_port_s *port =
            (struct rk3576_uart_port_s *)dev->priv;
          uint32_t lcr = rk3576_uart_lcr(port->config.uart);
          rk3576_uart_lcr_set(port->config.uart, lcr & ~RK3576_UART_LCR_BC);
          break;
        }

      default:
        {
          ret = -ENOTTY;
          break;
        }

#ifdef CONFIG_SERIAL_TERMIOS
      case TCGETS:
        {
          struct inode *inode = filep->f_inode;
          struct uart_dev_s *dev = inode->i_private;
          struct rk3576_uart_port_s *port =
            (struct rk3576_uart_port_s *)dev->priv;
          struct rk3576_uart_data *data = &port->data;
          struct termios *termiosp = (struct termios *)arg;
          if (termiosp == NULL)
            {
              ret = -EINVAL;
              break;
            }

          memset(termiosp, 0, sizeof(*termiosp));
          termiosp->c_ispeed = data->baud_rate;
          termiosp->c_ospeed = data->baud_rate;

          if (data->bits == 5)
            {
              termiosp->c_cflag = CS5;
            }
          else if (data->bits == 6)
            {
              termiosp->c_cflag = CS6;
            }
          else if (data->bits == 7)
            {
              termiosp->c_cflag = CS7;
            }
          else
            {
              termiosp->c_cflag = CS8;
            }

          if (data->stopbits2)
            {
              termiosp->c_cflag |= CSTOPB;
            }

          if (data->parity == 1)
            {
              termiosp->c_cflag |= (PARENB | PARODD);
            }
          else if (data->parity == 2)
            {
              termiosp->c_cflag |= PARENB;
            }

          termiosp->c_cflag |= CREAD | CLOCAL;
          break;
        }

      case TCSETS:
        {
          struct inode *inode = filep->f_inode;
          struct uart_dev_s *dev = inode->i_private;
          struct rk3576_uart_port_s *port =
            (struct rk3576_uart_port_s *)dev->priv;
          struct rk3576_uart_data *data = &port->data;
          const struct termios *termiosp = (const struct termios *)arg;
          if (termiosp == NULL)
            {
              ret = -EINVAL;
              break;
            }

          switch (termiosp->c_cflag & CSIZE)
            {
            case CS5:
              data->bits = 5;
              break;
            case CS6:
              data->bits = 6;
              break;
            case CS7:
              data->bits = 7;
              break;
            case CS8:
            default:
              data->bits = 8;
              break;
            }

          data->stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;

          if (termiosp->c_cflag & PARENB)
            {
              data->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
            }
          else
            {
              data->parity = 0;
            }

          data->baud_rate = termiosp->c_ospeed;

          /* Flush stale FIFO data before baud rate change */

          {
            struct rk3576_uart_port_s *port =
              (struct rk3576_uart_port_s *)dev->priv;
            putreg32(RK3576_UART_FCR_RFIFOR | RK3576_UART_FCR_XFIFOR,
                     port->config.uart + RK3576_UART_FCR_OFFSET);
          }

          ret = rk3576_uart_setup(dev);
          break;
        }

      case TIOCMGET:
        {
          if (arg == NULL)
            {
              ret = -EINVAL;
              break;
            }

          struct inode *inode = filep->f_inode;
          struct uart_dev_s *dev = inode->i_private;
          struct rk3576_uart_port_s *port =
            (struct rk3576_uart_port_s *)dev->priv;
          uint32_t mcr = rk3576_uart_mcr(port->config.uart);
          uint32_t msr = rk3576_uart_msr(port->config.uart);
          int *pins = (int *)arg;

          *pins = 0;

          /* Output signals from MCR */

          if (mcr & RK3576_UART_MCR_DTR)
            {
              *pins |= TIOCM_DTR;
            }

          if (mcr & RK3576_UART_MCR_RTS)
            {
              *pins |= TIOCM_RTS;
            }

          /* Input signals from MSR */

          if (msr & (1 << 0))  /* CTS */
            {
              *pins |= TIOCM_CTS;
            }

          if (msr & (1 << 1))  /* DSR */
            {
              *pins |= TIOCM_DSR;
            }

          if (msr & (1 << 2))  /* RI */
            {
              *pins |= TIOCM_RI;
            }

          if (msr & (1 << 3))  /* DCD */
            {
              *pins |= TIOCM_CD;
            }

          break;
        }

      case TIOCMSET:
        {
          if (arg == NULL)
            {
              ret = -EINVAL;
              break;
            }

          struct inode *inode = filep->f_inode;
          struct uart_dev_s *dev = inode->i_private;
          struct rk3576_uart_port_s *port =
            (struct rk3576_uart_port_s *)dev->priv;
          const int *pins = (const int *)arg;
          irqstate_t flags;
          uint32_t mcr;

          flags = spin_lock_irqsave(&port->lock);
          mcr = rk3576_uart_mcr(port->config.uart);

          if (*pins & TIOCM_DTR)
            {
              mcr |= RK3576_UART_MCR_DTR;
            }
          else
            {
              mcr &= ~RK3576_UART_MCR_DTR;
            }

          if (*pins & TIOCM_RTS)
            {
              mcr |= RK3576_UART_MCR_RTS;
            }
          else
            {
              mcr &= ~RK3576_UART_MCR_RTS;
            }

          putreg32(mcr, port->config.uart + RK3576_UART_MCR_OFFSET);
          spin_unlock_irqrestore(&port->lock, flags);
          break;
        }
#endif

#ifdef CONFIG_SERIAL_RS485
      case TIOCGRS485:
        {
          struct inode *inode = filep->f_inode;
          struct uart_dev_s *dev = inode->i_private;
          struct rk3576_uart_port_s *port =
            (struct rk3576_uart_port_s *)dev->priv;
          struct serial_rs485 *rs485p = (struct serial_rs485 *)arg;

          if (rs485p == NULL)
            {
              ret = -EINVAL;
              break;
            }

          memcpy(rs485p, &port->rs485_cfg, sizeof(*rs485p));
          break;
        }

      case TIOCSRS485:
        {
          struct inode *inode = filep->f_inode;
          struct uart_dev_s *dev = inode->i_private;
          struct rk3576_uart_port_s *port =
            (struct rk3576_uart_port_s *)dev->priv;
          const struct serial_rs485 *rs485p =
            (const struct serial_rs485 *)arg;
          irqstate_t flags;

          if (rs485p == NULL)
            {
              ret = -EINVAL;
              break;
            }

          memcpy(&port->rs485_cfg, rs485p, sizeof(port->rs485_cfg));

          /* Enable/disable RS-485 mode */

          flags = spin_lock_irqsave(&port->lock);
          port->data.rs485 =
            (port->rs485_cfg.flags & SER_RS485_ENABLED) != 0;

          /* Set initial RTS direction based on config */

          if (port->data.rs485)
            {
              uint32_t mcr = rk3576_uart_mcr(port->config.uart);
              if (port->rs485_cfg.flags & SER_RS485_RTS_ON_SEND)
                {
                  mcr |= RK3576_UART_MCR_RTS;
                }
              else
                {
                  mcr &= ~RK3576_UART_MCR_RTS;
                }

              putreg32(mcr, port->config.uart + RK3576_UART_MCR_OFFSET);
            }

          port->data.rs485_dir = false;
          spin_unlock_irqrestore(&port->lock, flags);
          break;
        }
#endif
    }

  return ret;
}

/***************************************************************************
 * Name: rk3576_uart_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character
 *   from the UART. Error bits associated with the receipt are provided
 *   in the return 'status'.
 *
 ***************************************************************************/

static int rk3576_uart_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct rk3576_uart_port_s *port = (struct rk3576_uart_port_s *)dev->priv;
  const struct rk3576_uart_config *config = &port->config;
  uint32_t rbr;

  *status = rk3576_uart_lsr(config->uart);
  rbr     = rk3576_uart_rbr(config->uart);
  return rbr;
}

/***************************************************************************
 * Name: rk3576_uart_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ***************************************************************************/

static void rk3576_uart_rxint(struct uart_dev_s *dev, bool enable)
{
  struct rk3576_uart_port_s *port = (struct rk3576_uart_port_s *)dev->priv;
  const struct rk3576_uart_config *config = &port->config;
  irqstate_t flags;
  uint32_t ier;

  flags = spin_lock_irqsave(&port->lock);
  ier = rk3576_uart_ier(config->uart);

  if (enable)
    {
      ier |= RK3576_UART_IER_ERBFI;
    }
  else
    {
      ier &= ~RK3576_UART_IER_ERBFI;
    }

  putreg32(ier, config->uart + RK3576_UART_IER_OFFSET);
  spin_unlock_irqrestore(&port->lock, flags);
}

/***************************************************************************
 * Name: rk3576_uart_rxavailable
 *
 * Description:
 *   Return true if the Receive FIFO is not empty
 *
 ***************************************************************************/

static bool rk3576_uart_rxavailable(struct uart_dev_s *dev)
{
  const struct rk3576_uart_port_s *port =
    (struct rk3576_uart_port_s *)dev->priv;
  const struct rk3576_uart_config *config = &port->config;

  /* Data Ready Bit (Line Status Register) is 1 if Rx Data is ready */

  return (rk3576_uart_lsr(config->uart) & RK3576_UART_LSR_DR) != 0;
}

/***************************************************************************
 * Name: rk3576_uart_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ***************************************************************************/

static void rk3576_uart_send(struct uart_dev_s *dev, int ch)
{
  struct rk3576_uart_port_s *port = (struct rk3576_uart_port_s *)dev->priv;
  const struct rk3576_uart_config *config = &port->config;

#ifdef CONFIG_SERIAL_RS485
  /* RS-485: Assert RTS (enable TX driver) before sending */

  if (port->data.rs485 && !port->data.rs485_dir)
    {
      irqstate_t flags = spin_lock_irqsave(&port->lock);
      uint32_t mcr = rk3576_uart_mcr(config->uart);
      mcr |= RK3576_UART_MCR_RTS;
      putreg32(mcr, config->uart + RK3576_UART_MCR_OFFSET);
      port->data.rs485_dir = true;
      spin_unlock_irqrestore(&port->lock, flags);
    }
#endif

  /* Write char to Transmit Holding Register (UART_THR) */

  rk3576_uart_thr(config->uart, ch);
}

/***************************************************************************
 * Name: rk3576_uart_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ***************************************************************************/

static void rk3576_uart_txint(struct uart_dev_s *dev, bool enable)
{
  struct rk3576_uart_port_s *port = (struct rk3576_uart_port_s *)dev->priv;
  const struct rk3576_uart_config *config = &port->config;
  irqstate_t flags;
  uint32_t ier;

  flags = spin_lock_irqsave(&port->lock);
  ier = rk3576_uart_ier(config->uart);

  if (enable)
    {
      ier |= RK3576_UART_IER_ETBEI;
    }
  else
    {
      ier &= ~RK3576_UART_IER_ETBEI;
    }

  putreg32(ier, config->uart + RK3576_UART_IER_OFFSET);
  spin_unlock_irqrestore(&port->lock, flags);
}

/***************************************************************************
 * Name: rk3576_uart_txready
 *
 * Description:
 *   Return true if the Transmit FIFO is not full
 *
 ***************************************************************************/

static bool rk3576_uart_txready(struct uart_dev_s *dev)
{
  const struct rk3576_uart_port_s *port =
    (struct rk3576_uart_port_s *)dev->priv;
  const struct rk3576_uart_config *config = &port->config;

  /* Tx FIFO is ready if THRE Bit is 1 (Tx Holding Register Empty) */

  return (rk3576_uart_lsr(config->uart) & RK3576_UART_LSR_THRE) != 0;
}

/***************************************************************************
 * Name: rk3576_uart_txempty
 *
 * Description:
 *   Return true if the Transmit FIFO is empty
 *
 ***************************************************************************/

static bool rk3576_uart_txempty(struct uart_dev_s *dev)
{
  struct rk3576_uart_port_s *port = (struct rk3576_uart_port_s *)dev->priv;
  const struct rk3576_uart_config *config = &port->config;
  bool empty;

  /* TEMT bit (Bit 6 of LSR) is 1 when both THR and shift register are empty */

  empty = (rk3576_uart_lsr(config->uart) & RK3576_UART_LSR_TEMT) != 0;

#ifdef CONFIG_SERIAL_RS485
  /* RS-485: De-assert RTS (enable RX driver) when transmitter is empty */

  if (empty && port->data.rs485 && port->data.rs485_dir)
    {
      irqstate_t flags = spin_lock_irqsave(&port->lock);
      uint32_t mcr = rk3576_uart_mcr(config->uart);
      mcr &= ~RK3576_UART_MCR_RTS;
      putreg32(mcr, config->uart + RK3576_UART_MCR_OFFSET);
      port->data.rs485_dir = false;
      spin_unlock_irqrestore(&port->lock, flags);
    }
#endif

  return empty;
}

/***************************************************************************
 * Name: rk3576_uart_wait_send
 *
 * Description:
 *   Wait for Transmit FIFO until it is not full, then transmit the
 *   character over UART.
 *
 ***************************************************************************/

static void rk3576_uart_wait_send(struct uart_dev_s *dev, int ch)
{
  int timeout;

  DEBUGASSERT(dev != NULL);

  for (timeout = 0; timeout < RK3576_UART_TIMEOUT_MS; timeout++)
    {
      if (rk3576_uart_txready(dev))
        {
          rk3576_uart_send(dev, ch);
          return;
        }

      up_mdelay(1);
    }
}

/***************************************************************************
 * Private Data
 ***************************************************************************/

/* UART Operations for Serial Driver */

static const struct uart_ops_s g_uart_ops =
{
  .setup    = rk3576_uart_setup,
  .shutdown = rk3576_uart_shutdown,
  .attach   = rk3576_uart_attach,
  .detach   = rk3576_uart_detach,
  .ioctl    = rk3576_uart_ioctl,
  .receive  = rk3576_uart_receive,
  .rxint    = rk3576_uart_rxint,
  .rxavailable = rk3576_uart_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol    = NULL,
#endif
  .send     = rk3576_uart_send,
  .txint    = rk3576_uart_txint,
  .txready  = rk3576_uart_txready,
  .txempty  = rk3576_uart_txempty,
};

/***************************************************************************
 * RK3576 UART Port Definitions
 *
 * RK3576 has 12 UART ports (UART0–UART11). We define support for
 * UART0-UART4 which are commonly available on development boards.
 * UART2 is the debug console (connected to USB-UART bridge).
 ***************************************************************************/

/* UART0 Port State */

#ifdef CONFIG_RK3576_UART0
static struct rk3576_uart_port_s g_uart0priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART0_BAUD,
      .parity     = CONFIG_UART0_PARITY,
      .bits       = CONFIG_UART0_BITS,
      .stopbits2  = CONFIG_UART0_2STOP
    },

  .config =
    {
      .uart       = RK3576_UART0_ADDR
    },

  .irq_num      = RK3576_UART0_IRQ,
  .lock         = SP_UNLOCKED
};

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

static struct uart_dev_s g_uart0port =
{
  .recv  =
    {
      .size   = CONFIG_UART0_RXBUFSIZE,
      .buffer = g_uart0rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART0_TXBUFSIZE,
      .buffer = g_uart0txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart0priv,
};
#endif /* CONFIG_RK3576_UART0 */

/* UART1 Port State */

#ifdef CONFIG_RK3576_UART1
static struct rk3576_uart_port_s g_uart1priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART1_BAUD,
      .parity     = CONFIG_UART1_PARITY,
      .bits       = CONFIG_UART1_BITS,
      .stopbits2  = CONFIG_UART1_2STOP
    },

  .config =
    {
      .uart       = RK3576_UART1_ADDR
    },

  .irq_num      = RK3576_UART1_IRQ,
  .lock         = SP_UNLOCKED
};

static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

static struct uart_dev_s g_uart1port =
{
  .recv  =
    {
      .size   = CONFIG_UART1_RXBUFSIZE,
      .buffer = g_uart1rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART1_TXBUFSIZE,
      .buffer = g_uart1txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart1priv,
};
#endif /* CONFIG_RK3576_UART1 */

/* UART2 Port State (Debug Console) */

#ifdef CONFIG_RK3576_UART2
static struct rk3576_uart_port_s g_uart2priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART2_BAUD,
      .parity     = CONFIG_UART2_PARITY,
      .bits       = CONFIG_UART2_BITS,
      .stopbits2  = CONFIG_UART2_2STOP
    },

  .config =
    {
      .uart       = RK3576_UART2_ADDR
    },

  .irq_num      = RK3576_UART2_IRQ,
  .lock         = SP_UNLOCKED
};

static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];

static struct uart_dev_s g_uart2port =
{
  .recv  =
    {
      .size   = CONFIG_UART2_RXBUFSIZE,
      .buffer = g_uart2rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART2_TXBUFSIZE,
      .buffer = g_uart2txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart2priv,
};
#endif /* CONFIG_RK3576_UART2 */

/* UART3 Port State */

#ifdef CONFIG_RK3576_UART3
static struct rk3576_uart_port_s g_uart3priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART3_BAUD,
      .parity     = CONFIG_UART3_PARITY,
      .bits       = CONFIG_UART3_BITS,
      .stopbits2  = CONFIG_UART3_2STOP
    },

  .config =
    {
      .uart       = RK3576_UART3_ADDR
    },

  .irq_num      = RK3576_UART3_IRQ,
  .lock         = SP_UNLOCKED
};

static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];

static struct uart_dev_s g_uart3port =
{
  .recv  =
    {
      .size   = CONFIG_UART3_RXBUFSIZE,
      .buffer = g_uart3rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART3_TXBUFSIZE,
      .buffer = g_uart3txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart3priv,
};
#endif /* CONFIG_RK3576_UART3 */

/* UART4 Port State */

#ifdef CONFIG_RK3576_UART4
static struct rk3576_uart_port_s g_uart4priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART4_BAUD,
      .parity     = CONFIG_UART4_PARITY,
      .bits       = CONFIG_UART4_BITS,
      .stopbits2  = CONFIG_UART4_2STOP
    },

  .config =
    {
      .uart       = RK3576_UART4_ADDR
    },

  .irq_num      = RK3576_UART4_IRQ,
  .lock         = SP_UNLOCKED
};

static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];

static struct uart_dev_s g_uart4port =
{
  .recv  =
    {
      .size   = CONFIG_UART4_RXBUFSIZE,
      .buffer = g_uart4rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART4_TXBUFSIZE,
      .buffer = g_uart4txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart4priv,
};
#endif /* CONFIG_RK3576_UART4 */

/* Pick ttys1.  This could be any of UART0, UART1, UART3, UART4. */

#if defined(CONFIG_RK3576_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0port /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_RK3576_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_RK3576_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS1_DEV           g_uart3port /* UART3 is ttyS1 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_RK3576_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS1_DEV           g_uart4port /* UART4 is ttyS1 */
#  define UART4_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of UART0, UART1, UART3, UART4. */

#if defined(CONFIG_RK3576_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS2_DEV           g_uart0port /* UART0 is ttyS2 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_RK3576_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS2_DEV           g_uart1port /* UART1 is ttyS2 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_RK3576_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS2_DEV           g_uart3port /* UART3 is ttyS2 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_RK3576_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS2_DEV           g_uart4port /* UART4 is ttyS2 */
#  define UART4_ASSIGNED      1
#endif

/* Pick ttys3.  This could be one of UART0, UART1, UART3, UART4. */

#if defined(CONFIG_RK3576_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS3_DEV           g_uart0port /* UART0 is ttyS3 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_RK3576_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS3_DEV           g_uart1port /* UART1 is ttyS3 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_RK3576_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS3_DEV           g_uart3port /* UART3 is ttyS3 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_RK3576_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS3_DEV           g_uart4port /* UART4 is ttyS3 */
#  define UART4_ASSIGNED      1
#endif

/* Pick ttys4.  This could be one of UART0, UART1, UART3, UART4. */

#if defined(CONFIG_RK3576_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS4_DEV           g_uart0port /* UART0 is ttyS4 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_RK3576_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS4_DEV           g_uart1port /* UART1 is ttyS4 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_RK3576_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS4_DEV           g_uart3port /* UART3 is ttyS4 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_RK3576_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS4_DEV           g_uart4port /* UART4 is ttyS4 */
#  define UART4_ASSIGNED      1
#endif

/* Pick ttys5.  This could be one of UART0, UART1, UART3, UART4. */

#if defined(CONFIG_RK3576_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS5_DEV           g_uart0port /* UART0 is ttyS5 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_RK3576_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS5_DEV           g_uart1port /* UART1 is ttyS5 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_RK3576_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS5_DEV           g_uart3port /* UART3 is ttyS5 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_RK3576_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS5_DEV           g_uart4port /* UART4 is ttyS5 */
#  define UART4_ASSIGNED      1
#endif

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: arm64_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that
 *   the serial console will be available during bootup. This must be
 *   called before arm64_serialinit.
 *
 *   NOTE: On RK3576, the debug console (UART2) is already configured
 *   by U-Boot. We only need to mark it as console and perform setup.
 *   Other UARTs are initialized lazily when opened.
 *
 ***************************************************************************/

void arm64_earlyserialinit(void)
{
  /* NOTE: This function assumes that UART2 low level hardware configuration
   * -- including all clocking and pin configuration -- was performed
   * earlier by U-Boot Bootloader.
   */

#ifdef CONSOLE_DEV
  /* Enable the console at UART2 */

  CONSOLE_DEV.isconsole = true;
  rk3576_uart_setup(&CONSOLE_DEV);
#endif
}

/***************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ***************************************************************************/

void up_putc(int ch)
{
#ifdef CONSOLE_DEV
  struct uart_dev_s *dev = &CONSOLE_DEV;

  rk3576_uart_wait_send(dev, ch);
#endif
}

/***************************************************************************
 * Name: arm64_serialinit
 *
 * Description:
 *   Register serial console and serial ports. This assumes that
 *   arm64_earlyserialinit was called previously.
 *
 ***************************************************************************/

void arm64_serialinit(void)
{
#ifdef CONSOLE_DEV
  int ret;

  ret = uart_register("/dev/console", &CONSOLE_DEV);
  if (ret < 0)
    {
      _err("Register /dev/console failed, ret=%d\n", ret);
    }

  ret = uart_register("/dev/ttyS0", &TTYS0_DEV);

  if (ret < 0)
    {
      _err("Register /dev/ttyS0 failed, ret=%d\n", ret);
    }

#ifdef TTYS1_DEV
  ret = uart_register("/dev/ttyS1", &TTYS1_DEV);

  if (ret < 0)
    {
      _err("Register /dev/ttyS1 failed, ret=%d\n", ret);
    }
#endif /* TTYS1_DEV */

#ifdef TTYS2_DEV
  ret = uart_register("/dev/ttyS2", &TTYS2_DEV);

  if (ret < 0)
    {
      _err("Register /dev/ttyS2 failed, ret=%d\n", ret);
    }
#endif /* TTYS2_DEV */

#ifdef TTYS3_DEV
  ret = uart_register("/dev/ttyS3", &TTYS3_DEV);

  if (ret < 0)
    {
      _err("Register /dev/ttyS3 failed, ret=%d\n", ret);
    }
#endif /* TTYS3_DEV */

#ifdef TTYS4_DEV
  ret = uart_register("/dev/ttyS4", &TTYS4_DEV);

  if (ret < 0)
    {
      _err("Register /dev/ttyS4 failed, ret=%d\n", ret);
    }
#endif /* TTYS4_DEV */

#ifdef TTYS5_DEV
  ret = uart_register("/dev/ttyS5", &TTYS5_DEV);

  if (ret < 0)
    {
      _err("Register /dev/ttyS5 failed, ret=%d\n", ret);
    }
#endif /* TTYS5_DEV */

#endif
}

#endif /* USE_SERIALDRIVER */
