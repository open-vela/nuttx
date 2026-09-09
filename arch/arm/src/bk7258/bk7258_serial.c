/****************************************************************************
 * arch/arm/src/bk7258/bk7258_serial.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * BK7258 mailbox UART0 NuttX serial lower-half.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/irq.h>
#include <nuttx/serial/serial.h>
#include <nuttx/syslog/syslog.h>

#include "arm_internal.h"
#include "hardware/bk7258_mbox.h"

#ifdef CONFIG_BK7258_MB_UART0_CONSOLE
#ifndef CONFIG_BK7258_MB_UART0_SERIAL_RXBUFSIZE
#  define CONFIG_BK7258_MB_UART0_SERIAL_RXBUFSIZE 256
#endif

#ifndef CONFIG_BK7258_MB_UART0_SERIAL_TXBUFSIZE
#  define CONFIG_BK7258_MB_UART0_SERIAL_TXBUFSIZE 512
#endif

struct bk7258_serial_priv
{
  bool rx_enabled;
  bool tx_enabled;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int bk7258_setup(struct uart_dev_s *dev)
{
  (void)dev;
  return OK;
}

static void bk7258_shutdown(struct uart_dev_s *dev)
{
  (void)dev;
}

static int bk7258_attach(struct uart_dev_s *dev)
{
  (void)dev;
  return OK;
}

static void bk7258_detach(struct uart_dev_s *dev)
{
  (void)dev;
}

static int bk7258_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  (void)filep;
  (void)cmd;
  (void)arg;
  return -ENOTTY;
}

static int bk7258_receive(struct uart_dev_s *dev, unsigned int *status)
{
  uint8_t byte;
  ssize_t ret;

  (void)dev;
  ret = bk7258_mbox_uart_read(&byte, 1, status);
  return ret == 1 ? byte : -1;
}

static void bk7258_rxint(struct uart_dev_s *dev, bool enable)
{
  struct bk7258_serial_priv *priv = dev->priv;

  priv->rx_enabled = enable;
  if (enable && bk7258_mbox_uart_rxavailable())
    {
      uart_recvchars(dev);
    }
}

static bool bk7258_rxavailable(struct uart_dev_s *dev)
{
  (void)dev;
  return bk7258_mbox_uart_rxavailable();
}

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool bk7258_rxflowcontrol(struct uart_dev_s *dev,
                                 unsigned int nbuffered, bool upper)
{
  (void)dev;
  (void)nbuffered;
  bk7258_mbox_uart_rxflowcontrol(upper);
  return upper;
}
#endif

static void bk7258_send(struct uart_dev_s *dev, int ch)
{
  uint8_t byte = ch;

  (void)dev;
  (void)bk7258_mbox_uart_write(&byte, 1);
}

static void bk7258_txint(struct uart_dev_s *dev, bool enable)
{
  struct bk7258_serial_priv *priv = dev->priv;

  priv->tx_enabled = enable;
  if (enable)
    {
      uart_xmitchars(dev);
    }
}

static bool bk7258_txready(struct uart_dev_s *dev)
{
  (void)dev;
  return bk7258_mbox_uart_txready();
}

static bool bk7258_txempty(struct uart_dev_s *dev)
{
  return dev->xmit.head == dev->xmit.tail &&
         bk7258_mbox_uart_txempty();
}

static int bk7258_release(struct uart_dev_s *dev)
{
  (void)dev;
  return OK;
}

static ssize_t bk7258_recvbuf(struct uart_dev_s *dev, void *buffer,
                              size_t length)
{
  unsigned int status;

  (void)dev;
  return bk7258_mbox_uart_read(buffer, length, &status);
}

static ssize_t bk7258_sendbuf(struct uart_dev_s *dev, const void *buffer,
                              size_t length)
{
  (void)dev;
  return bk7258_mbox_uart_write(buffer, length);
}

static const struct uart_ops_s g_bk7258_uart_ops =
{
  .setup         = bk7258_setup,
  .shutdown      = bk7258_shutdown,
  .attach        = bk7258_attach,
  .detach        = bk7258_detach,
  .ioctl         = bk7258_ioctl,
  .receive       = bk7258_receive,
  .rxint         = bk7258_rxint,
  .rxavailable   = bk7258_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = bk7258_rxflowcontrol,
#endif
  .send          = bk7258_send,
  .txint         = bk7258_txint,
  .txready       = bk7258_txready,
  .txempty       = bk7258_txempty,
  .release       = bk7258_release,
  .recvbuf       = bk7258_recvbuf,
  .sendbuf       = bk7258_sendbuf,
};

static char g_bk7258_rxbuffer[CONFIG_BK7258_MB_UART0_SERIAL_RXBUFSIZE];
static char g_bk7258_txbuffer[CONFIG_BK7258_MB_UART0_SERIAL_TXBUFSIZE];
static struct bk7258_serial_priv g_bk7258_priv;

static struct uart_dev_s g_bk7258_mb_uart0 =
{
  .isconsole = true,
  .recv =
    {
      .size = CONFIG_BK7258_MB_UART0_SERIAL_RXBUFSIZE,
      .buffer = g_bk7258_rxbuffer,
    },

  .xmit =
    {
      .size = CONFIG_BK7258_MB_UART0_SERIAL_TXBUFSIZE,
      .buffer = g_bk7258_txbuffer,
    },

  .ops = &g_bk7258_uart_ops,
  .priv = &g_bk7258_priv,
};

static void bk7258_serial_available(void *arg)
{
  struct uart_dev_s *dev = arg;
  struct bk7258_serial_priv *priv = dev->priv;

  if (priv->rx_enabled && bk7258_mbox_uart_rxavailable())
    {
      uart_recvchars(dev);
    }

  if (priv->tx_enabled && bk7258_mbox_uart_txready())
    {
      uart_xmitchars(dev);
    }
}
#endif

void arm_earlyserialinit(void)
{
  /* Physical auxiliary UART clocks and pinmux remain untouched. */
}

void arm_serialinit(void)
{
  int ret;

  ret = bk7258_mailbox_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "mailbox: logical init failed: %d\n", ret);
      return;
    }

  ret = bk7258_mb_uart_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "mb-uart0: init failed: %d\n", ret);
      return;
    }

  ret = bk7258_mailbox_start();
  if (ret < 0)
    {
      syslog(LOG_ERR, "mailbox: physical start failed: %d\n", ret);
      return;
    }

  bk7258_mb_uart_start();
#ifdef CONFIG_BK7258_MB_UART0_CONSOLE
  bk7258_mbox_uart_set_callback(bk7258_serial_available,
                                &g_bk7258_mb_uart0);
  ret = uart_register("/dev/console", &g_bk7258_mb_uart0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "mb-uart0: /dev/console register failed: %d\n", ret);
      return;
    }

  ret = uart_register("/dev/ttyMB0", &g_bk7258_mb_uart0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "mb-uart0: /dev/ttyMB0 register failed: %d\n", ret);
    }
#endif
}

void up_putc(int ch)
{
  if (ch == '\n')
    {
      arm_lowputc('\r');
    }

  arm_lowputc(ch);
}
