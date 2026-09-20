/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_usbserial.c
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
#include <string.h>
#include <assert.h>
#include <nuttx/debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#  include <nuttx/fs/ioctl.h>
#endif

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/serial/serial.h>
#include <arch/irq.h>

#include "riscv_internal.h"

#include "esp_config.h"
#include "esp_irq.h"

#include "esp_private/periph_ctrl.h"
#include "hal/uart_hal.h"
#include "hal/usb_serial_jtag_ll.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

#if !SOC_RCC_IS_INDEPENDENT
#define USJ_RCC_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#define USJ_RCC_ATOMIC()
#endif

/* The hardware buffer has a fixed size of 64 bytes */

#define ESP_USBCDC_BUFFERSIZE 64

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_priv_s
{
  const uint8_t  source;        /* Source ID */
  const uint8_t  irq;           /* IRQ number assigned to the source */
  int            cpuint;        /* CPU interrupt assigned */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_interrupt(int irq, void *context, void *arg);

/* Serial driver methods */

static int  esp_setup(struct uart_dev_s *dev);
static void esp_shutdown(struct uart_dev_s *dev);
static int  esp_attach(struct uart_dev_s *dev);
static void esp_detach(struct uart_dev_s *dev);
static void esp_txint(struct uart_dev_s *dev, bool enable);
static void esp_rxint(struct uart_dev_s *dev, bool enable);
static bool esp_rxavailable(struct uart_dev_s *dev);
static bool esp_txready(struct uart_dev_s *dev);
static void esp_send(struct uart_dev_s *dev, int ch);
static ssize_t esp_sendbuf(struct uart_dev_s *dev, const void *buf,
                           size_t len);
static int  esp_receive(struct uart_dev_s *dev, unsigned int *status);
static int  esp_ioctl(struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_rxbuffer[ESP_USBCDC_BUFFERSIZE];
static char g_txbuffer[ESP_USBCDC_BUFFERSIZE];

static struct esp_priv_s g_usbserial_priv =
{
  .source = ETS_USB_SERIAL_JTAG_INTR_SOURCE,
  .irq    = ESP_SOURCE2IRQ(ETS_USB_SERIAL_JTAG_INTR_SOURCE),
  .cpuint = -ENOMEM,
};

static struct uart_ops_s g_uart_ops =
{
  .setup       = esp_setup,
  .shutdown    = esp_shutdown,
  .attach      = esp_attach,
  .detach      = esp_detach,
  .txint       = esp_txint,
  .rxint       = esp_rxint,
  .rxavailable = esp_rxavailable,
  .txready     = esp_txready,
  .txempty     = NULL,
  .send        = esp_send,
  .sendbuf     = esp_sendbuf,
  .receive     = esp_receive,
  .ioctl       = esp_ioctl,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* RAM debug marker: every dbg_putc()/dbg_mark_char() records its character
 * to this FIXED SRAM address.  The value survives a warm (USB) reset because
 * the address sits in the heap region well above .bss (which crt0 zeroes on
 * boot) and far below the top of internal SRAM, so boot-time heap growth
 * never reaches it.  After a hang we boot back to NSH and read the word with
 * `mem 0x4ffae000` to learn the exact last debug point the CPU reached --
 * independent of the (lossy) USB console.
 */

#define DBG_MARK_ADDR    ((volatile uint32_t *)0x5010fff0) /* boot/driver markers */
#define DBG_APP_MARK_ADDR ((volatile uint32_t *)0x5010ffe0) /* app-only markers */
volatile uint32_t g_dbg_mark;   /* mirror, for debugger/symbolic access */

/* Gate for dbg_putc()'s console TX.  Boot keeps it enabled so the bringup M
 * dump reaches the host; esp_bringup() disables it once the dump is printed.
 * From then on dbg_putc() only records its char in the RAM marker and never
 * touches the USB-Serial-JTAG TX FIFO -- the polled per-byte path no longer
 * races the interrupt-driven esp_sendbuf() drain, so the app console (printf,
 * NSH prompt) stays clean.  Any stray polled markers (e.g. the ESP-HAL JPEG
 * engine's DPUT trace) still land in the marker for post-hang diagnosis. */

volatile bool g_dbg_console_tx = true;

uart_dev_t g_uart_usbserial =
{
  .isconsole = true,
  .recv =
    {
      .size = ESP_USBCDC_BUFFERSIZE,
      .buffer = g_rxbuffer,
    },
  .xmit =
    {
      .size = ESP_USBCDC_BUFFERSIZE,
      .buffer = g_txbuffer,
    },
  .ops = &g_uart_ops,
  .priv = &g_usbserial_priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_interrupt
 *
 * Description:
 *   This is the common UART interrupt handler. It will be invoked when an
 *   interrupt is received on the 'irq'. It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers. The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int esp_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  uint32_t int_status = usb_serial_jtag_ll_get_intsts_mask();

  /* Send buffer has room and can accept new data. */

  if ((int_status & USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY) != 0)
    {
      usb_serial_jtag_ll_clr_intsts_mask(
        USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY);
      uart_xmitchars(dev);

      /* If uart_xmitchars() just drained the xmit buffer empty, esp_txint()
       * leaves IN_EMPTY armed so that this handler runs once more once the
       * TX FIFO frees.  SERIAL_IN_EP_DATA_FREE is latched low right after a
       * flush until the USB engine reads the FIFO, so a flush issued at
       * drain-complete may be a no-op while the host is left holding a full
       * 64-byte packet as an incomplete USB IN transaction -- its read()
       * never returns and the console wedges.  This IN_EMPTY (FIFO freed)
       * is the right moment to send the zero-length packet that terminates
       * the transaction, then disarm. */

      if (dev->xmit.head == dev->xmit.tail &&
          usb_serial_jtag_ll_txfifo_writable())
        {
          usb_serial_jtag_ll_txfifo_flush();   /* zero-length packet */
          usb_serial_jtag_ll_disable_intr_mask(
            USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY);
        }
    }

  /* Data from the host are available to read. */

  if ((int_status & USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT) != 0)
    {
      usb_serial_jtag_ll_clr_intsts_mask(
        USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);
      uart_recvchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: esp_setup
 *
 * Description:
 *   This method is called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int esp_setup(struct uart_dev_s *dev)
{
  /* Zero the esp_send TX-drop diagnostics (0x5010ff58/5c) so the drop
   * counter is a clean per-boot count instead of adding to leftover RAM. */

  *((volatile uint32_t *)0x5010ff58) = 0;
  *((volatile uint32_t *)0x5010ff5c) = 0;
  return OK;
}

/****************************************************************************
 * Name: esp_shutdown
 *
 * Description:
 *   This method is called when the serial port is closed.
 *
 ****************************************************************************/

static void esp_shutdown(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: esp_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void esp_txint(struct uart_dev_s *dev, bool enable)
{
  if (enable)
    {
      /* Enable the TX-ready interrupt.  IN_EMPTY is level-triggered on the
       * "TX FIFO is empty" condition, so with the FIFO idle this fires
       * immediately and esp_interrupt() drains the xmit buffer -- no kick
       * flush needed here. */

      usb_serial_jtag_ll_ena_intr_mask(
        USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY);
    }
  else
    {
      /* Do not disarm IN_EMPTY here.  After the xmit buffer drains, a final
       * flush may have left a full 64-byte packet pending with SERIAL_IN_EP_
       * DATA_FREE latched low; disabling IN_EMPTY now would discard the
       * interrupt that fires when the FIFO frees, so the zero-length packet
       * that terminates the transaction would never be sent and the host
       * would keep the last full packet (console wedges).  esp_interrupt()
       * sends the ZLP and disarms once the FIFO is free. */
    }
}

/****************************************************************************
 * Name: esp_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

static void esp_rxint(struct uart_dev_s *dev, bool enable)
{
  if (enable)
    {
      usb_serial_jtag_ll_ena_intr_mask(
        USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);
    }
  else
    {
      usb_serial_jtag_ll_disable_intr_mask(
        USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);
    }
}

/****************************************************************************
 * Name: esp_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode. This method
 *   is called when the serial port is opened. Normally, this is just after
 *   the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling). The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int esp_attach(struct uart_dev_s *dev)
{
  struct esp_priv_s *priv = dev->priv;

  DEBUGASSERT(priv->cpuint == -ENOMEM);

  USJ_RCC_ATOMIC()
    {
      usb_serial_jtag_ll_enable_bus_clock(true);
    }

  usb_serial_jtag_ll_phy_set_defaults();

  usb_serial_jtag_ll_ena_intr_mask(USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);

  /* Try to attach the IRQ to a CPU int */

  priv->cpuint = esp_setup_irq(priv->source,
                               ESP_IRQ_PRIORITY_DEFAULT,
                               ESP_IRQ_TRIGGER_LEVEL,
                               esp_interrupt,
                               dev);
  if (priv->cpuint < 0)
    {
      return priv->cpuint;
    }

  /* Attach and enable the IRQ */

  if (priv->cpuint >= 0)
    {
      up_enable_irq(priv->irq);
    }
  else
    {
      up_disable_irq(priv->irq);
    }

  return OK;
}

/****************************************************************************
 * Name: esp_detach
 *
 * Description:
 *   Detach UART interrupts. This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void esp_detach(struct uart_dev_s *dev)
{
  struct esp_priv_s *priv = dev->priv;

  DEBUGASSERT(priv->cpuint != -ENOMEM);

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
  esp_teardown_irq(priv->source, priv->cpuint);

  priv->cpuint = -ENOMEM;
}

/****************************************************************************
 * Name: esp_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool esp_rxavailable(struct uart_dev_s *dev)
{
  return (bool)usb_serial_jtag_ll_rxfifo_data_available();
}

/****************************************************************************
 * Name: esp_txready
 *
 * Description:
 *   Return true if the transmit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool esp_txready(struct uart_dev_s *dev)
{
  return (bool)usb_serial_jtag_ll_txfifo_writable();
}

/****************************************************************************
 * Name: esp_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void esp_send(struct uart_dev_s *dev, int ch)
{
  uint8_t buf[1] = {
    (uint8_t)ch
  };

  /* Record the last char pushed through the console driver's TX interrupt
   * path (uart_xmitchars -> esp_send) at a fixed RAM marker.  dbg_putc
   * writes DBG_MARK_ADDR directly and never comes through here, so this
   * address distinguishes a K-flood flowing through the xmit buffer from a
   * polled-output flood (which would hit esp_usbserial_write / 0x5010ffd4). */

  *((volatile uint32_t *)0x5010ffd0) = (uint32_t)(uint8_t)ch;

  /* Write the character to the buffer and flush it out. */

  usb_serial_jtag_ll_write_txfifo(buf, sizeof(buf));
  usb_serial_jtag_ll_txfifo_flush();
}

/****************************************************************************
 * Name: esp_sendbuf
 *
 * Description:
 *   This method will send a block of data on the UART.
 *
 *   The USB-Serial-JTAG TX FIFO is a 64-byte hardware FIFO.  The per-byte
 *   esp_send() path issues a wr_done flush for every character; if a flush
 *   lands while the USB engine is still transmitting the previous packet it
 *   can be dropped, leaving bytes stuck in the FIFO with SERIAL_IN_EP_DATA_
 *   FREE latched low and IN_EMPTY never re-asserting -- the console wedges
 *   at an arbitrary byte count.  Writing a chunk and flushing ONCE (the
 *   ESP-IDF driver pattern) avoids racing the engine, so this method is used
 *   for the interrupt-driven drain while esp_send() remains for the polled
 *   paths (esp_usbserial_write / dbg_putc).
 *
 ****************************************************************************/

static ssize_t esp_sendbuf(struct uart_dev_s *dev, const void *buf,
                           size_t len)
{
  ssize_t sent = usb_serial_jtag_ll_write_txfifo(buf, len);
  usb_serial_jtag_ll_txfifo_flush();

  /* Keep the "last char through the TX interrupt path" RAM marker current
   * for the post-hang dump even though this path bypasses esp_send(). */

  if (sent > 0)
    {
      *((volatile uint32_t *)0x5010ffd0) =
        (uint32_t)((const uint8_t *)buf)[sent - 1];
    }

  return sent;
}

/****************************************************************************
 * Name: esp32_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character.
 *
 ****************************************************************************/

static int esp_receive(struct uart_dev_s *dev, unsigned int *status)
{
  uint8_t buf[1] = {
    0
  };

  *status = 0;
  usb_serial_jtag_ll_read_rxfifo(buf, sizeof(buf));

  return (int)buf[0];
}

/****************************************************************************
 * Name: esp_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int esp_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
  int                ret   = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
          }
        else
          {
            /* The USB Serial Console has fixed configuration of:
             *    9600 baudrate, no parity, 8 bits, 1 stopbit.
             */

            termiosp->c_cflag = CS8;
            cfsetispeed(termiosp, 9600);
          }
      }
      break;

    case TCSETS:
      ret = -ENOTTY;
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_usbserial_write
 *
 * Description:
 *   Write one character through the USB serial. Used mainly for early
 *   debugging.
 *
 ****************************************************************************/

void esp_usbserial_write(char ch)
{
  /* Record the last polled char (riscv_lowputc / dbg_mark_char path) at a
   * separate fixed marker so the K-flood source can be identified post-hang
   * even though this function is non-blocking and silently drops bytes. */

  *((volatile uint32_t *)0x5010ffd4) = (uint32_t)(uint8_t)ch;

  /* Non-blocking: if the USB-Serial-JTAG TX FIFO is full, drop the byte
   * instead of spinning on serial_in_ep_data_free.  A blocking wait here
   * deadlocks the caller when the console TX interrupt (SERIAL_IN_EMPTY)
   * is masked for polled debugging output: nothing drains the FIFO. */

  if (!esp_txready(&g_uart_usbserial))
    {
      return;
    }

  esp_send(&g_uart_usbserial, ch);
}

/****************************************************************************
 * Name: dbg_putc
 *
 * Description:
 *   Polled debug marker with a bounded wait and proper USB transaction
 *   handling.  Works with interrupts masked.  Never blocks forever.
 *
 *   The USB-Serial-JTAG TX FIFO auto-flushes a full 64-byte packet but
 *   without a "transaction complete" signal, so the host holds those 64
 *   bytes as an incomplete USB transaction unless we follow up with a
 *   zero-length packet once the FIFO frees.  We therefore wait for FIFO
 *   room, write one byte, flush, and then -- if the FIFO becomes full and
 *   frees again -- flush a second time to emit the ZLP (see ESP-IDF
 *   usb_serial_jtag_wait_tx_done_no_driver).  Each wait is bounded, so a
 *   host that stopped pulling costs at most one marker, never a hang.
 *
 ****************************************************************************/

void dbg_putc(int ch)
{
  volatile int tries = 0;
  uint8_t b = (uint8_t)ch;

  /* Record the character in the RAM marker before attempting TX. */

  *DBG_MARK_ADDR = b;
  g_dbg_mark = b;

  /* App phase: RAM marker only.  Disabling the polled console TX here keeps
   * dbg_putc() from racing the interrupt-driven esp_sendbuf() drain on the
   * same 64-byte USB-Serial-JTAG TX FIFO -- the root cause of the mangled
   * console around jpegenc.  The RAM marker is the authoritative post-hang
   * record either way. */

  if (!g_dbg_console_tx)
    {
      return;
    }

  /* Wait for FIFO room so the byte actually lands. */

  while (!esp_txready(&g_uart_usbserial))
    {
      if (++tries > 300000)
        {
          return;               /* host not pulling: give up, no spin */
        }
    }

  /* Write the byte and mark the transaction done. */

  usb_serial_jtag_ll_write_txfifo(&b, 1);
  usb_serial_jtag_ll_txfifo_flush();

  /* If the byte filled the FIFO to 64, the HW auto-flushes it as a
   * possibly-incomplete transaction.  Wait for the FIFO to free, then
   * send a zero-length packet so the host delivers the last 64 bytes. */

  tries = 0;
  while (!esp_txready(&g_uart_usbserial))
    {
      if (++tries > 300000)
        {
          return;
        }
    }

  usb_serial_jtag_ll_txfifo_flush();        /* zero-length packet */
}

/****************************************************************************
 * Name: dbg_console_tx_set
 *
 * Description:
 *   Enable/disable the polled console TX in dbg_putc().  esp_bringup() keeps
 *   it enabled while printing the RAM marker dump, then disables it before
 *   returning so app-phase polled markers never contend with the console's
 *   interrupt-driven drain.
 *
 ****************************************************************************/

void dbg_console_tx_set(bool enable)
{
  g_dbg_console_tx = enable;
}

/****************************************************************************
 * Name: dbg_mark_char
 *
 * Description:
 *   Record a character into the RAM marker and attempt a non-blocking TX of
 *   it (same behavior as the bare up_putc markers).  Used inside spin-lock
 *   critical sections where a bounded polled wait is undesirable; the RAM
 *   marker write is what matters for post-hang diagnosis.
 *
 ****************************************************************************/

void dbg_mark_char(int ch)
{
  /* App-only marker address: boot's dbg_putc() markers go to DBG_MARK_ADDR
   * (0x5010fff0), so 0x5010ffe0 survives a reset and the esp_bringup dump
   * can read the previous app run's last marker without boot pollution.
   */

  *DBG_APP_MARK_ADDR = (uint8_t)ch;
  g_dbg_mark = (uint8_t)ch;

  /* Deliberately no console TX here.  esp_usbserial_write() is a polled,
   * per-byte flush that races the interrupt-driven esp_sendbuf() drain on
   * the same 64-byte USB-Serial-JTAG TX FIFO: the two paths interleave and
   * drop bytes, corrupting console output (jpegnc's MARK chars visibly
   * mangled concurrent printf output).  The RAM marker is the authoritative
   * post-hang record, and it stays readable via the boot M dump.
   */
}
