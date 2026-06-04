/****************************************************************************
 * arch/xtensa/src/t113/t113_serial.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * T113-S3 HiFi4 DSP UART driver.  Copied from the ARM AP driver
 * (arch/arm/src/t113/t113_serial.c) -- same DesignWare-8250 IP -- and
 * adapted for the DSP: physical-address MMIO (no MMU), R_INTC interrupt
 * numbers, and the AP owns clock/pinmux so the SoC bring-up helpers are
 * stubbed.  The DMA path is preserved verbatim under CONFIG_T113_UART_DMA
 * (not enabled in the DSP defconfig yet) as the starting point for a
 * future DSP UART DMA driver -- the DSP can drive DMAC channels 8-15.
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
#include <inttypes.h>
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
#include <nuttx/fs/ioctl.h>
#include <nuttx/percpu.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#ifdef CONFIG_T113_UART_DMA
#  include <nuttx/cache.h>
#  include <nuttx/kmalloc.h>
#endif

#include "xtensa.h"
#include "hardware/t113_uart.h"
#include "hardware/t113_gpio.h"
#ifdef CONFIG_T113_UART_DMA
#  include "hardware/t113_dma.h"
#  include "t113_dma.h"
#endif
#include "t113_ccu.h"
#include "t113_config.h"
#include "t113_serial.h"

/* MMIO ordering barrier.  On ARM this is arm_dsb(15); the Xtensa
 * equivalent is the memw instruction, which orders memory-mapped device
 * accesses.  Used between paired CCU / DMA-REQ register writes.
 * nuttx/arch.h supplies an empty UP_DSB() fallback for arches with no DSB
 * concept; override it here since Xtensa does have an ordering instruction.
 */

#undef  UP_DSB
#define UP_DSB()  __asm__ __volatile__ ("memw" : : : "memory")

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#if defined(USE_SERIALDRIVER) && defined(HAVE_UART_DEVICE)

/* SCLK is the UART input clock (APB1 bus clock), defined in
 * hardware/t113_uart.h as BOARD_APB1_FREQUENCY.
 */

#ifdef CONFIG_T113_UART_DMA
#  define UART_DMA_BOUNCE_SIZE CONFIG_T113_UART_DMA_BOUNCE_SIZE
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t uartbase;    /* Base address of UART registers */
  uint32_t baud;        /* Configured baud */
  uint32_t ier;         /* Saved IER value */
  uint16_t fifo_depth;  /* HW FIFO depth: 64 (UART0) or 256 */
  uint8_t  irq;         /* IRQ associated with this UART */
  uint8_t  parity;      /* 0=none, 1=odd, 2=even */
  uint8_t  bits;        /* Number of bits (7 or 8) */
  bool     stopbits2;   /* true: Configure with 2 stop bits */
#ifdef CONFIG_T113_UART_DMA
  uint8_t  drq;         /* DRQ port number (14-19), 0=no DMA */
#endif
#ifdef CONFIG_T113_UART_DMA
  DMA_HANDLE        rxdma;           /* RX DMA channel handle */
  FAR uint8_t      *rxbounce;        /* Cache-aligned RX bounce buffer */
  bool              rxdma_started;   /* DMA is allocated & armed */
  size_t            rxdma_last_res;  /* Residual seen at previous poll */
  uint8_t           rxdma_stall_cnt; /* Consecutive polls with same res */
  uint32_t          rxoverrun;       /* Bytes dropped due to ring-full */
#endif
#ifdef CONFIG_T113_UART_DMA
  DMA_HANDLE        txdma;           /* TX DMA channel handle */
  FAR uint8_t      *txbounce;        /* Cache-aligned TX bounce buffer */
  volatile bool     txdma_active;    /* TX DMA transfer in progress */

#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  uart_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static ssize_t up_recvbuf(struct uart_dev_s *dev, void *buf, size_t len);
static void up_send(struct uart_dev_s *dev, int ch);
static ssize_t up_sendbuf(struct uart_dev_s *dev, const void *buf,
                          size_t len);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

#ifdef CONFIG_T113_UART_DMA
static void up_dmasend(FAR struct uart_dev_s *dev);
static void up_dmatxavail(FAR struct uart_dev_s *dev);
static void up_dma_txcallback(DMA_HANDLE handle, uint8_t status,
                               FAR void *arg);
#endif
#ifdef CONFIG_T113_UART_DMA
static void up_dma_rx_start(FAR struct up_dev_s *priv,
                            FAR struct uart_dev_s *dev,
                            bool rearm);
static void up_dma_rx_deliver(FAR struct up_dev_s *priv,
                              FAR struct uart_dev_s *dev,
                              size_t nbytes, bool from_dma);
static void up_dma_rx_drain(FAR struct up_dev_s *priv,
                            FAR struct uart_dev_s *dev);
static void up_dmareceive(FAR struct uart_dev_s *dev);
static void up_dmarxfree(FAR struct uart_dev_s *dev);
static void up_dma_rxcallback(DMA_HANDLE handle, uint8_t status,
                               FAR void *arg);
#endif

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
  .recvbuf        = up_recvbuf,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
#ifdef CONFIG_T113_UART_DMA
  .dmareceive     = up_dmareceive,
  .dmarxfree      = up_dmarxfree,
#endif
  .send           = up_send,
  .sendbuf        = up_sendbuf,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
#ifdef CONFIG_T113_UART_DMA
  .dmasend        = up_dmasend,
  .dmatxavail     = up_dmatxavail,
#endif
};

/* I/O buffers */

#ifdef CONFIG_T113_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_T113_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

#ifdef CONFIG_T113_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

#ifdef CONFIG_T113_UART3
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];
#endif

/* DMA bounce buffers - statically allocated, 64-byte aligned (T113 cache
 * line).  Pre-allocating here keeps up_setup() free of any sleeping
 * primitive (kmm_memalign internally takes mm_takesemaphore), which on
 * SMP would deadlock when uart_open holds dev->lock across uart_setup
 * (the task can migrate while owning the rspinlock).  Cost is bounded:
 * 5 ports x 2 directions x CONFIG_T113_UART_DMA_BOUNCE_SIZE bytes,
 * trivial on T113-S3's 128 MB DDR.
 *
 * Console UART0 is excluded - it never enters the DMA branch in
 * up_setup() (filtered by !dev->isconsole), so its bounce slot would be
 * dead weight.
 */

#ifdef CONFIG_T113_UART_DMA

#ifdef CONFIG_T113_UART1
static uint8_t g_uart1_rxbounce[UART_DMA_BOUNCE_SIZE] aligned_data(64);
static uint8_t g_uart1_txbounce[UART_DMA_BOUNCE_SIZE] aligned_data(64);
#endif

#ifdef CONFIG_T113_UART2
static uint8_t g_uart2_rxbounce[UART_DMA_BOUNCE_SIZE] aligned_data(64);
static uint8_t g_uart2_txbounce[UART_DMA_BOUNCE_SIZE] aligned_data(64);
#endif

#ifdef CONFIG_T113_UART3
static uint8_t g_uart3_rxbounce[UART_DMA_BOUNCE_SIZE] aligned_data(64);
static uint8_t g_uart3_txbounce[UART_DMA_BOUNCE_SIZE] aligned_data(64);
#endif

#endif /* CONFIG_T113_UART_DMA */

/* This describes the state of the T113-S3 UART0 port. */

#ifdef CONFIG_T113_UART0
static struct up_dev_s g_uart0priv =
{
  .uartbase       = T113_UART_VADDR(0),
  .baud           = CONFIG_UART0_BAUD,
  .fifo_depth     = 64,
  .irq            = T113_IRQ_UART0,
  .parity         = CONFIG_UART0_PARITY,
  .bits           = CONFIG_UART0_BITS,
  .stopbits2      = CONFIG_UART0_2STOP,
#ifdef CONFIG_T113_UART_DMA
  .drq            = DRQ_UART0_RX,
#endif
};

static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = CONFIG_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart0priv,
};
#endif

/* This describes the state of the T113-S3 UART1 port. */

#ifdef CONFIG_T113_UART1
static struct up_dev_s g_uart1priv =
{
  .uartbase       = T113_UART_VADDR(1),
  .baud           = CONFIG_UART1_BAUD,
  .fifo_depth     = 256,
  .irq            = T113_IRQ_UART1,
  .parity         = CONFIG_UART1_PARITY,
  .bits           = CONFIG_UART1_BITS,
  .stopbits2      = CONFIG_UART1_2STOP,
#ifdef CONFIG_T113_UART_DMA
  .drq            = DRQ_UART1_RX,
  .rxbounce       = g_uart1_rxbounce,
  .txbounce       = g_uart1_txbounce,
#endif
};

static uart_dev_t g_uart1port =
{
  .recv     =
  {
    .size   = CONFIG_UART1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart1priv,
};
#endif

/* This describes the state of the T113-S3 UART2 port. */

#ifdef CONFIG_T113_UART2
static struct up_dev_s g_uart2priv =
{
  .uartbase       = T113_UART_VADDR(2),
  .baud           = CONFIG_UART2_BAUD,
  .fifo_depth     = 256,
  .irq            = T113_IRQ_UART2,
  .parity         = CONFIG_UART2_PARITY,
  .bits           = CONFIG_UART2_BITS,
  .stopbits2      = CONFIG_UART2_2STOP,
#ifdef CONFIG_T113_UART_DMA
  .drq            = DRQ_UART2_RX,
  .rxbounce       = g_uart2_rxbounce,
  .txbounce       = g_uart2_txbounce,
#endif
};

static uart_dev_t g_uart2port =
{
  .recv     =
  {
    .size   = CONFIG_UART2_RXBUFSIZE,
    .buffer = g_uart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART2_TXBUFSIZE,
    .buffer = g_uart2txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart2priv,
};
#endif

/* This describes the state of the T113-S3 UART3 port. */

#ifdef CONFIG_T113_UART3
static struct up_dev_s g_uart3priv =
{
  .uartbase       = T113_UART_VADDR(3),
  .baud           = CONFIG_UART3_BAUD,
  .fifo_depth     = 256,
  .irq            = T113_IRQ_UART3,
  .parity         = CONFIG_UART3_PARITY,
  .bits           = CONFIG_UART3_BITS,
  .stopbits2      = CONFIG_UART3_2STOP,
#ifdef CONFIG_T113_UART_DMA
  .drq            = DRQ_UART3_RX,
  .rxbounce       = g_uart3_rxbounce,
  .txbounce       = g_uart3_txbounce,
#endif
};

static uart_dev_t g_uart3port =
{
  .recv     =
  {
    .size   = CONFIG_UART3_RXBUFSIZE,
    .buffer = g_uart3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART3_TXBUFSIZE,
    .buffer = g_uart3txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart3priv,
};
#endif

/* Which UART with be tty0/console and which tty1-5?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

/* First pick the console and ttys0.  This could be any of UART0-3 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart0port /* UART0 is console */
#    define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart1port /* UART1 is console */
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart2port /* UART2 is console */
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV         g_uart3port /* UART3 is console */
#    define TTYS0_DEV           g_uart3port /* UART3 is ttyS0 */
#    define UART3_ASSIGNED      1
#else
#  undef CONSOLE_DEV                        /* No console */
#  if defined(CONFIG_T113_UART0)
#    define TTYS0_DEV           g_uart0port /* UART0 is ttyS0 */
#    define UART0_ASSIGNED      1
#  elif defined(CONFIG_T113_UART1)
#    define TTYS0_DEV           g_uart1port /* UART1 is ttyS0 */
#    define UART1_ASSIGNED      1
#  elif defined(CONFIG_T113_UART2)
#    define TTYS0_DEV           g_uart2port /* UART2 is ttyS0 */
#    define UART2_ASSIGNED      1
#  elif defined(CONFIG_T113_UART3)
#    define TTYS0_DEV           g_uart3port /* UART3 is ttyS0 */
#    define UART3_ASSIGNED      1
#  endif
#endif

/* Pick ttys1.  This could be any of UART0-5 excluding the console UART. */

#if defined(CONFIG_T113_UART0) && !defined(UART0_ASSIGNED)
#  define TTYS1_DEV           g_uart0port /* UART0 is ttyS1 */
#  define UART0_ASSIGNED      1
#elif defined(CONFIG_T113_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_T113_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_uart2port /* UART2 is ttyS1 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_T113_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS1_DEV           g_uart3port /* UART3 is ttyS1 */
#  define UART3_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of UART1-5. It can't be UART0 because that
 * was either assigned as ttyS0 or ttys1.  One of UART 1-5 could also be the
 * console.
 */

#if defined(CONFIG_T113_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS2_DEV           g_uart1port /* UART1 is ttyS2 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_T113_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_uart2port /* UART2 is ttyS2 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_T113_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS2_DEV           g_uart3port /* UART3 is ttyS2 */
#  define UART3_ASSIGNED      1
#endif

/* Pick ttys3. This could be one of UART2-5. It can't be UART0-1 because
 * those have already been assigned to ttsyS0, 1, or 2.  One of
 * UART 2-5 could also be the console.
 */

#if defined(CONFIG_T113_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS3_DEV           g_uart2port /* UART2 is ttyS3 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_T113_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS3_DEV           g_uart3port /* UART3 is ttyS3 */
#  define UART3_ASSIGNED      1
#endif

/* Pick ttys4. This could be one of UART3-5. It can't be UART0-2 because
 * those have already been assigned to ttsyS0, 1, 2 or 3.  One of
 * UART 3-5 could also be the console.
 */

#if defined(CONFIG_T113_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS4_DEV           g_uart3port /* UART3 is ttyS4 */
#  define UART3_ASSIGNED      1
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset,
                                uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint32_t *ier)
{
  if (ier)
    {
      *ier = priv->ier & UART_IER_ALLIE;
    }

  priv->ier &= ~UART_IER_ALLIE;
  up_serialout(priv, T113_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint32_t ier)
{
  priv->ier |= ier & UART_IER_ALLIE;
  up_serialout(priv, T113_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_enablebreaks
 ****************************************************************************/

static inline void up_enablebreaks(struct up_dev_s *priv, bool enable)
{
  uint32_t lcr = up_serialin(priv, T113_UART_LCR_OFFSET);

  if (enable)
    {
      lcr |= UART_LCR_BC;
    }
  else
    {
      lcr &= ~UART_LCR_BC;
    }

  up_serialout(priv, T113_UART_LCR_OFFSET, lcr);
}

/****************************************************************************
 * Name: t113_uart0config, uart1config, uart2config, ..., uart5config
 *
 * Description:
 *   Configure the UART
 *
 ****************************************************************************/

#ifdef CONFIG_T113_UART0
static inline void t113_uart0config(void)
{
  /* When UART0 is the console, lowputc already initialized clock/pins.
   * Otherwise we must do it here.
   */

#ifndef CONFIG_UART0_SERIAL_CONSOLE
  /* Cycle reset: clear bit16 then set bit16+bit0 in two locked RMWs so
   * concurrent CCU writes from sibling drivers cannot lose updates.
   */

  t113_ccu_modify(T113_CCU_UART_BGR, (1 << 16), 0);
  UP_DSB();
  t113_ccu_modify(T113_CCU_UART_BGR, 0, (1 << 16) | (1 << 0));

  /* Configure I/O pins */

  t113_gpio_config(T113_UART0_TX);
  t113_gpio_config(T113_UART0_RX);
#endif
}
#endif

#ifdef CONFIG_T113_UART1
static inline void t113_uart1config(void)
{
#ifndef CONFIG_UART1_SERIAL_CONSOLE
  /* Enable clock gate and deassert reset (no assert-deassert cycle).
   * Asserting reset clears FCCR to a broken default (0x10003).
   * The power-on state set by BROM has the correct FCCR (0x4003).
   */

  t113_ccu_modify(T113_CCU_UART_BGR, 0, (1 << 17) | (1 << 1));

  /* Configure I/O pins */

  t113_gpio_config(T113_UART1_TX);
  t113_gpio_config(T113_UART1_RX);
#  if defined(T113_UART1_RTS) && defined(T113_UART1_CTS)
  t113_gpio_config(T113_UART1_RTS);
  t113_gpio_config(T113_UART1_CTS);
#  endif
#endif
}
#endif

#ifdef CONFIG_T113_UART2
static inline void t113_uart2config(void)
{
#ifndef CONFIG_UART2_SERIAL_CONSOLE
  /* Cycle reset, barrier, then deassert + enable clock gate */

  t113_ccu_modify(T113_CCU_UART_BGR, (1 << 18), 0);
  UP_DSB();
  t113_ccu_modify(T113_CCU_UART_BGR, 0, (1 << 18) | (1 << 2));

  /* Configure I/O pins */

  t113_gpio_config(T113_UART2_TX);
  t113_gpio_config(T113_UART2_RX);
#  if defined(T113_UART2_RTS) && defined(T113_UART2_CTS)
  t113_gpio_config(T113_UART2_RTS);
  t113_gpio_config(T113_UART2_CTS);
#  endif
#endif
}
#endif

#ifdef CONFIG_T113_UART3
static inline void t113_uart3config(void)
{
#ifndef CONFIG_UART3_SERIAL_CONSOLE
  /* Cycle reset, barrier, then deassert + enable clock gate */

  t113_ccu_modify(T113_CCU_UART_BGR, (1 << 19), 0);
  UP_DSB();
  t113_ccu_modify(T113_CCU_UART_BGR, 0, (1 << 19) | (1 << 3));

  /* Configure I/O pins */

  t113_gpio_config(T113_UART3_TX);
  t113_gpio_config(T113_UART3_RX);
#  if defined(T113_UART3_RTS) && defined(T113_UART3_CTS)
  t113_gpio_config(T113_UART3_RTS);
  t113_gpio_config(T113_UART3_CTS);
#  endif
#endif
}
#endif

/****************************************************************************
 * Name: t113_uartdl
 *
 * Description:
 *   Select a divider to produce the BAUD from the UART PCLK.
 *
 *     BAUD = PCLK / (16 * DL), or
 *     DL   = PCLK / BAUD / 16
 *
 ****************************************************************************/

static inline uint32_t t113_uartdl(uint32_t baud)
{
  /* Round-to-nearest divisor: dl = (CLK + (baud<<3)) / (baud<<4).
   * Truncation alone produces silent aliasing - e.g. with APB1=100MHz,
   * a 5 Mbaud request would give dl=floor(100M/(5M*16))=1, programming
   * 6.25 Mbaud (25% off) without warning.  Round-to-nearest minimises
   * the residual error, then we re-derive the actual baud and warn
   * loudly if the deviation exceeds 2%.
   */

  uint32_t dl     = (T113_UART_CLK + (baud << 3)) / (baud << 4);
  uint32_t actual;
  uint32_t err;

  if (dl < UART_MINDL)
    {
      uwarn("UART baud %" PRIu32 " -> DL %" PRIu32 ", clamped to %d\n",
            baud, dl, UART_MINDL);
      dl = UART_MINDL;
    }

  actual = T113_UART_CLK / (dl << 4);
  err    = actual > baud ? actual - baud : baud - actual;
  if (err * 50 > baud)
    {
      uwarn("UART baud %" PRIu32 " -> DL %" PRIu32 " -> actual %" PRIu32
            " (%" PRIu32 "%% off)\n",
            baud, dl, actual, (err * 100) / baud);
    }

  return dl;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is
 *   called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint16_t dl;
  uint32_t lcr;

  /* Follow the T113 User Manual initialization sequence:
   *   1. Reset and enable FIFOs
   *   2. Halt TX
   *   3. Set DLAB, program DLL/DLH (baud divisor)
   *   4. Clear DLAB
   *   5. Un-halt TX
   *   6. Configure LCR (data bits, stop, parity)
   *   7. Final FIFO configuration
   */

  /* Step 1: Clear and enable FIFOs */

  up_serialout(priv, T113_UART_FCR_OFFSET,
              (UART_FCR_RFIFOR | UART_FCR_XFIFOR));
  up_serialout(priv, T113_UART_FCR_OFFSET,
              (UART_FCR_FIFOE | UART_FCR_RT_ONE));

  /* Step 2: Halt TX before changing divisor */

  up_serialout(priv, T113_UART_HALT_OFFSET, UART_HALT_HALT_TX);

  /* Build LCR value */

  lcr = 0;

  switch (priv->bits)
    {
    case 5:
      lcr |= UART_LCR_DLS_5BITS;
      break;

    case 6:
      lcr |= UART_LCR_DLS_6BITS;
      break;

    case 7:
      lcr |= UART_LCR_DLS_7BITS;
      break;

    case 8:
    default:
      lcr |= UART_LCR_DLS_8BITS;
      break;
    }

  if (priv->stopbits2)
    {
      lcr |= UART_LCR_STOP;
    }

  if (priv->parity == 1)
    {
      lcr |= UART_LCR_PEN;
    }
  else if (priv->parity == 2)
    {
      lcr |= (UART_LCR_PEN | UART_LCR_EPS);
    }

  /* Step 3: Enter DLAB=1 and set baud divisor.
   * USR.BUSY is 0 at open time; no guard needed here.
   */

  up_serialout(priv, T113_UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

  dl = t113_uartdl(priv->baud);
  up_serialout(priv, T113_UART_DLH_OFFSET, dl >> 8);
  up_serialout(priv, T113_UART_DLL_OFFSET, dl & 0xff);

  /* Step 4: Clear DLAB, apply LCR */

  up_serialout(priv, T113_UART_LCR_OFFSET, lcr);

  /* Step 5: Un-halt TX */

  up_serialout(priv, T113_UART_HALT_OFFSET, 0);

  /* Step 6: Save IER state */

  priv->ier = up_serialin(priv, T113_UART_IER_OFFSET);

  /* Step 7: Final FIFO configuration */

  up_serialout(priv, T113_UART_FCR_OFFSET,
               (UART_FCR_RT_ONE | UART_FCR_XFIFOR | UART_FCR_RFIFOR |
                UART_FCR_FIFOE));

  /* Enable Auto-Flow Control in the Modem Control Register */

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
#  warning Missing logic
#endif

  /* DMA channel allocation and UART DMA register setup.
   * Only for UARTs that have DMA enabled via Kconfig (drq > 0).
   * Must happen AFTER baud rate and FIFO configuration above.
   */

#ifdef CONFIG_T113_UART_DMA
  /* Each direction (RX/TX) is allocated independently.  When the
   * shared DMAC pool is exhausted by other peripherals we keep the
   * port working on the PIO path for the failed direction - see
   * up_rxint/up_txint/up_dmatxavail/up_dmarxfree, all of which
   * fall through to the ERBFI/ETBEI path when their handle is
   * NULL.  Console UART skips DMA entirely (early-init, low-baud).
   *
   * Bounce buffers are statically pre-allocated per UART (see
   * g_uart{N}_{rx,tx}bounce above) and wired into priv at struct
   * init, so up_setup() does not need to call kmm_memalign().
   * Keeping this path free of sleeping primitives avoids the SMP
   * deadlock that would otherwise occur if the upper-half held
   * dev->lock across uart_setup() - the task can migrate while
   * holding the rspinlock and wedge it.
   */

  if (priv->drq > 0 && !dev->isconsole)
    {
      bool rx_dma = false;
      bool tx_dma = false;
      uint32_t halt;
      uint32_t req_en = 0;

      DEBUGASSERT(priv->rxbounce != NULL);
      DEBUGASSERT(priv->txbounce != NULL);

      /* RX direction */

      if (priv->rxdma == NULL)
        {
          priv->rxdma = t113_dmachannel();
        }

      if (priv->rxdma != NULL)
        {
          rx_dma = true;
        }

      /* TX direction */

      if (priv->txdma == NULL)
        {
          priv->txdma = t113_dmachannel();
        }

      if (priv->txdma != NULL)
        {
          tx_dma = true;
        }

      /* RX FIFO trigger:
       *   DMA RX  -> RT_ONE: the DMAC needs DRQ asserted per byte so
       *              waiting mode can poll between transfers without
       *              losing bytes (validated for 5-port concurrent
       *              load).
       *   PIO RX  -> RT_HALF (128 bytes on UART1-5's 256-byte FIFO):
       *              IRQ batches drain more bytes per entry; at
       *              3.125M baud with three concurrent PIO ports the
       *              RT_ONE rate exceeded the per-IRQ servicing budget
       *              and the FIFO overflowed mid-stream.  The 16550
       *              character-timeout IIR (4 idle char-times after
       *              non-empty FIFO) covers the stream tail and is
       *              independent of the trigger threshold.
       *
       * Override Step-7's RT_ONE for non-console ports so each
       * direction gets the threshold it actually needs.
       */

      up_serialout(priv, T113_UART_FCR_OFFSET,
                   UART_FCR_FIFOE | UART_FCR_XFIFOR | UART_FCR_RFIFOR |
                   (rx_dma ? UART_FCR_RT_ONE : UART_FCR_RT_HALF));

      /* Per-direction peripheral DMA registers.  Set only the bits
       * that correspond to a direction we actually got a channel
       * for; leaving DMA_REQ_*_EN clear keeps the UART on the PIO
       * IRQ path for that direction.  HSK is written whenever any
       * DMA is in use so DRQ generation matches the DMAC
       * Waiting-mode + BMODE=1 + burst=1 configuration.
       */

      if (rx_dma || tx_dma)
        {
          up_serialout(priv, T113_UART_HSK_OFFSET,
                       UART_HSK_HANDSHAKE);

          if (rx_dma)
            {
              req_en |= UART_DMA_REQ_RX_EN;
            }

          if (tx_dma)
            {
              req_en |= UART_DMA_REQ_TX_EN;
            }

          up_serialout(priv, T113_UART_DMA_REQ_EN_OFFSET, req_en);

          if (rx_dma)
            {
              halt  = up_serialin(priv, T113_UART_HALT_OFFSET);
              halt |= UART_HALT_PTE | UART_HALT_DMA_PTE_RX;
              up_serialout(priv, T113_UART_HALT_OFFSET, halt);

              /* Force ERBFI=0 for DMA-RX ports.  up_rxint() keeps
               * it cleared when the framework toggles RX IRQs;
               * tail drain comes from IID_TIMEOUT (char timeout
               * is not gated by ERBFI) and the HPWORK safety net.
               */

              priv->ier &= ~UART_IER_ERBFI;
              up_serialout(priv, T113_UART_IER_OFFSET, priv->ier);
            }
        }

      _info("UART@%08" PRIx32 ": TX=%s RX=%s\n",
            priv->uartbase,
            tx_dma ? "DMA" : "PIO",
            rx_dma ? "DMA" : "PIO");
    }
#endif /* CONFIG_T113_UART_DMA */

#endif /* !CONFIG_SUPPRESS_UART_CONFIG */
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_disableuartint(priv, NULL);

#ifdef CONFIG_T113_UART_DMA
  /* Tear down peripheral DMA wiring before releasing the channels
   * so a subsequent up_setup() that ends up fully PIO (because the
   * shared DMAC pool is now exhausted by another peripheral) does
   * not inherit stale DMA_REQ_*_EN / HALT_DMA_PTE_RX bits from this
   * session.  Without this, a PIO-only reopen would still see the
   * UART asserting DRQs to a non-existent channel.
   */

  if (priv->drq > 0 && (priv->rxdma != NULL || priv->txdma != NULL))
    {
      uint32_t halt;

      up_serialout(priv, T113_UART_DMA_REQ_EN_OFFSET, 0);

      halt  = up_serialin(priv, T113_UART_HALT_OFFSET);
      halt &= ~(UART_HALT_PTE | UART_HALT_DMA_PTE_RX);
      up_serialout(priv, T113_UART_HALT_OFFSET, halt);
    }

  if (priv->rxdma)
    {
      t113_dmastop(priv->rxdma);
      t113_dmafree(priv->rxdma);
      priv->rxdma = NULL;
      priv->rxdma_started = false;
    }

  if (priv->txdma)
    {
      t113_dmastop(priv->txdma);
      t113_dmafree(priv->txdma);
      priv->txdma = NULL;
      priv->txdma_active = false;
    }

  /* rxbounce / txbounce are statically allocated in this driver (see
   * g_uart{N}_{rx,tx}bounce above); they must persist across reopen
   * and never be freed.  No kmm_free here.
   */
#endif
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.
 *   This method is called when the serial port is opened.
 *   Normally, this is just after the  the setup() method is called,
 *   however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method
 *   (unless the hardware supports multiple levels of interrupt enabling).
 *   The RX and TX interrupts are not enabled until the txint() and rxint()
 *   methods are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  ret = irq_attach(priv->irq, uart_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.
 *   This method is called when the serial port is closed normally
 *   just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: uartN_interrupt and uart_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int uart_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s *priv;
  uint32_t status;
  int passes;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status */

      status = up_serialin(priv, T113_UART_IIR_OFFSET);

      /* Handle the interrupt by its interrupt ID field */

      switch (status & UART_IIR_IID_MASK)
        {
          /* Handle incoming, receive bytes (with or without timeout) */

          case UART_IIR_IID_RECV:
            {
#ifdef CONFIG_T113_UART_DMA
              /* DMA is actively draining the FIFO - IID_RECV
               * fires transiently whenever the level momentarily
               * reaches the trigger threshold (RT_ONE = 1 byte,
               * which happens on essentially every arriving byte
               * during streaming input).  Returning lets DMA keep
               * going without interruption; the UART will re-
               * assert IRQ only when genuine CPU attention is
               * required (char timeout, line status, etc).
               */

              if (priv->rxdma && priv->rxdma_started)
                {
                  return OK;
                }
#endif

              uart_recvchars(dev);
              break;
            }

          case UART_IIR_IID_TIMEOUT:
            {
#ifdef CONFIG_T113_UART_DMA
              /* Char timeout: FIFO has bytes and the line has
               * been idle for 4 character times.  When DMA is
               * armed this is the tail-drain signal - partial
               * bounce buffer + any residual FIFO bytes must be
               * delivered before the input resumes or the ring
               * buffer reader times out.  Route into the DMA
               * drain path (pause + stop DMA, deliver, drain
               * FIFO via CPU, let the framework re-arm).
               */

              if (priv->rxdma && priv->rxdma_started)
                {
                  up_dma_rx_drain(priv, dev);
                  break;
                }
#endif

              uart_recvchars(dev);
              break;
            }

          /* Handle outgoing, transmit bytes */

          case UART_IIR_IID_TXEMPTY:
            {
              uart_xmitchars(dev);
              break;
            }

          /* Just clear modem status interrupts (UART1 only) */

          case UART_IIR_IID_MODEM:
            {
              /* Read the modem status register (MSR) to clear */

              status = up_serialin(priv, T113_UART_MSR_OFFSET);
              break;
            }

          /* Just clear any line status interrupts */

          case UART_IIR_IID_LINESTATUS:
            {
              /* Read the line status register (LSR) to clear */

              status = up_serialin(priv, T113_UART_LSR_OFFSET);
              break;
            }

          /* Busy detect.
           * Just ignore.
           * Cleared by reading the status register
           */

          case UART_IIR_IID_BUSY:
            {
              /* Read from the UART status register
               * to clear the BUSY condition
               */

              status = up_serialin(priv, T113_UART_USR_OFFSET);
              break;
            }

          /* No further interrupts pending... return now */

          case UART_IIR_IID_NONE:
            {
              return OK;
            }

            /* Otherwise we have received an interrupt
             * that we cannot handle
             */

          default:
            {
              _err("ERROR: Unexpected IIR: %02" PRIx32 "\n", status);
              break;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  struct up_dev_s   *priv  = (struct up_dev_s *)dev->priv;
  int                ret   = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
        struct up_dev_s *user = (struct up_dev_s *)arg;
        if (!user)
          {
            ret = -EINVAL;
          }
        else
          {
            memcpy(user, dev, sizeof(struct up_dev_s));
          }
      }
      break;
#endif

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags = enter_critical_section();
        up_enablebreaks(priv, true);
        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;
        flags = enter_critical_section();
        up_enablebreaks(priv, false);
        leave_critical_section(flags);
      }
      break;

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* TODO:  Other termios fields are not yet returned.
         * Note that cfsetospeed is not necessary because we have
         * knowledge that only one speed is supported.
         * Both cfset(i|o)speed() translate to cfsetspeed.
         */

        cfsetispeed(termiosp, priv->baud);
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (struct termios *)arg;
        uint32_t           lcr;  /* Holds current values of line control */
        uint16_t           dl;   /* Divisor latch */
        irqstate_t         flags;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        priv->baud = cfgetispeed(termiosp);

        /* DW UART BUSY-protect: LCR writes are silently discarded when
         * USR.BUSY=1; the subsequent DLH write (offset 0x04, DLAB=0)
         * corrupts IER.  Poll here with IRQs enabled so the TX ISR can
         * drain the FIFO, then enter the critical section.
         */

        while (up_serialin(priv, T113_UART_USR_OFFSET) & UART_USR_BUSY)
          {
          }

        /* Use the HALT TX sequence per T113 User Manual.
         * Disable interrupts to avoid ISR reading DLL/DLH while
         * DLAB is set.
         */

        flags = enter_critical_section();

        {
          uint32_t halt_save;
          halt_save = up_serialin(priv, T113_UART_HALT_OFFSET);
          up_serialout(priv, T113_UART_HALT_OFFSET,
                       halt_save | UART_HALT_HALT_TX);

          lcr = getreg32(priv->uartbase + T113_UART_LCR_OFFSET);
          up_serialout(priv, T113_UART_LCR_OFFSET,
                       (lcr | UART_LCR_DLAB));

          dl = t113_uartdl(priv->baud);
          up_serialout(priv, T113_UART_DLH_OFFSET, dl >> 8);
          up_serialout(priv, T113_UART_DLL_OFFSET, dl & 0xff);

          up_serialout(priv, T113_UART_LCR_OFFSET, lcr);
          up_serialout(priv, T113_UART_HALT_OFFSET,
                       halt_save & ~UART_HALT_HALT_TX);
        }

        leave_critical_section(flags);
      }
      break;
#endif

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t rbr;

  *status = up_serialin(priv, T113_UART_LSR_OFFSET);
  rbr     = up_serialin(priv, T113_UART_RBR_OFFSET);

  /* If the LSR latched an Overrun Error, pulse FCR.RFIFOR to clear
   * the HW FIFO.  Without this, a single overrun can leave stale
   * state that keeps the RX path unreliable until the port is
   * re-opened.  FCR shares its offset with IIR (write-only), so
   * re-write the steady-state FIFO mode with RFIFOR set rather
   * than read-modify-write.  RFIFOR is a self-clearing pulse.
   */

  if (*status & UART_LSR_OE)
    {
      uinfo("UART@%08" PRIx32 ": FIFO overrun, clearing FIFO\n",
            priv->uartbase);
      up_serialout(priv, T113_UART_FCR_OFFSET,
                   UART_FCR_RT_ONE | UART_FCR_XFIFOR |
                   UART_FCR_RFIFOR | UART_FCR_FIFOE);
    }

  return rbr;
}

/****************************************************************************
 * Name: up_recvbuf
 *
 * Description:
 *   Batch-receive bytes from HW FIFO into caller's buffer.
 *   Reads RFL (RX FIFO Level) once, then drains that many bytes via
 *   tight RBR read loop.  Eliminates the per-byte LSR check overhead
 *   of the single-byte up_receive() path.
 *
 ****************************************************************************/

static ssize_t up_recvbuf(struct uart_dev_s *dev, void *buf, size_t len)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t rfl;
  size_t count;
  size_t i;
  char *p = (char *)buf;

  rfl   = up_serialin(priv, T113_UART_RFL_OFFSET);
  count = rfl < len ? rfl : len;

  for (i = 0; i < count; i++)
    {
      *p++ = (char)up_serialin(priv, T113_UART_RBR_OFFSET);
    }

  return (ssize_t)count;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = uart_spinlock(dev, false);

  /* For DMA-mode UARTs keep ERBFI=0 unconditionally.  With RT_ONE
   * trigger the FIFO stays above the threshold for essentially
   * every arriving byte during a stream, and the DMA-armed ISR
   * path only returns OK without clearing the IRQ - a level-
   * triggered IID_RECV then re-fires immediately and monopolizes
   * the CPU, starving the DMAC PKGDONE handler and locking up
   * the whole RX path (observed on BOUNCE=256 2 MB pressure).
   * Tail drain is handled by the HPWORK safety-net poll instead.
   */

#ifdef CONFIG_T113_UART_DMA
  if (priv->rxdma != NULL)
    {
      priv->ier &= ~UART_IER_ERBFI;
      up_serialout(priv, T113_UART_IER_OFFSET, priv->ier);
      uart_spinunlock(dev, false, flags);
      return;
    }
#endif

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= UART_IER_ERBFI;
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_ERBFI;
    }

  up_serialout(priv, T113_UART_IER_OFFSET, priv->ier);
  uart_spinunlock(dev, false, flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, T113_UART_LSR_OFFSET) & UART_LSR_DR) != 0);
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_serialout(priv, T113_UART_THR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_sendbuf
 *
 * Description:
 *   Batch-send bytes from caller's buffer into TX HW FIFO.
 *   Reads TFL (TX FIFO Level) once to compute free space, then
 *   burst-writes that many bytes.  Eliminates per-byte LSR check.
 *
 ****************************************************************************/

static ssize_t up_sendbuf(struct uart_dev_s *dev, const void *buf,
                          size_t len)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t tfl;
  size_t space;
  size_t count;
  size_t i;
  const char *p = (const char *)buf;

  tfl = up_serialin(priv, T113_UART_TFL_OFFSET);
  if (tfl >= priv->fifo_depth)
    {
      return 0;
    }

  space = priv->fifo_depth - tfl;
  count = space < len ? space : len;

  for (i = 0; i < count; i++)
    {
      up_serialout(priv, T113_UART_THR_OFFSET, (uint32_t)*p++);
    }

  return (ssize_t)count;
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

#ifdef CONFIG_T113_UART_DMA
  if (priv->txdma)
    {
      return;  /* DMA mode: TX interrupts handled by DMA */
    }
#endif

  flags = uart_spinlock(dev, false);
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= UART_IER_ETBEI;
      up_serialout(priv, T113_UART_IER_OFFSET, priv->ier);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_ETBEI;
      up_serialout(priv, T113_UART_IER_OFFSET, priv->ier);
    }

  uart_spinunlock(dev, false, flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, T113_UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, T113_UART_LSR_OFFSET) & UART_LSR_TEMT) != 0);
}

/****************************************************************************
 * DMA Functions
 ****************************************************************************/

#ifdef CONFIG_T113_UART_DMA

/****************************************************************************
 * Name: up_dma_txcallback
 *
 * Description:
 *   DMA TX completion callback.  Runs in ISR context.
 *   If there is a second segment (nlength > 0), start it.
 *   Otherwise notify the framework that TX DMA is done.
 *
 ****************************************************************************/

static void up_dma_txcallback(DMA_HANDLE handle, uint8_t status,
                               FAR void *arg)
{
  FAR struct uart_dev_s *dev = (FAR struct uart_dev_s *)arg;
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;

  size_t len;
  struct t113_dma_config_s txcfg;
  irqstate_t txflags;
  size_t done_nbytes;

  if (dev->dmatx.nlength > 0)
    {
      /* Second segment: copy nbuffer to bounce and DMA again */

      len = dev->dmatx.nlength;
      if (len > UART_DMA_BOUNCE_SIZE)
        {
          len = UART_DMA_BOUNCE_SIZE;
        }

      /* Accumulate bytes transferred from first segment.
       * length was already clamped to actual DMA size in
       * up_dmasend.
       */

      dev->dmatx.nbytes = dev->dmatx.length;
      dev->dmatx.nlength = len;

      memcpy(priv->txbounce, dev->dmatx.nbuffer, len);
      up_flush_dcache((uintptr_t)priv->txbounce,
                      (uintptr_t)priv->txbounce + len);

      txcfg.src_drq    = DRQ_DRAM;
      txcfg.dst_drq    = priv->drq;
      txcfg.src_width  = DMAC_WIDTH_8BIT;
      txcfg.dst_width  = DMAC_WIDTH_8BIT;
      txcfg.src_burst  = DMAC_BURST_1;
      txcfg.dst_burst  = DMAC_BURST_1;
      txcfg.src_linear = true;
      txcfg.dst_linear = false;
      txcfg.mode       = DMAC_MODE_DST_HANDSHAKE;
      txcfg.circular   = false;
      txcfg.bmode      = false;

      t113_dmasetup(priv->txdma,
                    (uintptr_t)priv->txbounce,
                    priv->uartbase + T113_UART_THR_OFFSET,
                    len, &txcfg);

      /* Clear nlength so the next callback finishes */

      dev->dmatx.nlength = 0;
      dev->dmatx.nbytes += len;

      t113_dmastart(priv->txdma, up_dma_txcallback, dev);
    }
  else
    {
      /* All segments done.  If nbytes was not set by second-segment
       * path, set it to the first (only) segment length.
       */

      if (dev->dmatx.nbytes == 0)
        {
          dev->dmatx.nbytes = dev->dmatx.length;
        }

      /* Inline the critical part of uart_xmitchars_done under
       * dev->lock: advance tail and clear xfer state BEFORE
       * setting txdma_active=false.  This prevents CPU1 from
       * seeing active==false, starting a new DMA, and then
       * having our tail advance / nbytes=0 clobber the new
       * transfer's state.
       *
       * uart_datasent (sem_post + poll_notify) is deferred
       * outside the lock -- it only wakes waiters and does not
       * touch xfer state.
       */

      txflags = uart_spinlock(dev, false);

      done_nbytes = dev->dmatx.nbytes;
      if (dev->dmatx.buffer ==
          &dev->xmit.buffer[dev->xmit.tail])
        {
          dev->xmit.tail = (dev->xmit.tail + done_nbytes) %
                           dev->xmit.size;
        }

      dev->dmatx.nbytes  = 0;
      dev->dmatx.length  = 0;
      dev->dmatx.nlength = 0;

      priv->txdma_active = false;
      uart_xmitchars_dma(dev);

      uart_spinunlock(dev, false, txflags);

      if (done_nbytes)
        {
          uart_datasent(dev);
        }
    }
}

/****************************************************************************
 * Name: up_dmasend
 *
 * Description:
 *   Start TX DMA transfer.  Called by the serial framework when there
 *   is data in the TX circular buffer ready to send.
 *
 ****************************************************************************/

static void up_dmasend(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  struct t113_dma_config_s txcfg;
  size_t len;

  DEBUGASSERT(priv->txdma != NULL && priv->txbounce != NULL);

  len = dev->dmatx.length;
  if (len > UART_DMA_BOUNCE_SIZE)
    {
      /* Segment 1 > BOUNCE: send the first BOUNCE bytes and drop
       * the segment-2 hint.  We cannot preserve the remainder of
       * segment 1 AND the original segment 2 with only two pointer
       * slots in struct uart_dmaxfer_s (nbuffer/nlength), and
       * overwriting nbuffer with the seg1 remainder would lose
       * seg2.  The tail-advance in up_dma_txcallback matches
       * bytes actually sent (BOUNCE), so the framework re-computes
       * the next segment pair on its next uart_xmitchars_dma call
       * and submits both the leftover seg1 bytes and the original
       * seg2 correctly - at the cost of one extra setup round-trip.
       */

      len = UART_DMA_BOUNCE_SIZE;
      dev->dmatx.nlength = 0;
    }

  dev->dmatx.length = len;

  memcpy(priv->txbounce, dev->dmatx.buffer, len);
  up_flush_dcache((uintptr_t)priv->txbounce,
                  (uintptr_t)priv->txbounce + len);

  /* Setup DMA: Memory(bounce) -> IO(UART THR) */

  txcfg.src_drq    = DRQ_DRAM;
  txcfg.dst_drq    = priv->drq;
  txcfg.src_width  = DMAC_WIDTH_8BIT;
  txcfg.dst_width  = DMAC_WIDTH_8BIT;
  txcfg.src_burst  = DMAC_BURST_1;
  txcfg.dst_burst  = DMAC_BURST_1;
  txcfg.src_linear = true;
  txcfg.dst_linear = false;
  txcfg.mode       = DMAC_MODE_DST_HANDSHAKE;
  txcfg.circular   = false;
  txcfg.bmode      = false;

  dev->dmatx.nbytes = 0;
  priv->txdma_active = true;

  t113_dmasetup(priv->txdma,
                (uintptr_t)priv->txbounce,
                priv->uartbase + T113_UART_THR_OFFSET,
                len, &txcfg);

  t113_dmastart(priv->txdma, up_dma_txcallback, dev);
}

/****************************************************************************
 * Name: up_dmatxavail
 *
 * Description:
 *   Notify the driver that data is available for TX DMA.
 *   If no TX DMA is currently in progress, kick it off via
 *   uart_xmitchars_dma() which will call up_dmasend().
 *
 ****************************************************************************/

static void up_dmatxavail(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  irqstate_t flags;

  /* Check and start TX DMA under dev->lock to prevent the ISR
   * callback on one CPU and a writer thread on another CPU from
   * both calling uart_xmitchars_dma simultaneously.
   */

  flags = uart_spinlock(dev, false);
  if (priv->txdma && !priv->txdma_active)
    {
      uart_xmitchars_dma(dev);
    }

  uart_spinunlock(dev, false, flags);
}
#endif /* CONFIG_T113_UART_DMA */

#ifdef CONFIG_T113_UART_DMA

/****************************************************************************
 * Name: up_dma_rx_start
 *
 * Description:
 *   Start (or re-arm) a single-shot RX DMA transfer.
 *   The DMAC reads UART_DMA_BOUNCE_SIZE bytes from the UART
 *   RBR into the bounce buffer.  On completion, the callback
 *   copies data to the ring buffer and re-arms immediately.
 *
 *   The UART 256-byte FIFO absorbs incoming data during the
 *   brief re-arm gap (~5 us at 1 GHz).
 *
 ****************************************************************************/

static void up_dma_rx_start(FAR struct up_dev_s *priv,
                            FAR struct uart_dev_s *dev,
                            bool rearm)
{
  struct t113_dma_config_s rxcfg;

  /* Re-arm: toggle DMA_REQ_EN to reset DRQ state.
   * Never toggle HSK - that corrupts the handshake FSM.
   */

  up_flush_dcache((uintptr_t)priv->rxbounce,
                  (uintptr_t)priv->rxbounce +
                  UART_DMA_BOUNCE_SIZE);

  rxcfg.src_drq    = priv->drq;
  rxcfg.dst_drq    = DRQ_DRAM;
  rxcfg.src_width  = DMAC_WIDTH_8BIT;
  rxcfg.dst_width  = DMAC_WIDTH_8BIT;
  rxcfg.src_burst  = DMAC_BURST_1;
  rxcfg.dst_burst  = DMAC_BURST_1;
  rxcfg.src_linear = false;
  rxcfg.dst_linear = true;
  rxcfg.mode       = DMAC_MODE_SRC_WAIT;
  rxcfg.circular   = false;
  rxcfg.bmode      = true;

  priv->rxdma_last_res  = UART_DMA_BOUNCE_SIZE;
  priv->rxdma_stall_cnt = 0;

  t113_dmasetup(priv->rxdma,
                priv->uartbase + T113_UART_RBR_OFFSET,
                (uintptr_t)priv->rxbounce,
                UART_DMA_BOUNCE_SIZE, &rxcfg);

  t113_dmastart(priv->rxdma, up_dma_rxcallback, dev);

  if (!rearm)
    {
      uint32_t req_en = UART_DMA_REQ_RX_EN;

      if (priv->txdma != NULL)
        {
          req_en |= UART_DMA_REQ_TX_EN;
        }

      up_serialout(priv, T113_UART_FCR_OFFSET,
                   UART_FCR_RT_ONE | UART_FCR_FIFOE);
      up_serialout(priv, T113_UART_DMA_REQ_EN_OFFSET, 0);
      UP_DSB();

      /* Re-assert only the DRQ enables that have a backing DMA
       * channel.  On a DMA-RX + PIO-TX port (possible when TX
       * channel alloc failed but RX succeeded) writing TX_EN here
       * leaves the UART asserting a TX DRQ that no DMA channel
       * services, while the PIO ETBEI path is what is actually
       * draining THR - at best wasted DRQ traffic, at worst a HW
       * state that interferes with the FIFO empty signal.
       */

      up_serialout(priv, T113_UART_DMA_REQ_EN_OFFSET, req_en);
    }
}

/****************************************************************************
 * Name: up_dma_rx_deliver
 *
 * Description:
 *   Copy nbytes from the DMA bounce buffer into the serial ring buffer.
 *   Shared by the DMA-complete ISR callback and the poll tail-drain path.
 *
 ****************************************************************************/

static void up_dma_rx_deliver(FAR struct up_dev_s *priv,
                              FAR struct uart_dev_s *dev,
                              size_t nbytes, bool from_dma)
{
  FAR struct uart_buffer_s *rxbuf = &dev->recv;
  irqstate_t flags;
  sbuf_size_t head;
  sbuf_size_t tail;
  size_t avail;
  size_t tocopy;
  size_t first;

  if (nbytes == 0)
    {
      return;
    }

  /* Invalidate only when the bounce buffer was written by DMA
   * (bypasses CPU cache).  Skip when data was placed by CPU
   * (FIFO drain) -- cache already holds the correct content.
   *
   * Round the end of the invalidate range up to the next cache-line
   * boundary so the tail line gets invalidated atomically with the
   * data we are about to read.  The bounce buffer is allocated with
   * 64-byte alignment and is dedicated to this driver, so over-
   * invalidating into the unused tail region is safe.  Without this,
   * a partial-line invalidate could miss bytes that the DMAC wrote
   * into the same line concurrently.
   */

  if (from_dma)
    {
      up_invalidate_dcache((uintptr_t)priv->rxbounce,
                           (uintptr_t)priv->rxbounce + nbytes);
    }

  flags = uart_spinlock(dev, false);

  head = rxbuf->head;
  tail = rxbuf->tail;
  avail = (head >= tail) ?
    (size_t)(rxbuf->size - 1) - (head - tail) :
    (size_t)(tail - head) - 1;

  tocopy = avail < nbytes ? avail : nbytes;
  if (tocopy < nbytes)
    {
      /* Ring buffer is too small / reader is too slow - bytes lost.
       * Bump the counter and log on the leading edge of an overrun
       * burst (rxoverrun was zero before this increment).  Flow
       * control is not wired up on this driver yet.
       */

      size_t dropped = nbytes - tocopy;
      if (priv->rxoverrun == 0)
        {
          _err("UART@%08" PRIx32 ": RX ring overrun, %zu byte(s) lost\n",
               priv->uartbase, dropped);
        }

      priv->rxoverrun += (uint32_t)dropped;
    }

  first = (size_t)rxbuf->size - (size_t)head;
  if (first >= tocopy)
    {
      memcpy(&rxbuf->buffer[head], priv->rxbounce, tocopy);
    }
  else
    {
      memcpy(&rxbuf->buffer[head], priv->rxbounce, first);
      memcpy(rxbuf->buffer, priv->rxbounce + first,
             tocopy - first);
    }

  head = (head + tocopy) % rxbuf->size;

  /* Ensure all buffer writes from memcpy are globally visible
   * before the head advance that makes them readable by the
   * consumer on another CPU.  The consumer uses SMP_RMB after
   * observing head to pair with this barrier.
   */

  SMP_WMB();
  rxbuf->head = head;

  uart_spinunlock(dev, false, flags);

  if (head != tail)
    {
      uart_datareceived(dev);
    }
}

static void up_dma_rxcallback(DMA_HANDLE handle, uint8_t status,
                               FAR void *arg)
{
  FAR struct uart_dev_s *dev = (FAR struct uart_dev_s *)arg;
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  irqstate_t flags;

  /* Hold dev->lock across check + deliver + re-arm to serialize
   * with the poll worker drain path.  rspin_lock is recursive,
   * so up_dma_rx_deliver's inner lock acquisition nests safely.
   */

  flags = uart_spinlock(dev, false);
  if (!priv->rxdma_started)
    {
      uart_spinunlock(dev, false, flags);
      return;
    }

  up_dma_rx_deliver(priv, dev, UART_DMA_BOUNCE_SIZE, true);
  up_dma_rx_start(priv, dev, true);
  uart_spinunlock(dev, false, flags);
}

/****************************************************************************
 * Name: up_dmareceive
 *
 * Description:
 *   Start RX DMA.  Called by the serial framework at port open.
 *   Uses single-shot transfers with immediate re-arm in the
 *   callback.  The T113 DMAC does not reliably support
 *   self-linking descriptors, so we re-arm explicitly.
 *
 ****************************************************************************/

static void up_dmareceive(FAR struct uart_dev_s *dev)
{
  /* The serial.c framework never calls dmareceive() directly.
   * DMA is started from up_dmarxfree() which IS called by the
   * framework at port open and when the ring buffer has space.
   * This function exists only to satisfy the ops table.
   */

  UNUSED(dev);
}

/****************************************************************************
 * Name: up_dmarxfree
 *
 * Description:
 *   Called from task context when the reader has consumed data.
 *   If the DMA callback set the re-arm flag, start a new DMA
 *   transfer.  This avoids re-arming DMA from within the DMA
 *   ISR handler.  The 256-byte FIFO absorbs incoming data
 *   during the gap between DMA completion and re-arm.
 *
 ****************************************************************************/

static void up_dmarxfree(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  irqstate_t flags;

  if (priv->rxdma == NULL)
    {
      return;
    }

  DEBUGASSERT(UART_DMA_BOUNCE_SIZE <
              (size_t)dev->recv.size);

  /* Take dev->lock to serialize with the ISR drain path which
   * clears rxdma_started under the same lock.  Without this,
   * we could restart DMA while the drain is still stopping it.
   */

  flags = uart_spinlock(dev, false);
  if (!priv->rxdma_started)
    {
      /* Program the DMAC and publish rxdma_started=true atomically
       * under dev->lock.  Other CPUs must not observe started=true
       * with stale DMAC CNT (before t113_dmastart programmed the
       * channel) or with stale last_res/stall_cnt from a previous
       * session.  up_dma_rx_start itself resets last_res/stall_cnt.
       */

      up_dma_rx_start(priv, dev, false);
      priv->rxdma_started = true;
    }

  uart_spinunlock(dev, false, flags);
}

/****************************************************************************
 * Name: up_dma_rx_drain
 *
 * Description:
 *   Event-driven tail drain for DMA RX.  Called from the UART ISR
 *   when IID_RECV or IID_TIMEOUT fires while DMA is armed.  The
 *   char-timeout interrupt fires after 4 empty character times on
 *   the line, which is exactly the signal "input has paused and
 *   the partial bounce buffer will not be completed soon".
 *
 *   Steps: mark DMA as not-started under dev->lock (so a concurrent
 *   completion callback on another CPU bails out), pause the DMAC
 *   to flush its internal FIFO, read the final residual, stop the
 *   channel, deliver the partial bounce bytes to the ring buffer,
 *   drain any remaining UART FIFO bytes via CPU reads, then return.
 *   The framework re-arms DMA via up_dmarxfree() once the reader
 *   has consumed enough ring buffer space.
 *
 *   Note on ERBFI: up_rxint() forces ERBFI=0 for DMA UARTs, so the
 *   only UART IRQs that reach uart_interrupt() for a DMA-armed port
 *   are line-status and character-timeout (IID_TIMEOUT).  IID_RECV
 *   never fires because the receive-data-available interrupt is
 *   gated by ERBFI.  See the comment block in up_rxint for why
 *   this is required on SMP.
 *
 ****************************************************************************/

static void up_dma_rx_drain(FAR struct up_dev_s *priv,
                            FAR struct uart_dev_s *dev)
{
  irqstate_t flags;
  size_t residual;
  size_t dma_bytes;
  uint32_t rfl;
  size_t fifo_bytes;
  size_t i;

  /* Serialize with the DMA completion callback (runs on any CPU).
   * Both paths clear/check rxdma_started under dev->lock so the
   * first one in wins and the other becomes a no-op.
   */

  flags = uart_spinlock(dev, false);
  if (!priv->rxdma_started)
    {
      uart_spinunlock(dev, false, flags);
      return;
    }

  priv->rxdma_started = false;
  uart_spinunlock(dev, false, flags);

  /* PAUSE waits for the DMAC internal FIFO to drain into DRAM,
   * then we read a stable residual and disable the channel.
   */

  t113_dmapause(priv->rxdma);
  residual = t113_dmaresidual(priv->rxdma);
  t113_dmastop(priv->rxdma);

  dma_bytes = UART_DMA_BOUNCE_SIZE - residual;
  if (dma_bytes > 0)
    {
      up_dma_rx_deliver(priv, dev, dma_bytes, true);
    }

  /* Drain any bytes still sitting in the UART FIFO (e.g. bytes
   * that arrived after the DMAC paused but before we stopped it).
   * Reading RBR also clears the char-timeout IIR flag so the ISR
   * caller will not re-enter in a loop.
   */

  rfl = up_serialin(priv, T113_UART_RFL_OFFSET);
  if (rfl > 0)
    {
      fifo_bytes = rfl;
      if (fifo_bytes > UART_DMA_BOUNCE_SIZE)
        {
          fifo_bytes = UART_DMA_BOUNCE_SIZE;
        }

      for (i = 0; i < fifo_bytes; i++)
        {
          priv->rxbounce[i] =
            (uint8_t)up_serialin(priv,
                                 T113_UART_RBR_OFFSET);
        }

      up_dma_rx_deliver(priv, dev, fifo_bytes, false);
    }
}

/****************************************************************************
 * Name: t113_serial_dma_poll
 *
 * Description:
 *   Low-frequency safety-net poll for partial RX bounce buffers.
 *   The IIR char-timeout interrupt handles most tail-drain cases
 *   (FIFO still holds the final byte when the line goes idle), but
 *   at high baud + burst=1 DMA the DMAC can empty the FIFO via DRQ
 *   before the 4-char-time counter arms - leaving a partial bounce
 *   with no hardware event to trigger delivery.
 *
 *   Drain is gated by a simple residual-unchanged check: we only
 *   stop DMA when the residual has not moved since the previous
 *   poll.  This ensures we never punch a hole in an active stream.
 *
 ****************************************************************************/

static void t113_dma_rx_poll_dev(FAR struct uart_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->priv;
  irqstate_t flags;
  size_t residual;
  size_t dma_bytes;

  flags = uart_spinlock(dev, false);
  if (priv->rxdma == NULL || !priv->rxdma_started)
    {
      uart_spinunlock(dev, false, flags);
      return;
    }

  residual = t113_dmaresidual(priv->rxdma);

  /* Skip the poll in any of these cases:
   *   - residual == 0: DMA completed the full bounce; the DMAC
   *     IRQ (PKGDONE) will fire and up_dma_rxcallback() will
   *     deliver + re-arm.  Draining now would race that path.
   *   - residual >= BOUNCE_SIZE: fresh descriptor, nothing
   *     transferred yet.
   *   - residual moved since last poll: stream still flowing.
   *     Record the new value, reset stall counter.
   */

  if (residual == 0 ||
      residual >= UART_DMA_BOUNCE_SIZE ||
      residual != priv->rxdma_last_res)
    {
      priv->rxdma_last_res  = residual;
      priv->rxdma_stall_cnt = 0;
      uart_spinunlock(dev, false, flags);
      return;
    }

  /* Residual is stuck below BOUNCE_SIZE.  Require two consecutive
   * polls (>=100 ms with no DMA progress) before declaring the
   * stream truly dead - at any supported baud rate DMA would move
   * thousands of bytes in 50 ms, so a single stuck observation can
   * be coincidence but two in a row cannot.
   */

  if (++priv->rxdma_stall_cnt < 2)
    {
      uart_spinunlock(dev, false, flags);
      return;
    }

  /* Claim the drain by clearing rxdma_started under the lock.  A
   * concurrent up_dma_rxcallback on another CPU will observe the
   * cleared flag and bail out, so the DMA channel will not be
   * re-armed under our feet while we stop it.
   */

  priv->rxdma_started = false;
  uart_spinunlock(dev, false, flags);

  /* Pause -> read residual -> stop.  Pause waits for the DMAC
   * internal FIFO to drain into DRAM before we read a stable
   * residual, then we disable the channel.
   */

  t113_dmapause(priv->rxdma);
  residual = t113_dmaresidual(priv->rxdma);
  t113_dmastop(priv->rxdma);

  dma_bytes = UART_DMA_BOUNCE_SIZE - residual;
  if (dma_bytes > 0)
    {
      up_dma_rx_deliver(priv, dev, dma_bytes, true);
    }

  /* Leave FIFO contents for the next DMA arm to pick up - the
   * framework will call up_dmarxfree() once the reader has space
   * and up_dma_rx_start() will re-arm the DMAC on a fresh
   * descriptor.  Draining the FIFO with CPU reads here risks
   * duplicating bytes if the DMAC has already moved them to the
   * bounce region.
   */
}

void t113_serial_dma_poll(void)
{
  /* Per-port tail-drain safety net: scan each DMA-armed RX channel
   * and deliver any bytes the DMAC has already moved into the
   * bounce buffer that the framework has not yet picked up.
   * Only channels 0-7 are used.
   */

#ifdef CONFIG_T113_UART0
  t113_dma_rx_poll_dev(&g_uart0port);
#endif
#ifdef CONFIG_T113_UART1
  t113_dma_rx_poll_dev(&g_uart1port);
#endif
#ifdef CONFIG_T113_UART2
  t113_dma_rx_poll_dev(&g_uart2port);
#endif
#ifdef CONFIG_T113_UART3
  t113_dma_rx_poll_dev(&g_uart3port);
#endif
}

#endif /* CONFIG_T113_UART_DMA */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: xtensa_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during boot up.  This must be called
 *   before xtensa_serialinit.
 *
 *   NOTE: the AP configured clock/pinmux before the DSP started; here we
 *   only program baud/FIFO and mark the console.
 *
 ****************************************************************************/

void xtensa_earlyserialinit(void)
{
  /* Configure all UARTs (except the CONSOLE UART) and disable interrupts */

#ifdef CONFIG_T113_UART0
  t113_uart0config();
#  ifndef CONFIG_UART0_SERIAL_CONSOLE
  up_disableuartint(&g_uart0priv, NULL);
#  endif
#endif

#ifdef CONFIG_T113_UART1
  t113_uart1config();
#  ifndef CONFIG_UART1_SERIAL_CONSOLE
  up_disableuartint(&g_uart1priv, NULL);
#  endif
#endif

#ifdef CONFIG_T113_UART2
  t113_uart2config();
#  ifndef CONFIG_UART2_SERIAL_CONSOLE
  up_disableuartint(&g_uart2priv, NULL);
#  endif
#endif

#ifdef CONFIG_T113_UART3
  t113_uart3config();
#  ifndef CONFIG_UART3_SERIAL_CONSOLE
  up_disableuartint(&g_uart3priv, NULL);
#  endif
#endif

  /* Configuration whichever one is the console */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: xtensa_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   xtensa_earlyserialinit was called previously.
 *
 ****************************************************************************/

void xtensa_serialinit(void)
{
#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV);
#endif
#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
#ifdef TTYS3_DEV
  uart_register("/dev/ttyS3", &TTYS3_DEV);
#endif
#ifdef TTYS4_DEV
  uart_register("/dev/ttyS4", &TTYS4_DEV);
#endif
#ifdef TTYS5_DEV
  uart_register("/dev/ttyS5", &TTYS5_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef CONSOLE_DEV
  /* The DSP has no arm_lowputc(); poll THRE on the console UART and write
   * THR directly.  CONSOLE_DEV resolves to the UART picked as console in
   * the DSP defconfig.
   */

  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;

  while ((up_serialin(priv, T113_UART_LSR_OFFSET) & UART_LSR_THRE) == 0)
    {
    }

  up_serialout(priv, T113_UART_THR_OFFSET, (uint32_t)ch);
#else
  UNUSED(ch);
#endif
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
  UNUSED(ch);
}

#endif /* USE_SERIALDRIVER */
