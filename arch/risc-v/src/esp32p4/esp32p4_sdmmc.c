/****************************************************************************
 * arch/risc-v/src/esp32p4/espressif/esp32p4_sdmmc.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <syslog.h>
#include "esp_cpu.h"
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/mmcsd.h>

#include "riscv_internal.h"
#include "esp_gpio.h"
#include "esp_irq.h"
#include "esp32p4_sdmmc.h"

#include "soc/gpio_sig_map.h"
#include "soc/interrupts.h"

/* The SDMMC clock tree (SDIO PLL power-up in the PMU, bus clock, module
 * reset, LS clock source/divider) spans the PMU, LP_AON_CLKRST and
 * HP_SYS_CLKRST blocks; use the HAL's authoritative sequence rather than
 * open-coding it.
 */

#include "hal/sdmmc_ll.h"
#include "hal/cache_ll.h"
#include "soc/cache_reg.h"
#include "esp_private/periph_ctrl.h"
#include "esp_ldo_regulator.h"

#ifdef CONFIG_ESP32P4_SDMMC_DMA
#  define ESP32P4_NC_ADDR(p) \
          ((uintptr_t)(p) + SOC_NON_CACHEABLE_OFFSET_SRAM)
#endif

#ifdef CONFIG_ESP32P4_SDMMC_DMA
#include "esp_cache.h"
#endif

#ifdef CONFIG_ESP32P4_SDMMC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MCI_DMADES0_OWN         (1UL << 31)
#define MCI_DMADES0_CH          (1 << 4)
#define MCI_DMADES0_FS          (1 << 3)
#define MCI_DMADES0_LD          (1 << 2)
#define MCI_DMADES0_DIC         (1 << 1)
#define MCI_DMADES1_MAXTR       4096
#define MCI_DMADES1_BS1(x)      (x)

/* GPIO matrix constant-level input sources (see esp_gpio_matrix_in) */

#define GPIO_MATRIX_CONST_ONE_INPUT   (0x38)
#define GPIO_MATRIX_CONST_ZERO_INPUT  (0x3c)

/* Timing : 100mS short timeout, 2 seconds for long one */

#define SDCARD_CMDTIMEOUT       MSEC2TICK(100)
#define SDCARD_LONGTIMEOUT      MSEC2TICK(2000)

/* FIFO size in bytes */

#define ESP32P4_TXFIFO_SIZE     (ESP32P4_TXFIFO_DEPTH | ESP32P4_TXFIFO_WIDTH)
#define ESP32P4_RXFIFO_SIZE     (ESP32P4_RXFIFO_DEPTH | ESP32P4_RXFIFO_WIDTH)

/* Number of DMA Descriptors */

#define ESP32P4_MULTIBLOCK_LIMIT  128
#define NUM_DMA_DESCRIPTORS \
        (1 + (ESP32P4_MULTIBLOCK_LIMIT * 512 / MCI_DMADES1_MAXTR))

#if (CONFIG_MMCSD_MULTIBLOCK_LIMIT == 0 || \
     CONFIG_MMCSD_MULTIBLOCK_LIMIT > ESP32P4_MULTIBLOCK_LIMIT)
#error "CONFIG_MMCSD_MULTIBLOCK_LIMIT is too big"
#endif

/* Data transfer interrupt mask bits */

#define SDCARD_RECV_MASK    (SDMMC_INT_DTO | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                             SDMMC_INT_EBE | SDMMC_INT_RXDR | SDMMC_INT_SBE)
#define SDCARD_SEND_MASK    (SDMMC_INT_DTO | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                             SDMMC_INT_EBE | SDMMC_INT_TXDR | SDMMC_INT_SBE)

#define SDCARD_DMARECV_MASK (SDMMC_INT_DTO | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                             SDMMC_INT_SBE | SDMMC_INT_EBE)
#define SDCARD_DMASEND_MASK (SDMMC_INT_DTO | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                             SDMMC_INT_EBE)

#define SDCARD_DMAERROR_MASK (SDMMC_IDINTEN_FBE | SDMMC_IDINTEN_DU | \
                              SDMMC_IDINTEN_AIS)

#define SDCARD_TRANSFER_ALL (SDMMC_INT_DTO | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                             SDMMC_INT_EBE | SDMMC_INT_TXDR | SDMMC_INT_RXDR | \
                             SDMMC_INT_SBE)

/* Event waiting interrupt mask bits */

#define SDCARD_INT_RESPERR  (SDMMC_INT_RE | SDMMC_INT_RCRC | SDMMC_INT_RTO)

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
#  define SDCARD_INT_CDET    SDMMC_INT_CDET
#else
#  define SDCARD_INT_CDET    0
#endif

#define SDCARD_CMDDONE_STA   (SDMMC_INT_CDONE)

#define SDCARD_CMDDONE_MASK  (SDMMC_INT_CDONE)
#define SDCARD_RESPDONE_MASK (SDMMC_INT_CDONE | SDCARD_INT_RESPERR)
#define SDCARD_XFRDONE_MASK  (0)

#define SDCARD_CMDDONE_CLEAR  (SDMMC_INT_CDONE)
#define SDCARD_RESPDONE_CLEAR (SDMMC_INT_CDONE | SDCARD_INT_RESPERR)

#define SDCARD_XFRDONE_CLEAR  (SDCARD_TRANSFER_ALL)

#define SDCARD_WAITALL_CLEAR (SDCARD_CMDDONE_CLEAR | SDCARD_RESPDONE_CLEAR | \
                              SDCARD_XFRDONE_CLEAR)

/* Let's wait until we have both SD card transfer complete and DMA
 * complete.
 */

#define SDCARD_XFRDONE_FLAG  (1)
#define SDCARD_DMADONE_FLAG  (2)
#define SDCARD_ALLDONE       (3)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sdmmc_dma_s
{
  volatile uint32_t des0;        /* Control and status */
  volatile uint32_t des1;        /* Buffer size(s) */
  volatile uint32_t des2;        /* Buffer address pointer 1 */
  volatile uint32_t des3;        /* Next descriptor (chained) */
  uint32_t reserved[12];         /* Pad to 64B (P4 L1 cache line) */
} __attribute__((aligned(64)));

/* This structure lists GPIO Matrix signal numbers for the SD bus signals.
 * Field names match SD bus signal names.
 */

typedef struct
{
  uint8_t clk;
  uint8_t cmd;
  uint8_t d0;
  uint8_t d1;
  uint8_t d2;
  uint8_t d3;
  uint8_t d4;
  uint8_t d5;
  uint8_t d6;
  uint8_t d7;
} sdmmc_slot_io_info_t;

/* Common SDMMC slot info (card detect / write protect / card interrupt) */

typedef struct
{
  uint8_t card_detect;    /* Card detect signal in GPIO Matrix */
  uint8_t write_protect;  /* Write protect signal in GPIO Matrix */
  uint8_t card_int;       /* Card interrupt signal in GPIO Matrix */
} sdmmc_slot_info_t;

/* This structure defines the state of the ESP32-P4 SDIO interface */

struct esp32p4_dev_s
{
  struct sdio_dev_s  dev;             /* Standard, base SDIO interface */

  /* ESP32-P4-specific extensions */

  /* Event support */

  sem_t              waitsem;         /* Implements event waiting */
  sdio_eventset_t    waitevents;      /* Set of events to be waited for */
  uint32_t           waitmask;        /* Interrupt enables for event wait */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  struct wdog_s      waitwdog;        /* Event-timeout watchdog */

  /* Callback support */

  sdio_statset_t     cdstatus;        /* Card status */
  sdio_eventset_t    cbevents;        /* Set of events to cause callbacks */
  worker_t           callback;        /* Registered callback function */
  void              *cbarg;           /* Registered callback argument */
  struct work_s      cbwork;          /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t          *buffer;          /* Address of current R/W buffer */
  size_t             remaining;       /* Bytes remaining in the transfer */
  uint32_t           xfrmask;         /* Interrupt enables for data xfr */
#ifdef CONFIG_ESP32P4_SDMMC_DMA
  uint32_t           dmamask;         /* Interrupt enables for DMA xfr */
  uint8_t           *dmarxbuf;        /* DMA receive buffer, for post-xfr
                                       * cache invalidate */
  size_t             dmarxlen;        /* Length of dmarxbuf */
  volatile struct sdmmc_dma_s dma_desc[NUM_DMA_DESCRIPTORS];
#endif
  bool               wrdir;           /* True: Writing False: Reading */

  /* DMA data transfer support */

  int                slot;            /* SDMMC slot number */
  int                cpuint;          /* Allocated CPU interrupt */

  const sdmmc_slot_io_info_t *sdio_pins;
  const sdmmc_slot_info_t *slot_info;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* #define CONFIG_ESP32P4_SDMMC_REGDEBUG */

#ifdef CONFIG_ESP32P4_SDMMC_REGDEBUG
static uint32_t __esp32p4_getreg(const char *func, uint32_t addr);
static void __esp32p4_putreg(const char *func, uint32_t val, uint32_t addr);

#  define esp32p4_getreg(addr)     __esp32p4_getreg(__func__, addr)
#  define esp32p4_putreg(val,addr) __esp32p4_putreg(__func__, val,addr)
#else
#  define esp32p4_getreg(addr)     getreg32(addr)
#  define esp32p4_putreg(val,addr) putreg32(val,addr)
#endif

/* Data Transfer Helpers ****************************************************/

static void esp32p4_eventtimeout(wdparm_t arg);

/* PROBE: SDMMC interrupt hit counter and last MINTSTS value. */

extern volatile uint32_t g_irq_nohandler_count;
extern volatile uint32_t g_irq_nohandler_last;

static volatile uint32_t g_sdmmc_isr_count;
static volatile uint32_t g_sdmmc_isr_last;
static void esp32p4_endwait(struct esp32p4_dev_s *priv,
                            sdio_eventset_t wkupevent);
static void esp32p4_endtransfer(struct esp32p4_dev_s *priv,
                                sdio_eventset_t wkupevent);
static void esp32p4_drain_fifo(struct esp32p4_dev_s *priv);

/* Interrupt Handling *******************************************************/

static int  esp32p4_interrupt(int irq, void *context, void *arg);

/* SDIO interface methods ***************************************************/

/* Mutual exclusion */

#ifdef CONFIG_SDIO_MUXBUS
static int esp32p4_lock(struct sdio_dev_s *dev, bool lock);
#endif

/* Initialization/setup */

static void esp32p4_reset(struct sdio_dev_s *dev);
static sdio_capset_t esp32p4_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t esp32p4_status(struct sdio_dev_s *dev);
static void esp32p4_widebus(struct sdio_dev_s *dev, bool enable);
static void esp32p4_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate);
static int  esp32p4_attach(struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  esp32p4_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                            uint32_t arg);
#ifdef CONFIG_SDIO_BLOCKSETUP
static void esp32p4_blocksetup(struct sdio_dev_s *dev, unsigned int blocklen,
                               unsigned int nblocks);
#endif
static int  esp32p4_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                              size_t nbytes);
static int  esp32p4_sendsetup(struct sdio_dev_s *dev, const uint8_t *buffer,
                              size_t nbytes);
static int  esp32p4_cancel(struct sdio_dev_s *dev);

static int  esp32p4_waitresponse(struct sdio_dev_s *dev, uint32_t cmd);
static int  esp32p4_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                                 uint32_t *rshort);
static int  esp32p4_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t rlong[4]);
static int  esp32p4_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                              uint32_t *rshort);

/* EVENT handler */

static void esp32p4_waitenable(struct sdio_dev_s *dev,
                               sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t esp32p4_eventwait(struct sdio_dev_s *dev);
static void esp32p4_callbackenable(struct sdio_dev_s *dev,
                                   sdio_eventset_t eventset);
static int  esp32p4_registercallback(struct sdio_dev_s *dev,
                                     worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_ESP32P4_SDMMC_DMA
static int  esp32p4_dmarecvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                                 size_t buflen);
static int  esp32p4_dmasendsetup(struct sdio_dev_s *dev,
                                 const uint8_t *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void esp32p4_callback(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct esp32p4_dev_s g_sdiodev =
{
  .dev =
  {
#ifdef CONFIG_SDIO_MUXBUS
    .lock             = esp32p4_lock,
#endif
    .reset            = esp32p4_reset,
    .capabilities     = esp32p4_capabilities,
    .status           = esp32p4_status,
    .widebus          = esp32p4_widebus,
    .clock            = esp32p4_clock,
    .attach           = esp32p4_attach,
    .sendcmd          = esp32p4_sendcmd,
#ifdef CONFIG_SDIO_BLOCKSETUP
    .blocksetup       = esp32p4_blocksetup,
#endif
    .recvsetup        = esp32p4_recvsetup,
    .sendsetup        = esp32p4_sendsetup,
    .cancel           = esp32p4_cancel,
    .waitresponse     = esp32p4_waitresponse,
    .recv_r1          = esp32p4_recvshortcrc,
    .recv_r2          = esp32p4_recvlong,
    .recv_r3          = esp32p4_recvshort,
    .recv_r4          = esp32p4_recvshort,
    .recv_r5          = esp32p4_recvshortcrc,
    .recv_r6          = esp32p4_recvshortcrc,
    .recv_r7          = esp32p4_recvshort,
    .waitenable       = esp32p4_waitenable,
    .eventwait        = esp32p4_eventwait,
    .callbackenable   = esp32p4_callbackenable,
    .registercallback = esp32p4_registercallback,
#ifdef CONFIG_ESP32P4_SDMMC_DMA
    .dmarecvsetup     = esp32p4_dmarecvsetup,
    .dmasendsetup     = esp32p4_dmasendsetup,
#endif
  },
  .waitsem = SEM_INITIALIZER(0),
};

/* Common SDMMC slot info.  Index 0 is slot 0, index 1 is slot 1.  The
 * numeric constants come from soc/gpio_sig_map.h (hw_ver3): the "_1_"
 * signals belong to slot 0 and the "_2_" signals to slot 1.
 */

static const sdmmc_slot_info_t g_sdmmc_slot_info[] =
{
  {
    .card_detect   = SD_CARD_DETECT_N_1_PAD_IN_IDX,
    .write_protect = SD_CARD_WRITE_PRT_1_PAD_IN_IDX,
    .card_int      = SD_CARD_INT_N_1_PAD_IN_IDX,
  },

  {
    .card_detect   = SD_CARD_DETECT_N_2_PAD_IN_IDX,
    .write_protect = SD_CARD_WRITE_PRT_2_PAD_IN_IDX,
    .card_int      = SD_CARD_INT_N_2_PAD_IN_IDX,
  }
};

/* GPIO Matrix signal indices for the SD bus signals.  On the P4 slot 0 is
 * IOMUX-only (no GPIO-matrix signals), so only slot 1 is populated; the
 * C6 SDIO link uses slot 1.  IN and OUT signal indices are equal on the P4
 * (e.g. SD_CARD_CCMD_2_PAD_IN_IDX == SD_CARD_CCMD_2_PAD_OUT_IDX), so the
 * OUT index is stored and reused for the matrix-in routing.
 */

static const sdmmc_slot_io_info_t g_sdmmc_slot_gpio_sig[] =
{
  {
    .clk = 0,
    .cmd = 0,
    .d0  = 0,
    .d1  = 0,
    .d2  = 0,
    .d3  = 0,
    .d4  = 0,
    .d5  = 0,
    .d6  = 0,
    .d7  = 0,
  },

  {
    .clk = SD_CARD_CCLK_2_PAD_OUT_IDX,
    .cmd = SD_CARD_CCMD_2_PAD_OUT_IDX,
    .d0  = SD_CARD_CDATA0_2_PAD_OUT_IDX,
    .d1  = SD_CARD_CDATA1_2_PAD_OUT_IDX,
    .d2  = SD_CARD_CDATA2_2_PAD_OUT_IDX,
    .d3  = SD_CARD_CDATA3_2_PAD_OUT_IDX,
    .d4  = SD_CARD_CDATA4_2_PAD_OUT_IDX,
    .d5  = SD_CARD_CDATA5_2_PAD_OUT_IDX,
    .d6  = SD_CARD_CDATA6_2_PAD_OUT_IDX,
    .d7  = SD_CARD_CDATA7_2_PAD_OUT_IDX,
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32p4_getreg
 *
 * Description:
 *   This function may be used to intercept and monitor register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   addr - The register address to read
 *
 * Returned Value:
 *   The value read from the register
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32P4_SDMMC_REGDEBUG
static uint32_t __esp32p4_getreg(const char *func, uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval   = 0;
  static uint32_t count    = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              mcerr("%s: ...\n", func);
            }

          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          mcerr("%s: [repeats %d more times]\n", func, count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  mcerr("%s: %08x->%08x\n", func, addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: esp32p4_putreg
 *
 * Description:
 *   This function may be used to intercept and monitor register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   val  - The value to write to the register
 *   addr - The register address to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32P4_SDMMC_REGDEBUG
static void __esp32p4_putreg(const char *func, uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  mcerr("%s: %08x<-%08x\n", func, addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: esp32p4_ciu_sendcmd
 *
 * Description:
 *   Function to send command to Card interface unit (CIU)
 *
 * Input Parameters:
 *   cmd - The command to be executed
 *   arg - The argument to use with the command.
 *
 * Returned Value:
 *   Returns zero on success.  One will be returned on a timeout.
 *
 ****************************************************************************/

static int esp32p4_ciu_sendcmd(uint32_t cmd, uint32_t arg)
{
  clock_t watchtime;

  watchtime = clock_systime_ticks();

  while ((esp32p4_getreg(ESP32P4_SDMMC_CMD) & SDMMC_CMD_STARTCMD) != 0)
    {
      if (watchtime - clock_systime_ticks() > SDCARD_CMDTIMEOUT)
        {
          mcerr("TMO Timed out (%08X)\n",
                esp32p4_getreg(ESP32P4_SDMMC_CMD));
          return 1;
        }
    }

  /* Set command arg reg */

  cmd |= SDMMC_CMD_STARTCMD | SDMMC_CMD_USE_HOLE;

  esp32p4_putreg(arg, ESP32P4_SDMMC_CMDARG);
  esp32p4_putreg(cmd, ESP32P4_SDMMC_CMD);

  mcinfo("cmd=0x%x arg=0x%x\n", cmd, arg);

  return 0;
}

/****************************************************************************
 * Name: configure_pin
 *
 * Description:
 *   Configure one SD bus GPIO and route it through the P4 GPIO matrix.
 *
 ****************************************************************************/

static void configure_pin(int gpio_pin, uint8_t sdio_pin,
                          gpio_pinattr_t attr)
{
  /* Match ESP-IDF sdmmc configure_pin_gpio_matrix(): GPIO function,
   * pulldown off, drive strength 3.  Cancel any previous matrix output
   * before attaching the SDMMC signal so a leftover OE does not hold DAT
   * high (that produces SBE on a 4-bit read).
   */

  attr |= DRIVE_3;
  esp_gpio_matrix_out(gpio_pin, 0x100, false, false);
  esp_configgpio(gpio_pin, attr);

  if (attr & INPUT)
    {
      esp_gpio_matrix_in(gpio_pin, sdio_pin, false);
    }

  if (attr & OUTPUT)
    {
      esp_gpio_matrix_out(gpio_pin, sdio_pin, false, false);
    }
}

/****************************************************************************
 * Name: esp32p4_enable_ints
 *
 * Description:
 *   Enable/disable SD card interrupts per functional settings.
 *
 * Input Parameters:
 *   priv - A reference to the SD card device state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32p4_enable_ints(struct esp32p4_dev_s *priv)
{
  uint32_t regval;

#ifdef CONFIG_ESP32P4_SDMMC_DMA
  mcinfo("waitmask=%04lx xfrmask=%04lx dmamask=%04lx RINTSTS=%08lx\n",
         (unsigned long)priv->waitmask, (unsigned long)priv->xfrmask,
         (unsigned long)priv->dmamask,
         (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_RINTSTS));
#else
  mcinfo("waitmask=%04lx xfrmask=%04lx RINTSTS=%08lx\n",
         (unsigned long)priv->waitmask, (unsigned long)priv->xfrmask,
         (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_RINTSTS));
#endif

  /* Enable SDMMC interrupts */

  regval = priv->xfrmask | priv->waitmask | SDCARD_INT_CDET;
  esp32p4_putreg(regval, ESP32P4_SDMMC_INTMASK);
}

/****************************************************************************
 * Name: esp32p4_disable_allints
 *
 * Description:
 *   Disable all SD card interrupts.
 *
 * Input Parameters:
 *   priv - A reference to the SD card device state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32p4_disable_allints(struct esp32p4_dev_s *priv)
{
#ifdef CONFIG_ESP32P4_SDMMC_DMA
  /* Disable DMA-related interrupts */

  priv->dmamask = 0;
#endif

  /* Disable all SDMMC interrupts (except card detect) */

  esp32p4_putreg(SDCARD_INT_CDET, ESP32P4_SDMMC_INTMASK);
  priv->waitmask = 0;
  priv->xfrmask  = 0;
}

/****************************************************************************
 * Name: esp32p4_config_waitints
 *
 * Description:
 *   Enable/disable SD card interrupts needed to support the wait function
 *
 * Input Parameters:
 *   priv       - A reference to the SD card device state structure
 *   waitmask   - The set of bits in the SD card INTMASK register to set
 *   waitevents - Waited for events
 *   wkupevent  - Wake-up events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32p4_config_waitints(struct esp32p4_dev_s *priv,
                                    uint32_t waitmask,
                                    sdio_eventset_t waitevents,
                                    sdio_eventset_t wkupevent)
{
  irqstate_t flags;

  mcinfo("waitevents=%04x wkupevent=%04x\n",
         (unsigned)waitevents, (unsigned)wkupevent);

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags            = enter_critical_section();
  priv->waitevents = waitevents;
  priv->wkupevent  = wkupevent;
  priv->waitmask   = waitmask;

  esp32p4_enable_ints(priv);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32p4_config_xfrints
 *
 * Description:
 *   Enable SD card interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the SD card device state structure
 *   xfrmask - The set of bits in the SD card MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32p4_config_xfrints(struct esp32p4_dev_s *priv,
                                   uint32_t xfrmask)
{
  irqstate_t flags;
  flags = enter_critical_section();

  priv->xfrmask = xfrmask;
  esp32p4_enable_ints(priv);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32p4_config_dmaints
 *
 * Description:
 *   Enable DMA transfer interrupts
 *
 * Input Parameters:
 *   priv    - A reference to the SD card device state structure
 *   xfrmask - The set of bits in the SD card MASK register to set
 *   dmamask - The set of bits in the DMA MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32P4_SDMMC_DMA
static void esp32p4_config_dmaints(struct esp32p4_dev_s *priv,
                                   uint32_t xfrmask, uint32_t dmamask)
{
  irqstate_t flags;
  flags = enter_critical_section();

  priv->xfrmask = xfrmask;
  priv->dmamask = dmamask;
  esp32p4_enable_ints(priv);

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: esp32p4_eventtimeout
 *
 * Description:
 *   The watchdog timeout setup when the event wait start has expired
 *   without any other waited-for event occurring.
 *
 * Input Parameters:
 *   arg    - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void esp32p4_eventtimeout(wdparm_t arg)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)arg;

  /* There is always race conditions with timer expirations. */

  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0 ||
              priv->wkupevent != 0);

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. wake up any waiting threads */

      esp32p4_endwait(priv, SDIOWAIT_TIMEOUT);
      syslog(LOG_ERR,
             "SDMMCPROBE: timeout remaining=%d isr_count=%lu isr_last=0x%08lx\n"
             "SDMMCPROBE: CTRL=0x%08lx INTMASK=0x%08lx MINTSTS=0x%08lx "
             "RINTSTS=0x%08lx\n"
             "SDMMCPROBE: STATUS=0x%08lx IDSTS=0x%08lx IDINTEN=0x%08lx "
             "BYTCNT=0x%08lx BLKSIZ=0x%08lx\n"
             "SDMMCPROBE: cpuint=%d clic_en=0x%08lx nohandler=%lu last=%lu\n",
             priv->remaining,
             (unsigned long)g_sdmmc_isr_count,
             (unsigned long)g_sdmmc_isr_last,
             (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_CTRL),
             (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_INTMASK),
             (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_MINTSTS),
             (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_RINTSTS),
             (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_STATUS),
             (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_IDSTS),
             (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_IDINTEN),
             (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_BYTCNT),
             (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_BLKSIZ),
             priv->cpuint,
             (unsigned long)esp_cpu_intr_get_enabled_mask(),
             (unsigned long)g_irq_nohandler_count,
             (unsigned long)g_irq_nohandler_last);
    }
}

/****************************************************************************
 * Name: esp32p4_endwait
 *
 * Description:
 *   Wake up a waiting thread if the waited-for event has occurred.
 *
 * Input Parameters:
 *   priv      - An instance of the SDIO device interface
 *   wkupevent - The event that caused the wait to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void esp32p4_endwait(struct esp32p4_dev_s *priv,
                            sdio_eventset_t wkupevent)
{
  mcinfo("wkupevent=%04x\n", (unsigned)wkupevent);

  /* Cancel the watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Disable event-related interrupts */

  esp32p4_config_waitints(priv, 0, 0, wkupevent);

  /* Wake up the waiting thread */

  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: esp32p4_endtransfer
 *
 * Description:
 *   Terminate a transfer with the provided status.  This function is called
 *   only from the SDIO interrupt handler when end-of-transfer conditions
 *   are detected.
 *
 * Input Parameters:
 *   priv      - An instance of the SDIO device interface
 *   wkupevent - The event that caused the transfer to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void esp32p4_drain_fifo(struct esp32p4_dev_s *priv)
{
  unsigned int before;
  unsigned int now;
  unsigned int next;
  uint32_t word;
  int i;

  if (priv->wrdir || priv->buffer == NULL || priv->remaining == 0)
    {
      return;
    }

  before = SDMMC_STATUS_FIFOCOUNT(esp32p4_getreg(ESP32P4_SDMMC_STATUS));

  for (i = 0; priv->remaining > 0; i++)
    {
      now = SDMMC_STATUS_FIFOCOUNT(esp32p4_getreg(ESP32P4_SDMMC_STATUS));
      if (now == 0)
        {
          break;
        }

      word = getreg32(ESP32P4_SDMMC_DATA);
      next = SDMMC_STATUS_FIFOCOUNT(esp32p4_getreg(ESP32P4_SDMMC_STATUS));

      /* Same-address reads did not pop the P4 FIFO (count stayed at the
       * programmed word count and the first word repeated).  Walk the
       * 0x200-0x3fc window; some DW ports decode any address in that
       * range as a FIFO pop.
       */

      if (next == now)
        {
          word = getreg32(ESP32P4_SDMMC_DATA + ((i * 4) & 0x1fc));
          next = SDMMC_STATUS_FIFOCOUNT(
                   esp32p4_getreg(ESP32P4_SDMMC_STATUS));
        }

      if (i < 6)
        {
          syslog(LOG_INFO,
                 "SDMMCPROBE: fifo[%d] %u->%u word=0x%08lx\n",
                 i, now, next, (unsigned long)word);
        }

      *priv->buffer++ = word;
      priv->remaining -= 4;

      if (next == now)
        {
          syslog(LOG_ERR,
                 "SDMMCPROBE: FIFO read does not pop (count=%u)\n", now);
          break;
        }
    }

  UNUSED(before);
  UNUSED(i);
}

static void esp32p4_endtransfer(struct esp32p4_dev_s *priv,
                                sdio_eventset_t wkupevent)
{
  mcinfo("wkupevent=%04x\n", (unsigned)wkupevent);

  /* Disable all transfer related interrupts */

  esp32p4_config_xfrints(priv, 0);

  /* Clearing pending interrupt status on all transfer related interrupts */

  esp32p4_putreg(priv->waitmask, ESP32P4_SDMMC_RINTSTS);

  /* Mark the transfer finished */

  priv->remaining = 0;

  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      esp32p4_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Name: esp32p4_interrupt
 *
 * Description:
 *   SDIO interrupt handler
 *
 * Input Parameters:
 *   irq     - Interrupt number
 *   context - Saved interrupt context
 *   arg     - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int esp32p4_interrupt(int irq, void *context, void *arg)
{
  struct esp32p4_dev_s *priv = arg;
  uint32_t enabled;
  uint32_t pending;

  g_sdmmc_isr_count++;
  g_sdmmc_isr_last = esp32p4_getreg(ESP32P4_SDMMC_MINTSTS);

  /* Loop while there are pending interrupts.  Check the SD card status
   * register.  Mask out all bits that don't correspond to enabled
   * interrupts.  (This depends on the fact that bits are ordered the same
   * in both the STA and MASK register).  If there are non-zero bits
   * remaining, then we have work to do here.
   */

  while ((enabled = esp32p4_getreg(ESP32P4_SDMMC_MINTSTS)) != 0)
    {
      /* Clear pending status */

      esp32p4_putreg(enabled, ESP32P4_SDMMC_RINTSTS);

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
      /* Handle in card detection events ************************************/

      if ((enabled & SDMMC_INT_CDET) != 0)
        {
          sdio_statset_t cdstatus;

          /* Update card status */

          cdstatus = priv->cdstatus;
          if ((esp32p4_getreg(ESP32P4_SDMMC_CDETECT) &
              SDMMC_CDETECT_NOTPRESENT(priv->slot)) == 0)
            {
              priv->cdstatus |= SDIO_STATUS_PRESENT;

#ifdef CONFIG_MMCSD_HAVE_WRITEPROTECT
              if ((esp32p4_getreg(ESP32P4_SDMMC_WRTPRT) &
                  SDMMC_WRTPRT_PROTECTED(priv->slot)) != 0)
                {
                  priv->cdstatus |= SDIO_STATUS_WRPROTECTED;
                }
              else
#endif
                {
                  priv->cdstatus &= ~SDIO_STATUS_WRPROTECTED;
                }
            }
          else
            {
              priv->cdstatus &=
                ~(SDIO_STATUS_PRESENT | SDIO_STATUS_WRPROTECTED);
            }

          mcinfo("cdstatus OLD: %02x NEW: %02x\n", cdstatus,
                 priv->cdstatus);

          /* Perform any requested callback if the status has changed */

          if (cdstatus != priv->cdstatus)
            {
              esp32p4_callback(priv);
            }
        }
#endif

      /* Handle data transfer events ****************************************/

      pending = enabled & priv->xfrmask;
      if (pending != 0)
        {
          /* Handle data request events */

          if ((pending & SDMMC_INT_TXDR) != 0)
            {
              uint32_t status;

              /* Transfer data to the TX FIFO */

              DEBUGASSERT(priv->wrdir);

              for (status = esp32p4_getreg(ESP32P4_SDMMC_STATUS);
                   (status & SDMMC_STATUS_FIFOFULL) == 0 &&
                   priv->remaining > 0;
                   status = esp32p4_getreg(ESP32P4_SDMMC_STATUS))
                {
                  esp32p4_putreg(*priv->buffer, ESP32P4_SDMMC_DATA);
                  priv->buffer++;
                  priv->remaining -= 4;
                }
            }
          else if ((pending & SDMMC_INT_RXDR) != 0)
            {
              DEBUGASSERT(!priv->wrdir);
              esp32p4_drain_fifo(priv);
            }

          /* Check for transfer errors.  DCRC often arrives with DTO on a
           * short hosted frame: finish the transfer so IDMAC can write back
           * instead of aborting first and flushing the FIFO.
           */

          if ((pending & SDMMC_INT_DCRC) != 0 &&
              (pending & SDMMC_INT_DTO) == 0)
            {
#ifdef CONFIG_ESP32P4_SDMMC_DMA
              volatile struct sdmmc_dma_s *ncdesc;
              int spin;
#endif

              UNUSED(pending);

#ifdef CONFIG_ESP32P4_SDMMC_DMA
              /* Do not abort: IDMAC is often still in DESC_CLOSE with the
               * hosted payload sitting in the FIFO.  Wait for DTO / OWN.
               */

              ncdesc = (volatile struct sdmmc_dma_s *)
                       ESP32P4_NC_ADDR(&priv->dma_desc[0]);
              for (spin = 0; spin < 1000; spin++)
                {
                  uint32_t rint = esp32p4_getreg(ESP32P4_SDMMC_RINTSTS);
                  uint32_t idsts = esp32p4_getreg(ESP32P4_SDMMC_IDSTS);

                  if ((ncdesc->des0 & MCI_DMADES0_OWN) == 0 ||
                      (rint & SDMMC_INT_DTO) != 0 ||
                      ((idsts >> 10) & 0xf) == 0)
                    {
                      break;
                    }

                  esp32p4_putreg(1, ESP32P4_SDMMC_PLDMND);
                }

              UNUSED(spin);

              esp32p4_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
#else
              esp32p4_drain_fifo(priv);
              esp32p4_endtransfer(priv,
                                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
#endif
            }

          /* Handle data timeout error */

          else if ((pending & SDMMC_INT_DRTO) != 0)
            {
              syslog(LOG_ERR,
                     "SDMMCPROBE: DRTO pending=0x%08lx remaining=%d\n",
                     (unsigned long)pending, priv->remaining);

              esp32p4_endtransfer(priv,
                                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT);
            }

          /* Handle RX FIFO overrun error */

          else if ((pending & SDMMC_INT_FRUN) != 0)
            {
              syslog(LOG_ERR,
                     "SDMMCPROBE: FRUN pending=0x%08lx remaining=%d\n",
                     (unsigned long)pending, priv->remaining);

              esp32p4_endtransfer(priv,
                                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle start bit / end bit errors */

          else if ((pending & (SDMMC_INT_SBE | SDMMC_INT_EBE)) != 0)
            {
              syslog(LOG_ERR,
                     "SDMMCPROBE: SBE/EBE pending=0x%08lx remaining=%d\n",
                     (unsigned long)pending, priv->remaining);

              esp32p4_endtransfer(priv,
                                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle data end events.  Note that RXDR may accompany DTO, DTO
           * will be set on received while there is still data in the FIFO.
           * So for the case of receiving, we don't actually even enable the
           * DTO interrupt.
           */

          else if ((pending & SDMMC_INT_DTO) != 0)
            {
#ifdef CONFIG_ESP32P4_SDMMC_DMA
              int spin;
#endif

              /* DTO+DCRC is treated as transfer-complete on this 1-bit
               * hosted path; do not log it as an error on every CMD53.
               */

#ifdef CONFIG_ESP32P4_SDMMC_DMA
              {
                volatile struct sdmmc_dma_s *ncdesc =
                  (volatile struct sdmmc_dma_s *)
                  ESP32P4_NC_ADDR(&priv->dma_desc[0]);
                volatile uint32_t *ncbuf = priv->buffer ?
                  (volatile uint32_t *)ESP32P4_NC_ADDR(priv->buffer) :
                  NULL;

                for (spin = 0; spin < 64; spin++)
                  {
                    if ((ncdesc->des0 & MCI_DMADES0_OWN) == 0)
                      {
                        break;
                      }
                  }

                UNUSED(spin);
                UNUSED(ncbuf);
              }
#endif

              esp32p4_drain_fifo(priv);
              esp32p4_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
            }
        }

      /* Handle wait events *************************************************/

      pending = enabled & priv->waitmask;
      if (pending != 0)
        {
          /* Is this a response error event? */

          if ((pending & SDCARD_INT_RESPERR) != 0)
            {
              /* If response errors are enabled, then we must certainly be
               * waiting for a response.
               */

              DEBUGASSERT((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0);

              /* Wake the thread up */

              mcerr("ERROR: Response error, pending=%08x\n", pending);
              esp32p4_endwait(priv, SDIOWAIT_RESPONSEDONE | SDIOWAIT_ERROR);
            }

          /* Is this a command (plus response) completion event? */

          else if ((pending & SDMMC_INT_CDONE) != 0)
            {
              /* Yes.. Is their a thread waiting for response done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  esp32p4_endwait(priv, SDIOWAIT_RESPONSEDONE);
                }

              /* NO.. Is their a thread waiting for command done? */

              else if ((priv->waitevents & SDIOWAIT_CMDDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  esp32p4_endwait(priv, SDIOWAIT_CMDDONE);
                }
            }
        }
    }

#ifdef CONFIG_ESP32P4_SDMMC_DMA
  /* DMA error events *******************************************************/

  pending = esp32p4_getreg(ESP32P4_SDMMC_IDSTS);
  if ((pending & priv->dmamask) != 0)
    {
      syslog(LOG_ERR, "SDMMCPROBE: IDSTS=0x%08lx dmamask=0x%08lx\n",
             (unsigned long)pending, (unsigned long)priv->dmamask);

      /* Clear the pending interrupts */

      esp32p4_putreg(pending, ESP32P4_SDMMC_IDSTS);

      /* Abort the transfer */

      esp32p4_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: esp32p4_lock
 *
 * Description:
 *   Locks the bus. Function calls low-level multiplexed bus routines to
 *   resolve bus requests and acknowledgment issues.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   lock - TRUE to lock, FALSE to unlock.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_MUXBUS
static int esp32p4_lock(struct sdio_dev_s *dev, bool lock)
{
  /* The multiplex bus is part of board support package. */

  return OK;
}
#endif

/****************************************************************************
 * Name: esp32p4_reset
 *
 * Description:
 *   Reset the SDIO controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32p4_reset(struct sdio_dev_s *dev)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  irqstate_t flags;
  uint32_t regval;

  mcinfo("Resetting...\n");

  flags = enter_critical_section();

  /* Reset all blocks */

  esp32p4_putreg(SDMMC_CTRL_CNTLRRESET | SDMMC_CTRL_FIFORESET |
                 SDMMC_CTRL_DMARESET, ESP32P4_SDMMC_CTRL);

  regval = 1000000;
  while ((esp32p4_getreg(ESP32P4_SDMMC_CTRL) &
         (SDMMC_CTRL_CNTLRRESET | SDMMC_CTRL_FIFORESET |
          SDMMC_CTRL_DMARESET)) != 0)
    {
      if (--regval == 0)
        {
          mcerr("Controller reset timeout (no SDMMC clock?)\n");
          break;
        }
    }

  /* Select clock divider (slot N selects clock divider N).  Unlike the S3,
   * the LS clock source and host divider live in HP_SYS_CLKRST (set up in
   * esp32p4_sdmmc_sdio_initialize), so nothing is written to the SDMMC
   * 0x800 register here.
   */

  regval = esp32p4_getreg(ESP32P4_SDMMC_CLKSRC);
  regval &= ~SDMMC_CLKSRC_MASK(priv->slot);
  regval |= SDMMC_CLKSRC_CLKDIV(priv->slot, priv->slot);
  esp32p4_putreg(regval, ESP32P4_SDMMC_CLKSRC);

#ifdef CONFIG_ESP32P4_SDMMC_DMA
  /* Program the DMA descriptor list base address.
   *
   * NOTE: This uses the CPU address of the descriptor array.  On the P4 the
   * internal SRAM is DMA-capable and identity-mapped, but confirm on
   * hardware that no address translation is required for the IDMAC.
   */

  esp32p4_putreg((uint32_t)(uintptr_t)&priv->dma_desc[0],
                 ESP32P4_SDMMC_DBADDR);
#endif

  /* Reset data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitmask   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */

  wd_cancel(&priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;      /* Address of current R/W buffer */
  priv->remaining  = 0;      /* Number of bytes remaining in the transfer */
  priv->xfrmask    = 0;      /* Interrupt enables for data transfer */
#ifdef CONFIG_ESP32P4_SDMMC_DMA
  priv->dmamask    = 0;      /* Interrupt enables for DMA transfer */
#endif

  /* DMA data transfer support */

  priv->cdstatus   = 0;      /* Card status is unknown */

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32p4_capabilities
 *
 * Description:
 *   Get capabilities (and limitations) of the SDIO driver (optional)
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see SDIO_CAPS_* defines)
 *
 ****************************************************************************/

static sdio_capset_t esp32p4_capabilities(struct sdio_dev_s *dev)
{
  sdio_capset_t caps = 0;

  caps |= SDIO_CAPS_DMABEFOREWRITE;
  caps |= SDIO_CAPS_MMC_HS_MODE;

#ifdef CONFIG_SDIO_WIDTH_D1_ONLY
  caps |= SDIO_CAPS_1BIT_ONLY;
#endif
#ifdef CONFIG_ESP32P4_SDMMC_DMA
  caps |= SDIO_CAPS_DMASUPPORTED;
#endif

  return caps;
}

/****************************************************************************
 * Name: esp32p4_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see SDIO_STATUS_* defines)
 *
 ****************************************************************************/

static sdio_statset_t esp32p4_status(struct sdio_dev_s *dev)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
  if ((esp32p4_getreg(ESP32P4_SDMMC_CDETECT) &
       SDMMC_CDETECT_NOTPRESENT(priv->slot)) == 0)
    {
      priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }
#endif

  mcinfo("cdstatus=%02x\n", priv->cdstatus);

  return priv->cdstatus;
}

/****************************************************************************
 * Name: esp32p4_widebus
 *
 * Description:
 *   Called after change in Bus width has been selected (via ACMD6).  Most
 *   controllers will need to perform some special operations to work
 *   correctly in the new bus mode.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   wide - true: wide bus (4-bit) bus mode enabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32p4_widebus(struct sdio_dev_s *dev, bool wide)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  uint32_t regval;

  regval = esp32p4_getreg(ESP32P4_SDMMC_CTYPE);
  regval &= ~(SDMMC_CTYPE_WIDTH4_MASK(priv->slot));
  regval &= ~(SDMMC_CTYPE_WIDTH8_MASK(priv->slot));

#ifndef CONFIG_SDIO_WIDTH_D1_ONLY
  if (wide)
    {
      regval |= SDMMC_CTYPE_WIDTH4_MASK(priv->slot);

      configure_pin(CONFIG_ESP32P4_SDMMC_D1, priv->sdio_pins->d1,
                    INPUT | OUTPUT | PULLUP);
      configure_pin(CONFIG_ESP32P4_SDMMC_D2, priv->sdio_pins->d2,
                    INPUT | OUTPUT | PULLUP);
      configure_pin(CONFIG_ESP32P4_SDMMC_D3, priv->sdio_pins->d3,
                    INPUT | OUTPUT | PULLUP);
    }
#endif

  esp32p4_putreg(regval, ESP32P4_SDMMC_CTYPE);
}

/****************************************************************************
 * Name: sdmmc_host_clock_update_command
 *
 * Description:
 *   Issue the "update clock registers only" command to the CIU so that the
 *   new CLKDIV/CLKSRC/CLKENA values take effect.
 *
 ****************************************************************************/

static int sdmmc_host_clock_update_command(struct esp32p4_dev_s *priv)
{
  /* Clock update command; not a real command, just updates CIU registers */

  uint32_t cmd = SDMMC_CMD_UPDCLOCK | SDMMC_CMD_WAITPREV |
                 SDMMC_CMD_CARD_NUMBER(priv->slot);
  uint32_t regval;
  bool repeat = true;
  int timeout_ms = 100;
  int ret;

  while (repeat)
    {
      ret = esp32p4_ciu_sendcmd(cmd, 0);
      if (ret)
        {
          return ret;
        }

      while (timeout_ms)
        {
          regval = esp32p4_getreg(ESP32P4_SDMMC_RINTSTS);
          if (regval & SDMMC_INT_HLE)
            {
              esp32p4_putreg(SDMMC_INT_HLE, ESP32P4_SDMMC_RINTSTS);
              break;
            }

          if ((esp32p4_getreg(ESP32P4_SDMMC_CMD) & SDMMC_CMD_STARTCMD)
              == 0)
            {
              repeat = false;
              break;
            }

          timeout_ms--;
          up_mdelay(1);
        }
    }

  return timeout_ms > 0 ? OK : -ETIMEDOUT;
}

/****************************************************************************
 * Name: sdmmc_host_get_clk_dividers
 *
 * Description:
 *   Compute the host divider (feeding the LS clock in HP_SYS_CLKRST) and the
 *   card divider (SDMMC CLKDIV) for a requested card clock frequency.  The
 *   source clock is PLL160M as selected in the init function.
 *
 ****************************************************************************/

static void sdmmc_host_get_clk_dividers(uint32_t freq_khz, int *host_div,
                                        int *card_div)
{
  uint32_t clk_src_freq_hz = ESP32P4_SDMMC_SRC_FREQ_HZ;

  /* Calculate new dividers */

  if (freq_khz >= 40 * 1000)
    {
      *host_div = 4;       /* 160 MHz / 4 = 40 MHz */
      *card_div = 0;
    }
  else if (freq_khz == 20 * 1000)
    {
      *host_div = 8;       /* 160 MHz / 8 = 20 MHz */
      *card_div = 0;
    }
  else if (freq_khz == 400)
    {
      *host_div = 10;      /* 160 MHz / 10 / (20 * 2) = 400 kHz */
      *card_div = 20;
    }
  else
    {
      /* For custom frequencies use maximum range of host divider (1-16),
       * find the closest <= div. combination.  If exceeded, combine with the
       * card divider to keep reasonable precision.
       */

      *host_div = (clk_src_freq_hz) / (freq_khz * 1000);
      if (*host_div > 15)
        {
          *host_div = 2;
          *card_div = (clk_src_freq_hz / 2) / (2 * freq_khz * 1000);
          if (((clk_src_freq_hz / 2) % (2 * freq_khz * 1000)) > 0)
            {
              (*card_div)++;
            }
        }
      else if ((clk_src_freq_hz % (freq_khz * 1000)) > 0)
        {
          (*host_div)++;
        }
    }
}

/****************************************************************************
 * Name: sdmmc_host_set_clk_div
 *
 * Description:
 *   Program the host divider and card divider.
 *
 *   KEY S3->P4 DIFFERENCE: on the S3 the host divider (h/l/n edge factors)
 *   was written to the SDMMC 0x800 CLOCK register.  On the P4 that divider
 *   moved into HP_SYS_CLKRST.PERI_CLK_CTRL02 (reg_sdio_ls_clk_edge_*), so we
 *   write it there and toggle the edge-config-update bit.  The card divider
 *   still lives in the SDMMC CLKDIV register as before.
 *
 ****************************************************************************/

static void sdmmc_host_set_clk_div(uint32_t slot, uint32_t host_div,
                                   uint32_t card_div)
{
  irqstate_t flags = enter_critical_section();

  /* Duty cycle 1/2: negedge at l, posedge at h, counter resets at n. */

  ASSERT(host_div > 1 && host_div <= 16);
  int l = host_div - 1;
  int h = host_div / 2 - 1;
  int n = host_div - 1;
  uint32_t regval;
  uint32_t divider;

  /* Set card divider (SDMMC CLKDIV, indexed by the divider selected in
   * esp32p4_reset()).
   */

  regval = esp32p4_getreg(ESP32P4_SDMMC_CLKSRC);
  divider = (regval & SDMMC_CLKSRC_MASK(slot)) >> SDMMC_CLKSRC_SHIFT(slot);

  regval = esp32p4_getreg(ESP32P4_SDMMC_CLKDIV);
  regval &= ~SDMMC_CLKDIV_MASK(divider);
  regval |= SDMMC_CLKDIV(divider, card_div);
  esp32p4_putreg(regval, ESP32P4_SDMMC_CLKDIV);

  /* Set the host divider and the drive/sample/self phase clocks in
   * HP_SYS_CLKRST.PERI_CLK_CTRL02.  ESP-IDF's sdmmc_ll_init_phase_delay()
   * does this on every clock change: drv/sam/slf clocks enabled, drive
   * edge = 1, sample/self edge = 0.  Leaving those clocks off lets CMD52
   * (CMD line) work while the DAT sampling path never starts, which is
   * exactly the CMD53 "R5 clean, FIFO empty, no DTO" failure.
   */

  regval = esp32p4_getreg(ESP32P4_HP_SYS_CLKRST_PERI_CLK_CTRL02);
  regval &= ~(HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_L_M |
              HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_H_M |
              HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_N_M |
              HP_SYS_CLKRST_SDIO_LS_SLF_EDGE_SEL_M |
              HP_SYS_CLKRST_SDIO_LS_DRV_EDGE_SEL_M |
              HP_SYS_CLKRST_SDIO_LS_SAM_EDGE_SEL_M);
  regval |= (l << HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_L_S) &
            HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_L_M;
  regval |= (h << HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_H_S) &
            HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_H_M;
  regval |= (n << HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_N_S) &
            HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_N_M;
  /* drv=1, sam=0, slf=0 -- ESP-IDF sdmmc_ll_init_phase_delay() defaults.
   * The sample edge can be overridden later by
   * esp32p4_sdmmc_set_sample_phase().
   */

  regval |= (1u << HP_SYS_CLKRST_SDIO_LS_DRV_EDGE_SEL_S);
  regval |= HP_SYS_CLKRST_SDIO_LS_DRV_CLK_EN |
            HP_SYS_CLKRST_SDIO_LS_SAM_CLK_EN |
            HP_SYS_CLKRST_SDIO_LS_SLF_CLK_EN;
  esp32p4_putreg(regval, ESP32P4_HP_SYS_CLKRST_PERI_CLK_CTRL02);

  /* Latch the new edge configuration */

  regval |= HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_UPD;
  esp32p4_putreg(regval, ESP32P4_HP_SYS_CLKRST_PERI_CLK_CTRL02);
  regval &= ~HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_UPD;
  esp32p4_putreg(regval, ESP32P4_HP_SYS_CLKRST_PERI_CLK_CTRL02);

  leave_critical_section(flags);

  /* Wait for the clock to propagate */

  up_udelay(10);
}

/****************************************************************************
 * Name: esp32p4_clock
 *
 * Description:
 *   Enable/disable SDIO clocking
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   rate - Specifies the clocking to use (see enum sdio_clock_e)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32p4_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  uint32_t freq_khz;
  uint32_t regval;
  bool clk_en = true;
  int host_div = 0;   /* clock divider of the host (HP_SYS_CLKRST) */
  int card_div = 0;   /* 1/2 of card clock divider (SDMMC.clkdiv) */
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;

  switch (rate)
    {
      /* Disable clocking (with default ID mode divisor) */

      default:
      case CLOCK_SDIO_DISABLED:
        freq_khz = 400;
        clk_en = false;
        break;

      /* Enable in initial ID mode clocking (<400KHz) */

      case CLOCK_IDMODE:
        freq_khz = 400;
        break;

      /* Enable in MMC normal operation clocking */

      case CLOCK_MMC_TRANSFER:
        if (esp32p4_capabilities(dev) & SDIO_CAPS_MMC_HS_MODE)
          {
            freq_khz = 40 * 1000;
          }
        else
          {
            freq_khz = 20 * 1000;
          }
        break;

      /* SD normal operation clocking (wide 4-bit mode) */

      case CLOCK_SD_TRANSFER_4BIT:
#ifndef CONFIG_SDIO_WIDTH_D1_ONLY
        freq_khz = 20 * 1000;
        esp32p4_widebus(dev, true);
        break;
#endif

      /* SD normal operation clocking (narrow 1-bit mode) */

      case CLOCK_SD_TRANSFER_1BIT:
        freq_khz = 20 * 1000;
        esp32p4_widebus(dev, false);
        break;
    }

  /* Disable clock first */

  regval = esp32p4_getreg(ESP32P4_SDMMC_CLKENA);
  regval &= ~(SDMMC_CLKENA_ENABLE(priv->slot)
              | SDMMC_CLKENA_LOWPOWER(priv->slot));
  esp32p4_putreg(regval, ESP32P4_SDMMC_CLKENA);
  if (sdmmc_host_clock_update_command(priv) != OK)
    {
      mcerr("disabling clk failed\n");
      return;
    }

  /* Program card clock settings, send them to the CIU */

  sdmmc_host_get_clk_dividers(freq_khz, &host_div, &card_div);
  sdmmc_host_set_clk_div(priv->slot, host_div, card_div);
  if (sdmmc_host_clock_update_command(priv) != OK)
    {
      mcerr("setting clk div failed\n");
      return;
    }

  /* Re-enable clocks */

  if (clk_en)
    {
      regval = esp32p4_getreg(ESP32P4_SDMMC_CLKENA);
      /* Do NOT set CLKENA.LOWPOWER: it gates the card clock whenever the bus
       * goes idle, and an SDIO slave (the on-board ESP32-C6) needs a running
       * clock to keep its state machine and interrupt path alive.  With the
       * low-power bit set, CMD52 starts timing out ~0.5 s after function 1 is
       * enabled and never recovers.
       */

      regval |= SDMMC_CLKENA_ENABLE(priv->slot);
      esp32p4_putreg(regval, ESP32P4_SDMMC_CLKENA);
      if (sdmmc_host_clock_update_command(priv) != OK)
        {
          mcerr("re-enabling clk failed\n");
          return;
        }
    }

  /* Set data timeout to 100ms */

  if (freq_khz * 100 > (SDMMC_TMOUT_DATA_MASK >> SDMMC_TMOUT_DATA_SHIFT))
    {
      regval = SDMMC_TMOUT_DATA_MASK;
    }
  else
    {
      regval = freq_khz * 100 << SDMMC_TMOUT_DATA_SHIFT;
    }

  /* Always set response timeout to highest value; it's small enough */

  regval |= SDMMC_TMOUT_RESPONSE_MASK;
  esp32p4_putreg(regval, ESP32P4_SDMMC_TMOUT);
}

/****************************************************************************
 * Name: esp32p4_attach
 *
 * Description:
 *   Attach and prepare interrupts
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK on success; A negated errno on failure.
 *
 ****************************************************************************/

static int esp32p4_attach(struct sdio_dev_s *dev)
{
  uint32_t regval;
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;

  /* Stage-1 (SDIO link bring-up) is fully polled: esp32p4_waitresponse()
   * spins on RINTSTS.CMDDONE, so CMD5/CMD3/CMD7 enumeration and the CMD52
   * CCCR/CIS reads need no SDMMC interrupt at all.  Keep the controller
   * silent -- mask every source (main + IDMAC), drop the global interrupt
   * enable and clear all pending status -- and leave the CPU interrupt
   * un-attached.
   *
   * Wiring the SDMMC source through the ESP-IDF interrupt allocator
   * (esp_setup_irq -> esp_intr_alloc) currently hangs on the P4 the instant
   * the allocator's critical section is released (nuttx_exit_critical ->
   * up_irq_restore) as the SDMMC CPU interrupt is connected.  That only
   * gates the DMA data path (Stage 2+), so it must not block the link probe.
   */

  esp32p4_putreg(0, ESP32P4_SDMMC_INTMASK);
  esp32p4_putreg(0, ESP32P4_SDMMC_IDINTEN);
  regval  = esp32p4_getreg(ESP32P4_SDMMC_CTRL);
  regval &= ~(SDMMC_CTRL_INTENABLE | SDMMC_CTRL_INTDMA);
  esp32p4_putreg(regval, ESP32P4_SDMMC_CTRL);
  esp32p4_putreg(0xffffffff, ESP32P4_SDMMC_RINTSTS);
  esp32p4_putreg(0xffffffff, ESP32P4_SDMMC_IDSTS);

  /* Allocate the CPU interrupt for the SDIO host source and attach the
   * handler.  On the P4 esp_setup_irq() both allocates the CPU interrupt and
   * attaches the handler (see esp_i2c.c / esp_spi.c).
   */

  priv->cpuint = esp_setup_irq(ETS_SDIO_HOST_INTR_SOURCE,
                               ESP_IRQ_PRIORITY_DEFAULT,
                               ESP_IRQ_TRIGGER_LEVEL,
                               esp32p4_interrupt, priv);
  if (priv->cpuint < 0)
    {
      mcerr("ERROR: esp_setup_irq failed: %d\n", priv->cpuint);
      return priv->cpuint;
    }

  up_enable_irq(ESP_SOURCE2IRQ(ETS_SDIO_HOST_INTR_SOURCE));

  /* Re-enable the controller's global interrupt output.  It was cleared
   * above along with INTDMA so that a stale condition could not storm while
   * the handler was being registered, and nothing else in the driver ever
   * sets it again.  Without it the SDMMC never drives its interrupt line:
   * every data transfer ends in a wait timeout with MINTSTS non-zero and the
   * ISR entered zero times.
   */

  regval  = esp32p4_getreg(ESP32P4_SDMMC_CTRL);
  regval |= SDMMC_CTRL_INTENABLE;
  esp32p4_putreg(regval, ESP32P4_SDMMC_CTRL);

  return OK;
}

/****************************************************************************
 * Name: esp32p4_sendcmd
 *
 * Description:
 *   Send the SDIO command
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *   cmd - The command to send (32-bits, encoded)
 *   arg - 32-bit argument required with some commands
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int esp32p4_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t arg)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  uint32_t regval = 0;

  mcinfo("cmd=%04x arg=%04x\n", cmd, arg);

  /* Clear any stale command-done / response-error status left over from the
   * previous command so esp32p4_waitresponse() polls only this command's
   * result.  This matters because R3/R4 responses carry no CRC or command
   * index and therefore always leave RE (and RCRC) asserted, which would
   * otherwise be mis-attributed to the next command.
   */

  esp32p4_putreg(SDCARD_RESPDONE_CLEAR, ESP32P4_SDMMC_RINTSTS);

  if (cmd == MMCSD_CMD12)
    {
      regval |= SDMMC_CMD_STOPABORT;
    }
  else if (cmd == MMCSD_CMD0)
    {
      /* The CMD0 needs the SENDINIT CMD */

      regval |= SDMMC_CMD_SENDINIT;
    }
  else
    {
      regval |= SDMMC_CMD_WAITPREV;
    }

  /* Is this a Read/Write Transfer Command ? */

  if ((cmd & MMCSD_WRDATAXFR) == MMCSD_WRDATAXFR)
    {
      regval |= SDMMC_CMD_DATAXFREXPTD | SDMMC_CMD_WRITE;
    }
  else if ((cmd & MMCSD_RDDATAXFR) == MMCSD_RDDATAXFR)
    {
      regval |= SDMMC_CMD_DATAXFREXPTD;
    }

  /* Set WAITRESP bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      regval |= SDMMC_CMD_NORESPONSE;
      break;

    case MMCSD_R1B_RESPONSE:
      regval |= SDMMC_CMD_RESPCRC;
      regval |= SDMMC_CMD_WAITPREV;
      regval |= SDMMC_CMD_SHORTRESPONSE;
      break;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R4_RESPONSE:
      regval |= SDMMC_CMD_SHORTRESPONSE;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
    case MMCSD_R7_RESPONSE:
      regval |= SDMMC_CMD_RESPCRC;
      regval |= SDMMC_CMD_SHORTRESPONSE;
      break;

    case MMCSD_R2_RESPONSE:
      regval |= SDMMC_CMD_LONGRESPONSE;
      regval |= SDMMC_CMD_RESPCRC;
      break;
    }

  /* Set the command index */

  regval |= (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval |= SDMMC_CMD_CARD_NUMBER(priv->slot);

  /* Write the SD card CMD */

  esp32p4_ciu_sendcmd(regval, arg);

  return OK;
}

/****************************************************************************
 * Name: esp32p4_blocksetup
 *
 * Description:
 *   Configure block size and the number of blocks for next transfer
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   blocklen - The selected block size.
 *   nblocks  - The number of blocks to transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_BLOCKSETUP
static void esp32p4_blocksetup(struct sdio_dev_s *dev, unsigned int blocklen,
                               unsigned int nblocks)
{
  /* Configure block size for next transfer */

  esp32p4_putreg(blocklen, ESP32P4_SDMMC_BLKSIZ);
  esp32p4_putreg(blocklen * nblocks, ESP32P4_SDMMC_BYTCNT);
}
#endif

/****************************************************************************
 * Name: esp32p4_recvsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card in
 *   non-DMA (interrupt driven mode).  This method will do whatever
 *   controller setup is necessary.  This would be called for SD memory just
 *   BEFORE sending CMD13 (SEND_STATUS), CMD17 (READ_SINGLE_BLOCK), CMD18
 *   (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR), etc.  Normally,
 *   SDIO_WAITEVENT will be called to receive the transfer-complete
 *   indication.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer in which to receive the data
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int esp32p4_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                             size_t nbytes)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
#ifdef CONFIG_ESP32P4_SDMMC_DMA
  uint32_t regval;
#endif

  mcinfo("nbytes=%ld\n", (long)nbytes);

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)(uintptr_t)buffer & 3) == 0);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
  priv->wrdir     = false;

  /* Configure the FIFO so that we will receive the RXDR interrupt whenever
   * there are more than 1 words (at least 8 bytes) in the RX FIFO.
   */

  esp32p4_putreg(SDMMC_FIFOTH_RXWMARK(1), ESP32P4_SDMMC_FIFOTH);

#ifdef CONFIG_ESP32P4_SDMMC_DMA
  /* Make sure that internal DMA is disabled */

  esp32p4_putreg(0, ESP32P4_SDMMC_BMOD);

  regval  = esp32p4_getreg(ESP32P4_SDMMC_CTRL);
  regval &= ~(SDMMC_CTRL_INTDMA | SDMMC_CTRL_DMAENABLE);
  esp32p4_putreg(regval, ESP32P4_SDMMC_CTRL);
#endif

  /* Flush ints before we start */

  esp32p4_putreg(SDCARD_TRANSFER_ALL, ESP32P4_SDMMC_RINTSTS);

  /* Configure the transfer interrupts */

  esp32p4_config_xfrints(priv, SDCARD_RECV_MASK);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_sendsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card.  This
 *   method will do whatever controller setup is necessary.  This would be
 *   called for SD memory just AFTER sending CMD24 (WRITE_BLOCK), CMD25
 *   (WRITE_MULTIPLE_BLOCK), ... and before SDIO_SENDDATA is called.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer containing the data to send
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int esp32p4_sendsetup(struct sdio_dev_s *dev, const uint8_t *buffer,
                             size_t nbytes)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
#ifdef CONFIG_ESP32P4_SDMMC_DMA
  uint32_t regval;
#endif

  mcinfo("nbytes=%ld\n", (long)nbytes);

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)(uintptr_t)buffer & 3) == 0);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
  priv->wrdir     = true;

  /* Configure the FIFO so that we will receive the TXDR interrupt whenever
   * the TX FIFO is at least half empty.
   */

  esp32p4_putreg(SDMMC_FIFOTH_TXWMARK(ESP32P4_TXFIFO_DEPTH / 2),
                 ESP32P4_SDMMC_FIFOTH);

#ifdef CONFIG_ESP32P4_SDMMC_DMA
  /* Make sure that internal DMA is disabled */

  esp32p4_putreg(0, ESP32P4_SDMMC_BMOD);

  regval  = esp32p4_getreg(ESP32P4_SDMMC_CTRL);
  regval &= ~(SDMMC_CTRL_INTDMA | SDMMC_CTRL_DMAENABLE);
  esp32p4_putreg(regval, ESP32P4_SDMMC_CTRL);
#endif

  /* Flush ints before we start */

  esp32p4_putreg(SDCARD_TRANSFER_ALL, ESP32P4_SDMMC_RINTSTS);

  /* Configure the transfer interrupts */

  esp32p4_config_xfrints(priv, SDCARD_SEND_MASK);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_cancel
 *
 * Description:
 *   Cancel the data transfer setup of SDIO_RECVSETUP, SDIO_SENDSETUP,
 *   SDIO_DMARECVSETUP or SDIO_DMASENDSETUP.  This must be called to cancel
 *   the data transfer setup if, for some reason, you cannot perform the
 *   transfer.
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int esp32p4_cancel(struct sdio_dev_s *dev)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;

  mcinfo("Cancelling..\n");

  /* Disable all transfer- and event- related interrupts */

  esp32p4_disable_allints(priv);

  /* Clearing pending interrupt status on all transfer- and event- related
   * interrupts
   */

  esp32p4_putreg(SDCARD_WAITALL_CLEAR, ESP32P4_SDMMC_RINTSTS);

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Mark no transfer in progress */

  priv->remaining = 0;
  return OK;
}

/****************************************************************************
 * Name: esp32p4_waitresponse
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *   cmd - The command that was sent.  See 32-bit command definitions above.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int esp32p4_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  int ret = OK;
  volatile int32_t timeout;
  clock_t watchtime;
  uint32_t respmask;

  mcinfo("cmd=%04x\n", cmd);

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      timeout = SDCARD_CMDTIMEOUT;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
      timeout = SDCARD_LONGTIMEOUT;
      break;

    default:
      return -EINVAL;
    }

  watchtime = clock_systime_ticks();

  /* We should wait for CMDDONE, even if there is a response error. */

  while ((esp32p4_getreg(ESP32P4_SDMMC_RINTSTS) & SDMMC_INT_CDONE) == 0)
    {
      if (clock_systime_ticks() - watchtime > timeout)
        {
          mcerr("ERROR: Timeout cmd: %04x STA: %08x RINTSTS: %08x\n",
                cmd, esp32p4_getreg(ESP32P4_SDMMC_STATUS),
                esp32p4_getreg(ESP32P4_SDMMC_RINTSTS));

          ret = -ETIMEDOUT;
        }
    }

  /* Check if there is a response error.  R3 and R4 responses carry neither a
   * CRC (the CRC field is all ones) nor the command index (it is reserved and
   * transmitted as all ones), so the DesignWare controller always raises
   * RE/RCRC for them -- only a response timeout (RTO) is a genuine error in
   * that case.  For every other response type keep the full check.
   */

  respmask = SDCARD_INT_RESPERR;
  if ((cmd & MMCSD_RESPONSE_MASK) == MMCSD_R3_RESPONSE ||
      (cmd & MMCSD_RESPONSE_MASK) == MMCSD_R4_RESPONSE)
    {
      respmask = SDMMC_INT_RTO;
    }

  if (esp32p4_getreg(ESP32P4_SDMMC_RINTSTS) & respmask)
    {
      mcerr("ERROR: SDMMC failure cmd: %04x STA: %08x RINTSTS: %08x\n",
            cmd, esp32p4_getreg(ESP32P4_SDMMC_STATUS),
            esp32p4_getreg(ESP32P4_SDMMC_RINTSTS));
      ret = -EIO;
    }

  esp32p4_putreg(SDCARD_CMDDONE_CLEAR, ESP32P4_SDMMC_RINTSTS);
  return ret;
}

/****************************************************************************
 * Name: esp32p4_recvshortcrc
 *
 * Description:
 *   Receive response to SDIO command.  Only the critical payload is
 *   returned -- 32 bits for 48 bit status.  The driver implementation
 *   verifies the correctness of the remaining, non-returned bits (CRCs, CMD
 *   index, etc.).
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   cmd    - The command that was sent
 *   rshort - Buffer in which to receive the response
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 ****************************************************************************/

static int esp32p4_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                                uint32_t *rshort)
{
  uint32_t regval;
  int ret = OK;

  mcinfo("cmd=%04x\n", cmd);

#ifdef CONFIG_DEBUG_FEATURES
  if (!rshort)
    {
      mcerr("ERROR: rshort=NULL\n");
      ret = -EINVAL;
    }

  /* Check that this is the correct response to this command */

  else if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1B_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R5_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R6_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%04x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = esp32p4_getreg(ESP32P4_SDMMC_RINTSTS);
      if ((regval & SDMMC_INT_RTO) != 0)
        {
          mcerr("ERROR: Command timeout: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & SDMMC_INT_RCRC) != 0)
        {
          mcerr("ERROR: CRC failure: %08x\n", regval);
          ret = -EIO;
        }
    }

  /* Clear all pending message completion events and return the R1/R6
   * response.
   */

  esp32p4_putreg(SDCARD_RESPDONE_CLEAR | SDCARD_CMDDONE_CLEAR,
                 ESP32P4_SDMMC_RINTSTS);
  *rshort = esp32p4_getreg(ESP32P4_SDMMC_RESP0);
  mcinfo("CRC=%04x\n", *rshort);

  return ret;
}

/****************************************************************************
 * Name: esp32p4_recvlong
 *
 * Description:
 *   Receive response to SDIO command.  Only the critical payload is
 *   returned -- 128 bits for 136 bit status.  The driver implementation
 *   verifies the correctness of the remaining, non-returned bits (CRCs, CMD
 *   index, etc.).
 *
 * Input Parameters:
 *   dev   - An instance of the SDIO device interface
 *   cmd   - The command that was sent
 *   rlong - Buffer in which to receive the response
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 ****************************************************************************/

static int esp32p4_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                            uint32_t rlong[4])
{
  uint32_t regval;
  int ret = OK;

  mcinfo("cmd=%04x\n", cmd);

#ifdef CONFIG_DEBUG_FEATURES
  /* Check that R1 is the correct response to this command */

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R2_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%04x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = esp32p4_getreg(ESP32P4_SDMMC_RINTSTS);
      if (regval & SDMMC_INT_RTO)
        {
          mcerr("ERROR: Timeout STA: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & SDMMC_INT_RCRC)
        {
          mcerr("ERROR: CRC fail STA: %08x\n", regval);
          ret = -EIO;
        }
    }

  /* Return the long response */

  esp32p4_putreg(SDCARD_RESPDONE_CLEAR | SDCARD_CMDDONE_CLEAR,
                 ESP32P4_SDMMC_RINTSTS);
  if (rlong)
    {
      rlong[0] = esp32p4_getreg(ESP32P4_SDMMC_RESP3);
      rlong[1] = esp32p4_getreg(ESP32P4_SDMMC_RESP2);
      rlong[2] = esp32p4_getreg(ESP32P4_SDMMC_RESP1);
      rlong[3] = esp32p4_getreg(ESP32P4_SDMMC_RESP0);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32p4_recvshort
 *
 * Description:
 *   Receive response to SDIO command.  Only the critical payload is
 *   returned -- 32 bits for 48 bit status.  The driver implementation
 *   verifies the correctness of the remaining, non-returned bits (CMD
 *   index, etc., not including CRC).
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   cmd    - The command that was sent
 *   rshort - Buffer in which to receive the response
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 ****************************************************************************/

static int esp32p4_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t *rshort)
{
  uint32_t regval;
  int ret = OK;

  mcinfo("cmd=%04x\n", cmd);

  /* Check that this is the correct response to this command */

#ifdef CONFIG_DEBUG_FEATURES
  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R4_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%04x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout occurred (Apparently a CRC error can terminate
       * a good response)
       */

      regval = esp32p4_getreg(ESP32P4_SDMMC_RINTSTS);
      if (regval & SDMMC_INT_RTO)
        {
          mcerr("ERROR: Timeout STA: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
    }

  esp32p4_putreg(SDCARD_RESPDONE_CLEAR | SDCARD_CMDDONE_CLEAR,
                 ESP32P4_SDMMC_RINTSTS);
  if (rshort)
    {
      *rshort = esp32p4_getreg(ESP32P4_SDMMC_RESP0);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32p4_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   SDIO_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling esp32p4_eventwait.  This is done in this way
 *   to help the driver to eliminate race conditions between the command
 *   setup and the subsequent events.
 *
 *   The enabled events persist until either (1) SDIO_WAITENABLE is called
 *   again specifying a different set of wait events, or (2) SDIO_EVENTWAIT
 *   returns.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOWAIT_*
 *              definitions). 0=disable; 1=enable.
 *   timeout  - Maximum time in milliseconds to wait
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32p4_waitenable(struct sdio_dev_s *dev,
                               sdio_eventset_t eventset, uint32_t timeout)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  uint32_t waitmask;

  mcinfo("eventset=%04x\n", (unsigned int)eventset);
  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  esp32p4_config_waitints(priv, 0, 0, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

  waitmask = 0;
  if ((eventset & SDIOWAIT_CMDDONE) != 0)
    {
      waitmask |= SDCARD_CMDDONE_MASK;
    }

  if ((eventset & SDIOWAIT_RESPONSEDONE) != 0)
    {
      waitmask |= SDCARD_RESPDONE_MASK;
    }

  if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
    {
      waitmask |= SDCARD_XFRDONE_MASK;
    }

  /* Enable event-related interrupts */

  esp32p4_config_waitints(priv, waitmask, eventset, 0);

  /* Check if the timeout event is specified in the event set */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      int delay;
      int ret;

      /* Handle a cornercase: The user requested a timeout event but with
       * timeout == 0?
       */

      if (!timeout)
        {
          priv->wkupevent = SDIOWAIT_TIMEOUT;
          return;
        }

      /* Start the watchdog timer */

      delay = MSEC2TICK(timeout);
      ret   = wd_start(&priv->waitwdog, delay,
                       esp32p4_eventtimeout, (wdparm_t)priv);
      if (ret < 0)
        {
          mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: esp32p4_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDIO_WAITEVENTS are disabled when
 *   esp32p4_eventwait returns.  SDIO_WAITEVENTS must be called again before
 *   esp32p4_eventwait can be used again.
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   Event set containing the event(s) that ended the wait.  Should always
 *   be non-zero.  All events are disabled after the wait concludes.
 *
 ****************************************************************************/

static sdio_eventset_t esp32p4_eventwait(struct sdio_dev_s *dev)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  sdio_eventset_t wkupevent = 0;
  irqstate_t flags;
  clock_t start;
  int ret;

  /* There is a race condition here... the event may have completed before
   * we get here.  In this case waitevents will be zero, but wkupevents will
   * be non-zero (and, hopefully, the semaphore count will also be non-zero).
   */

  flags = enter_critical_section();
  DEBUGASSERT(priv->waitevents != 0 || priv->wkupevent != 0);

  /* Polled completion.
   *
   * The SDMMC CPU interrupt cannot be relied on here.  esp_setup_irq() ->
   * esp_intr_alloc() may share one CPU interrupt line between several
   * peripheral sources, while riscv_dispatch_irq() resolves a cpuint to a
   * single irq by scanning the handle map and taking the first match -- so a
   * shared line delivers only one source's handler, the other source's
   * condition is never cleared, and the line storms.  The defence added in
   * riscv_dispatch_irq() (masking a handler-less cpuint) stops the storm but
   * leaves the SDMMC line masked for good: observed as MINTSTS non-zero with
   * zero ISR entries and every data transfer ending in a wait timeout.
   *
   * Stage-1 already runs the whole command path polled, so do the same for
   * the data path: drive the exact same handler from this context until it
   * posts a wake-up event.  The critical section is dropped around each pass
   * so the timeout watchdog can still run.  A tick deadline is the backstop
   * if the watchdog never fires from this context.
   */

  start = clock_systime_ticks();

  for (; ; )
    {
      wkupevent = priv->wkupevent;
      if (wkupevent != 0)
        {
          break;
        }

      if ((clock_systime_ticks() - start) > MSEC2TICK(1500))
        {
          wkupevent = SDIOWAIT_TIMEOUT;
          priv->wkupevent = wkupevent;
          wd_cancel(&priv->waitwdog);
          break;
        }

      leave_critical_section(flags);
      esp32p4_interrupt(0, NULL, priv);
      flags = enter_critical_section();
    }

  UNUSED(ret);

  /* The IDMAC wrote straight into memory, so drop any cache lines the CPU
   * may still hold for that buffer.  The invalidate done in
   * esp32p4_dmarecvsetup() only covers the pre-transfer state; without this
   * second one the caller reads the stale (all-zero) contents even though
   * the descriptor consumed every byte.
   */

  if (priv->dmarxbuf != NULL)
    {
      size_t sync = (priv->dmarxlen + 63u) & ~63u;
      int mret;

      /* M2C + UNALIGNED is rejected by esp_cache_msync (no-op).  The RX
       * buffer is 64-byte aligned; round the length up to a cache line
       * so the invalidate actually runs.
       */

      mret = esp_cache_msync(priv->dmarxbuf, sync,
                             ESP_CACHE_MSYNC_FLAG_DIR_M2C);
      UNUSED(mret);
      priv->dmarxbuf = NULL;
    }

  if ((wkupevent & (SDIOWAIT_ERROR | SDIOWAIT_TIMEOUT)) != 0)
    {
#ifdef CONFIG_ESP32P4_SDMMC_DMA
      esp_cache_msync((void *)(uintptr_t)priv->dma_desc,
                      sizeof(struct sdmmc_dma_s),
                      ESP_CACHE_MSYNC_FLAG_DIR_M2C);
#endif
      syslog(LOG_ERR,
             "SDMMCPROBE: wait error evt=0x%02x remaining=%d "
             "RINTSTS=0x%08lx STATUS=0x%08lx IDSTS=0x%08lx CTYPE=0x%08lx\n",
             wkupevent, priv->remaining,
             (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_RINTSTS),
             (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_STATUS),
             (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_IDSTS),
             (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_CTYPE));

#ifdef CONFIG_ESP32P4_SDMMC_DMA
      syslog(LOG_ERR,
             "SDMMCPROBE: DBADDR=0x%08lx desc@%p des0=0x%08lx des1=0x%08lx "
             "des2=0x%08lx des3=0x%08lx OWN=%d\n",
             (unsigned long)esp32p4_getreg(ESP32P4_SDMMC_DBADDR),
             (void *)priv->dma_desc,
             (unsigned long)priv->dma_desc[0].des0,
             (unsigned long)priv->dma_desc[0].des1,
             (unsigned long)priv->dma_desc[0].des2,
             (unsigned long)priv->dma_desc[0].des3,
             (priv->dma_desc[0].des0 & MCI_DMADES0_OWN) ? 1 : 0);
#endif
    }

  /* Disable all transfer- and event- related interrupts */

  esp32p4_disable_allints(priv);

out:
  leave_critical_section(flags);

  mcinfo("wkupevent=%04x\n", wkupevent);
  return wkupevent;
}

/****************************************************************************
 * Name: esp32p4_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   SDIO callback sequence.  The set of events is configured to enable
 *   callbacks to the function provided in esp32p4_registercallback.
 *
 *   Events are automatically disabled once the callback is performed and no
 *   further callback events will occur until they are again enabled by
 *   calling this method.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOMEDIA_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32p4_callbackenable(struct sdio_dev_s *dev,
                                   sdio_eventset_t eventset)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;

  mcinfo("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  esp32p4_callback(priv);
}

/****************************************************************************
 * Name: esp32p4_registercallback
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change.  Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to SDIO_CALLBACKENABLE
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   callback - The function to call on the media change
 *   arg      - A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

static int esp32p4_registercallback(struct sdio_dev_s *dev,
                                    worker_t callback, void *arg)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;

  /* Disable callbacks and register this callback and is argument */

  mcinfo("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: esp32p4_fill_dma_desc
 *
 * Description:
 *   Build the IDMAC descriptor chain for the current transfer and write the
 *   descriptor memory back from cache so the IDMAC observes it.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32P4_SDMMC_DMA
static int esp32p4_fill_dma_desc(struct esp32p4_dev_s *priv)
{
  uint32_t ctrl;
  uint32_t maxs;
  int i = 0;
  size_t buflen = priv->remaining;
  uint32_t buffer = (uint32_t)(uintptr_t)priv->buffer;

  /* Setup DMA list */

  while (buflen > 0)
    {
      /* Limit size of the transfer to maximum buffer size */

      maxs = buflen;

      if (maxs > MCI_DMADES1_MAXTR)
        {
          maxs = MCI_DMADES1_MAXTR;
        }

      buflen -= maxs;

      /* Set buffer size */

      priv->dma_desc[i].des1 = MCI_DMADES1_BS1(maxs);

      /* IDMAC sees the CPU SRAM address.  The caller reads the payload
       * back through the 0x8FFxxxxx non-cacheable alias.
       */

      priv->dma_desc[i].des2 = buffer + (i * MCI_DMADES1_MAXTR);

      /* Setup basic control */

      ctrl = MCI_DMADES0_OWN | MCI_DMADES0_CH;

      if (i == 0)
        {
          ctrl |= MCI_DMADES0_FS; /* First DMA buffer */
        }

      /* No more data?  Then this is the last descriptor */

      if (buflen == 0)
        {
          ctrl |= MCI_DMADES0_LD;
          priv->dma_desc[i].des3 = 0;
        }
      else
        {
          ctrl |= MCI_DMADES0_DIC;
          priv->dma_desc[i].des3 =
            (uint32_t)(uintptr_t)&priv->dma_desc[i + 1];
        }

      priv->dma_desc[i].des0 = ctrl;
      memset((void *)priv->dma_desc[i].reserved, 0,
             sizeof(priv->dma_desc[i].reserved));
      i++;
    }

  DEBUGASSERT(i < NUM_DMA_DESCRIPTORS);

  /* Write the descriptor list back from cache so the IDMAC sees it.  P4 has
   * a data cache; the descriptors live in cached internal SRAM.
   */

  esp_cache_msync((void *)(uintptr_t)priv->dma_desc,
                  sizeof(struct sdmmc_dma_s) * (size_t)i,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  return 0;
}
#endif

/****************************************************************************
 * Name: esp32p4_dmarecvsetup
 *
 * Description:
 *   Setup to perform a read DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For read transfers this may mean
 *   invalidating the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32P4_SDMMC_DMA
static void esp32p4_idmac_arm(struct esp32p4_dev_s *priv, bool rx)
{
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);
  uint32_t bmod;
  int wait;

  /* IDMAC burst of 8 x 32-bit words.  RX watermark 0 so a 44-byte hosted
   * frame still generates a DMA request.  Do not enable CARDTHR: a 512-byte
   * threshold never trips on a short SDIO frame.
   */

  if (rx)
    {
      /* Burst=1: an 8-beat burst reread the first FIFO word (observed
       * 0x00200005 three times) before the pointer advanced.
       */

      esp32p4_putreg(SDMMC_FIFOTH_RXWMARK(0) | SDMMC_FIFOTH_DMABURST_1XFR,
                     ESP32P4_SDMMC_FIFOTH);
      esp32p4_putreg(0, ESP32P4_SDMMC_CARDTHR);
    }
  else
    {
      esp32p4_putreg(SDMMC_FIFOTH_TXWMARK(ESP32P4_TXFIFO_DEPTH / 2) |
                     SDMMC_FIFOTH_DMABURST_1XFR,
                     ESP32P4_SDMMC_FIFOTH);
      esp32p4_putreg(0, ESP32P4_SDMMC_CARDTHR);
    }

  /* ESP-IDF calls init_dma() once at host init (already done in
   * esp32p4_sdmmc_sdio_initialize).  Re-running BMOD.SWR here left IDMAC
   * in reset across the CMD53 data phase (OWN stuck, buffer zero).
   */

  bmod = esp32p4_getreg(ESP32P4_SDMMC_BMOD);
  if ((bmod & SDMMC_BMOD_SWR) != 0)
    {
      for (wait = 0; wait < 1000; wait++)
        {
          if ((esp32p4_getreg(ESP32P4_SDMMC_BMOD) & SDMMC_BMOD_SWR) == 0)
            {
              break;
            }
        }
    }

  sdmmc_ll_set_desc_addr(hw, (uint32_t)(uintptr_t)&priv->dma_desc[0]);
  sdmmc_ll_enable_dma(hw, true);

  /* enable_dma() leaves BMOD.PBL at the reset default (8).  Force single
   * beat so it matches FIFOTH and the SDIO slave FIFO pop behaviour.
   */

  esp32p4_putreg(SDMMC_BMOD_DE | SDMMC_BMOD_FB, ESP32P4_SDMMC_BMOD);
  sdmmc_ll_poll_demand(hw);

}

static int esp32p4_dmarecvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                                size_t buflen)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(buffer != NULL && buflen > 0 &&
              ((uint32_t)(uintptr_t)buffer & 3) == 0);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->wrdir     = false;
  priv->dmarxbuf  = buffer;
  priv->dmarxlen  = buflen;

  /* Setup DMA list */

  if (esp32p4_fill_dma_desc(priv))
    {
      return -ENOMEM;
    }

  /* Flush any CPU paint/zeroing into SRAM so a failed DMA is visible as
   * the pre-fill pattern.  After the transfer, eventwait invalidates so
   * the CPU sees IDMAC writes.
   */

  esp_cache_msync(buffer, buflen,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                  ESP_CACHE_MSYNC_FLAG_UNALIGNED);

  /* Flush ints before we start */

  esp32p4_putreg(SDCARD_TRANSFER_ALL, ESP32P4_SDMMC_RINTSTS);
  esp32p4_putreg(0xffffffff, ESP32P4_SDMMC_IDSTS);

  esp32p4_idmac_arm(priv, true);

  /* Setup DMA error interrupts */

  esp32p4_config_dmaints(priv, SDCARD_DMARECV_MASK, SDCARD_DMAERROR_MASK);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp32p4_dmasendsetup
 *
 * Description:
 *   Setup to perform a write DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For write transfers, this may mean
 *   flushing the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA into
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32P4_SDMMC_DMA
static int esp32p4_dmasendsetup(struct sdio_dev_s *dev,
                                const uint8_t *buffer, size_t buflen)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;

  DEBUGASSERT(priv != NULL);

  mcinfo("buflen=%lu\n", (unsigned long)buflen);
  DEBUGASSERT(buffer != NULL && buflen > 0 &&
              ((uint32_t)(uintptr_t)buffer & 3) == 0);

  /* Save the source buffer information for use by the interrupt handler. */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->wrdir     = true;
  priv->dmarxbuf  = NULL;
  priv->dmarxlen  = 0;

  /* Setup DMA descriptor list */

  if (esp32p4_fill_dma_desc(priv))
    {
      return -ENOMEM;
    }

  /* Write the source buffer back from cache so the IDMAC reads valid data */

  esp_cache_msync((void *)buffer, buflen,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                  ESP_CACHE_MSYNC_FLAG_UNALIGNED);

  /* Flush ints before we start */

  esp32p4_putreg(SDCARD_TRANSFER_ALL, ESP32P4_SDMMC_RINTSTS);
  esp32p4_putreg(0xffffffff, ESP32P4_SDMMC_IDSTS);

  esp32p4_idmac_arm(priv, false);

  /* Setup DMA error interrupts */

  esp32p4_config_dmaints(priv, SDCARD_DMASEND_MASK, SDCARD_DMAERROR_MASK);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp32p4_callback
 *
 * Description:
 *   Perform callback.
 *
 * Assumptions:
 *   This function does not execute in the context of an interrupt handler.
 *   It may be invoked on any user thread or scheduled on the work thread
 *   from an interrupt handler.
 *
 ****************************************************************************/

static void esp32p4_callback(void *arg)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)arg;

  /* Is a callback registered? */

  DEBUGASSERT(priv != NULL);
  mcinfo("Callback %p(%p) cbevents: %02x cdstatus: %02x\n",
         priv->callback, priv->cbarg, priv->cbevents, priv->cdstatus);

  if (priv->callback)
    {
      /* Yes.. Check for enabled callback events */

      if ((priv->cdstatus & SDIO_STATUS_PRESENT) != 0)
        {
          /* Media is present.  Is the media inserted event enabled? */

          if ((priv->cbevents & SDIOMEDIA_INSERTED) == 0)
            {
              /* No... return without performing the callback */

              return;
            }
        }
      else
        {
          /* Media is not present.  Is the media eject event enabled? */

          if ((priv->cbevents & SDIOMEDIA_EJECTED) == 0)
            {
              /* No... return without performing the callback */

              return;
            }
        }

      /* Perform the callback, disabling further callbacks.  Of course, the
       * callback can (and probably should) re-enable callbacks.
       */

      priv->cbevents = 0;

      /* Callbacks cannot be performed in the context of an interrupt
       * handler.  If we are in an interrupt handler, then queue the
       * callback to be performed later on the work thread.
       */

      if (up_interrupt_context())
        {
          /* Yes.. queue it */

          mcinfo("Queuing callback to %p(%p)\n",
                 priv->callback, priv->cbarg);
          work_queue(HPWORK, &priv->cbwork, priv->callback,
                     priv->cbarg, 0);
        }
      else
        {
          /* No.. then just call the callback here */

          mcinfo("Callback to %p(%p)\n", priv->callback, priv->cbarg);
          priv->callback(priv->cbarg);
        }
    }
}

/****************************************************************************
 * Name: esp32p4_sdmmc_enable_clock_reset
 *
 * Description:
 *   Enable the SDMMC bus clock, pulse the module reset, select the LS clock
 *   source (PLL160M) and initialise the clock phase.  These use the P4
 *   HP_SYS_CLKRST / LP_CLKRST blocks -- the S3 did the equivalent in the
 *   SYSTEM_PERIP_CLK_EN1 / SYSTEM_PERIP_RST_EN1 registers and in the SDMMC
 *   0x800 CLOCK register.  This is the KEY clock adaptation to verify on
 *   hardware.
 *
 ****************************************************************************/

static void esp32p4_sdmmc_enable_clock_reset(void)
{
  sdmmc_dev_t *hw = SDMMC_LL_GET_HW(0);

  /* PERIPH_RCC_ATOMIC() provides the __DECLARE_RCC_ATOMIC_ENV that the HAL
   * clock helpers require and the surrounding critical section.
   */

  PERIPH_RCC_ATOMIC()
    {
      /* Enable the AHB bus clock and pulse the module reset */

      sdmmc_ll_enable_bus_clock(0, true);
      sdmmc_ll_reset_register(0);

      /* Power up the SDIO PLL (PMU) -- without this the selected clock
       * source produces no clock and the controller reset never completes.
       */

      sdmmc_ll_enable_sdio_pll(hw, true);

      /* Select PLL160M as the LS clock source and set the host divider */

      sdmmc_ll_select_clk_source(hw, SDMMC_CLK_SRC_PLL160M);
      sdmmc_ll_set_clock_div(hw, 1);
      sdmmc_ll_init_phase_delay(hw);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32p4_sdmmc_sdio_initialize
 *
 * Description:
 *   Initialize the SDMMC host for SDIO operation.
 *
 * Input Parameters:
 *   slotno - The SDMMC slot (0 or 1).  The on-board C6 is on slot 1.
 *
 * Returned Value:
 *   A reference to an SDIO interface structure.  NULL is returned on
 *   failures.
 *
 ****************************************************************************/

struct sdio_dev_s *esp32p4_sdmmc_sdio_initialize(int slotno)
{
  struct esp32p4_dev_s *priv = &g_sdiodev;

  DEBUGASSERT(slotno == 0 || slotno == 1);

  priv->slot      = slotno;
  priv->cpuint    = -1;
  priv->slot_info = &g_sdmmc_slot_info[slotno];
  priv->sdio_pins = &g_sdmmc_slot_gpio_sig[slotno];

  /* Official FreeRTOS hosted host (sd_pwr_ctrl_new_on_chip_ldo): LDO_VO4
   * at 3.3 V powers the SDMMC IO rail on the P4 function-ev board
   * (CONFIG_ESP_HOSTED_SD_PWR_CTRL_LDO_IO_ID=4).  Without it, CMD52 and
   * 1-bit 400 kHz can still work, but 4-bit CMD53 fails with SBE.
   */

    {
      static bool s_sdmmc_ldo_on;
      esp_ldo_channel_handle_t ldo = NULL;
      esp_ldo_channel_config_t cfg;

      if (!s_sdmmc_ldo_on)
        {
          memset(&cfg, 0, sizeof(cfg));
          cfg.chan_id    = 4;
          cfg.voltage_mv = 3300;
          if (esp_ldo_acquire_channel(&cfg, &ldo) == 0)
            {
              s_sdmmc_ldo_on = true;
              syslog(LOG_INFO, "SDMMC: LDO_VO4 3300mV enabled\n");
            }
          else
            {
              syslog(LOG_ERR, "SDMMC: LDO_VO4 acquire failed\n");
            }
        }
    }

  /* Enable the bus clock, reset the module and set up the LS clock source
   * and phase.
   */

  esp32p4_sdmmc_enable_clock_reset();

  /* Reset the controller */

  priv->dev.reset(&priv->dev);

  /* Allow DMA through L2 (default SHUT_DMA=1 blocks AHB masters from
   * internal SRAM).  IDMAC is re-armed per transfer.
   */

#ifdef CONFIG_ESP32P4_SDMMC_DMA
    {
      uint32_t l2ctrl = getreg32(CACHE_L2_CACHE_CTRL_REG);

      l2ctrl &= ~CACHE_L2_CACHE_SHUT_DMA;
      putreg32(l2ctrl, CACHE_L2_CACHE_CTRL_REG);
      syslog(LOG_INFO, "SDMMC: L2_CACHE_CTRL=0x%08lx\n",
             (unsigned long)getreg32(CACHE_L2_CACHE_CTRL_REG));
    }
#endif

  sdmmc_ll_init_dma(SDMMC_LL_GET_HW(0));

  /* Pin configuration (slot 1 is routed through the GPIO matrix).  CLK is
   * output-only; CMD and D0 are bidirectional with a pull-up.  The extra
   * data lines D1..D3 are configured by esp32p4_widebus() when 4-bit mode
   * is selected.
   */

  configure_pin(CONFIG_ESP32P4_SDMMC_CLK, priv->sdio_pins->clk, OUTPUT);
  configure_pin(CONFIG_ESP32P4_SDMMC_CMD, priv->sdio_pins->cmd,
                INPUT | OUTPUT | PULLUP);
  configure_pin(CONFIG_ESP32P4_SDMMC_D0, priv->sdio_pins->d0,
                INPUT | OUTPUT | PULLUP);
  configure_pin(CONFIG_ESP32P4_SDMMC_D1, priv->sdio_pins->d1,
                INPUT | OUTPUT | PULLUP);
  configure_pin(CONFIG_ESP32P4_SDMMC_D2, priv->sdio_pins->d2,
                INPUT | OUTPUT | PULLUP);
  configure_pin(CONFIG_ESP32P4_SDMMC_D3, priv->sdio_pins->d3,
                INPUT | OUTPUT | PULLUP);

  /* Tie the card-interrupt input high (inactive), card-detect low (card
   * present -- the C6 is hard-wired) and write-protect inactive, all
   * through GPIO-matrix constant inputs.
   */

  esp_gpio_matrix_in(GPIO_MATRIX_CONST_ONE_INPUT,
                     priv->slot_info->card_int, false);
  esp_gpio_matrix_in(GPIO_MATRIX_CONST_ZERO_INPUT,
                     priv->slot_info->card_detect, false);
  esp_gpio_matrix_in(GPIO_MATRIX_CONST_ONE_INPUT,
                     priv->slot_info->write_protect, true);

  return &priv->dev;
}

/****************************************************************************
 * Name: esp32p4_sdmmc_set_sample_phase
 ****************************************************************************/

void esp32p4_sdmmc_set_sample_phase(unsigned int phase)
{
  irqstate_t flags;
  uint32_t regval;

  phase &= 3u;

  flags = enter_critical_section();
  regval = esp32p4_getreg(ESP32P4_HP_SYS_CLKRST_PERI_CLK_CTRL02);
  regval &= ~HP_SYS_CLKRST_SDIO_LS_SAM_EDGE_SEL_M;
  regval |= (phase << HP_SYS_CLKRST_SDIO_LS_SAM_EDGE_SEL_S) &
            HP_SYS_CLKRST_SDIO_LS_SAM_EDGE_SEL_M;
  regval |= HP_SYS_CLKRST_SDIO_LS_SAM_CLK_EN;
  esp32p4_putreg(regval, ESP32P4_HP_SYS_CLKRST_PERI_CLK_CTRL02);

  regval |= HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_UPD;
  esp32p4_putreg(regval, ESP32P4_HP_SYS_CLKRST_PERI_CLK_CTRL02);
  regval &= ~HP_SYS_CLKRST_SDIO_LS_CLK_EDGE_UPD;
  esp32p4_putreg(regval, ESP32P4_HP_SYS_CLKRST_PERI_CLK_CTRL02);
  leave_critical_section(flags);

  up_udelay(10);
  syslog(LOG_INFO, "SDMMC: sample phase %u\n", phase);
}

#endif /* CONFIG_ESP32P4_SDMMC */
