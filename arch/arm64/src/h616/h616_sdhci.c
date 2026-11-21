/****************************************************************************
 * arch/arm64/src/h616/h616_sdhci.c
 *
 * H616 (sun50iw9) SD/MMC host controller driver.
 *
 * Implements the NuttX SDIO interface (struct sdio_dev_s) on top of the
 * Allwinner sunxi-mmc (SDHCI-compatible) controller.  This provides eMMC
 * block access (/dev/mmcsd0) through the mmcsd framework.
 *
 * Ported from the NuttX SDIO driver model (stm32_sdio.c style: lock/reset
 * capabilities/status/widebus/clock/attach/sendcmd/blocksetup/recvsetup
 * sendsetup/cancel/waitresponse/recv_rX/eventwait/callbackenable
 * registercallback) with register definitions from the Allwinner sunxi-mmc
 * block (Tina U-Boot drivers/mmc/sunxi_mmc.c + linux sunxi-mmc.h, v4p6x).
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
 ***************************************************************************
 */

/****************************************************************************
 * Included Files
 ***************************************************************************
 */

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/kthread.h>
#include <unistd.h>

#include "arm64_internal.h"
#include "chip.h"
#include "hardware/h616_sdhci.h"
#include "hardware/h616_memorymap.h"

/* Per transfer diagnostic prints.  The driver is chatty on purpose (every
 * interesting register is dumped once per data transfer), but those lines
 * share the console UART with NSH: the two streams end up character
 * interleaved and the boot log becomes unreadable.  Route them through
 * H616_DIAG and leave the switch off unless a register level trace is being
 * chased.  The "ladder" verdict prints (bus width experiment) deliberately
 * stay on _info so a boot log always shows the outcome of that test.
 */

static bool g_sdio_diag_muted = true;

/* Data path selection.  The CPU/AHB FIFO path is the one the boot is
 * verified with (1 bit, 8 bit SDR, mount, file I/O), so it stays the default
 * and the
 * boot can never be broken by an experiment on the other one.  U-Boot runs
 * the IDMAC instead: at the prompt GCTRL reads 0x20000000 (ACCESS_BY_AHB
 * *clear*, bit29 set) with DMAC = 0x300, IDIE = 0x10800080, DLBA/CHDA
 * programmed -- and CSDC = 6 with the start bit detect set, the exact pair
 * that stops the AHB path dead (proven by the N11 probe: "ub-sdr" uses the
 * handover timing with CSDC = 6 / DSBD = 1 and the data phase never starts,
 * while "ub-sdr-c3", which differs only in those two fields, verifies).  So
 * the IDMAC is the only remaining candidate for the handover timing and for
 * DDR52, and the delayed probe flips this flag at run time to find out
 * whether the DMA path works on this controller at all.
 */

/* Boot time default for the data path (see g_rx_use_ahb).  U-Boot moves the
 * payload with plain CPU FIFO accesses (mmc_trans_data_by_cpu) and that is
 * the path this driver has always booted with, so it stays the default; the
 * IDMAC code below is compiled in either way and can be selected at run
 * time.
 */

#define H616_RX_USE_AHB 1

static volatile bool g_rx_use_ahb = (H616_RX_USE_AHB != 0);

/* Run time data path selection (see h616_sdhci.h).
 */

void h616_sdhci_set_datapath(int use_ahb)
{
  g_rx_use_ahb = (use_ahb != 0);
}

#define H616_DIAG(...) \
  do { if (!g_sdio_diag_muted) { _info(__VA_ARGS__); } } while (0)

#ifdef CONFIG_H616_SDHCI

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************
 */

#define H616_SDMMC2_IRQ             H616_IRQ_SD_MMC2  /* GIC SPI 37 */

/* Interrupt-driven waiting.  The wait loops below remain fully functional
 * pollers; the ISR only makes them react instantly and lets them sleep
 * instead of hot-spinning (a 2 MiB IDMAC read used to burn a whole CPU
 * for ~20 ms).  Every wait still polls the raw RINTR/ latch as well, so
 * losing the interrupt costs latency, never correctness.
 */

#ifdef CONFIG_H616_SDMMC_IRQ
#  define H616_SDIO_USEIRQ           1
#endif

#define H616_SDIO_CMDBITS            (H616_SDXC_COMMAND_DONE | \
                                      H616_SDXC_RESP_ERROR | \
                                      H616_SDXC_RESP_CRC_ERROR | \
                                      H616_SDXC_RESP_TIMEOUT)

/* Iterations a 1-tick sleep is worth (see CONFIG_BOARD_LOOPSPERMSEC:
 * the calibrated loop rate is ~1165000 iterations per 10 ms tick).
 */

#define H616_SDIO_POLLS_PER_TICK     116500

/* Interrupt statistics (see h616_interrupt)
 */

#ifndef CONFIG_H616_SDHCI_CLK
#  define CONFIG_H616_SDHCI_CLK      24000000
#endif

/* ID mode clocking (card init) is <400KHz.
 */

/* The data path default lives at the top of this file (H616_RX_USE_AHB).
 *
 */
#define H616_SDMMC_IDCLK             400000

/* Normal operation clocking (eMMC).
 *
 * This MUST NOT exceed the clock that is legal for the card's CURRENT
 * speed mode.  h616_reset() pulses the card's hardware reset (HWRST) and
 * then re-runs CMD0/CMD1, which puts the eMMC back into its power-on
 * default: backward-compatible (default speed) timing and a 1 bit bus.
 * Default speed is specified up to 26MHz, and an eMMC that is still in
 * that mode cannot drive a 512 byte data burst at 100MHz -- measured on
 * this board: with the card freshly reset, a CMD8 (SEND_EXT_CSD) at
 * 100MHz never starts its data phase (CBCR stays 0, DATA_OVER never
 * fires, the FIFO stays empty), while the identical command with the
 * card still in the HS200/8-bit state U-Boot left behind at 200MHz reads
 * 512 bytes correctly.  Every earlier "100MHz works" observation was
 * made against that HS200 card state.
 *
 * So keep the transfer clock at a default-speed legal rate until a real
 * HS_TIMING switch (CMD6 index 185) is implemented and verified.
 */

#define H616_SDMMC_MAXCLK            26000000

/* eMMC bus width / speed ladder.
 *
 * mmcsd finishes its bring-up with SDIO_CLOCK(CLOCK_MMC_TRANSFER), so that
 * call is the last chance the host has to move the card out of its power on
 * default (1 bit, default speed).  The ladder below runs there, tries the
 * faster modes one at a time and verifies each of them with a real block
 * read against a reference read taken in the known good 1 bit
 * configuration.  A step that fails to transfer, or that returns different
 * data, is rolled back immediately, so a wrong delay value or a mode this
 * card/board cannot do degrades gracefully instead of breaking the boot.
 *
 * The steps were brought up one at a time, each verified on hardware:
 *   1: 8 bit SDR (unchanged clock)   2: 8 bit DDR52   3: HS400
 */

#define H616_LADDER_8BIT             1
#define H616_LADDER_DDR52            1
#define H616_LADDER_HS400            1

/* 0: the ladder measures 8 bit and then hands the bus back to 1 bit, so the
 * rest of the boot (and the FAT mount) runs in the configuration that is
 * known to work.  1: keep 8 bit when it verified -- the point of the N3
 * test:
 * the FAT mount, the directory scan and the file read then all run on the 8
 * bit bus, which is the only way to find out whether 8 bit survives real
 * file system traffic (not just single block reads).
 */

#define H616_LADDER_KEEP8BIT         1

/* Boot time ladder.  Now 0: the 8 bit switch has moved into h616_widebus(),
 * which is where the mmcsd layer asks the driver to select the bus width, so
 * it happens on the real path instead of behind the stack's back.  The
 * ladder
 * is kept (and still compiled) as the platform for the next steps -- 8 bit
 * DDR52 and HS400 -- which need their own delay programming and their own
 * verification; H616_LADDER_KEEP8BIT is inert while this is 0.  Set to 1 to
 * run the old boot time experiment instead.
 */

#define H616_LADDER_AT_BOOT          0

/* Delay after a CMD6 mode switch and before the first transfer on the new
 * bus.  The card is busy writing its EXT_CSD byte for a while, and a data
 * command issued while it is busy either never starts a data phase at all
 * (RINTR = 0, CBCR = 0) or moves garbage -- the two failure modes this board
 * shows on the first transfer after a width change.
 */

#define H616_LADDER_SETTLE_MS        5

/* Reference reads must be repeatable, not just complete: a reference has
 * nothing it can be compared against, so the only way to tell a real
 * transfer from noise is to read the same block twice and require the two
 * copies to be identical.
 */

#define H616_LADDER_REF_TRIES        4

/* The CIU reset values of the two delay registers are the known good 1 bit
 * 8 bit SDR setup on this board: sampled delay 0x18, data strobe delay 0x0d,
 * both with the enable bit set and extra bits the small-value helper below
 * would drop.  Faster modes sweep their own values and put these back.
 *
 */

#define H616_SAMP_DL_SDR             0x1898u
#define H616_DS_DL_SDR               0x0d8du

/* Attempt 8 bit DDR52 from h616_widebus() once 8 bit SDR has verified, so a
 * failure falls back to a bus that is already known to work.
 */

/* NTSR (0x5c) bits.  bit31 is the vendor's "2x timing" mode, bits[9:8]
 * carry the data and bits[5:4] the command sampling phase (Linux v4p1x
 * sunxi_mmc_set_dly_raw(); the u-boot tm4 driver writes bit31 clear).
 */

#define H616_SDXC_NTSR_2X_TIMING      (1u << 31)
#define H616_SDXC_NTSR_DAT_PH(x)      ((((uint32_t)(x)) & 3u) << 8)
#define H616_SDXC_NTSR_CMD_PH(x)      ((((uint32_t)(x)) & 3u) << 4)

#define H616_WIDEBUS_DDR52           1
#define H616_WIDEBUS_SDR50           1

/* 8 bit DDR52 on the boot bus.  It runs after the 8 bit SDR bus has been
 * verified, and an attempt that fails is rolled back to that bus, so the
 * boot still reaches its mount either way.  Set to 0 to stay on 8 bit SDR.
 *
 */

#define H616_DDR52_SUPPORT           1

/* CMD6 (SWITCH) argument for writing one EXT_CSD byte: access = 3 (write
 * bytes), index = EXT_CSD byte, value = one bit per byte written
 * (JEDEC 6.6.2, the same encoding the driver already used for the earlier
 * bus width switch).
 */

#define H616_CMD6_ARG(index, value) \
  ((3u << 24) | ((uint32_t)(index) << 16) | ((uint32_t)(value) << 8))

#define H616_EXT_CSD_BUS_WIDTH       183u
#define H616_EXT_CSD_HS_TIMING       185u

#define H616_BUS_WIDTH_8BIT          2u
#define H616_BUS_WIDTH_8BIT_DDR      6u

#define H616_HS_TIMING_HS            1u
#define H616_HS_TIMING_HS400         3u

#define H616_R1_SWITCH_ERROR         (1u << 7)
#define H616_R1_ILLEGAL_COMMAND      (1u << 22)

/* Every step is verified by reading the first block of the openvela boot
 * image at 0x20000: fel_flash writes it on every flash, so it is non-zero,
 * stable and always present.  The throughput measurement behind each step
 * runs for at least this many system ticks, or this many blocks, whichever
 * comes first.
 */

#define H616_LADDER_LBA              0x00020000u
#define H616_LADDER_LBA2             0x00020200u
/* Throughput measurement window: long enough for the 1 ms system tick to be
 * more than a rounding error, short enough not to hold up the boot.
 */

#define H616_LADDER_TICKLIMIT        50
#define H616_LADDER_MAXBLOCKS        8192

/* Sample / strobe delays from the H616 timing mode 4 table for a 50MHz card
 * clock (Tina U-Boot sunxi_host_mmc.c, def_sdly / def_dsdly): DDR50 needs
 * 0x0e, HS400 needs 0x11 for the inbound sample delay and 0x18 for the data
 * strobe.  The drive delay is PH180 for legacy/HS/DDR50 and PH90 for
 * HS200/HS400 (drv_dl bits[17:16]).
 */

#define H616_SAMP_DLY_DDR50          0x0eu
#define H616_SAMP_DLY_HS400          0x18u
#define H616_DS_DLY_HS400            0x0du
#define H616_DRV_DLY_PH180           1u
#define H616_DRV_DLY_PH90            0u

/* CCU module clock to program for each mode.  Timing mode 4 divides the
 * module clock inside the controller by two (by four for 8 bit DDR52), so
 * 50MHz DDR needs 4 x 50MHz and HS400 needs 2 x 100MHz.  8 bit SDR keeps
 * the 25MHz module clock that is already known to work.
 */

#define H616_LADDER_CLK_8BIT_SDR     H616_SDMMC_MAXCLK
#define H616_LADDER_CLK_DDR52        50000000u
#define H616_LADDER_CLK_HS400        200000000u

/* Timeout values in ticks of the 24MHz module clock
 */

#define H616_SDMMC_CMDTIMEOUT        1000000
#define H616_SDMMC_LONGTIMEOUT       10000000

/* Poll budget for the two register polling loops (h616_waitresponse() and
 * h616_eventwait()).  The boot value is what the rest of the driver has
 * always used; the DDR52 probe lowers it while it sweeps, because a sample
 * point that does not work pays the whole budget and nine seconds per point
 * is what makes a sweep of 128 points take half an hour.
 */

#define H616_SDMMC_EVENTPOLLS        10000000
#define H616_SDMMC_PROBE_POLLS        1000000

static volatile int32_t g_sdio_poll_limit = H616_SDMMC_EVENTPOLLS;

/* DMA descriptor: 4-word 32-bit descriptor used by the sunxi IDMAC.
 * See sunxi_mmc_tuning.c / Linux sunxi-mmc.c (struct sunxi_mmc_idma_desc).
 *
 */

struct h616_idma_desc_s
{
  uint32_t config;      /* 0x0 */
  uint32_t size;        /* 0x4 */
  uint32_t addr;        /* 0x8 */
  uint32_t next;        /* 0xc */
};

#define H616_IDMA_DESC_DIC             (1 << 1)  /* Disable interrupt on completion */
#define H616_IDMA_DESC_FD              (1 << 2)  /* First descriptor */
#define H616_IDMA_DESC_LD              (1 << 3)  /* Last descriptor */
#define H616_IDMA_DESC_OWN             (1 << 31) /* Owned by IDMAC */

/****************************************************************************
 * Private Types
 ***************************************************************************
 */

struct h616_sdhci_priv_s
{
  struct sdio_dev_s      dev;         /* Standard SDIO interface (first) */

  uintptr_t              base;        /* Controller base address */
  uint32_t               clock;       /* Current card clock */
  uint8_t                buswidth;    /* Current bus width */
  bool                   wide;        /* Wide bus (4/8 bit) */
  bool                   ddr;         /* 8 bit DDR52 (card and host) */
  bool                   hs400;       /* HS400 (card HS_TIMING=3, host strobe) */
  uint32_t               bus_mod_hz;  /* CCU module clock of the kept bus mode */
  uint32_t               bus_clkdiv;  /* CLKCR[7:0] of the kept bus mode */
  bool                   ladderdone;  /* Speed ladder already run */
  bool                   dmamode;     /* DMA enabled */
  uint32_t               xfrflags;    /* Transfer state flags */
  sdio_eventset_t        waitevents;  /* Events being waited for */
  volatile sdio_eventset_t wkupevent; /* Event that woke up the waiter */
  sdio_eventset_t        cbevents;    /* Events to cause callbacks */
  sdio_capset_t          caps;        /* Supported capabilities */
  struct h616_idma_desc_s *dma_desc;  /* DMA descriptor (unused, PIO mode) */
  uint32_t              *dma_buffer;  /* DMA bounce buffer (unused) */
  uint32_t               xfrmask;     /* Transfer-related interrupt mask */
  sem_t                  waitsem;     /* Wait for events */
#ifdef H616_SDIO_USEIRQ
  sem_t                  cmdsem;      /* Wait for command completion */
  volatile uint32_t      cmdrintr;    /* Command RINTR bits latched by ISR */
  volatile uint32_t      rintr_latch; /* Data RINTR bits latched by ISR */
#endif
  bool                   wdogactive;

  /* Data path state (IDMAC)
   */

  FAR uint8_t           *buffer;      /* User data buffer */
  size_t                 remaining;   /* Bytes left to transfer */
  size_t                 total;       /* Total bytes of this transfer */
  bool                   xfrdir;      /* false = RX (read), true = TX (write) */
};

/****************************************************************************
 * Private Function Prototypes
 ***************************************************************************
 */

/* One host timing configuration.  Every field is what gets written to the
 * controller: the CCU module clock, the CLKCR internal divider, the NTSR
 * image (2x timing bit and sampling phases), the DRV_DL output phase, the
 * SAMP_DL sampled delay, GCTRL DDR mode and the DSBD HS400 bit.
 */

struct h616_bus_profile_s
{
  FAR const char *tag;    /* Name used in the log */
  uint32_t mod_hz;        /* CCU module clock feeding the controller */
  uint32_t clkdiv;        /* CLKCR[7:0] internal divider */
  uint32_t ntsr;          /* 0x5c image: 2x timing bit + sampling phases */
  int      odly;          /* DRV_DL output phase, -1 leaves it alone */
  int      sdly;          /* SAMP_DL value, -1 leaves it alone */
  bool     ddr;           /* GCTRL DDR mode */
  bool     dsbd;          /* DSBD bit31 (HS400 start bit detection) */
};

/* The configuration this driver has always used for 1 bit and 8 bit SDR --
 * the whole boot runs from it: CCU module clock equal to the card clock, no
 * internal divider, no NTSR phase shift, the CIU reset sample delay (0x1898
 * = enable + 0x18 + two phase bits) and no output phase shift.
 */

static const struct h616_bus_profile_s g_sdr25_profile =
{
  "sdr25", H616_SDMMC_MAXCLK, 0u, 0u, 0, 0x18, false, false
};

static void h616_bus_apply(struct h616_sdhci_priv_s *priv,
                           FAR const struct h616_bus_profile_s *p);
static bool h616_bus_verify(struct h616_sdhci_priv_s *priv);
static void h616_ladder_fixciu(struct h616_sdhci_priv_s *priv);
static bool h616_ddr52_boot(struct h616_sdhci_priv_s *priv);
static bool h616_bus_sweep(struct h616_sdhci_priv_s *priv,
                           FAR const char *what,
                           FAR const struct h616_bus_profile_s *table,
                           size_t count,
                           FAR struct h616_bus_profile_s *kept);
static bool h616_ladder_sdr50(struct h616_sdhci_priv_s *priv,
                              FAR struct h616_bus_profile_s *kept);

/* N72: interrupt path disabled -- the driver is fully poll driven.
 */

/* Low-level helpers
 */

static inline uint32_t h616_getreg(const struct h616_sdhci_priv_s *priv,
                                   unsigned int offset);
static inline void h616_putreg(struct h616_sdhci_priv_s *priv,
                               unsigned int offset, uint32_t value);
static inline void h616_modifyreg(struct h616_sdhci_priv_s *priv,
                                  unsigned int offset, uint32_t clr,
                                  uint32_t set);
static void h616_setclock(struct h616_sdhci_priv_s *priv, uint32_t clk);
static void h616_setbuswidth(struct h616_sdhci_priv_s *priv, int width);
static int  h616_sendcmdhw(struct h616_sdhci_priv_s *priv, uint32_t cmd,
                           uint32_t arg);
static void h616_recvfifo(struct h616_sdhci_priv_s *priv)
  __attribute__((unused));
static void h616_sendfifo(struct h616_sdhci_priv_s *priv)
  __attribute__((unused));
static void h616_configxfrints(struct h616_sdhci_priv_s *priv,
                               uint32_t enxfr);
static void h616_configwaitints(struct h616_sdhci_priv_s *priv,
                                uint32_t waitints,
                                sdio_eventset_t wkupevents);

/* SDIO interface
 */

static void h616_reset(struct sdio_dev_s *dev);
static sdio_capset_t h616_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t h616_status(struct sdio_dev_s *dev);
static void h616_widebus(struct sdio_dev_s *dev, bool enable);
static void h616_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate);
static int  h616_attach(struct sdio_dev_s *dev);
static int  h616_sendcmd(struct sdio_dev_s *dev, uint32_t cmd, uint32_t arg);
static void h616_blocksetup(struct sdio_dev_s *dev, unsigned int blocklen,
                            unsigned int nblocks);
static int  h616_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                           size_t nbytes);
static int  h616_sendsetup(struct sdio_dev_s *dev,
                           const uint8_t *buffer, size_t nbytes);
static int  h616_cancel(struct sdio_dev_s *dev);
static int  h616_waitresponse(struct sdio_dev_s *dev, uint32_t cmd);
static int  h616_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                              uint32_t *rshort);
static int  h616_recvlongcrc(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t rlong[4]);
static int  h616_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t *rshort);
static void h616_waitenable(struct sdio_dev_s *dev,
                            sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t h616_eventwait(struct sdio_dev_s *dev);
static void h616_callbackenable(struct sdio_dev_s *dev,
                                sdio_eventset_t eventset);
static int  h616_registercallback(struct sdio_dev_s *dev,
                                  worker_t callback, void *arg);

/****************************************************************************
 * Private Data
 ***************************************************************************
 */

static struct h616_sdhci_priv_s g_sdio =
{
  .dev =
  {
    .reset            = h616_reset,
    .capabilities     = h616_capabilities,
    .status           = h616_status,
    .widebus          = h616_widebus,
    .clock            = h616_clock,
    .attach           = h616_attach,
    .sendcmd          = h616_sendcmd,
    .blocksetup       = h616_blocksetup,
    .recvsetup        = h616_recvsetup,
    .sendsetup        = h616_sendsetup,
    .cancel           = h616_cancel,
    .waitresponse     = h616_waitresponse,
    .recv_r1          = h616_recvshortcrc,
    .recv_r2          = h616_recvlongcrc,
    .recv_r3          = h616_recvshort,
    .recv_r4          = h616_recvshort,
    .recv_r5          = h616_recvshortcrc,
    .recv_r6          = h616_recvshortcrc,
    .recv_r7          = h616_recvshortcrc,
    .waitenable       = h616_waitenable,
    .eventwait        = h616_eventwait,
    .callbackenable   = h616_callbackenable,
    .registercallback = h616_registercallback,

    /* gotextcsd: NULL (not needed; SDIO_GOTEXTCSD macro is NULL-safe) */
  },
  .base = H616_SDMMC2_ADDR,
  .xfrmask = 0,
  .waitsem = SEM_INITIALIZER(0),
#ifdef H616_SDIO_USEIRQ
  .cmdsem = SEM_INITIALIZER(0),
#endif
};

/****************************************************************************
 * Private Functions
 ***************************************************************************
 */

static inline uint32_t h616_getreg(const struct h616_sdhci_priv_s *priv,
                                   unsigned int offset)
{
  return getreg32(priv->base + offset);
}

static inline void h616_putreg(struct h616_sdhci_priv_s *priv,
                               unsigned int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static inline void h616_modifyreg(struct h616_sdhci_priv_s *priv,
                                  unsigned int offset, uint32_t clr,
                                  uint32_t set)
{
  uint32_t val = getreg32(priv->base + offset);

  val &= ~clr;
  val |= set;
  putreg32(val, priv->base + offset);
}

/* Enable the controller module clock via the CCU and set its rate.
 * Mirrors Tina U-Boot mmc_set_mod_clk()/mmc_clk_io_onoff() for H616
 * (sun50iw9): the module clock divider lives in the CCU SD2_CLK_CFG
 * register, and the controller CLKCR divider is left at divide-by-1.
 *
 *   CCU output = source / (2^N) / M    (M in bits[4:0], N in bits[11:8])
 *   For f <= 24MHz: source = OSC24M, else PLL6x2.
 */

static void h616_clockenable(struct h616_sdhci_priv_s *priv, uint32_t clk)
{
  uintptr_t ccu = H616_CCU_ADDR;
  uint32_t regval;
  uint32_t pll;
  uint32_t pll_hz;
  uint32_t div;
  uint32_t n;
  uint32_t m;

  /* Deassert reset + enable gate for MMC2 (H616: gate=bit2, rst=bit18)
   */

  regval = getreg32(ccu + H616_CCU_SD_GATE_RESET);
  regval |= H616_CCM_MMC_RST(2) | H616_CCM_MMC_GATE(2);
  putreg32(regval, ccu + H616_CCU_SD_GATE_RESET);

  /* Select the clock source and compute the divider
   */

  if (clk <= H616_OSC24M)
    {
      pll     = H616_CCM_MMC_CTRL_OSCM24;
      pll_hz  = H616_OSC24M;
    }
  else
    {
      pll     = H616_CCM_MMC_CTRL_PLL6X2;
      pll_hz  = H616_PLL6X2_HZ;
    }

  div = (pll_hz + clk - 1) / clk;

  n = 0;
  while (div > 16)
    {
      n++;
      div = (div + 1) / 2;
    }

  m = div;

  /* Write the module clock: enable + source + N multiplier + M divider
   */

  regval = H616_CCM_MMC_CTRL_ENABLE | pll |
           H616_CCM_MMC_CTRL_N(n) | H616_CCM_MMC_CTRL_M(m);
  putreg32(regval, ccu + H616_CCU_SD2_CLK_CFG);

  priv->clock = clk;
}

/* Set the controller-internal clock.  With the CCU module clock already
 * set to the target rate (h616_clockenable), the controller CLKCR divider
 * is left at divide-by-1 and the clock is (re)triggered via CMDR with
 * SDXC_UPCLK_ONLY (sunxi_mmc_update_clk) so the new rate takes effect.
 *
 */

static void h616_setclock(struct h616_sdhci_priv_s *priv, uint32_t clk)
  __attribute__((unused));

static void h616_setclock(struct h616_sdhci_priv_s *priv, uint32_t clk)
{
  uint32_t regval;
  volatile unsigned int timeout;

  /* Keep the function compilable while it is unused (-Werror); it is
   * retained for the future 50MHz transfer-clock path.
   */

  UNUSED(regval);
  UNUSED(timeout);

  /* Mirror mmcdiag's clock_update(): keep CARD_CLOCK_ON set while the
   * clock update runs.  Clearing it first (the previous version) left the
   * controller wedged after the UPCLK trigger.
   */

  regval = h616_getreg(priv, H616_SDXC_CLKCR);
  regval |= H616_SDXC_MASK_DATA0;
  regval |= H616_SDXC_CARD_CLOCK_ON;
  h616_putreg(priv, H616_SDXC_CLKCR, regval);

  /* Set the divider to divide-by-1
   */

  regval = h616_getreg(priv, H616_SDXC_CLKCR);
  regval &= ~0xff;
  regval |= 0;
  h616_putreg(priv, H616_SDXC_CLKCR, regval);

  /* Trigger the clock update (UPCLK_ONLY command) and wait for it to
   * complete.
   */

  h616_putreg(priv, H616_SDXC_CMDR,
              H616_SDXC_START | H616_SDXC_UPCLK_ONLY |
              H616_SDXC_WAIT_PRE_OVER);

  timeout = 100000;
  while ((h616_getreg(priv, H616_SDXC_CMDR) & H616_SDXC_START) != 0)
    {
      if (--timeout == 0)
        {
          _err("ERROR: h616_setclock update timeout\n");
          break;
        }
    }

  /* Unmask DAT0, enable the card clock
   */

  regval = h616_getreg(priv, H616_SDXC_CLKCR);
  regval &= ~H616_SDXC_MASK_DATA0;
  regval |= H616_SDXC_CARD_CLOCK_ON;
  h616_putreg(priv, H616_SDXC_CLKCR, regval);

  priv->clock = clk;
}

/* IDMAC descriptor (must be 16-byte aligned and 4-byte aligned addresses).
 * This controller does NOT implement the CPU/PIO data path: with no IDMAC
 * descriptor programmed the data phase of a DATA_EXPECT command never starts
 * (RINTR stays 0), while the IDMAC is the path both Tina Linux
 * (sunxi_mmc_start_dma) and Tina U-Boot use.
 */

/* One IDMAC descriptor may not cover an arbitrary large buffer: a single
 * 32KB descriptor stalled mid transfer on this board (18 sectors in, then
 * the FIFO sat full with RX_DATA_REQUEST latched while the IDMAC stopped
 * fetching).  Linux sunxi_mmc caps every descriptor at one page and chains
 * FD..LD, which is the known-good recipe, so the same layout is used here:
 * 4096-byte chunks, every descriptor CH|OWN, first FD, last LD|ER.
 *
 * 512 descriptors x 4096 bytes = 2 MiB per transfer.  Larger multiblock
 * writes pipeline better inside the eMMC (the card keeps programming while
 * it receives), which raised sequential write throughput from 43 to 51 MB/s
 * on this board; the gain saturates there because the controller DATA_OVER
 * for writes only fires after the card releases DAT0 busy.
 */

#define H616_IDMA_CHUNK          4096u
#define H616_IDMA_NDESC          512
#define H616_IDMA_MAX_XFER       (H616_IDMA_CHUNK * H616_IDMA_NDESC)

static struct h616_idma_desc_s g_idma_desc[H616_IDMA_NDESC]
  __attribute__((aligned(16)));

/* Program the IDMAC for one block transfer of nbytes into/out of buffer.
 * Mirrors Linux sunxi_mmc_start_dma(): descriptor -> FIFO reset -> DMA
 * interface reset -> DMAC soft reset -> GCTRL.DMA_ENABLE -> IDIE (reads) ->
 * DMAC = FIX_BURST | IDMA_ON.
 *
 * PROVEN ORDER (2026-10-05, bisected live at the U-Boot prompt on this
 *   board):
 *   1. DLBA MUST be written AFTER the DMAC soft reset (0x80 bit0).  If DLBA
 *      is
 *      written before the reset, the reset clears it and the IDMAC silently
 *      falls back to CHDA = 0x40000000 (a stale U-Boot value) and DMAs to
 *      that address instead -- the descriptor at DLBA is never read.
 *      Measured on hardware: with DLBA programmed before DMAC-on, CHDA read
 *      back 0x40000000; written after, CHDA = DLBA.
 *   2. GCTRL must select IDMAC access (ACCESS_BY_AHB *clear*) and have
 *      DMA_ENABLE set BEFORE CMDR is issued, otherwise the data phase never
 *      starts (CBCR stays 0, RINTR never sees DATA_OVER).
 *   3. DLBA is expressed in WORD units on H616 (descriptor address >> 2),
 *      matching U-Boot (sun50iw9) and Tina's des_addr_shift = 2.
 */

/* IDMAC programming, used when g_rx_use_ahb is cleared.  The notes below
 * record what was bisected live at the U-Boot prompt on 2026-10-05, where a
 * raw CMD17 did complete through this path (512 bytes landed intact with
 * CBCR = 0x200) -- the driver's own attempt has never reached a data phase,
 * which is exactly what the run time probe is meant to settle.
 */

static void h616_start_dma(struct h616_sdhci_priv_s *priv, uintptr_t buffer,
                           size_t nbytes, bool write)
{
  struct h616_idma_desc_s *des = g_idma_desc;
  uint32_t left = (uint32_t)nbytes;
  uint32_t off  = 0;
  int ndes = 0;
  uint32_t regval;
  volatile int t;

  /* Chained descriptors, Linux sunxi_mmc style: every descriptor covers at
   * most one 4096-byte chunk, is marked CH|OWN, the first one FD, the last
   * one LD|ER.  A single 32KB descriptor was proven bad on this board: the
   * IDMAC stalled ~18 sectors in with the FIFO full and RX_DATA_REQUEST
   * latched (eventwait TIMEOUT, CBCR=0x2400), so chunking is not optional.
   * Addresses (addr and next) are in word units, like DLBA.
   */

  while (left > 0)
    {
      uint32_t chunk = left > H616_IDMA_CHUNK ? H616_IDMA_CHUNK : left;

      des[ndes].config = H616_IDMAC_DES0_CH | H616_IDMAC_DES0_OWN |
                         H616_IDMAC_DES0_DIC;
      des[ndes].size   = chunk;
      des[ndes].addr   = (uint32_t)(buffer + off) >> 2;
      des[ndes].next   = (uint32_t)(uintptr_t)&des[ndes + 1] >> 2;
      left -= chunk;
      off  += chunk;
      ndes++;
    }

  des[0].config        |= H616_IDMAC_DES0_FD;
  des[ndes - 1].config |= H616_IDMAC_DES0_LD | H616_IDMAC_DES0_ER;
  des[ndes - 1].next    = (uint32_t)(uintptr_t)&des[0] >> 2;

  /* The CPU writes the descriptor into the data cache, but the IDMAC reads
   * it straight from DRAM.  With CONFIG_ARCH_DCACHE=y and no MMU the
   * descriptor would otherwise never reach memory and the IDMAC parks
   * waiting for it (IDST = 0x2000, data phase never starts).  Same
   * reasoning for the payload of a write: clean it out before the DMA
   * reads it.
   */

  up_clean_dcache((uintptr_t)des,
                  (uintptr_t)des + ndes * sizeof(*des));

  if (write)
    {
      up_clean_dcache(buffer, buffer + nbytes);
    }

  /* GCTRL: DMA access path + DMA enable + DMA reset, clearing the PIO/AHB
   * selection and the interrupt-enable bit.  This MUST be in place before
   * CMDR (see note 2 above).  The exact value proven on hardware at the
   * U-Boot prompt for a successful 512-byte IDMAC read was 0x20000024
   * (ACCESS_BY_DMA | DMA_ENABLE | DMA_RESET, bit4 clear), so reproduce that
   * rather than Linux's simpler DMA_ENABLE-only write.
   */

  regval  = h616_getreg(priv, H616_SDXC_GCTRL);
  regval &= ~(H616_SDXC_ACCESS_BY_AHB | H616_SDXC_INTERRUPT_ENABLE);
  regval |= H616_SDXC_DMA_ENABLE | H616_SDXC_DMA_RESET;

  /* Zero the byte counter across the DMA-enable write, then restore it.
   * This mirrors the only data transfer ever observed to complete on this
   * board (a raw CMD17 issued from the U-Boot prompt: 512 bytes landed
   * intact with CBCR = 0x200).  In that sequence GCTRL was written while
   * BCNTR was still 0 and the byte count was programmed afterwards; issuing
   * the DMA-enable write with a non-zero BCNTR instead makes the CIU take a
   * timeout and the card transfers nothing (CBCR/BBCR stay 0).
   */

  h616_putreg(priv, H616_SDXC_BCNTR, 0);
  h616_putreg(priv, H616_SDXC_GCTRL, regval);
  h616_putreg(priv, H616_SDXC_BCNTR, (uint32_t)nbytes);

  /* IDMAC soft reset, then IDMA on.  The soft reset clears DMAC and any
   * previously latched descriptor base, so DLBA is written afterwards.
   *
   */

  h616_putreg(priv, H616_SDXC_IDST, 0xffffffff);
  h616_putreg(priv, H616_SDXC_DMAC, H616_SDXC_IDMAC_SOFT_RESET);
  t = 100000;
  while ((h616_getreg(priv, H616_SDXC_DMAC) &
          H616_SDXC_IDMAC_SOFT_RESET) && --t);

  h616_putreg(priv, H616_SDXC_DMAC,
              H616_SDXC_IDMAC_FIX_BURST | H616_SDXC_IDMAC_IDMA_ON);

  if (!write)
    {
      h616_putreg(priv, H616_SDXC_IDIE, H616_SDXC_IDMAC_RECEIVE_INTERRUPT);
    }
  else
    {
      h616_putreg(priv, H616_SDXC_IDIE, H616_SDXC_IDMAC_TRANSMIT_INTERRUPT);
    }

  /* DLBA LAST -- after the DMAC soft reset.  The IDMAC only latches the
   * descriptor base here; writing it before the reset is a silent no-op
   * (proven on hardware, see note 1 above).
   */

  h616_putreg(priv, H616_SDXC_DLBA, (uint32_t)(uintptr_t)des >> 2);

  /* Make sure the descriptor is visible before the IDMAC walks it
   */

  __asm__ __volatile__("dsb sy" ::: "memory");

  H616_DIAG("DIAG: start_dma buf=%08" PRIx32 " nbytes=%u %s DLBA=%08" PRIx32
        " DMAC=%08" PRIx32 " GCTRL=%08" PRIx32 "\n",
        (uint32_t)buffer, (unsigned int)nbytes, write ? "WR" : "RD",
        h616_getreg(priv, H616_SDXC_DLBA),
        h616_getreg(priv, H616_SDXC_DMAC),
        h616_getreg(priv, H616_SDXC_GCTRL));
}

/* Set the data bus width
 */

static void h616_setbuswidth(struct h616_sdhci_priv_s *priv, int width)
{
  uint32_t w = H616_SDXC_WIDTH1;

  switch (width)
    {
    case 1:
      w = H616_SDXC_WIDTH1;
      break;
    case 4:
      w = H616_SDXC_WIDTH4;
      break;
    case 8:
      w = H616_SDXC_WIDTH8;
      break;
    }

  /* Do NOT force 8-bit here.
   *
   * The host and the CARD must agree on the bus width, and after the card
   * hardware reset (HWRST) in h616_reset() the card is back at its 1-bit
   * power-on default.  Forcing the host to 8-bit before the card has been
   * switched made the FIRST data command -- CMD8 SEND_EXT_CSD, the very read
   * that identifies the card -- run with host 8-bit vs card 1-bit, and the
   * card then drives no data at all (CBCR = 0).  That also made the mmcsd
   * layer fail to identify the card, so it never created /dev/mmcsd0.
   *
   * h616_widebus() sets the CARD to the matching width via CMD6 and then
   * calls here; keeping this function honest means both sides change
   * together.
   */

  h616_putreg(priv, H616_SDXC_WIDTH, w);
  priv->buswidth = width;
}

/* Issue a hardware command.  data = true adds the data-expect / write bits
 * and the byte-count setup (done by the caller via blocksetup).
 */

#ifdef H616_SDIO_USEIRQ
static void h616_irq_lateinit(struct h616_sdhci_priv_s *priv);
#endif

static int h616_sendcmdhw(struct h616_sdhci_priv_s *priv, uint32_t cmd,
                          uint32_t arg)
{
  uint32_t cmdidx = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  uint32_t regval;

#ifdef H616_SDIO_USEIRQ
  /* First command after boot: the GIC is up by now, so the interrupt can
   * finally be attached (h616_attach runs too early, see its comment).
   *
   */

  h616_irq_lateinit(priv);
#endif

  /* Argument
   */

  /* NOTE: CARG is written in the mmcdiag-order block below (after IMASK);
   * do not write it here so the sequence matches mmcdiag exactly.
   */

  /* Command register
   */

  regval = cmdidx | H616_SDXC_START;

  /* CMD0 (GO_IDLE) needs the init-sequence bit to properly reset the card
   * into the idle state (per mainline U-Boot sunxi_mmc.c).
   */

  if (cmdidx == 0)
    {
      regval |= H616_SDXC_SEND_INIT_SEQUENCE;
    }

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
    case MMCSD_R7_RESPONSE:
      regval |= H616_SDXC_RESP_EXPECT | H616_SDXC_CHECK_RESPONSE_CRC;
      break;

    case MMCSD_R2_RESPONSE:
      regval |= H616_SDXC_RESP_EXPECT | H616_SDXC_LONG_RESPONSE |
                H616_SDXC_CHECK_RESPONSE_CRC;
      break;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R4_RESPONSE:
      regval |= H616_SDXC_RESP_EXPECT;
      break;

    default:
      return -EINVAL;
    }

  /* Set data direction bits based on the command's data transfer mask.
   * U-Boot additionally sets SEND_AUTO_STOP for multi-block transfers
   * (mmc_trans_data_by_dma's AUTO_STOP); the IDMAC state machine needs the
   * CIU to terminate the transfer for the descriptor to close, otherwise it
   * parks in *_REQUEST_WAIT and the data phase never reaches DATA_OVER.
   *
   */

  if ((cmd & MMCSD_DATAXFR_MASK) != MMCSD_NODATAXFR)
    {
      regval |= H616_SDXC_DATA_EXPECT | H616_SDXC_WAIT_PRE_OVER;

      /* N66: do NOT arm the controller's auto CMD12 for a multi block
       * transfer.  mmcsd has already handed the block count to the card with
       * CMD23 (mmcsd_sdio.c mmcsd_setblockcount) before CMD25, so the card
       * terminates the transfer on its own; the controller only has to
       * report the end of the data phase with DATA_OVER.
       *
       * Arming SEND_AUTO_STOP on top of that is actively harmful here: the
       * card, having already self-terminated, sees the CIU's CMD12 while in
       * tran state and answers R1 = 0x00400900 (ILLEGAL_COMMAND).  Worse,
       * AUTO_COMMAND_DONE latches when the CMD12 is ISSUED -- measurably
       * before its response is captured (N62: RESP0 still held the CMD25's
       * clean 0x00000900 at that moment) -- so waiting for the bit and then
       * clearing RINTR leaves the CMD12 response completion to latch
       * COMMAND_DONE into RINTR a moment later.  The next real command's
       * h616_waitresponse() then inherits that stale COMMAND_DONE and reads
       * the CMD12's ILLEGAL response out of RESP0 (N65: cmd=13 resp=00400900
       * with RINTR=0), and mmcsd aborts the next write with "Card not ready"
       * (-EIO) after a few blocks.  The extra diagnostic CMD13 is what made
       * the N62 test pass -- it consumed the stale completion -- but that is
       * papering over the real problem.
       *
       * With CMD23 + no auto CMD12 the transfer ends exactly like the single
       * block CMD24 path, which has never shown this failure: DATA_OVER,
       * card drops into prg, DAT0 released when programming is done (the
       * post-transfer CARD_DATA_BUSY wait below covers that), and the next
       * command gets a clean response.
       */

      if ((cmd & MMCSD_WRXFR) != 0)
        {
          regval |= H616_SDXC_WRITE;
        }
    }

  /* Clear any stale interrupt/response state before issuing the command
   * (mmcdiag proved RESP0 latches correctly only when RINTR is cleared
   * first -- otherwise a stale COMMAND_DONE makes waitresponse return
   * immediately and RESP0 holds the PREVIOUS command's value).
   */

  /* DIAG: command issue order matches mmcdiag's driver-style send_cmd:
   * CARG -> RINTR -> CMDR via raw volatile pointers with no putreg32/DMB.
   *
   * NOTE: IMASK is deliberately NOT written here any more.  A blanket RMW
   * used to clear every data bit in IMASK, which silently disabled the data
   * path configured by h616_recvsetup/h616_sendsetup (their enables live in
   * the same register), so DATA_OVER/RX_DATA_REQUEST never reached the ISR
   * and the first block read (CMD8/CMD17) hung the boot with no NSH prompt.
   * h616_waitresponse polls the RAW status register RINTR, so it needs no
   * IMASK bits of its own.
   */

  /* Never mask DATA0 here.  N45 tried masking it on command-only transfers
   * (believing the card's DAT0 busy phase was mangling CMD13); U-Boot, the
   * one
   * implementation known to write on this board, never does that, and the
   * experiment changed nothing -- the ILLEGAL_COMMAND that follows a multi
   * block write has another cause (see h616_eventwait and
   * MMCSD_NO_MULTIBLOCK_WRITE in drivers/mmcsd/Kconfig).
   */

  *(volatile uint32_t *)(priv->base + H616_SDXC_CARG) = arg;
  *(volatile uint32_t *)(priv->base + H616_SDXC_RINTR) = 0xffffffff;

#ifdef H616_SDIO_USEIRQ
  /* Arm the command-class sources so the ISR can wake a sleeping waiter.
   * A careful RMW that only sets these bits: the data bits configured by
   * h616_recvsetup/h616_sendsetup live in the same register.
   */

  priv->cmdrintr = 0;
  h616_modifyreg(priv, H616_SDXC_IMASK, 0, H616_SDIO_CMDBITS);
#endif

  /* N52 safety net: a data FIFO still holding unread payload wedges the CIU
   * -- the command is written but never starts (CMDR reads back with START
   * set, no COMMAND_DONE, no RESP_TIMEOUT).  Both data paths now reset the
   * FIFO at completion (see h616_eventwait), but any future path that
   * leaves residue gets caught here instead of hanging the whole transfer
   * chain.  Bounded log so a recurring residue shows up as a count, not a
   * flood.
   */

  if ((*(volatile uint32_t *)(priv->base + H616_SDXC_STAS) &
       H616_SDXC_FIFO_EMPTY) == 0)
    {
      static int fifo_rescue_n = 0;

      if (fifo_rescue_n < 24)
        {
          fifo_rescue_n++;
          _err("SDIO: FIFO not empty before cmd=%08" PRIx32
               " STAS=%08" PRIx32 " (rescue #%d)\n",
               cmd, *(volatile uint32_t *)(priv->base + H616_SDXC_STAS),
               fifo_rescue_n);
        }

      h616_modifyreg(priv, H616_SDXC_GCTRL, 0, H616_SDXC_FIFO_RESET);
      h616_modifyreg(priv, H616_SDXC_GCTRL, H616_SDXC_FIFO_RESET, 0);
    }

  *(volatile uint32_t *)(priv->base + H616_SDXC_CMDR) = regval;

  /* DIAG: poll BOTH MISTA and RINTR (the command's START bit clears when
   * the controller accepts/executes it; COMMAND_DONE should appear in
   * RINTR).  Report whichever fires first so we know whether the command
   * executes and where the completion signal goes.
   */

  {
    volatile int t = 5000000;
    uint32_t mista = 0;
    uint32_t rintr = 0;
    while (t--)
      {
        mista = *(volatile uint32_t *)(priv->base + H616_SDXC_MISTA);
        rintr = *(volatile uint32_t *)(priv->base + H616_SDXC_RINTR);
#ifdef H616_SDIO_USEIRQ
        rintr |= priv->cmdrintr;
#endif
        if ((mista | rintr) & H616_SDXC_COMMAND_DONE)
          {
            break;
          }

#ifdef H616_SDIO_USEIRQ
        /* After a short hot spin, sleep on the command semaphore: the ISR
         * posts it the moment COMMAND_DONE fires.  Raw polling continues
         * after every wakeup, so a lost interrupt only costs latency.
         */

        if (t < 5000000 - 20000)
          {
            nxsem_tickwait(&priv->cmdsem, 1);
            t -= H616_SDIO_POLLS_PER_TICK;
            if (t < 0)
              {
                t = 0;
              }
          }
#endif
      }

    if (t <= 0)
      {
        _err("SDIO: sendcmdhw poll timeout cmd=%08" PRIx32
             " MISTA=%08" PRIx32 " RINTR=%08" PRIx32 " CMDR=%08" PRIx32
             " STAS=%08" PRIx32 " PIO0=%08" PRIx32 " PIO1=%08" PRIx32
             " PIO2=%08" PRIx32 "\n", cmd, mista, rintr,
             *(volatile uint32_t *)(priv->base + H616_SDXC_CMDR),
             *(volatile uint32_t *)(priv->base + H616_SDXC_STAS),
             getreg32(H616_PIO_ADDR + 0x48),
             getreg32(H616_PIO_ADDR + 0x4c),
             getreg32(H616_PIO_ADDR + 0x50));
      }
    else
      {
        H616_DIAG("SDIO: sendcmdhw COMMAND_DONE cmd=%08" PRIx32
                  " MISTA=%08" PRIx32 " RINTR=%08" PRIx32
                  " CMDR=%08" PRIx32 "\n",
              cmd, mista, rintr,
              *(volatile uint32_t *)(priv->base + H616_SDXC_CMDR));
      }
  }

  return OK;
}

/* Receive data from the FIFO.  sunxi-mmc FIFO is 16 words (64 bytes) at
 * offset 0x200.  RXWL_FLAG (STAS bit 0) indicates the RX FIFO has data to
 * the watermark.  We drain whatever is available.
 */

static void h616_recvfifo(struct h616_sdhci_priv_s *priv)
{
  unsigned int in_fifo;
  uint32_t stas;

  /* U-Boot's mmc_trans_data_by_cpu drain recipe, adapted to be non-blocking
   * (this is called from the eventwait poll loop, which owns the timeout):
   *
   *   stas   = rd(STAS)
   *   in_fifo= (stas >> 17) & 0x3fff        <- FIFO level is bits 30:17
   *   if (in_fifo == 0 && (stas & FIFO_FULL)) in_fifo = 32
   *   while (in_fifo-- && enough_left) *buff++ = rd(FIFO)
   *
   * The previous version had the FIFO_EMPTY test INVERTED -- it read only
   * when the FIFO was *not* empty, which blocks forever on the first call --
   * and never looked at the level field at all.  This is the one read path
   * mainline U-Boot uses, and mainline U-Boot is the only code observed to
   * read successfully on this board.
   */

  stas = h616_getreg(priv, H616_SDXC_STAS);

  if ((stas & H616_SDXC_FIFO_EMPTY) != 0)
    {
      return;                                  /* nothing to take yet */
    }

  in_fifo = (stas >> 17) & 0x3fff;
  if (in_fifo == 0 && (stas & H616_SDXC_FIFO_FULL))
    {
      in_fifo = 32;
    }

  while (in_fifo-- > 0 && priv->remaining >= sizeof(uint32_t))
    {
      uint32_t w = h616_getreg(priv, H616_SDXC_FIFO);
      memcpy(priv->buffer, &w, sizeof(uint32_t));
      priv->buffer    += sizeof(uint32_t);
      priv->remaining -= sizeof(uint32_t);
    }
}

/* Send data to the FIFO.  TXWL_FLAG (STAS bit 1) indicates the TX FIFO is
 * below the watermark and can accept more data.
 */

static void h616_sendfifo(struct h616_sdhci_priv_s *priv)
{
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } data;

  /* U-Boot's write side: block while the FIFO is FULL (STAS bit3).
   */

  while (priv->remaining > 0 &&
         (h616_getreg(priv, H616_SDXC_STAS) & H616_SDXC_FIFO_FULL) == 0)
    {
      if (priv->remaining >= sizeof(uint32_t))
        {
          data.w = *((uint32_t *)priv->buffer);
          priv->buffer += sizeof(uint32_t);
          priv->remaining -= sizeof(uint32_t);
        }
      else
        {
          uint8_t *ptr = (uint8_t *)priv->buffer;
          int i;

          data.w = 0;
          for (i = 0; i < (int)priv->remaining; i++)
            {
              data.b[i] = *ptr++;
            }

          priv->remaining = 0;
        }

      h616_putreg(priv, H616_SDXC_FIFO, data.w);
    }
}

/* Enable/disable the transfer-related interrupt mask
 */

static void h616_configxfrints(struct h616_sdhci_priv_s *priv,
                               uint32_t enxfr)
{
  uint32_t regval;

  regval = h616_getreg(priv, H616_SDXC_IMASK);
  regval &= ~(H616_SDXC_DATA_OVER | H616_SDXC_TX_DATA_REQUEST |
              H616_SDXC_RX_DATA_REQUEST | H616_SDXC_DATA_CRC_ERROR |
              H616_SDXC_DATA_TIMEOUT | H616_SDXC_FIFO_RUN_ERROR);
  regval |= enxfr;
  h616_putreg(priv, H616_SDXC_IMASK, regval);
  priv->xfrmask = enxfr;

#ifdef H616_SDIO_USEIRQ
  /* The boot-time ladder and DDR52 bring-up re-write GCTRL (soft reset,
   * AHB/DMA toggle, restore of a pre-ladder snapshot) and drop the global
   * interrupt-enable bit, which silences the GIC line even though MISTA
   * asserts.  Re-assert it at every interrupt-arming point so the line
   * is guaranteed live by the time transfers run.
   */

  h616_modifyreg(priv, H616_SDXC_GCTRL, 0, H616_SDXC_INTERRUPT_ENABLE);
#endif
}

/* Configure the wait-for-event interrupt mask
 */

static void h616_configwaitints(struct h616_sdhci_priv_s *priv,
                                uint32_t waitints,
                                sdio_eventset_t wkupevents)
{
  uint32_t regval;

  regval = h616_getreg(priv, H616_SDXC_IMASK);
  regval &= ~(H616_SDXC_COMMAND_DONE | H616_SDXC_RESP_ERROR |
              H616_SDXC_RESP_CRC_ERROR | H616_SDXC_RESP_TIMEOUT |
              H616_SDXC_DATA_OVER | H616_SDXC_DATA_CRC_ERROR |
              H616_SDXC_DATA_TIMEOUT | H616_SDXC_FIFO_RUN_ERROR |
              H616_SDXC_HARD_WARE_LOCKED | H616_SDXC_START_BIT_ERROR |
              H616_SDXC_END_BIT_ERROR);
  regval |= waitints;
  h616_putreg(priv, H616_SDXC_IMASK, regval);

  priv->waitevents = waitints;
  priv->wkupevent  = wkupevents;
}

/****************************************************************************
 * SDIO Interface Implementation
 ***************************************************************************
 */

/* Locking: the mmcsd layer serializes slot access on its own mutex, so no
 * driver-level lock is needed (CONFIG_SDIO_MUXBUS is not enabled).
 */

/* Reset the controller to a known state
 */

/* DIAG helper: issue a raw CMD0 and report whether COMMAND_DONE latches.
 * Used to bisect which step of the reset/clock sequence kills the
 * controller (it is provably alive at handover).
 */

static void h616_probe_cmd0(struct h616_sdhci_priv_s *priv, const char *tag)
{
  volatile unsigned int timeout = 200000;
  uint32_t rintr;

  h616_putreg(priv, H616_SDXC_RINTR, 0xffffffff);
  h616_putreg(priv, H616_SDXC_CARG, 0xf0f0f0f0);
  h616_putreg(priv, H616_SDXC_CMDR,
              H616_SDXC_SEND_INIT_SEQUENCE | H616_SDXC_START);

  while (timeout-- != 0 &&
         (h616_getreg(priv, H616_SDXC_RINTR) & H616_SDXC_COMMAND_DONE) == 0)
    {
    }

  rintr = h616_getreg(priv, H616_SDXC_RINTR);
  H616_DIAG("DIAG: probe[%s] STAS=%08" PRIx32 " RINTR=%08" PRIx32
        " CLKCR=%08" PRIx32 " CCU838=%08" PRIx32 " (%s)\n",
        tag, h616_getreg(priv, H616_SDXC_STAS), rintr,
        h616_getreg(priv, H616_SDXC_CLKCR),
        getreg32(H616_CCU_ADDR + H616_CCU_SD2_CLK_CFG),
        (rintr & H616_SDXC_COMMAND_DONE) != 0 ? "OK" : "DEAD");
}

static void h616_reset(struct sdio_dev_s *dev)
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;
  uint32_t regval;

  /* Record U-Boot's controller state before anything here modifies it.
   * h400d7: the boot_package U-Boot read boot.img from this same card at
   * HS400@100MHz moments earlier, so these are the registers of a KNOWN
   * WORKING HS400 configuration.  The ladder later re-derives everything
   * from scratch and has failed to reproduce HS400 at 100MHz (36/36 cells
   * with CBCR=0), so replicate this state instead of guessing at it.
   */

    {
      static bool dumped;

      if (!dumped)
        {
          dumped = true;
          _info("handoff: GCTRL=%08" PRIx32 " CLKCR=%08" PRIx32
                " WIDTH=%08" PRIx32 " TMOUT=%08" PRIx32 "\n",
                h616_getreg(priv, H616_SDXC_GCTRL),
                h616_getreg(priv, H616_SDXC_CLKCR),
                h616_getreg(priv, H616_SDXC_WIDTH),
                h616_getreg(priv, H616_SDXC_TMOUT));
          _info("handoff: FTRGL=%08" PRIx32 " THLDC=%08" PRIx32
                " NTSR=%08" PRIx32 " CSDC=%08" PRIx32 "\n",
                h616_getreg(priv, H616_SDXC_FTRGL),
                h616_getreg(priv, H616_SDXC_THLDC),
                h616_getreg(priv, H616_SDXC_NTSR),
                h616_getreg(priv, H616_SDXC_CSDC));
          _info("handoff: SAMP_DL=%08" PRIx32 " DS_DL=%08" PRIx32
                " DRV_DL=%08" PRIx32 " DSBD=%08" PRIx32 "\n",
                h616_getreg(priv, H616_SDXC_SAMP_DL),
                h616_getreg(priv, H616_SDXC_DS_DL),
                h616_getreg(priv, H616_SDXC_DRV_DL),
                h616_getreg(priv, H616_SDXC_DSBD));
        }
    }

  /* DIAG: probe the controller the moment the driver takes it over, BEFORE
   * any reset of our own.  U-Boot has just used this same eMMC controller
   * successfully, so a raw CMD0 here must latch COMMAND_DONE if the handover
   * state is good.  This distinguishes "we broke it" from "it arrived
   * broken".
   */

  h616_probe_cmd0(priv, "takeover");

  /* Controller reset, reproduced from the sequence measured working on this
   * board (mmcdiag).  The order is load-bearing and was isolated step by
   * step; the measured facts are:
   *
   *   GCTRL reset -> CCU clock write -> UPCLK      => controller ALIVE
   *   GCTRL reset -> [any command]   -> CCU -> UPCLK => controller WEDGED
   *
   * i.e. NOTHING may be issued between the software reset and the clock
   * update.  In particular a diagnostic CMD0 there (which this driver used
   * to do) wedges it, and the UPCLK then never completes (CMDR reads back
   * 80202000 with START still set).  That was the real cause of the long
   * standing "reset kills the controller" behaviour.
   *
   * So: no probe/probe_cmd0 between these statements.
   */

  regval = H616_SDXC_SOFT_RESET | H616_SDXC_FIFO_RESET |
           H616_SDXC_DMA_RESET;
  h616_putreg(priv, H616_SDXC_GCTRL, regval);
    {
      volatile int t = 100000;
      while ((h616_getreg(priv, H616_SDXC_GCTRL) &
              (H616_SDXC_SOFT_RESET | H616_SDXC_FIFO_RESET |
               H616_SDXC_DMA_RESET)) != 0 && --t != 0);
    }

  h616_putreg(priv, H616_SDXC_GCTRL, H616_SDXC_INTERRUPT_ENABLE);
  h616_putreg(priv, H616_SDXC_WIDTH, H616_SDXC_WIDTH1);
  h616_putreg(priv, H616_SDXC_RINTR, 0xffffffff);

  /* CCU module clock (set_ccu_clock in mmcdiag), then the controller clock
   * update.  Both must follow the reset with no intervening command.
   */

  h616_clockenable(priv, H616_SDMMC_IDCLK);
  h616_setclock(priv, H616_SDMMC_IDCLK);

  /* Now the CARD hardware reset -- the piece U-Boot has and mmcdiag does
   * not.  It is what un-wedges an eMMC that a failed data command left
   * stuck (otherwise every later access reports CBCR=0 for the whole boot).
   * HWRST is active low.
   */

  h616_putreg(priv, H616_SDXC_HWRST, 0);                     /* assert   */
  up_udelay(10);
  h616_putreg(priv, H616_SDXC_HWRST, H616_SDXC_CARD_RESET);  /* deassert */
  up_udelay(300);

  /* The card is now back in its default speed mode (1-bit, HS_TIMING = 0),
   * so the controller has to stop sampling like an HS400 bus.  U-Boot boots
   * this eMMC in HS400 (100MHz with data strobe sampling) and leaves its
   * HS400
   * switch behind in the controller: DSBD bit31 = 1 and CSDC[3:0] = 6 (see
   * mmc_hs400_mode_onoff in sunxi_mmc.c).  Those two fields are NOT reset by
   * the CIU software reset above (only DSBD bit31 reads back sticky).
   *
   * Measured with mmcdiag u39 on this board: after the card HWRST with the
   * two
   * fields left set, EVERY data command times out with CBCR = 0 and an empty
   * FIFO (not one byte is transferred, the CIU reports DATA_TIMEOUT), while
   * clearing exactly these fields makes the same CMD17 return all 512 bytes
   * byte-exactly at 400kHz and at 25MHz.  This was the long-standing "the
   * driver can never read eMMC" failure.
   */

  h616_modifyreg(priv, H616_SDXC_DSBD, H616_SDXC_DSBD_HS400, 0);
  h616_modifyreg(priv, H616_SDXC_CSDC, H616_SDXC_CSDC_STROBE_MASK, 0);
  h616_putreg(priv, H616_SDXC_NTSR, 0);
  h616_modifyreg(priv, H616_SDXC_GCTRL, H616_SDXC_DDR_MODE, 0);

  h616_probe_cmd0(priv, "after-reset+hwrst");

  /* Program the host-side registers U-Boot expects.  These were previously
   * set AHB/PIO here, which is wrong: the data path on this controller is
   * the IDMAC only (see h616_start_dma).
   */

  h616_modifyreg(priv, H616_SDXC_GCTRL,
                 H616_SDXC_ACCESS_BY_AHB | H616_SDXC_INTERRUPT_ENABLE, 0);

  /* TMOUT as U-Boot programs it:
   * (data timeout 0xffffff << 8) | response timeout 0xff = 0xffffffff.
   */

  h616_putreg(priv, H616_SDXC_TMOUT, 0xffffffff);

  /* FTRGL for sdc2 (host 2) is 0x300f00f0 per both U-Boot and Tina Linux
   * (sunxi_mmc_thld_ctl_for_sdmmc2 / host_no == 2 branch).
   */

  h616_putreg(priv, H616_SDXC_FTRGL, 0x300f00f0);

  /* THLDC (card threshold control) MUST be enabled for the data phase to
   * work on H616 -- U-Boot sets it as part of the very same reset sequence.
   *   mainline:  READ_THLD(512) | WRITE_EN | READ_EN = 0x02000005
   *   Tina:      (512 << 16) | (1 << 2) | (1 << 0)
   */

  h616_putreg(priv, H616_SDXC_THLDC, 0x02000005);

  /* WIDTH: the eMMC bus width is a PERSISTENT card setting (EXT_CSD byte
   * 183).  Following the hardware reset above the card comes back in its
   * power-on default (1-bit), so start there and let h616_setbuswidth()
   * raise it once the card has been re-identified.
   */

  h616_putreg(priv, H616_SDXC_WIDTH, H616_SDXC_WIDTH1);

  /* The card is back at 1 bit / default speed, so the ladder has to run
   * again on the next bring-up.
   */

  priv->ladderdone = false;
  priv->ddr = false;
  priv->bus_mod_hz = 0;
  priv->bus_clkdiv = 0;

  h616_probe_cmd0(priv, "after-restore");
}

/* Ladder scratch buffers: 512 bytes each, init time only.  g_ladder_ref
 * holds the block read in the known good 1 bit configuration, g_ladder_buf
 * the block read in the configuration currently being tested.
 */

/* Order matters: h616_eventwait() invalidates [buffer+total, buffer+2*total)
 * (a stale IDMAC path leftover), i.e. the 512 bytes immediately AFTER
 * g_ladder_buf, so the reference copies must not be allocated there.
 */

static uint32_t g_ladder_ref[128];
static uint32_t g_ladder_ref2[128];
static uint32_t g_ladder_buf[128];
static uint32_t g_ladder_scratch[128];

/* g_sdio_diag_muted is declared near the top of the file, together with the
 * H616_DIAG() macro that uses it.
 */

/* Register snapshot taken before the ladder changes anything.  This board's
 * known good 1 bit setup depends on the CIU reset values of the delay
 * registers (SAMPDL=0x1898, DS_DL=0x0d8d), so a rollback has to put back
 * exactly what was there instead of hardcoding "delays off".
 */

static struct h616_ladder_save_s
{
  uint32_t gctrl;
  uint32_t clkcr;
  uint32_t width;
  uint32_t drvdl;
  uint32_t sampdl;
  uint32_t dsdl;
  uint32_t csdc;
  uint32_t dsbd;
  uint32_t ntsr;
  uint32_t ftrgl;
  uint32_t thldc;
  uint32_t tmout;
} g_ladder_save;

/* Unjam the CIU after a failed or timed out data transfer.
 *
 * Port of the Linux sunxi-mmc retry sequence (sunxi_mmc_handle_bottom_half,
 * mrq_retry path).  At HS400 a failed read (h400d4b boot log:
 * START_BIT_ERROR
 * on the first CMD17 at the 100MHz card clock) leaves the CIU refusing every
 * later command -- the next CMD17 is written with START set and never
 * starts,
 * and the delay sweep dies at its first failing cell.  Linux recovers with:
 * raw CMD12 stop, wait CARD_DATA_BUSY clear, save the IP registers, toggle
 * the CCU module gate+reset off/on (full IP reset), restore the registers,
 * then re-latch the clock with an UPCLK-only command.
 *
 * This runs for every failed data transfer, from the ladder's verification
 * reads and from mmcsd's runtime transfers, so a runtime data error cannot
 * leave the controller wedged until the next power cycle.
 */

/* Abort a data phase with CMD12 and ignore the response.  Used by the
 * recovery path and by the HS400 exit sequence, which must not reset the
 * controller while the card is still in HS400 (a reset stops the card
 * clock, and an HS400 card that loses its clock stops answering).
 */

static void h616_cmd12_abort(struct h616_sdhci_priv_s *priv)
{
  int t = 1000000;

  h616_putreg(priv, H616_SDXC_CARG, 0);
  h616_putreg(priv, H616_SDXC_RINTR, 0xffffffff);
  h616_putreg(priv, H616_SDXC_CMDR,
              H616_SDXC_START | H616_SDXC_STOP_ABORT_CMD | 12);

  while (t-- > 0 &&
         (h616_getreg(priv, H616_SDXC_RINTR) &
          (H616_SDXC_COMMAND_DONE | H616_SDXC_INTERRUPT_ERROR_BIT)) == 0)
    ;

  h616_putreg(priv, H616_SDXC_RINTR, 0xffffffff);
}

static void h616_data_recover(struct h616_sdhci_priv_s *priv)
{
  /* 1. raw CMD12 so the card leaves its data state.  The CIU still accepts
   * commands right after the failed transfer -- it is the NEXT command that
   * wedges.
   */

  h616_cmd12_abort(priv);

  /* 2. wait for the card to release the data lines */

    {
      volatile int t = 2000000;
      while (--t > 0 &&
             (h616_getreg(priv, H616_SDXC_STAS) &
              H616_SDXC_CARD_DATA_BUSY) != 0)
        ;
    }

  /* 3+4. save the IP registers, toggle the CCU gate+reset (this is the
   * only reset deep enough to unjam the data FSM), restore.  Linux does
   * exactly this via sunxi_mmc_regs_save/bus_clk_en(0)/bus_clk_en(1)
   * regs_restore.
   */

    {
      uint32_t gctrl = h616_getreg(priv, H616_SDXC_GCTRL);
      uint32_t clkcr = h616_getreg(priv, H616_SDXC_CLKCR);
      uint32_t tmout = h616_getreg(priv, H616_SDXC_TMOUT);
      uint32_t width = h616_getreg(priv, H616_SDXC_WIDTH);
      uint32_t ftrgl = h616_getreg(priv, H616_SDXC_FTRGL);
      uint32_t funs  = h616_getreg(priv, H616_SDXC_FUNS);
      uint32_t dsbd  = h616_getreg(priv, H616_SDXC_DSBD);
      uint32_t drvdl = h616_getreg(priv, H616_SDXC_DRV_DL);
      uint32_t samp  = h616_getreg(priv, H616_SDXC_SAMP_DL);
      uint32_t dsdl  = h616_getreg(priv, H616_SDXC_DS_DL);
      uint32_t csdc  = h616_getreg(priv, H616_SDXC_CSDC);
      uint32_t gate  = getreg32(H616_CCU_ADDR + H616_CCU_SD_GATE_RESET);

      if (priv->hs400)
        {
          /* HS400 runs from the card's data strobe and does not tolerate the
           * card clock being stopped mid-transfer: gating the CCU module
           * clock here (the DDR52 recovery path) left the card unresponsive
           * to every later CMD12/CMD13/CMD6 (RINTR = COMMAND_DONE +
           * RESP_TIMEOUT
           * for all of them).  A CIU soft reset unjams the controller just
           * as well and keeps the card clock running, which is what the card
           * needs to finish its block and return to tran.
           */

          volatile int t = 100000;

          h616_putreg(priv, H616_SDXC_GCTRL,
                      H616_SDXC_SOFT_RESET | H616_SDXC_FIFO_RESET |
                      H616_SDXC_DMA_RESET);
          while ((h616_getreg(priv, H616_SDXC_GCTRL) &
                  (H616_SDXC_SOFT_RESET | H616_SDXC_FIFO_RESET |
                   H616_SDXC_DMA_RESET)) != 0 && --t != 0)
            ;
        }
      else
        {
          putreg32(gate & ~(H616_CCM_MMC_RST(2) | H616_CCM_MMC_GATE(2)),
                   H616_CCU_ADDR + H616_CCU_SD_GATE_RESET);
          up_udelay(20);
          putreg32(gate, H616_CCU_ADDR + H616_CCU_SD_GATE_RESET);
        }

      h616_putreg(priv, H616_SDXC_GCTRL, gctrl);
      h616_putreg(priv, H616_SDXC_CLKCR, clkcr);
      h616_putreg(priv, H616_SDXC_TMOUT, tmout);
      h616_putreg(priv, H616_SDXC_WIDTH, width);
      h616_putreg(priv, H616_SDXC_FTRGL, ftrgl);
      h616_putreg(priv, H616_SDXC_FUNS, funs);
      h616_putreg(priv, H616_SDXC_DSBD, dsbd);
      h616_putreg(priv, H616_SDXC_DRV_DL, drvdl);
      h616_putreg(priv, H616_SDXC_SAMP_DL, samp);
      h616_putreg(priv, H616_SDXC_DS_DL, dsdl);
      h616_putreg(priv, H616_SDXC_CSDC, csdc);
    }

  /* 6. re-latch the clock inside the CIU (UPCLK-only with MASK_DATA0,
   * mirroring __sunxi_mmc_do_oclk_onoff).  The soft reset above clears the
   * CIU clock state, so this is needed at HS400 too -- without it the
   * controller stops driving the card clock and every following command
   * stays at RINTR = 0 forever (measured at fallback9).
   */

    {
      uint32_t clkcr = h616_getreg(priv, H616_SDXC_CLKCR);

      h616_putreg(priv, H616_SDXC_CLKCR, clkcr | H616_SDXC_MASK_DATA0);
      h616_putreg(priv, H616_SDXC_CMDR,
                  H616_SDXC_START | H616_SDXC_UPCLK_ONLY |
                  H616_SDXC_WAIT_PRE_OVER);
        {
          volatile int t = 1000000;
          while (--t > 0 &&
                 (h616_getreg(priv, H616_SDXC_CMDR) & H616_SDXC_START) != 0)
            ;
        }

      h616_putreg(priv, H616_SDXC_CLKCR, clkcr);
      h616_putreg(priv, H616_SDXC_RINTR, 0xffffffff);
    }
}

/* Read one block through exactly the path the mmcsd layer uses and, when
 * expect is not NULL, compare it against the reference block.  Both the
 * transfer itself and the data have to be right, so a mode that samples at
 * the wrong point is caught instead of silently returning garbage.
 */

static int h616_ladder_read(struct h616_sdhci_priv_s *priv, uint32_t lba,
                            FAR const uint32_t *expect)
{
  struct sdio_dev_s *dev = &priv->dev;
  sdio_eventset_t events;
  uint32_t sum = 0;
  uint32_t changed = 0;
  int i;

  /* Poison the destination before every read.  Without this, g_ladder_buf
   * still holds the *reference* read of the very same block, so a read that
   * transferred nothing -- or one that was served out of a stale FIFO --
   * still compares equal and the mode is accepted on evidence that does not
   * exist.
   */

  for (i = 0; i < 128; i++)
    {
      g_ladder_buf[i] = 0xdeadbeefu;
    }

  h616_blocksetup(dev, 512, 1);
  h616_waitenable(dev, SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT |
                  SDIOWAIT_ERROR, 0);
  h616_recvsetup(dev, (FAR uint8_t *)g_ladder_buf, 512);

  if (h616_sendcmd(dev, MMCSD_CMD17, lba) != OK)
    {
      return -EIO;
    }

  if (h616_waitresponse(dev, MMCSD_CMD17) != OK)
    {
      return -EIO;
    }

  events = h616_eventwait(dev);
  if ((events & SDIOWAIT_TRANSFERDONE) == 0)
    {
      H616_DIAG("ladder: lba %08" PRIx32 " read failed (events=%02x)"
            " WIDTH=%08" PRIx32 " CLKCR=%08" PRIx32 " STAS=%08" PRIx32
            " RINTR=%08"
            PRIx32 " CBCR=%08" PRIx32 " BCNTR=%08" PRIx32 " BLKSZ=%08"
            PRIx32 " remaining=%zu total=%zu\n",
            lba, (unsigned int)events,
            h616_getreg(priv, H616_SDXC_WIDTH),
            h616_getreg(priv, H616_SDXC_CLKCR),
            h616_getreg(priv, H616_SDXC_STAS),
            h616_getreg(priv, H616_SDXC_RINTR),
            h616_getreg(priv, H616_SDXC_CBCR),
            h616_getreg(priv, H616_SDXC_BCNTR),
            h616_getreg(priv, H616_SDXC_BLKSZ),
            priv->remaining, priv->total);

      return -EIO;
    }

  /* Note: priv->remaining is always 0 once eventwait returns (it zeroes the
   * field before handing back TRANSFERDONE), so it cannot be used to detect
   * a partial transfer -- the buffer itself is checked instead.
   */

  for (i = 0; i < 128; i++)
    {
      sum += g_ladder_buf[i];
      if (g_ladder_buf[i] != 0xdeadbeefu)
        {
          changed++;
        }
    }

  H616_DIAG("ladder: lba %08" PRIx32 " -> %08" PRIx32 " %08" PRIx32 " %08"
        PRIx32 " %08" PRIx32 " ... %08" PRIx32 " %08" PRIx32 " %08"
        PRIx32 " %08" PRIx32 " sum=%08" PRIx32 " changed=%" PRIu32 "\n",
        lba, g_ladder_buf[0], g_ladder_buf[1], g_ladder_buf[2],
        g_ladder_buf[3], g_ladder_buf[124], g_ladder_buf[125],
        g_ladder_buf[126], g_ladder_buf[127], sum, changed);

  if (changed == 0)
    {
      H616_DIAG("ladder: lba %08" PRIx32
                " moved no data (buffer untouched)\n", lba);
      return -EIO;
    }

  if (expect != NULL)
    {
      for (i = 0; i < 128; i++)
        {
          if (g_ladder_buf[i] != expect[i])
            {
              H616_DIAG("ladder: lba %08" PRIx32 " MISMATCH word %d: got %08"
                        PRIx32 " want %08" PRIx32 "\n", lba, i,
                        g_ladder_buf[i], expect[i]);
              return -EIO;
            }
        }
    }

  return OK;
}

/* Same read, one retry.
 *
 * The 8 bit mode is not reliably good on the first attempt on this board:
 * one
 * boot verifies both reference blocks, the next one times out on the very
 * first 8 bit read and has to fall back to 1 bit.  Reporting which attempt
 * succeeded is what tells the two failure modes apart -- "the width switch
 * is
 * marginal" (second attempt works) versus "this bus configuration never
 * works" (both attempts fail).
 */

static int h616_ladder_read_try2(struct h616_sdhci_priv_s *priv,
                                uint32_t lba, FAR const uint32_t *expect)
{
  int attempt;
  int ret = -EIO;

  /* A width change leaves the card busy for a short while: the first data
   * transfer after it either never starts its data phase (RINTR = 0, CBCR =
   * 0) or moves garbage.  Retrying is what carries the transfer, so keep
   * trying a few times before believing a failure.  A byte comparison
   * mismatch is retried too -- it is the same hardware behaviour seen
   * through a different symptom.
   */

  for (attempt = 1; attempt <= 3; attempt++)
    {
      ret = h616_ladder_read(priv, lba, expect);
      if (ret == OK)
        {
          break;
        }

      H616_DIAG("ladder: lba %08" PRIx32 " attempt %d failed, retrying\n",
                lba, attempt);
    }

  if (attempt > 1)
    {
      _info("ladder: lba %08" PRIx32 " attempt %d %s\n", lba, attempt,
            ret == OK ? "OK" : "also failed");
    }

  return ret;
}

/* Take a reference read.
 *
 * A reference cannot be compared against anything, so the only way to tell a
 * real transfer from noise is to read the same block twice and require both
 * copies to be identical.  h616_ladder_read() reports only that the
 * controller completed the transfer -- it will happily complete one that
 * moved garbage -- and its "changed" counter only proves that something was
 * written, so without this check a bus width switch can be verified against
 * noise.  That is exactly what happened on hardware: the first reference
 * read after a width change came back as zeroes and 0xffffffff, the honest
 * 8 bit data was then rejected against it, and the driver fell back to 1
 * bit.
 */

static int h616_ladder_refread(struct h616_sdhci_priv_s *priv, uint32_t lba,
                               FAR uint32_t *dst, int tries)
{
  int attempt;

  for (attempt = 1; attempt <= tries; attempt++)
    {
      if (h616_ladder_read(priv, lba, NULL) == OK)
        {
          memcpy(g_ladder_scratch, g_ladder_buf, sizeof(g_ladder_scratch));

          if (h616_ladder_read(priv, lba, NULL) == OK &&
              memcmp(g_ladder_scratch, g_ladder_buf,
                     sizeof(g_ladder_scratch)) == 0)
            {
              memcpy(dst, g_ladder_buf, sizeof(g_ladder_scratch));
              _info("ladder: reference lba %08" PRIx32 " agreed (try %d)\n",
                    lba, attempt);
              return OK;
            }
        }

      up_mdelay(1);
    }

  _info("ladder: reference lba %08" PRIx32 " never read the same twice\n",
        lba);
  return -EIO;
}

/* Measure the bus for real.
 *
 * The registers cannot tell a working bus from a dead one (a stalled
 * transfer reports RINTR = 0 / CBCR = 0, and a live one at the same width
 * reports exactly the same values), so the only honest confirmation that a
 * mode took effect -- and the only way to see how fast it is -- is to move
 * data and time it.  Single 512 byte blocks, the same shape the file system
 * uses, for a fixed window that is long enough for the 1 ms system tick to
 * be more than a rounding error.
 *
 * This also answers something the vendor sources leave open: how the CIU
 * divides the CCU module clock.  If the 8 bit number is ~8x the 1 bit one
 * then the bus is the limit, and the absolute value reveals the real card
 * clock (25 MHz 8 bit SDR is about 20 MB/s, 50 MHz about 40 MB/s).
 */

static uint32_t h616_ladder_speed(struct h616_sdhci_priv_s *priv,
                                  FAR const char *mode)
{
  clock_t start = clock_systime_ticks();
  clock_t ticks;
  uint32_t blocks = 0;
  uint32_t kbs = 0;

  /* The per read register dump is worth ~100ms per block at 115200 baud,
   * which swamps the measurement -- keep it muted for the whole loop.
   */

  bool saved_diag = g_sdio_diag_muted;
  g_sdio_diag_muted = true;

  while (blocks < H616_LADDER_MAXBLOCKS &&
         clock_systime_ticks() - start < H616_LADDER_TICKLIMIT)
    {
      if (h616_ladder_read(priv, H616_LADDER_LBA, NULL) != OK)
        {
          _info("ladder: %s throughput read failed after %lu blocks\n",
                mode, (unsigned long)blocks);
          g_sdio_diag_muted = saved_diag;
          return 0;
        }

      blocks++;
    }

  ticks = clock_systime_ticks() - start;
  if (ticks > 0)
    {
      /* 512 bytes per block, 1024 bytes per KB, one tick per millisecond.
       *
       */

      kbs = (uint32_t)(((uint64_t)blocks * 512u / 1024u) * 1000u /
                       (uint64_t)ticks);
    }

  _info("ladder: %s: %lu blocks in %lu ticks = %lu KB/s (WIDTH=%08" PRIx32
        " CLKCR=%08" PRIx32 " GCTRL=%08" PRIx32 " NTSR=%08" PRIx32
        " SAMP_DL=%08" PRIx32 " DRV_DL=%08" PRIx32 " CCU838=%08" PRIx32
        ")\n", mode, (unsigned long)blocks, (unsigned long)ticks,
        (unsigned long)kbs, h616_getreg(priv, H616_SDXC_WIDTH),
        h616_getreg(priv, H616_SDXC_CLKCR),
        h616_getreg(priv, H616_SDXC_GCTRL),
        h616_getreg(priv, H616_SDXC_NTSR),
        h616_getreg(priv, H616_SDXC_SAMP_DL),
        h616_getreg(priv, H616_SDXC_DRV_DL),
        getreg32(H616_CCU_ADDR + H616_CCU_SD2_CLK_CFG));

  g_sdio_diag_muted = saved_diag;
  return kbs;
}

/* Issue a CMD6 (SWITCH) that writes one EXT_CSD byte.  R1 bit 7 reports
 * whether the card accepted the switch (JEDEC: SWITCH_ERROR).
 */

static int h616_ladder_cmd6(struct h616_sdhci_priv_s *priv, uint32_t index,
                            uint32_t value)
{
  struct sdio_dev_s *dev = &priv->dev;

  if (h616_sendcmd(dev, MMCSD_CMD6, H616_CMD6_ARG(index, value)) != OK)
    {
      return -EIO;
    }

  if (h616_waitresponse(dev, MMCSD_CMD6) != OK)
    {
      return -EIO;
    }

  if ((h616_getreg(priv, H616_SDXC_RESP0) &
       (H616_R1_SWITCH_ERROR | H616_R1_ILLEGAL_COMMAND)) != 0)
    {
      return -EIO;
    }

  return OK;
}

/* A CMD6 write only takes effect after the card has run its internal
 * programming cycle: it signals busy on DAT0 and sets READY_FOR_DATA (R1 bit
 * 8) when it is done.  Issuing the next data command before that either
 * transfers with the old bus width -- the card has not applied the new one
 * yet, so the transfer still succeeds and proves nothing at all -- or comes
 * back with no data phase whatsoever.  Both were seen while probing bus
 * widths, so wait for the card here: poll CMD13 (SEND_STATUS) until it
 * reports READY_FOR_DATA in TRAN, or until the attempts run out.
 */

#define H616_LADDER_RDY_TRIES  100
#define H616_R1_READY_FOR_DATA (1u << 8)

static int h616_ladder_ready(struct h616_sdhci_priv_s *priv,
                             FAR const char *tag)
{
  uint32_t resp = 0;
  uint32_t state = 0xffu;
  int attempt;

  for (attempt = 0; attempt < H616_LADDER_RDY_TRIES; attempt++)
    {
      uint32_t rintr = 0;
      int t;

      h616_putreg(priv, H616_SDXC_RINTR, 0xffffffff);
      h616_putreg(priv, H616_SDXC_CARG, 0x00010000);  /* RCA 1 << 16 */
      h616_putreg(priv, H616_SDXC_CMDR,
                  H616_SDXC_START | H616_SDXC_RESP_EXPECT |
                  H616_SDXC_CHECK_RESPONSE_CRC | 13);

      for (t = 5000000; t > 0; t--)
        {
          rintr = h616_getreg(priv, H616_SDXC_RINTR);
          if ((rintr & (H616_SDXC_COMMAND_DONE |
                        H616_SDXC_RESP_TIMEOUT)) != 0)
            {
              break;
            }
        }

      resp  = h616_getreg(priv, H616_SDXC_RESP0);
      state = (resp >> 9) & 0xf;

      if (t > 0 && (rintr & H616_SDXC_RESP_TIMEOUT) == 0 &&
          (resp & H616_R1_READY_FOR_DATA) != 0 && state == 4u)
        {
          h616_putreg(priv, H616_SDXC_RINTR, 0xffffffff);

          if (tag != NULL)
            {
              _info("DDR52: card ready after %s (attempts=%d R1=%08"
                    PRIx32 ")\n", tag, attempt + 1, resp);
            }

          return OK;
        }

      up_mdelay(1);
    }

    {
      uint32_t rintr = h616_getreg(priv, H616_SDXC_RINTR);
      uint32_t stas  = h616_getreg(priv, H616_SDXC_STAS);
      uint32_t cmdr  = h616_getreg(priv, H616_SDXC_CMDR);

      h616_putreg(priv, H616_SDXC_RINTR, 0xffffffff);
      _info("DDR52: card NOT ready after %s: R1=%08" PRIx32 " state=%"
            PRIu32 " RINTR=%08" PRIx32 " STAS=%08" PRIx32 " CMDR=%08" PRIx32
            "\n", tag, resp, state, rintr, stas, cmdr);
    }

  return -EIO;
}

/* Program the sample delay (0x144) and the data strobe delay (0x148).  A
 * negative value leaves that delay disabled -- the card cannot assert the
 * strobe outside HS400, and 1 bit/default speed needs no sample delay.
 *
 */

static inline void h616_ladder_delays(struct h616_sdhci_priv_s *priv,
                                      int samp_dl, int ds_dl)
{
  uint32_t regval;

  regval  = h616_getreg(priv, H616_SDXC_SAMP_DL);
  regval &= ~(H616_SDXC_DLY_VALUE_MASK | H616_SDXC_DLY_ENABLE);
  if (samp_dl >= 0)
    {
      regval |= ((uint32_t)samp_dl & H616_SDXC_DLY_VALUE_MASK) |
                H616_SDXC_DLY_ENABLE;
    }

  h616_putreg(priv, H616_SDXC_SAMP_DL, regval);

  regval  = h616_getreg(priv, H616_SDXC_DS_DL);
  regval &= ~(H616_SDXC_DLY_VALUE_MASK | H616_SDXC_DLY_ENABLE);
  if (ds_dl >= 0)
    {
      regval |= ((uint32_t)ds_dl & H616_SDXC_DLY_VALUE_MASK) |
                H616_SDXC_DLY_ENABLE;
    }

  h616_putreg(priv, H616_SDXC_DS_DL, regval);
}

/* Drive delay: the output phase lives in drv_dl bits[17:16] and both bits
 * are written together (sunxi_mmc.c:379-428).
 */

static inline void h616_ladder_drvdl(struct h616_sdhci_priv_s *priv,
                                     uint32_t phase)
{
  uint32_t regval = h616_getreg(priv, H616_SDXC_DRV_DL);

  regval &= ~(3u << 16);
  regval |= ((phase & 1u) << 16) | ((phase & 1u) << 17);
  h616_putreg(priv, H616_SDXC_DRV_DL, regval);
}

/* Switch the CCU module clock and re-enable the card clock.  Deliberately
 * no UPCLK trigger: that wedges this controller (see h616_reset).
 */

static inline void h616_ladder_clock(struct h616_sdhci_priv_s *priv,
                                     uint32_t modulehz)
{
  h616_clockenable(priv, modulehz);
  h616_modifyreg(priv, H616_SDXC_CLKCR, 0, H616_SDXC_CARD_CLOCK_ON);
}

/* Put host and card back into the known good 1 bit / default speed state
 * after every faster mode has failed.
 */

static void h616_ladder_rollback1(struct h616_sdhci_priv_s *priv)
{
  h616_ladder_cmd6(priv, H616_EXT_CSD_BUS_WIDTH, 0);
  h616_setbuswidth(priv, 1);
  priv->wide = false;

  /* Restore the exact register image captured before the ladder ran rather
   * than an approximation of it: the working 1 bit configuration uses the
   * CIU reset values of the sample/strobe delay registers.
   */

  h616_putreg(priv, H616_SDXC_GCTRL, g_ladder_save.gctrl);
  h616_putreg(priv, H616_SDXC_CLKCR, g_ladder_save.clkcr);
  h616_putreg(priv, H616_SDXC_DRV_DL, g_ladder_save.drvdl);
  h616_putreg(priv, H616_SDXC_SAMP_DL, g_ladder_save.sampdl);
  h616_putreg(priv, H616_SDXC_DS_DL, g_ladder_save.dsdl);
  h616_putreg(priv, H616_SDXC_CSDC, g_ladder_save.csdc);
  h616_putreg(priv, H616_SDXC_DSBD, g_ladder_save.dsbd);
  h616_putreg(priv, H616_SDXC_WIDTH, g_ladder_save.width);

  _info("ladder: rollback -> WIDTH=%08" PRIx32 " CLKCR=%08" PRIx32
        " GCTRL=%08" PRIx32 " SAMP_DL=%08" PRIx32 " DS_DL=%08" PRIx32
        "\n", h616_getreg(priv, H616_SDXC_WIDTH),
        h616_getreg(priv, H616_SDXC_CLKCR),
        h616_getreg(priv, H616_SDXC_GCTRL),
        h616_getreg(priv, H616_SDXC_SAMP_DL),
        h616_getreg(priv, H616_SDXC_DS_DL));
}

/* The ladder itself.  Runs once per card initialisation, from
 * h616_clock(CLOCK_MMC_TRANSFER), which is the last call of the mmcsd
 * bring-up sequence.
 */

static void h616_ladder(struct h616_sdhci_priv_s *priv)
{
  bool eight = false;
  bool eightok = false;
  bool ddr = false;

  if (priv->ladderdone)
    {
      return;
    }

  priv->ladderdone = true;

  /* Snapshot every register the ladder may touch, so a failed step can be
   * undone exactly.
   */

  g_ladder_save.gctrl  = h616_getreg(priv, H616_SDXC_GCTRL);
  g_ladder_save.clkcr  = h616_getreg(priv, H616_SDXC_CLKCR);
  g_ladder_save.width  = h616_getreg(priv, H616_SDXC_WIDTH);
  g_ladder_save.drvdl  = h616_getreg(priv, H616_SDXC_DRV_DL);
  g_ladder_save.sampdl = h616_getreg(priv, H616_SDXC_SAMP_DL);
  g_ladder_save.dsdl   = h616_getreg(priv, H616_SDXC_DS_DL);
  g_ladder_save.csdc   = h616_getreg(priv, H616_SDXC_CSDC);
  g_ladder_save.dsbd   = h616_getreg(priv, H616_SDXC_DSBD);
  g_ladder_save.ntsr   = h616_getreg(priv, H616_SDXC_NTSR);
  g_ladder_save.ftrgl  = h616_getreg(priv, H616_SDXC_FTRGL);
  g_ladder_save.thldc  = h616_getreg(priv, H616_SDXC_THLDC);
  g_ladder_save.tmout  = h616_getreg(priv, H616_SDXC_TMOUT);

  /* h400d7: this is the exact handoff state left by the boot_package
   * U-Boot (which just read boot.img successfully at HS400@100MHz on the
   * same card).  Replicating this state instead of re-deriving it is the
   * whole point of the dump.
   */

  _info("ladder: save GCTRL=%08" PRIx32 " CLKCR=%08" PRIx32 " WIDTH=%08"
        PRIx32 " DRV_DL=%08" PRIx32 " SAMP_DL=%08" PRIx32 " DS_DL=%08"
        PRIx32 " CSDC=%08" PRIx32 " DSBD=%08" PRIx32 "\n",
        g_ladder_save.gctrl, g_ladder_save.clkcr, g_ladder_save.width,
        g_ladder_save.drvdl, g_ladder_save.sampdl, g_ladder_save.dsdl,
        g_ladder_save.csdc, g_ladder_save.dsbd);

  _info("ladder: save NTSR=%08" PRIx32 " FTRGL=%08" PRIx32 " THLDC=%08"
        PRIx32 " TMOUT=%08" PRIx32 "\n",
        g_ladder_save.ntsr, g_ladder_save.ftrgl, g_ladder_save.thldc,
        g_ladder_save.tmout);

  /* Reference reads in the configuration that is known to work.  Two
   * *different* blocks, because the 8 bit check below must not be
   * satisfiable
   * by a buffer that still holds the reference or by stale FIFO content.
   *
   */

  if (h616_ladder_refread(priv, H616_LADDER_LBA, g_ladder_ref,
                          H616_LADDER_REF_TRIES) != OK ||
      h616_ladder_refread(priv, H616_LADDER_LBA2, g_ladder_ref2,
                          H616_LADDER_REF_TRIES) != OK)
    {
      _info("ladder: 1 bit reference read failed, staying at 1 bit\n");
      return;
    }

  _info("ladder: 1 bit references OK (lba %08" PRIx32 "/%08" PRIx32
        " WIDTH=%08" PRIx32 " SAMP_DL=%08" PRIx32 " DS_DL=%08" PRIx32
        ")\n", H616_LADDER_LBA, H616_LADDER_LBA2,
        h616_getreg(priv, H616_SDXC_WIDTH),
        h616_getreg(priv, H616_SDXC_SAMP_DL),
        h616_getreg(priv, H616_SDXC_DS_DL));

  h616_ladder_speed(priv, "1 bit");

#if H616_LADDER_8BIT
  /* Step 1: 8 bit bus at the clock already in use.  This also proves that
   * DAT4..DAT7 really reach the controller on this board.
   */

  if (h616_ladder_cmd6(priv, H616_EXT_CSD_BUS_WIDTH,
                       H616_BUS_WIDTH_8BIT) != OK)
    {
      _info("ladder: CMD6 BUS_WIDTH=8 rejected by the card\n");
    }
  else
    {
      h616_setbuswidth(priv, 8);
      priv->wide = true;

      /* Both blocks are checked: the first read proves that the 8 bit path
       * moves data at all (its destination was poisoned), the second one --
       * a different LBA whose content differs from the first -- proves that
       * it really is the data that block holds and not the previous read's
       * leftovers or stale FIFO content.
       */

      if (h616_ladder_read_try2(priv, H616_LADDER_LBA2,
                                g_ladder_ref2) == OK &&
          h616_ladder_read_try2(priv, H616_LADDER_LBA,
                                g_ladder_ref) == OK)
        {
          eight  = true;
          eightok = true;
          _info("ladder: 8 bit SDR verify OK (2 lbas, byte compared)\n");

          if (H616_LADDER_KEEP8BIT)
            {
              h616_ladder_speed(priv, "8 bit");
            }
        }
      else
        {
          _info("ladder: 8 bit SDR verify FAILED, back to 1 bit\n");
          h616_ladder_rollback1(priv);
          eight = false;
        }
    }
#endif

#if !H616_LADDER_KEEP8BIT
  /* Safety gate: 8 bit is measured and then handed back.  Two verified block
   * reads are not proof that the mmcsd layer (scatter/gather, multi block
   * reads, FAT) survives 8 bit -- and the last attempt at keeping it left
   * the
   * board wedged right after this point -- so the rest of the boot runs in
   * the 1 bit configuration that is known to work.
   */

  if (eight)
    {
      h616_ladder_rollback1(priv);
      eight = false;
      _info("ladder: 8 bit verified but given back (KEEP8BIT=0)\n");
    }
#endif

  /* Prove the way back before the mmcsd layer takes over.
   */

  if (h616_getreg(priv, H616_SDXC_WIDTH) == H616_SDXC_WIDTH1)
    {
      if (h616_ladder_read(priv, H616_LADDER_LBA2, g_ladder_ref2) == OK &&
          h616_ladder_read(priv, H616_LADDER_LBA, g_ladder_ref) == OK)
        {
          _info("ladder: back at 1 bit, both blocks verify again\n");
        }
      else
        {
          _info("ladder: WARNING 1 bit re-verify FAILED after rollback\n");
        }
    }

#if H616_LADDER_DDR52
  /* Step 2: 8 bit DDR52 (50MHz DDR = 100MB/s).  JEDEC only defines the DDR
   * bus widths for 4/8 bit, so the card must already be at 8 bit.
   *
   * The whole step is h616_ddr52_boot() (defined with the rest of the DDR52
   * machinery below): the recipe, the switch order and the way the host
   * clock change is applied are the ones N35 verified byte for byte, and the
   * bus is only kept when two different blocks read back identical.
   */

  if (eight)
    {
      ddr = h616_ddr52_boot(priv);

      if (!ddr)
        {
          _info("ladder: DDR52 not taken, the boot keeps 8 bit SDR\n");
        }
    }
#endif

  _info("ladder: done 8bit=%d eight_ok=%d ddr52=%d WIDTH=%08"
        PRIx32 " CLKCR=%08" PRIx32 "\n", eight, eightok, ddr,
        h616_getreg(priv, H616_SDXC_WIDTH),
        h616_getreg(priv, H616_SDXC_CLKCR));
}

/* Return the supported capabilities.  The controller supports 1/4/8-bit
 * buses (8-bit for eMMC), MMC/SD/SDIO, and no DMA on the simplest path.
 *
 */

static sdio_capset_t h616_capabilities(struct sdio_dev_s *dev)
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;
  return priv->caps;
}

/* Return the card status.  eMMC is always present (non-removable).
 */

static sdio_statset_t h616_status(struct sdio_dev_s *dev)
{
  sdio_statset_t status = 0;

  /* Card present (eMMC soldered, always inserted)
   */

  status |= SDIO_STATUS_PRESENT;

  return status;
}

/* Set the bus width
 */

/* Read the card's EXT_CSD over the same PIO path the block reads use.  No
 * comparison here: the content is the card's own state, read out to report
 * which mode the card believes it is in (byte 183 bus width, byte 185
 * HS_TIMING) and what it can do (byte 196 device type, which also answers
 * whether HS400 is physically available on this card).
 */

static int h616_ladder_extcsd(struct h616_sdhci_priv_s *priv,
                              FAR uint8_t *buf)
{
  struct sdio_dev_s *dev = &priv->dev;

  h616_blocksetup(dev, 512, 1);
  h616_waitenable(dev, SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT |
                  SDIOWAIT_ERROR, 0);
  h616_recvsetup(dev, buf, 512);

  if (h616_sendcmd(dev, MMC_CMD8, 0) != OK ||
      h616_waitresponse(dev, MMC_CMD8) != OK)
    {
      return -EIO;
    }

  return (h616_eventwait(dev) & SDIOWAIT_TRANSFERDONE) != 0 ? OK : -EIO;
}

/* Bus timing profiles.
 *
 * The vendor stacks drive this controller very differently from what this
 * driver needed to get 1 bit and 8 bit SDR working, and the difference is
 * exactly what DDR52 needs:
 *
 *   - the CCU module clock is a multiple of the card clock, because the
 *     controller divides it down internally: "mod_hz = hz * 2" for SDR (and
 *     for 4 bit / HS400 DDR), "mod_hz = hz * 4" for 8 bit DDR52, with
 *     CLKCR[7:0] = 0 for SDR and 1 for 8 bit DDR52 (tm4 mmc_set_mod_clk(),
 *     sunxi_mmc_host_tm4.c:253-466);
 *   - the sample point of the incoming data is NTSR (0x5c): bit31 is the
 *     "2x timing" mode, bits[9:8] the data and bits[5:4] the command
 *     sampling phase (Linux v4p1x sunxi_mmc_set_dly_raw(); tm4 writes bit31
 *     clear, "Clear NTSR bit 31 to shutdown 2x mode!");
 *   - the outgoing phases live in DRV_DL[17:16] (bit16 command, bit17 data,
 *     both written together, 1 = 180 degrees) and the sampled delay in
 *     SAMP_DL[6:0] with bit7 as enable (tm4 mmc_config_delay(),
 *     sunxi_mmc_host_tm4.c:195-250);
 *   - DDR52 8 bit on this board is module clock 4x, CLKCR[7:0] = 1, DRV_DL
 *     180 degrees, SAMP_DL 0x0e (tm4 defaults, sunxi_mmc_host_tm4.c:71-83);
 *   - DDR52 mode = GCTRL[10] set, DSBD(0x10c)[31] clear, CSDC(0x54)[3:0] = 3
 *     (tm4 sunxi_mmc_set_speed_mode(), sunxi_mmc_host_tm4.c:467-525).
 *
 * Getting this wrong is expensive on this board: the bus either stops
 * transferring altogether (RINTR = 0, CBCR = 0) or moves wrong data, and
 * neither shows up in the register dump.  So every profile below is verified
 * with real data, and a profile that fails falls back to the last one that
 * verified.
 */

static void h616_bus_apply(struct h616_sdhci_priv_s *priv,
                           FAR const struct h616_bus_profile_s *p)
{
  uint32_t regval;

  /* Take the card clock down first: the timing registers, the internal
   * divider and the DDR mode bit must not move under a running bus.
   */

  h616_modifyreg(priv, H616_SDXC_CLKCR, H616_SDXC_CARD_CLOCK_ON, 0);

  h616_clockenable(priv, p->mod_hz);

  /* NTSR has to be written with bit31 clear and then set again for the
   * sampling phases to be latched (both vendor stacks do it that way).
   *
   */

  h616_putreg(priv, H616_SDXC_NTSR, p->ntsr & ~H616_SDXC_NTSR_2X_TIMING);
  if ((p->ntsr & H616_SDXC_NTSR_2X_TIMING) != 0)
    {
      h616_putreg(priv, H616_SDXC_NTSR, p->ntsr);
    }

  if (p->odly >= 0)
    {
      h616_ladder_drvdl(priv, (uint32_t)p->odly);
    }

  if (p->sdly >= 0)
    {
      regval  = h616_getreg(priv, H616_SDXC_SAMP_DL);
      regval &= ~(H616_SDXC_DLY_VALUE_MASK | H616_SDXC_DLY_ENABLE);
      regval |= ((uint32_t)p->sdly & H616_SDXC_DLY_VALUE_MASK) |
                H616_SDXC_DLY_ENABLE;
      h616_putreg(priv, H616_SDXC_SAMP_DL, regval);
    }

  h616_modifyreg(priv, H616_SDXC_GCTRL, H616_SDXC_DDR_MODE,
                 p->ddr ? H616_SDXC_DDR_MODE : 0);
  h616_modifyreg(priv, H616_SDXC_DSBD, H616_SDXC_DSBD_HS400,
                 p->dsbd ? H616_SDXC_DSBD_HS400 : 0);

  h616_modifyreg(priv, H616_SDXC_CLKCR, 0xffu, p->clkdiv & 0xffu);
  h616_modifyreg(priv, H616_SDXC_CLKCR, 0, H616_SDXC_CARD_CLOCK_ON);

  priv->bus_mod_hz = p->mod_hz;
  priv->bus_clkdiv = p->clkdiv & 0xffu;
  priv->ddr        = p->ddr;

  /* A mode change leaves the card busy for a while: without this the first
   * transfer on the new timing either never starts its data phase or moves
   * garbage (both measured on this board).
   */

  up_mdelay(H616_LADDER_SETTLE_MS);
}

/* Verify the bus with real data: both reference blocks, byte compared.
 */

static bool h616_bus_verify(struct h616_sdhci_priv_s *priv)
{
  return
    h616_ladder_read_try2(priv, H616_LADDER_LBA2, g_ladder_ref2) == OK &&
    h616_ladder_read_try2(priv, H616_LADDER_LBA, g_ladder_ref) == OK;
}

/* Try every profile in order and keep the first one whose data verifies.
 * The kept profile is copied out so the caller can fall back to it.
 */

static bool h616_bus_sweep(struct h616_sdhci_priv_s *priv,
                           FAR const char *what,
                           FAR const struct h616_bus_profile_s *table,
                           size_t count,
                           FAR struct h616_bus_profile_s *kept)
{
  size_t i;

  for (i = 0; i < count; i++)
    {
      h616_bus_apply(priv, &table[i]);

      if (h616_bus_verify(priv))
        {
          *kept = table[i];

          _info("widebus: %s profile \"%s\" verified (mod=%" PRIu32
                " clkdiv=%" PRIu32 " ntsr=%08" PRIx32 " odly=%d sdly=%d"
                " WIDTH=%08" PRIx32 " GCTRL=%08" PRIx32 " SAMP_DL=%08"
                PRIx32 " DRV_DL=%08" PRIx32 " CCU838=%08" PRIx32 ")\n",
                what, table[i].tag, table[i].mod_hz, table[i].clkdiv,
                table[i].ntsr, table[i].odly, table[i].sdly,
                h616_getreg(priv, H616_SDXC_WIDTH),
                h616_getreg(priv, H616_SDXC_GCTRL),
                h616_getreg(priv, H616_SDXC_SAMP_DL),
                h616_getreg(priv, H616_SDXC_DRV_DL),
                getreg32(H616_CCU_ADDR + H616_CCU_SD2_CLK_CFG));
          return true;
        }
    }

  _info("widebus: %s gave nothing (last: STAS=%08" PRIx32 " RINTR=%08"
        PRIx32 " CBCR=%08" PRIx32 " BCNTR=%08" PRIx32 " WIDTH=%08" PRIx32
        " CLKCR=%08" PRIx32 " GCTRL=%08" PRIx32 " NTSR=%08" PRIx32
        " SAMP_DL=%08" PRIx32 " DRV_DL=%08" PRIx32 " CCU838=%08" PRIx32
        ")\n", what, h616_getreg(priv, H616_SDXC_STAS),
        h616_getreg(priv, H616_SDXC_RINTR),
        h616_getreg(priv, H616_SDXC_CBCR),
        h616_getreg(priv, H616_SDXC_BCNTR),
        h616_getreg(priv, H616_SDXC_WIDTH),
        h616_getreg(priv, H616_SDXC_CLKCR),
        h616_getreg(priv, H616_SDXC_GCTRL),
        h616_getreg(priv, H616_SDXC_NTSR),
        h616_getreg(priv, H616_SDXC_SAMP_DL),
        h616_getreg(priv, H616_SDXC_DRV_DL),
        getreg32(H616_CCU_ADDR + H616_CCU_SD2_CLK_CFG));

  return false;
}

/* 8 bit SDR at 50 MHz (high speed timing, single edge).
 *
 * The card only allows 52 MHz in high speed timing, and it has to be told
 * while the bus is still running at the old clock, so HS_TIMING is written
 * first.  The host side is then swept: this is where the vendor's module
 * clock scheme gets tried, because "just raise the CCU clock" is not
 * obviously the same thing on a controller that divides it down.
 */

static bool h616_ladder_sdr50(struct h616_sdhci_priv_s *priv,
                              FAR struct h616_bus_profile_s *kept)
{
  /* Keep the delays that already work and only move the clock.  The
   * "vendor" rows additionally drive phase 180 degrees with sampled delay 0.
   *
   */

  static const struct h616_bus_profile_s table[] =
  {
    { "mod100-2x",      100000000u, 0u, H616_SDXC_NTSR_2X_TIMING, -1, -1,
      false, false },
    { "mod100",         100000000u, 0u, 0u, -1, -1, false, false },
    { "mod50",           50000000u, 0u, 0u, -1, -1, false, false },
    { "mod100-vendor",  100000000u, 0u, 0u, (int)H616_DRV_DLY_PH180, 0,
      false, false },
    { "mod100-vendor2x", 100000000u, 0u,
      H616_SDXC_NTSR_2X_TIMING | H616_SDXC_NTSR_DAT_PH(1) |
      H616_SDXC_NTSR_CMD_PH(1), (int)H616_DRV_DLY_PH180, 0, false, false },
  };

  if (h616_ladder_cmd6(priv, H616_EXT_CSD_HS_TIMING, H616_HS_TIMING_HS) !=
      OK)
    {
      _info("widebus: CMD6 HS_TIMING=1 rejected by the card\n");
      return false;
    }

  return h616_bus_sweep(priv, "8 bit SDR 50M", table,
                        sizeof(table) / sizeof(table[0]), kept);
}

/* 8 bit DDR52 (50 MHz clock, data on both edges).
 *
 * Only attempted after 8 bit SDR has verified, so a failure falls back to a
 * bus that is already known to work.
 *
 * JEDEC sequence: HS_TIMING first and then the DDR bus width, because the
 * card latches the DDR mode together with the width; the host raises its own
 * clock only once the card has moved.  Coming back, the host is put on the
 * SDR timing it had before and the card is taken out of DDR afterwards, so
 * the card is never driven with a timing it cannot follow.
 */

#if H616_DDR52_SUPPORT

/* 8 bit DDR52: the card clock stays at 50 MHz and both bus edges carry data,
 * so the bus moves twice the bytes per clock.  The host side is the vendor
 * driver's DDR52 path (sunxi_mmc_set_ios() + sunxi_mmc_clk_set_rate_for_
 * sdmmc_v4p5x() + sunxi_mmc_set_clk_dly()):
 *
 *   module clock 200 MHz (four times the card clock), CLKCR[7:0] = 1,
 *   no 2x timing, DRV_DL = 0x00010000 (command output phase 180 degrees),
 *   SAMP_DL low six bits 0, CSDC[3:0] = 3, HS400 start bit detect clear,
 *   GCTRL device-to-controller DDR mode set.
 *
 * Two things make that work on this controller, both measured here:
 *
 *   - the clock change has to be latched by one CIU clock update
 *     (CMDR START | UPCLK_ONLY | WAIT_PRE_OVER with the card clock off),
 *     not by writing CLKCR and waiting;
 *   - GCTRL.DDR_MODE must be written while the module clock is running: the
 *     CCU gate that the vendor's "disable mclk around set_ios" maps to on
 *     this SoC silently drops the write.
 *
 * The card is switched with CMD6 bus width 6 first and HS_TIMING = 1 second
 * (this card rejects the JEDEC order), and the host bus is only kept when
 * two
 * different reference blocks read back byte for byte.  Otherwise the card is
 * put back on the verified 8 bit SDR timing before returning.
 */

struct h616_ddr52_profile_s
{
  FAR const char *tag;      /* Name used in the log */
  uint32_t mod_hz;          /* CCU module clock */
  uint32_t clkdiv;          /* CLKCR[7:0] internal divider */
  uint32_t ntsr;            /* 0x5c image (2x timing bit + sampling phases) */
  uint32_t drv_dl;          /* 0x140 raw */
  uint32_t samp_dl;         /* 0x144 raw, or H616_DDR52_DLY_UNTOUCHED */
  uint32_t ds_dl;           /* 0x148 raw, or H616_DDR52_DLY_UNTOUCHED */
  uint32_t csdc;            /* 0x54 low nibble */
  bool     dsbd;            /* 0x10c bit31 */
  bool     ddr;             /* GCTRL bit10 */
};

/* A delay field set to this leaves the register alone.  The controller this
 * board has (sun50iw10p1.dtsi binds sdc2 to allwinner,sunxi-mmc-v5p3x) is
 * driven by a kernel that never writes SAMP_DL or DS_DL, so the values the
 * boot left behind are part of that controller's real DDR52 configuration
 * and overwriting them with another driver's numbers only hides the truth.
 *
 */

#define H616_DDR52_DLY_UNTOUCHED 0xffffffffu

/* DRV_DL bits the vendor driver actually drives: bit16 is the command output
 * phase, bit17 the data output phase (sunxi-mmc-v4p5x.c:160).
 */

#define H616_DDR52_DRV_PH_MASK   0x00030000u

/* Apply one profile with the card clock stopped, exactly like
 * h616_bus_apply, but with the raw delay/strobe registers the vendor dump
 * describes.
 */

/* The card clock switch, ported line for line from the vendor driver's
 * __sunxi_mmc_do_oclk_onoff() (sunxi-mmc-v4p5x.c:383).
 *
 * The part this driver was missing is the update itself: on this controller
 * a CLKCR change only takes effect when the CIU is told to apply it, and the
 * vendor does that by issuing CMDR = START | UPCLK_ONLY | WAIT_PRE_OVER and
 * waiting for START to clear, with DAT0 masked so the card does not see the
 * glitch.  Writing CLKCR and waiting for a while (what this driver did) is
 * not the same thing: the divider, the delays and the DDR mode bit are
 * latched by that command.
 */

static void h616_ddr52_oclk(struct h616_sdhci_priv_s *priv, bool on)
{
  uint32_t regval;
  volatile int32_t timeout;

  regval  = h616_getreg(priv, H616_SDXC_CLKCR);
  regval &= ~(H616_SDXC_CARD_CLOCK_ON | H616_SDXC_LOW_POWER_ON |
              H616_SDXC_MASK_DATA0);
  if (on)
    {
      regval |= H616_SDXC_CARD_CLOCK_ON;
    }

  /* The vendor masks DAT0 for every clock update (ignore_dat0 = 1).
   */

  regval |= H616_SDXC_MASK_DATA0;
  h616_putreg(priv, H616_SDXC_CLKCR, regval);

  h616_putreg(priv, H616_SDXC_CMDR,
              H616_SDXC_START | H616_SDXC_UPCLK_ONLY |
              H616_SDXC_WAIT_PRE_OVER);

  timeout = 250000;
  while ((h616_getreg(priv, H616_SDXC_CMDR) & H616_SDXC_START) != 0)
    {
      if (--timeout == 0)
        {
          _err("ERROR: ddr52 clock update timeout (CLKCR=%08" PRIx32 ")\n",
               h616_getreg(priv, H616_SDXC_CLKCR));
          break;
        }
    }

  /* Clear the status the update command latched (the vendor leaves the SDIO
   * bit alone; every other bit is write-one-to-clear here).
   */

  regval = h616_getreg(priv, H616_SDXC_RINTR) & ~H616_SDXC_SDIO_INTERRUPT;
  h616_putreg(priv, H616_SDXC_RINTR, regval);

  regval  = h616_getreg(priv, H616_SDXC_CLKCR);
  h616_putreg(priv, H616_SDXC_CLKCR, regval & ~H616_SDXC_MASK_DATA0);
}

/* Apply one profile the way sunxi_mmc_set_ios() +
 * sunxi_mmc_clk_set_rate_for_sdmmc_v4p5x() do it (sunxi-mmc.c:1002,
 * sunxi-mmc-v4p5x.c:446):
 *
 *   1. bus width (the caller has already set 8 bit),
 *   2. GCTRL.DDR_MODE while the module clock is gated
 *      ("If we set ddr mode, we should disable mclk first"),
 *   3. card clock off (CLKCR + UPCLK_ONLY),
 *   4. CCU rate -> CLKCR[7:0] divider -> EDSD (HS400 bit) -> CSDC -> the
 *      three delay registers,
 *   5. card clock on (CLKCR + UPCLK_ONLY).
 *
 * The register *values* are the ones the earlier profiles already used; what
 * changes here is the order and the clock update around them.
 */

static void h616_ddr52_apply(struct h616_sdhci_priv_s *priv,
                             FAR const struct h616_ddr52_profile_s *p)
{
  uint32_t regval;

  /* Step 1 is the caller's h616_setbuswidth(priv, 8).
   */

  h616_ddr52_oclk(priv, false);

  /* Step 2: DDR mode bit.  The vendor gates clk_mmc (the functional clock)
   * around this write, over an AHB that stays clocked; gating the CCU gate
   * we have here instead loses the write (measured in N34: GCTRL read back
   * 0x20000010, DDR_MODE clear).  The card clock is already stopped by the
   * clock update above, so the bus is quiet either way.
   */

  h616_modifyreg(priv, H616_SDXC_GCTRL, H616_SDXC_DDR_MODE,
                 p->ddr ? H616_SDXC_DDR_MODE : 0);

  /* NTSR is not touched by the v4p5x driver at all; the profiles that ask
   * for 2x timing still need it, and bit31 has to be cleared before the rest
   * of the register latches.
   */

  h616_putreg(priv, H616_SDXC_NTSR, p->ntsr & ~H616_SDXC_NTSR_2X_TIMING);
  if ((p->ntsr & H616_SDXC_NTSR_2X_TIMING) != 0)
    {
      h616_putreg(priv, H616_SDXC_NTSR, p->ntsr);
    }

  /* Step 4: clock rate, divider, EDSD, CSDC and the delays, card clock off.
   *
   */

  h616_clockenable(priv, p->mod_hz);

  h616_modifyreg(priv, H616_SDXC_CLKCR, 0xffu, p->clkdiv & 0xffu);

  h616_modifyreg(priv, H616_SDXC_DSBD, H616_SDXC_DSBD_HS400,
                 p->dsbd ? H616_SDXC_DSBD_HS400 : 0);

  regval  = h616_getreg(priv, H616_SDXC_CSDC);
  regval &= ~H616_SDXC_CSDC_STROBE_MASK;
  regval |= (p->csdc & H616_SDXC_CSDC_STROBE_MASK);
  h616_putreg(priv, H616_SDXC_CSDC, regval);

  /* DRV_DL: the vendor only ever drives bits[17:16] (command and data
   * output phase, 1 = 180 degrees) and leaves the rest of the register as
   * the handover left it.
   */

  regval  = h616_getreg(priv, H616_SDXC_DRV_DL);
  regval &= ~(H616_DDR52_DRV_PH_MASK);
  regval |= (p->drv_dl & H616_DDR52_DRV_PH_MASK);
  h616_putreg(priv, H616_SDXC_DRV_DL, regval);

  if (p->samp_dl != H616_DDR52_DLY_UNTOUCHED)
    {
      regval  = h616_getreg(priv, H616_SDXC_SAMP_DL);
      regval &= ~(H616_SDXC_DLY_VALUE_MASK | H616_SDXC_DLY_ENABLE);
      regval |= ((uint32_t)p->samp_dl & H616_SDXC_DLY_VALUE_MASK) |
                H616_SDXC_DLY_ENABLE;
      h616_putreg(priv, H616_SDXC_SAMP_DL, regval);
    }

  if (p->ds_dl != H616_DDR52_DLY_UNTOUCHED)
    {
      regval  = h616_getreg(priv, H616_SDXC_DS_DL);
      regval &= ~(H616_SDXC_DLY_VALUE_MASK | H616_SDXC_DLY_ENABLE);
      regval |= ((uint32_t)p->ds_dl & H616_SDXC_DLY_VALUE_MASK) |
                H616_SDXC_DLY_ENABLE;
      h616_putreg(priv, H616_SDXC_DS_DL, regval);
    }

  /* Step 5: the card clock back on, applied by the CIU.
   */

  h616_ddr52_oclk(priv, true);

  priv->bus_mod_hz = p->mod_hz;
  priv->bus_clkdiv = p->clkdiv & 0xffu;
  priv->ddr        = p->ddr;

  up_mdelay(H616_LADDER_SETTLE_MS);
}

/* One log line per attempt: the profile, followed by the controller state it
 * actually left behind (the raw writes are checked by reading them back).
 *
 */

static void h616_ddr52_log(struct h616_sdhci_priv_s *priv,
                           FAR const char *tag,
                           FAR const struct h616_ddr52_profile_s *p)
{
  _info("DDR52: \"%s\" applied (mod=%08" PRIx32 " clkdiv=%08" PRIx32
        " ntsr=%08" PRIx32 " drv=%08" PRIx32 " samp=%08" PRIx32
        " ds=%08" PRIx32 " csdc=%" PRIu32 " dsbd=%d ddr=%d) WIDTH=%08"
        PRIx32 " CLKCR=%08" PRIx32 " GCTRL=%08" PRIx32 " NTSR=%08" PRIx32
        " CSDC=%08" PRIx32 " DSBD=%08" PRIx32 " CCU838=%08" PRIx32 "\n",
        tag, p->mod_hz, p->clkdiv, p->ntsr, p->drv_dl, p->samp_dl, p->ds_dl,
        p->csdc, p->dsbd ? 1 : 0, p->ddr ? 1 : 0,
        h616_getreg(priv, H616_SDXC_WIDTH),
        h616_getreg(priv, H616_SDXC_CLKCR),
        h616_getreg(priv, H616_SDXC_GCTRL),
        h616_getreg(priv, H616_SDXC_NTSR),
        h616_getreg(priv, H616_SDXC_CSDC),
        h616_getreg(priv, H616_SDXC_DSBD),
        getreg32(H616_CCU_ADDR + H616_CCU_SD2_CLK_CFG));
}

/* Send one CMD6 EXT_CSD write from the probe and say exactly which step
 * failed: the command not going out at all, no response, or the card
 * answering with SWITCH_ERROR.  One "the card stopped accepting" message hid
 * all three, and they mean completely different things.
 */

static int h616_ddr52_cmd6(struct h616_sdhci_priv_s *priv, uint32_t index,
                           uint32_t value, FAR const char *tag)
{
  struct sdio_dev_s *dev = &priv->dev;
  uint32_t resp;

  if (h616_sendcmd(dev, MMCSD_CMD6, H616_CMD6_ARG(index, value)) != OK)
    {
      _info("DDR52: CMD6 %s (index %" PRIu32 " value %" PRIu32 "): "
            "send failed STAS=%08" PRIx32 "\n", tag, index, value,
            h616_getreg(priv, H616_SDXC_STAS));
      return -EIO;
    }

  if (h616_waitresponse(dev, MMCSD_CMD6) != OK)
    {
      _info("DDR52: CMD6 %s: no response RINTR=%08" PRIx32 " STAS=%08"
            PRIx32 "\n", tag, h616_getreg(priv, H616_SDXC_RINTR),
            h616_getreg(priv, H616_SDXC_STAS));
      return -EIO;
    }

  resp = h616_getreg(priv, H616_SDXC_RESP0);
  _info("DDR52: CMD6 %s: OK R1=%08" PRIx32 " state=%" PRIu32
        "\n", tag, resp, (resp >> 9) & 0xfu);
  if ((resp & (H616_R1_SWITCH_ERROR | H616_R1_ILLEGAL_COMMAND)) != 0)
    {
      _info("DDR52: CMD6 %s: R1 error=%08" PRIx32 " state=%"
            PRIu32 "\n", tag, resp, (resp >> 9) & 0xfu);
      return -EIO;
    }

  return OK;
}

/* Read one block and say what happened to its bytes, even when the transfer
 * itself reported an error.
 *
 * The verify path above stops at the first eventwait() that does not report
 * TRANSFERDONE and never looks at the buffer, which answers the wrong
 * question in phase 2.  A DDR52 read that fails on the trailing end bit
 * still
 * moves the whole block (CBCR ends at BCNTR, RX_DATA_REQUEST stays asserted,
 * STAS carries FIFO_RUN_ERROR/END_BIT_ERROR) and those bytes are sitting in
 * the destination buffer.  Only a compare can say whether that sample point
 * samples the bus correctly, so this reads the block, prints the transfer
 * state and then reports the comparison against the reference whether or not
 * the transfer was declared good.
 */

static int h616_ddr52_read_once(struct h616_sdhci_priv_s *priv,
                                uint32_t lba, FAR const uint32_t *expect,
                                int attempt)
{
  struct sdio_dev_s *dev = &priv->dev;
  sdio_eventset_t events;
  uint32_t r1;
  uint32_t changed = 0;
  uint32_t bad = 0;
  int first_bad = -1;
  int i;

  for (i = 0; i < 128; i++)
    {
      g_ladder_buf[i] = 0xdeadbeefu;
    }

  h616_blocksetup(dev, 512, 1);
  h616_waitenable(dev, SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT |
                  SDIOWAIT_ERROR, 0);
  h616_recvsetup(dev, (FAR uint8_t *)g_ladder_buf, 512);

  if (h616_sendcmd(dev, MMCSD_CMD17, lba) != OK ||
      h616_waitresponse(dev, MMCSD_CMD17) != OK)
    {
      _info("DDR52: lba %08" PRIx32 " attempt %d/3 never went out "
            "(STAS=%08" PRIx32 " RINTR=%08" PRIx32 " CMDR=%08" PRIx32
            ")\n", lba, attempt,
            h616_getreg(priv, H616_SDXC_STAS),
            h616_getreg(priv, H616_SDXC_RINTR),
            h616_getreg(priv, H616_SDXC_CMDR));
      return -EIO;
    }

  r1     = h616_getreg(priv, H616_SDXC_RESP0);
  events = h616_eventwait(dev);

  for (i = 0; i < 128; i++)
    {
      if (g_ladder_buf[i] != 0xdeadbeefu)
        {
          changed++;
        }

      if (g_ladder_buf[i] != expect[i])
        {
          bad++;
          if (first_bad < 0)
            {
              first_bad = i;
            }
        }
    }

  _info("DDR52: lba %08" PRIx32 " attempt %d/3 events=%02x R1=%08"
        PRIx32 " CBCR=%08" PRIx32 " BCNTR=%08" PRIx32 " STAS=%08" PRIx32
        " RINTR=%08" PRIx32 " moved=%" PRIu32 "/128 mismatched=%" PRIu32
        " first_bad=%d\n",
        lba, attempt, (unsigned int)events, r1,
        h616_getreg(priv, H616_SDXC_CBCR),
        h616_getreg(priv, H616_SDXC_BCNTR),
        h616_getreg(priv, H616_SDXC_STAS),
        h616_getreg(priv, H616_SDXC_RINTR),
        changed, bad, first_bad);

  _info("DDR52: lba %08" PRIx32 " got %08" PRIx32 " %08" PRIx32
        " %08" PRIx32 " %08" PRIx32 " want %08" PRIx32 " %08" PRIx32
        " %08" PRIx32 " %08" PRIx32 "\n",
        lba, g_ladder_buf[0], g_ladder_buf[1], g_ladder_buf[2],
        g_ladder_buf[3], expect[0], expect[1], expect[2], expect[3]);

  if (changed == 0)
    {
      _info("DDR52: lba %08" PRIx32 " moved no data at all\n", lba);
      return -EIO;
    }

  return bad == 0 ? OK : -EIO;
}

/* The same read, tried a few times the way the vendor driver retries.
 *
 * A transfer that starts right after the card clock is reprogrammed usually
 * fails without ever beginning its data phase, while the one after it moves
 * the block: a retry is what carries the transfer (sunxi_mmc_judge_retry_
 * v4p6x() exists in the vendor driver for exactly this).  Every attempt
 * re-arms the transfer and re-poisons the destination, so a later attempt
 * can never be credited with an earlier one's bytes.
 */

static int h616_ddr52_read_anyway(struct h616_sdhci_priv_s *priv,
                                  uint32_t lba, FAR const uint32_t *expect)
{
  int attempt;
  int ret = -EIO;

  for (attempt = 1; attempt <= 3; attempt++)
    {
      ret = h616_ddr52_read_once(priv, lba, expect, attempt);
      if (ret == OK)
        {
          break;
        }

      up_mdelay(1);
    }

  return ret;
}

/* Un-wedge the controller after a failed data phase.  A read that ended in a
 * start bit error -- which is exactly what a host/card width mismatch
 * produces -- leaves the CIU answering nothing at all: RINTR stays 0, CMDR
 * never completes, and every later command times out.  That is why the first
 * bus width probe could not even write the width back.  Same reset the
 * driver's own reset path uses, and the CCU clock write has to follow it
 * with no command in between.
 */

static void h616_ladder_fixciu(struct h616_sdhci_priv_s *priv)
{
  volatile int t = 100000;

  h616_putreg(priv, H616_SDXC_GCTRL, H616_SDXC_SOFT_RESET |
              H616_SDXC_FIFO_RESET | H616_SDXC_DMA_RESET);
  while ((h616_getreg(priv, H616_SDXC_GCTRL) &
          (H616_SDXC_SOFT_RESET | H616_SDXC_FIFO_RESET |
           H616_SDXC_DMA_RESET)) != 0 && --t != 0);

  h616_putreg(priv, H616_SDXC_GCTRL, H616_SDXC_INTERRUPT_ENABLE);
  h616_putreg(priv, H616_SDXC_RINTR, 0xffffffff);
  h616_clockenable(priv, H616_SDMMC_IDCLK);
  h616_setclock(priv, H616_SDMMC_IDCLK);
}

/* Timing the card and host return to when HS400 is left.  g_ddr52_restore
 * is the 50MHz DDR52 profile the boot verified; the HS400 values below are
 * the pair this board's card was measured to work with (samp_dl=0x18,
 * ds_dl=0x0d).
 */

static const struct h616_ddr52_profile_s g_ddr52_restore =
{
  "ddr-restore", 200000000u, 1u, 0u, 0x00010000u, 0x00000000u,
  H616_DDR52_DLY_UNTOUCHED, 3u, false, true
};

/* Re-program the three delay registers to the known good HS400 timing
 * WITHOUT touching CLKCR.  A controller reset (h616_ladder_fixciu) clears
 * the card clock enable inside the CIU, and an HS400 card that loses its
 * clock stops answering for good: with the reset first, a teardown after
 * even 36 perfectly healthy sweep reads left CMD13 and CMD6 at RINTR =
 * COMMAND_DONE + RESP_TIMEOUT, RESP0 = 0.  Delay-register writes change the
 * sampling phase while the clock keeps running.
 */

static void h616_hs400_retime(struct h616_sdhci_priv_s *priv)
{
  uint32_t regval;

  regval  = h616_getreg(priv, H616_SDXC_SAMP_DL);
  regval &= ~(H616_SDXC_DLY_VALUE_MASK | H616_SDXC_DLY_ENABLE);
  regval |= 0x18u | H616_SDXC_DLY_ENABLE;
  h616_putreg(priv, H616_SDXC_SAMP_DL, regval);

  regval  = h616_getreg(priv, H616_SDXC_DS_DL);
  regval &= ~(H616_SDXC_DLY_VALUE_MASK | H616_SDXC_DLY_ENABLE);
  regval |= 0x0du | H616_SDXC_DLY_ENABLE;
  h616_putreg(priv, H616_SDXC_DS_DL, regval);

  regval  = h616_getreg(priv, H616_SDXC_DRV_DL);
  regval &= ~H616_DDR52_DRV_PH_MASK;
  h616_putreg(priv, H616_SDXC_DRV_DL, regval);

  up_mdelay(2);
}

/* Take the card and the host out of HS400 and land on 8 bit DDR52.
 *
 * The order is the opposite of a normal mode change:
 *
 *   1. re-time the host in place (clock keeps running), abort a possible
 *      data phase with CMD12, wait for the card,
 *   2. CMD6 HS_TIMING = 1 while the clock runs -- the card leaves HS400,
 *   3. only NOW may the controller be reset and the clock re-programmed:
 *      clock stop/start is fine in DDR52, not in HS400.
 */

static bool h616_hs400_leave(struct h616_sdhci_priv_s *priv,
                             FAR const char *why)
{
  int attempt;

  if (!priv->hs400)
    {
      return true;
    }

  for (attempt = 1; attempt <= 3; attempt++)
    {
      h616_hs400_retime(priv);
      h616_cmd12_abort(priv);
      up_mdelay(2);

      h616_ladder_ready(priv, "exit HS400: host retimed");

      if (h616_ddr52_cmd6(priv, H616_EXT_CSD_HS_TIMING, H616_HS_TIMING_HS,
                          "exit HS400: HS_TIMING=1") == OK)
        {
          break;
        }
    }

  if (attempt > 3)
    {
      _err("ERROR: %s: the card refused HS_TIMING=1, still at HS400\n", why);
      return false;
    }

  priv->hs400 = false;

  /* The card is out of HS400 now: reset the controller and move the card
   * clock to the DDR52 rate, then wait for the card to latch it.
   */

  h616_ladder_fixciu(priv);
  h616_ddr52_apply(priv, &g_ddr52_restore);
  h616_ladder_ready(priv, "exit HS400: card at DDR52");
  _info("widebus: HS400 -> 8 bit DDR52 (%s, attempt %d)\n", why, attempt);
  return true;
}

static bool h616_ddr52_restore_sdr(struct h616_sdhci_priv_s *priv)
{
  static const struct h616_ddr52_profile_s sdr =
  {
    "sdr-restore", 100000000u, 0u, H616_SDXC_NTSR_2X_TIMING,
    0x00000000u, 0x00001898u, 0x00000d8du, 0u, false, false
  };

  int attempt;

  for (attempt = 1; attempt <= 3; attempt++)
    {
      h616_ladder_fixciu(priv);
      h616_ddr52_apply(priv, &sdr);
      h616_setbuswidth(priv, 8);
      up_mdelay(H616_LADDER_SETTLE_MS);

      h616_ladder_ready(priv, "restore");

      if (h616_ddr52_cmd6(priv, H616_EXT_CSD_BUS_WIDTH, H616_BUS_WIDTH_8BIT,
                          "restore BUS_WIDTH=2") != OK)
        {
          continue;
        }

      h616_ladder_ready(priv, "restore BUS_WIDTH=2");

      if (h616_bus_verify(priv))
        {
          _info("DDR52: card back at 8 bit SDR (restore attempt %d)\n",
                attempt);
          return true;
        }

      _info("DDR52: restore attempt %d did not read back\n", attempt);
    }

  return false;
}

static bool h616_ddr52_recover(struct h616_sdhci_priv_s *priv)
{
  /* A card parked in HS400 cannot answer a host that has already dropped to
   * SDR: leave HS400 first, and only fall through to the SDR restore once
   * the card is back on HS_TIMING = 1.  If HS400 cannot even be left there
   * is no safe bus left to advertise, so report the failure instead of
   * pretending an SDR switch worked.
   */

  if (priv->hs400 && !h616_hs400_leave(priv, "recover"))
    {
      return false;
    }

  return h616_ddr52_restore_sdr(priv);
}

/* HS400 boot attempt (H616_LADDER_HS400 = 1).  Runs from the DDR52 boot
 * path once 8 bit DDR52 has verified byte for byte, so the card is in
 * HS_TIMING = 1 with BUS_WIDTH = 6 -- the state from which it accepts
 * HS_TIMING = 3 -- and both reference blocks are filled.
 *
 * The host moves to strobe sampling first (DSBD bit31, CSDC = 6, DDR mode
 * bit cleared, command output phase 90 degrees, module clock 200MHz
 * CLKCR = 1 for the 100MHz card clock), then the card is switched with
 * CMD6 HS_TIMING = 3 and the 4 x 4 sample/data-strobe delay table inherited
 * from the ladder step is swept until both reference blocks read back
 * byte for byte.  samp_dl/ds_dl are reprogrammed directly per sweep step
 * (h616_ladder_delays); the clock and strobe setup come from the profile.
 *
 * Failure rolls back to the verified DDR52 profile and, if even that does
 * not read back, to 8 bit SDR via the DDR52 recovery path, so every outcome
 * leaves a usable card on a verified bus.
 *
 * While this is enabled the capabilities do not advertise
 * SDIO_CAPS_MMC_HS_MODE: mmcsd would send its own CMD6 HS_TIMING = 1 right
 * after widebus, which on a card parked in HS_TIMING = 3 is not a no-op but
 * a downgrade to DDR52 with the host still in strobe mode.
 */

static bool h616_hs400_boot(struct h616_sdhci_priv_s *priv)
{
  static const struct h616_ddr52_profile_s hs400 =
  {
    "boot-hs400", 200000000u, 0u, 0u, 0x00000000u, 0x00000011u,
          /* N53b: clkdiv 0 => mod 200MHz / 2 = 100MHz card clock, matching
           * Linux sunxi-mmc-v4p5x (HS400 = mod card<<1, div 0).  clkdiv 1
           * was a half-speed HS400 that killed TX.
           */

    0x00000018u, 6u, true, false
  };

  /* h400d8: same re-ordering as the boot sweep -- lead with the pair the
   * boot_package U-Boot had tuned and left behind at handoff (samp=0x18,
   * ds=0x0d), see the h400d8 comment in the boot ladder sweep.
   */

  static const uint32_t sampdl[6] =
  {
    0x18, 0x12, 0x1f, 0x11, 0x1a, 0x14
  };

  static const uint32_t dsdl[6] =
  {
    0x0d, 0x18, 0x12, 0x06, 0x03, 0x08
  };

  int i;
  int j;
  bool ok = false;

  priv->hs400 = false;

  /* Host first: strobe sampling at the HS400 clock while the card is still
   * in 8 bit DDR52.  The profile clears the DDR mode bit, sets DSBD and
   * CSDC = 6, and brings the 100MHz card clock up with the bus quiet.
   */

  h616_ddr52_apply(priv, &hs400);
  h616_ddr52_log(priv, "boot-hs400-host", &hs400);

  if (h616_ddr52_cmd6(priv, H616_EXT_CSD_HS_TIMING, H616_HS_TIMING_HS400,
                      "enter HS400: HS_TIMING=3") != OK)
    {
      _info("HS400: the card rejected HS_TIMING=3, staying at DDR52\n");

      /* The card stayed at DDR52 while the host is already on the HS400
       * 100MHz strobe profile; put the host back on the DDR52 timing that
       * matches the card before returning.
       */

      h616_ddr52_apply(priv, &g_ddr52_restore);
    }
  else
    {
      /* The card has accepted HS_TIMING = 3; from here until HS_TIMING = 1
       * is written back the card only answers at HS400 timing, so the exit
       * path must know that even when the sweep fails.
       */

      priv->hs400 = true;
      h616_ladder_ready(priv, "HS400 switch");

      for (i = 0; i < 6 && !ok; i++)
        {
          for (j = 0; j < 6; j++)
            {
              h616_ladder_delays(priv, sampdl[i], dsdl[j]);
              _info("ladder: HS400 cell %d/%d samp=%02" PRIx32
                    " ds=%02" PRIx32 "\n", i, j, sampdl[i], dsdl[j]);
              if (h616_ladder_read(priv, H616_LADDER_LBA,
                                   g_ladder_ref) == OK &&
                  h616_ladder_read(priv, H616_LADDER_LBA2,
                                   g_ladder_ref2) == OK)
                {
                  ok = true;
                  _info("ladder: HS400 verified samp_dl=%02" PRIx32
                        " ds_dl=%02" PRIx32 " (2 lbas, byte compared)\n",
                        sampdl[i], dsdl[j]);
                  break;
                }
            }
        }
    }

  if (ok)
    {
      h616_ladder_speed(priv, "HS400");
      return true;
    }

  _info("ladder: HS400 verify FAILED, back to 8 bit DDR52\n");

  /* Leave through the known good HS400 timing, not through the cell that
   * just failed, then verify the DDR52 profile the boot already trusted.
   *
   */

  if (h616_hs400_leave(priv, "HS400 verify failed") &&
      h616_ddr52_read_anyway(priv, H616_LADDER_LBA2, g_ladder_ref2) == OK &&
      h616_ddr52_read_anyway(priv, H616_LADDER_LBA, g_ladder_ref) == OK)
    {
      h616_ladder_speed(priv, "8 bit DDR52");
      return false;
    }

  _err("ERROR: HS400: DDR52 restore did not verify, trying SDR recovery\n");
  h616_ddr52_recover(priv);
  return false;
}

#if H616_LADDER_HS400
/* Leave HS400: the card gives up HS_TIMING = 3 while the host still drives
 * the HS400 clock, then the host drops back to the DDR52 timing that the
 * normal h616_widebus(false) path already knows how to unwind (DDR52 -> 8
 * bit SDR -> 1 bit).  Without this the card would stay at HS_TIMING = 3
 * while the host switches to single-edge 1 bit, and the next CMD6 would
 * write
 * BUS_WIDTH = 0 into a card in the wrong speed mode.
 */

static void h616_hs400_exit(struct h616_sdhci_priv_s *priv)
{
  if (!h616_hs400_leave(priv, "widebus"))
    {
      _info("widebus: HS400 teardown failed, bus left unchanged\n");
    }
}
#endif

/* N27: this card only leaves 8 bit SDR when the bus width is written before
 * HS_TIMING.  With the JEDEC order (HS_TIMING first) it accepts both writes,
 * reports no SWITCH_ERROR and keeps driving single rate -- which is why
 * every
 * host DDR timing before this was measured against a bus the card was not
 * on.
 * The proof is in the SDR oracle: after this order the very same read comes
 * back with data moved and every host byte being every other card byte.
 *
 */

static int h616_ddr52_enter_ddr(struct h616_sdhci_priv_s *priv)
{
  int ret;

  ret = h616_ddr52_cmd6(priv, H616_EXT_CSD_BUS_WIDTH,
                        H616_BUS_WIDTH_8BIT_DDR,
                        "enter DDR: BUS_WIDTH=6 first");
  if (ret != OK)
    {
      return ret;
    }

  ret = h616_ddr52_cmd6(priv, H616_EXT_CSD_HS_TIMING, H616_HS_TIMING_HS,
                        "enter DDR: HS_TIMING=1 second");
  if (ret != OK)
    {
      return ret;
    }

  h616_ladder_ready(priv, "entering 8 bit DDR");
  return OK;
}

/* Switch the boot bus to 8 bit DDR52 and keep it, using the recipe N35
 * verified byte for byte:
 *
 *   module clock 200MHz (4x the 50MHz card clock), CLKCR[7:0] = 1,
 *   no 2x timing, DRV_DL = 0x00010000 (command output phase 180 degrees),
 *   SAMP_DL low six bits 0 (the sample point that carried data),
 *   DS_DL left at the handover value, CSDC[3:0] = 3, HS400 strobe bit clear,
 *   host DDR mode on.
 *
 * What made this work, after N19..N33 guessed at the numbers: the host
 * change
 * is applied by one CIU clock update (CMDR START|UPCLK_ONLY) with the card
 * clock stopped, and the DDR mode bit is written with the bus clocked -- the
 * CCU gate that the vendor's clk_mmc disable maps to on this controller
 * drops
 * the GCTRL write (N34 measured exactly that).
 *
 * The bus is only kept when two different reference blocks come back byte
 * for
 * byte; otherwise the card is put back on the 8 bit SDR timing the boot
 * verified before this function returns.
 */

static bool h616_ddr52_boot(struct h616_sdhci_priv_s *priv)
{
  static const struct h616_ddr52_profile_s boot =
  {
    "boot-ddr52", 200000000u, 1u, 0u, 0x00010000u, 0x00000000u,
    H616_DDR52_DLY_UNTOUCHED, 3u, false, true
  };

  if (h616_ddr52_enter_ddr(priv) != OK)
    {
      _info("ladder: DDR52: the card refused the switch, staying at 8 bit "
            "SDR\n");
      return false;
    }

  h616_ddr52_apply(priv, &boot);
  h616_ddr52_log(priv, "boot-ddr52", &boot);

  if (h616_ddr52_read_anyway(priv, H616_LADDER_LBA2, g_ladder_ref2) == OK &&
      h616_ddr52_read_anyway(priv, H616_LADDER_LBA, g_ladder_ref) == OK)
    {
      _info("ladder: 8 bit DDR52 verified (2 lbas, byte compared) GCTRL=%08"
            PRIx32 " CLKCR=%08" PRIx32 " SAMP_DL=%08" PRIx32 " DS_DL=%08"
            PRIx32 " DRV_DL=%08" PRIx32 " CSDC=%08" PRIx32 " CCU838=%08"
            PRIx32 "\n",
            h616_getreg(priv, H616_SDXC_GCTRL),
            h616_getreg(priv, H616_SDXC_CLKCR),
            h616_getreg(priv, H616_SDXC_SAMP_DL),
            h616_getreg(priv, H616_SDXC_DS_DL),
            h616_getreg(priv, H616_SDXC_DRV_DL),
            h616_getreg(priv, H616_SDXC_CSDC),
            getreg32(H616_CCU_ADDR + H616_CCU_SD2_CLK_CFG));

      h616_ladder_speed(priv, "8 bit DDR52");
#if H616_LADDER_HS400
      h616_hs400_boot(priv);
#endif
      return true;
    }

  _info("ladder: DDR52: the data did not verify, back to 8 bit SDR\n");

  if (!h616_ddr52_recover(priv))
    {
      /* The card would not come back on its own; put the host back on the
       * timing that was verified and let mmcsd see whatever state that
       * leaves.  This is the branch that has never been reached on hardware.
       *
       */

      _err("ERROR: ladder: the card did not return to 8 bit SDR\n");
    }

  return false;
}
#endif /* H616_DDR52_SUPPORT */

static void h616_widebus(struct sdio_dev_s *dev, bool enable)
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;

  /* The driver owns the bus width, because only it can move the card and the
   * host together.
   *
   * The mmcsd layer has no notion of an 8 bit bus: for MMC it writes
   * EXT_CSD BUS_WIDTH = 1 (4 bit) when it thinks it is appropriate and then
   * calls here with enable = true, leaving the choice to the driver ("it may
   * well be 1, 4 or 8 bits").  The card is at 1 bit when this function runs
   * -- the error this driver was fixed for was exactly a mismatch here
   * (h616_reset() leaves the card at its 1 bit power-on default, and a host
   * set to 8 bit while the card is at 1 bit drives no data at all, CBCR =
   * 0) -- so the safe sequence is: settle the card at 1 bit on both sides,
   * take reference reads, then switch the card with CMD6 and the host with
   * h616_setbuswidth() in that order.
   *
   * The switch is then *verified* before the mmcsd layer gets to use the
   * bus.  On hardware the first data transfer after a bus width change has
   * been observed to never start its data phase at all (RINTR = 0, CBCR = 0,
   * no controller timeout either) while the retry immediately after it
   * transfers the correct data -- so a mode is only accepted once real data
   * has come across it, and a retry absorbs that first transfer.  Both
   * reference blocks are re-read into a poisoned destination and byte
   * compared, so a transfer that moved nothing, or that replayed stale FIFO
   * content, cannot pass.  The references themselves are only trusted when
   * two reads of the same block agree, because a reference read that came
   * back as noise would make this comparison reject perfectly good 8 bit
   * data (observed on this board).  If any of that fails the bus goes back
   * to 1 bit
   * and the boot continues in the configuration that is known to work.
   */

  if (!enable)
    {
      /* HS400 first: the card has to leave HS_TIMING = 3 while the host
       * still clocks it at HS400, then the host drops to the verified DDR52
       * timing that the branch below unwinds.  Leaving the card in HS400
       * while the host moves to 1 bit SDR is exactly the mismatch this step
       * prevents.
       *
       */

#if H616_DDR52_SUPPORT && H616_LADDER_HS400
      h616_hs400_exit(priv);
#endif

      /* Drop out of DDR52 before the width: lower the clock first, then the
       * card, then the host sampling and the delays.
       */

      if (priv->ddr)
        {
          /* Host back onto the single edge timing first (the card is still
           * clocked by it), then the card out of DDR: bus width before
           * timing, JEDEC order.
           */

          h616_bus_apply(priv, &g_sdr25_profile);
          h616_ladder_cmd6(priv, H616_EXT_CSD_BUS_WIDTH,
                           H616_BUS_WIDTH_8BIT);
          h616_ladder_cmd6(priv, H616_EXT_CSD_HS_TIMING, 0);
        }

      if (priv->wide)
        {
          h616_ladder_cmd6(priv, H616_EXT_CSD_BUS_WIDTH, 0);
        }

      h616_setbuswidth(priv, 1);
      priv->wide = false;
      _info("widebus: 1 bit (host and card)\n");
      return;
    }

  if (priv->wide && h616_getreg(priv, H616_SDXC_WIDTH) == H616_SDXC_WIDTH8)
    {
      _info("widebus: already at 8 bit\n");
      return;
    }

  /* Make sure the card really is at 1 bit before taking references, whatever
   * the layer above may have programmed.  BUS_WIDTH = 0 is legal and is the
   * power-on default, so this is a no-op in the normal case.
   */

  h616_ladder_cmd6(priv, H616_EXT_CSD_BUS_WIDTH, 0);
  h616_setbuswidth(priv, 1);

  /* Let the card finish the internal programming cycle the CMD6 started
   * before the first transfer is issued.
   */

  up_mdelay(H616_LADDER_SETTLE_MS);

  /* References are only accepted when two reads of the same block agree,
   * because there is nothing else they can be checked against.
   */

  if (h616_ladder_refread(priv, H616_LADDER_LBA, g_ladder_ref,
                          H616_LADDER_REF_TRIES) != OK ||
      h616_ladder_refread(priv, H616_LADDER_LBA2, g_ladder_ref2,
                          H616_LADDER_REF_TRIES) != OK)
    {
      _info("widebus: 1 bit reference read failed, staying at 1 bit\n");
      return;
    }

  _info("widebus: 1 bit references OK (lba %08" PRIx32 "/%08" PRIx32 ")\n",
        (uint32_t)H616_LADDER_LBA, (uint32_t)H616_LADDER_LBA2);

  /* Baseline before anything is changed.
   */

  h616_ladder_speed(priv, "1 bit");

  /* Card first, then host: the width only matches once both have moved.
   */

  if (h616_ladder_cmd6(priv, H616_EXT_CSD_BUS_WIDTH,
                       H616_BUS_WIDTH_8BIT) != OK)
    {
      _info("widebus: CMD6 BUS_WIDTH=8 rejected by the card, staying at 1 "
            "bit\n");
      return;
    }

  h616_setbuswidth(priv, 8);
  priv->wide = true;

  up_mdelay(H616_LADDER_SETTLE_MS);

  if (h616_ladder_read_try2(priv, H616_LADDER_LBA2, g_ladder_ref2) == OK &&
      h616_ladder_read_try2(priv, H616_LADDER_LBA, g_ladder_ref) == OK)
    {
      _info("widebus: 8 bit SDR verified (2 lbas, byte compared, WIDTH=%08"
            PRIx32 ")\n", h616_getreg(priv, H616_SDXC_WIDTH));

      h616_ladder_speed(priv, "8 bit SDR 25M");

#if H616_WIDEBUS_SDR50
      /* Next step: the same single edge protocol at twice the clock.  The
       * card has to be in high speed timing for that, and the host has to be
       * driven the way the vendor drives it -- a module clock above the card
       * clock, with the controller's internal divider making up the
       * difference.  A failure here falls back to the profile that was just
       * verified.
       */

        {
          struct h616_bus_profile_s kept = g_sdr25_profile;

          if (h616_ladder_sdr50(priv, &kept))
            {
              h616_ladder_speed(priv, "8 bit SDR 50M");
            }
          else
            {
              h616_bus_apply(priv, &g_sdr25_profile);
              if (h616_bus_verify(priv))
                {
                  _info("widebus: back at 8 bit SDR 25M\n");
                }
              else
                {
                  _info("widebus: 8 bit SDR 25M did not come back\n");
                }
            }

  #if H616_WIDEBUS_DDR52
          /* And then the real DDR52 step, taken from the 8 bit SDR bus that
           * just verified.  The card side switch order, the host timing and
           * the way the host clock change is applied are the ones N35
           * verified
           * byte for byte (see h616_ddr52_boot); if the data does not come
           * back identical the card is put back on the SDR timing before
           * this
           * returns, so the mount below never runs on a half switched bus.
           *
           */

          if (!h616_ddr52_boot(priv))
            {
              _info("widebus: staying at 8 bit SDR\n");
            }
  #endif
        }
#endif

      if (h616_ladder_extcsd(priv, (FAR uint8_t *)g_ladder_scratch) == OK)
        {
          FAR uint8_t *e = (FAR uint8_t *)g_ladder_scratch;

          _info("widebus: card extcsd hs_timing=%u device_type=%02x\n",
                e[185], e[196]);
        }
      else
        {
          _info("widebus: card extcsd read failed\n");
        }

      return;
    }

  _info("widebus: 8 bit SDR verify FAILED, back to 1 bit\n");
  h616_ladder_cmd6(priv, H616_EXT_CSD_BUS_WIDTH, 0);
  h616_setbuswidth(priv, 1);
  priv->wide = false;
}

/* Set the clock.  The rate is applied through the CCU module clock and
 * re-triggered via the controller.
 */

static void h616_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;
  uint32_t clk;

  switch (rate)
    {
    case CLOCK_SDIO_DISABLED:
      h616_modifyreg(priv, H616_SDXC_CLKCR, H616_SDXC_CARD_CLOCK_ON, 0);
      break;

    case CLOCK_IDMODE:
      /* Unexpectedly the driver stack calls into h616_clock here, and it
       * runs the full setclock path: enable the CCU module clock, then
       * trigger a controller clock update.  The UPCLK trigger re-programs
       * the controller's clock path and was the piece that finally broke
       * the controller.  Probe before/after each step so the next boot log
       * shows exactly where the card clock stops being usable.
       */

      h616_clockenable(priv, H616_SDMMC_IDCLK);

      /* Re-enable the card clock with a plain register write.  Do NOT run
       * h616_setclock() here: its UPCLK trigger is what killed the
       * controller (probe[CLOCK_IDMODE] with CLKCR=0 -> OK,
       * probe[CLOCK_IDMODE-setclock] with CLKCR=0x10000 -> DEAD), because
       * it clears CARD_CLOCK_ON before issuing the UPCLK command and the
       * controller then never completes another command.
       */

      h616_modifyreg(priv, H616_SDXC_CLKCR, 0, H616_SDXC_CARD_CLOCK_ON);

      H616_DIAG("SDIO: CLOCK_IDMODE CLKCR=%08" PRIx32 " STAS=%08" PRIx32
            " CCU838=%08" PRIx32 "\n",
            h616_getreg(priv, H616_SDXC_CLKCR),
            h616_getreg(priv, H616_SDXC_STAS),
            getreg32(H616_CCU_ADDR + H616_CCU_SD2_CLK_CFG));

      h616_probe_cmd0(priv, "CLOCK_IDMODE");
      break;

    case CLOCK_MMC_TRANSFER:
    case CLOCK_SD_TRANSFER_1BIT:
    case CLOCK_SD_TRANSFER_4BIT:
      /* Data transfers need a real transfer clock.  Every path that ever
       * completed a data phase on this board ran the card at 100 MHz
       * (U-Boot's working CMD17/CMD8 reads); every path that failed was
       * still at the 400 kHz identification clock, where the CIU never even
       * starts counting (CBCR stays 0 and DATA_OVER never fires).
       *
       * U-Boot reaches 100 MHz from the 1.2 GHz PLL6x2 source with
       * M = 12 (1200 / 12 = 100 MHz), N = 0, no controller divider.  Use the
       * same module clock, but do NOT trigger a UPCLK here: on this part a
       * UPCLK while the card clock is being toggled wedges the controller
       * (see h616_reset notes).  Just program the CCU divider and re-enable
       * the card clock with a plain write.
       */

      /* Use the module clock and the internal divider belonging to the bus
       * mode that widebus() last verified rather than a fixed rate.  The
       * vendor scheme feeds the controller a multiple of the card clock and
       * divides it down internally, so programming only the CCU here would
       * silently change the card clock of a mode that is already in use --
       * and the mmcsd layer calls this again right after it has switched the
       * card to high speed timing.
       */

      clk = priv->bus_mod_hz != 0 ? priv->bus_mod_hz : H616_SDMMC_MAXCLK;
      h616_clockenable(priv, clk);
      h616_modifyreg(priv, H616_SDXC_CLKCR, 0xffu, priv->bus_clkdiv);
      h616_modifyreg(priv, H616_SDXC_CLKCR, 0, H616_SDXC_CARD_CLOCK_ON);

      H616_DIAG("DIAG: XFER clock CCU838=%08" PRIx32 " CLKCR=%08" PRIx32
            " WIDTH=%08" PRIx32 " GCTRL=%08" PRIx32 " NTSR=%08" PRIx32
            "\n", getreg32(H616_CCU_ADDR + H616_CCU_SD2_CLK_CFG),
            h616_getreg(priv, H616_SDXC_CLKCR),
            h616_getreg(priv, H616_SDXC_WIDTH),
            h616_getreg(priv, H616_SDXC_GCTRL),
            h616_getreg(priv, H616_SDXC_NTSR));

      /* This is the last call of the mmcsd bring-up sequence, so it is the
       * only place left where the card can still be moved to a faster bus
       * width or speed mode (see the ladder above).
       */

      /* Kept in the binary (and compiler-visible) so re-enabling the ladder
       * for the next test is a one line change; mmcdiag can also drive the
       * same steps from the shell.
       */

      if (H616_LADDER_AT_BOOT)
        {
          h616_ladder(priv);
        }

      break;
    }
}

/* Attach the interrupt handler
 */

#ifdef H616_SDIO_USEIRQ
/****************************************************************************
 * Name: h616_interrupt
 *
 *   The SDXC GIC handler.  Latches every fired RINTR bit for the polling
 *   wait loops, clears the raw status (so the level line drops and no
 *   storm can develop) and wakes whichever waiter the event class belongs
 *   to.  The wait loops themselves still poll the latch + raw register, so
 *   this only accelerates them.
 *
 ***************************************************************************
 */

static int h616_interrupt(int irq, FAR void *context, FAR void *arg)
{
  struct h616_sdhci_priv_s *priv = &g_sdio;
  uint32_t rintr = h616_getreg(priv, H616_SDXC_RINTR);
  uint32_t imask;

  if (rintr == 0)
    {
      return OK;
    }

  imask = h616_getreg(priv, H616_SDXC_IMASK);

  /* Latch first, then clear every bit that fired.  Clearing the whole
   * snapshot (not just the bits we know) is what prevents the historical
   * interrupt storm: no raw bit is ever left asserted.
   */

  priv->cmdrintr   |= rintr & H616_SDIO_CMDBITS;
  priv->rintr_latch |= rintr;

  h616_putreg(priv, H616_SDXC_RINTR, rintr);

  if ((rintr & H616_SDIO_CMDBITS) != 0)
    {
      nxsem_post(&priv->cmdsem);
    }

  /* Only report data completion when the data sources are actually
   * armed (h616_configxfrints); stray data-class bits from an unmasked
   * source are just cleared above.
   */

  if ((rintr & imask & (H616_SDXC_DATA_OVER | H616_SDXC_DATA_CRC_ERROR |
                        H616_SDXC_DATA_TIMEOUT |
                        H616_SDXC_FIFO_RUN_ERROR)) != 0)
    {
      if ((rintr & (H616_SDXC_DATA_CRC_ERROR | H616_SDXC_DATA_TIMEOUT |
                    H616_SDXC_FIFO_RUN_ERROR)) != 0)
        {
          priv->wkupevent |= SDIOWAIT_ERROR;
        }

      nxsem_post(&priv->waitsem);
    }

  return OK;
}
#endif

static int h616_attach(struct sdio_dev_s *dev)
{
  /* mmcsd_bind() calls this from board initialization - BEFORE
   * arm64_gic_initialize() runs.  Any GIC write made here (enable,
   * trigger config) is wiped when the GIC is initialized afterwards
   * (measured: ISENABLER read back 0 and ICFGR still 0x55555555 after
   * boot).  The real attachment therefore happens lazily, on the first
   * command - see h616_irq_lateinit().
   *
   * N72 history: the very first IRQ attempt stormed the CPU, and the
   * conclusion then was "never attach the interrupt".  The storm had two
   * real causes that are both gone now:
   *
   *   1. The IRQ number came from the A64-derived irq.h and was simply
   *      wrong (94 instead of 69), so the line was never this device's.
   *   2. The old ISR only cleared a fixed subset of the raw bits, so a
   *      bit it did not know (AUTO_COMMAND_DONE, ...) stayed asserted.
   *
   * The handler above clears its whole snapshot and the waits below keep
   * polling the raw register, so correctness no longer depends on the
   * interrupt at all.
   */

  return OK;
}

#ifdef H616_SDIO_USEIRQ
static void h616_irq_lateinit(struct h616_sdhci_priv_s *priv)
{
  static bool done;

  if (done)
    {
      return;
    }

  done = true;

  /* GICD at 0x03021000, ICFGR at +0xc00 (see h616_dma.c).  Configure the
   * SPI level-triggered like the DMAC: the bootloader marks every SPI
   * edge-triggered, and with an edge config a level line that re-asserts
   * without a sampled low period can lose completions.
   */

    {
      uintptr_t icfgr = 0x03021000 + 0xc00 + (H616_SDMMC2_IRQ / 16) * 4;
      uint32_t shift  = (H616_SDMMC2_IRQ % 16) * 2;
      uint32_t val    = getreg32(icfgr) & ~(3u << shift);

      putreg32(val, icfgr);
    }

  h616_putreg(priv, H616_SDXC_IMASK, 0);   /* quiet until armed */
  h616_putreg(priv, H616_SDXC_RINTR, 0xffffffff);

  /* The one gate everything else was missing: GCTRL bit 4 routes the
   * masked interrupt status out of the controller to the GIC (seen in
   * Linux sunxi_mmc_init_host(): rval |= SDXC_INTERRUPT_ENABLE_BIT).
   * Without it MISTA asserts but the line stays low - exactly what the
   * pendscan/probe experiments measured.
   */

    {
      uint32_t gctl = h616_getreg(priv, H616_SDXC_GCTRL);

      gctl |= H616_SDXC_INTERRUPT_ENABLE;
      h616_putreg(priv, H616_SDXC_GCTRL, gctl);
    }

  irq_attach(H616_SDMMC2_IRQ, h616_interrupt, NULL);
  up_enable_irq(H616_SDMMC2_IRQ);
}
#endif

/* Send a command
 */

static int h616_sendcmd(struct sdio_dev_s *dev, uint32_t cmd, uint32_t arg)
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;
  int ret;

  /* Byte-for-byte mirror mmcdiag's send_cmd: no up_irq_save, no register
   * reads before issuing (reads may interfere with the command state
   * machine).  Just issue and return.
   */

  /* N44 debug tracing of the command stream was removed after N51: the
   * multi block write failure it hunted is understood now (see
   * MMCSD_NO_MULTIBLOCK_WRITE in drivers/mmcsd/Kconfig), and the trace
   * itself cost about 100 bytes of 115200 baud serial per data command.
   *
   */

  ret = h616_sendcmdhw(priv, cmd, arg);
  return ret;
}

/* Set up a block transfer.  sunxi-mmc: BLKSZ holds the block size only;
 * BCNTR holds the total byte count (blksz * nblocks).
 */

static void h616_blocksetup(struct sdio_dev_s *dev, unsigned int blocklen,
                            unsigned int nblocks)
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;

  /* Raise the card clock out of the 400 kHz identification rate before the
   * data command goes out.  The mmcsd layer only calls
   * SDIO_CLOCK(CLOCK_MMC_TRANSFER) from inside a CMD6/HS-timing branch that
   * this board never reaches, so without this the controller would issue
   * every data command -- including the very first CMD8 (SEND_EXT_CSD)
   * during card identification -- at 400 kHz, where the CIU never starts
   * counting (CBCR stays 0, DATA_OVER never fires).  Every working data
   * transfer measured on this board ran at ~50-100 MHz.
   *
   * Do NOT trigger a UPCLK: a UPCLK while the card clock is being toggled
   * wedges this controller (see the h616_reset notes).  Programming the CCU
   * divider plus a plain CARD_CLOCK_ON write is enough.
   */

  if (priv->clock < H616_SDMMC_MAXCLK)
    {
      /* Apply the transfer clock the way mmcdiag's clock_update() and
       * U-Boot's mmc_config_clock() do it, which is proven to work on this
       * board: mask DAT0, program the CCU divider, issue the controller's
       * UPCLK update command and wait for it to complete, then unmask DAT0.
       * Programming the CCU alone is NOT enough -- the controller's internal
       * clock path is only re-synchronised by the UPCLK trigger, and without
       * it the card sees a clock whose data phase never runs (CBCR and BBCR
       * both stay 0: the card transfers nothing).
       *
       * The earlier "UPCLK wedges the controller" finding only applied to
       * the old sequence that CLEARED CARD_CLOCK_ON first; keeping the card
       * clock on across the update is what makes this safe.
       */

      h616_clockenable(priv, H616_SDMMC_MAXCLK);

      h616_putreg(priv, H616_SDXC_CLKCR,
                  h616_getreg(priv, H616_SDXC_CLKCR) | H616_SDXC_MASK_DATA0);

      /* CLKCR divider stays divide-by-1; the rate comes from the CCU.
       */

      h616_putreg(priv, H616_SDXC_CMDR,
                  H616_SDXC_START | H616_SDXC_UPCLK_ONLY |
                  H616_SDXC_WAIT_PRE_OVER);

        {
          volatile int t = 100000;
          while ((h616_getreg(priv, H616_SDXC_CMDR) &
                  H616_SDXC_START) != 0 && --t != 0);
        }

      h616_modifyreg(priv, H616_SDXC_CLKCR, H616_SDXC_MASK_DATA0,
                     H616_SDXC_CARD_CLOCK_ON);
    }

  h616_putreg(priv, H616_SDXC_BLKSZ, blocklen);
  h616_putreg(priv, H616_SDXC_BCNTR, blocklen * nblocks);

  H616_DIAG("DIAG: blocksetup %u x %u -> BLKSZ=%08" PRIx32
        " BCNTR=%08" PRIx32 " CLKCR=%08" PRIx32 " CCU838=%08" PRIx32 "\n",
        blocklen, nblocks, h616_getreg(priv, H616_SDXC_BLKSZ),
        h616_getreg(priv, H616_SDXC_BCNTR),
        h616_getreg(priv, H616_SDXC_CLKCR),
        getreg32(H616_CCU_ADDR + H616_CCU_SD2_CLK_CFG));
}

/* Set to 1 to drive reads through the CPU/AHB FIFO path instead of the
 * IDMAC.  The IDMAC read never makes the card drive DAT (CBCR stays 0), so
 * this selects the FIFO path used by the vendor NuttX driver
 * (__mci_trans_by_ahb) and by this driver's earlier PIO implementation.
 * Those earlier PIO attempts all predate the reset-sequence fix, so the path
 * deserves a re-test on a healthy controller.
 */

/* H616_RX_USE_AHB is defined at the top of this file: the CPU/AHB FIFO
 * data path is used for both read and write transfers.
 */

/* Send CMD13 (SEND_STATUS) to the selected card and log R1 plus the state
 * field.  R1 bits [12:9] are CURRENT_STATE:
 * 0 idle 1 ready 2 ident 3 stby 4 tran 5 data 6 rcv 7 prg
 * Used around the data command: every data read transfers nothing (CBCR=0)
 * while all non-data commands work, so this shows whether the CARD thinks it
 * is in TRAN (ready) or has been left in DATA (mid-transfer).
 */

static void h616_dbg_card_state(struct h616_sdhci_priv_s *priv,
                                const char *tag)
{
  static const char *st[] =
    {
      "idle", "ready", "ident", "stby", "tran", "data",
      "rcv",  "prg",   "dis",   "btst", "slp"
    };

  uint32_t rintr = 0;
  uint32_t resp;
  uint32_t state;
  int t;

  h616_putreg(priv, H616_SDXC_RINTR, 0xffffffff);
  h616_putreg(priv, H616_SDXC_CARG, 0x00010000);  /* RCA 1 << 16 */
  h616_putreg(priv, H616_SDXC_CMDR,
              H616_SDXC_START | H616_SDXC_RESP_EXPECT |
              H616_SDXC_CHECK_RESPONSE_CRC | 13);

  for (t = 2000000; t > 0; t--)
    {
      rintr = h616_getreg(priv, H616_SDXC_RINTR);
      if ((rintr & (H616_SDXC_COMMAND_DONE | H616_SDXC_RESP_TIMEOUT)) != 0)
        {
          break;
        }
    }

  resp  = h616_getreg(priv, H616_SDXC_RESP0);
  state = (resp >> 9) & 0xf;

  H616_DIAG("DIAG: cardstate[%s] R1=%08" PRIx32 " CURRENT_STATE=%" PRIu32
        "(%s) RINTR=%08" PRIx32 "\n",
        tag, resp, state, state < 11 ? st[state] : "?", rintr);

  h616_putreg(priv, H616_SDXC_RINTR, 0xffffffff);
}

/* Set up a receive (read)
 */

static int h616_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                          size_t nbytes)
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;

#ifdef H616_SDIO_USEIRQ
  /* A new transfer starts: drop any data bits latched by the ISR for the
   * previous one (eventwait merges this latch into its RINTR read).
   */

  priv->rintr_latch = 0;
#endif

  /* The IDMAC descriptor chain has a hard capacity of H616_IDMA_MAX_XFER
   * bytes.  Requests above it would walk off g_idma_desc and corrupt
   * memory.  The block layer is expected to chunk requests
   * (CONFIG_MMCSD_MULTIBLOCK_LIMIT), but never trust it.
   */

  if (!g_rx_use_ahb && nbytes > H616_IDMA_MAX_XFER)
    {
      ferr("ERROR: IDMAC transfer %zu bytes exceeds %d descriptor "
           "capacity\n", nbytes, H616_IDMA_MAX_XFER);
      return -E2BIG;
    }

  /* Save the buffer and count for the data path
   */

  priv->buffer    = buffer;
  priv->remaining = nbytes;
  priv->total     = nbytes;
  priv->xfrdir    = false;   /* RX */

  if (!g_sdio_diag_muted)
    {
      h616_dbg_card_state(priv, "pre-read");
    }

  H616_DIAG("DIAG: recvsetup buf=%p nbytes=%u BLKSZ=%08" PRIx32
        " BCNTR=%08" PRIx32 " GCTRL=%08" PRIx32 " CLKCR=%08" PRIx32
        " WIDTH=%08" PRIx32 " FTRGL=%08" PRIx32 " STAS=%08" PRIx32 "\n",
        buffer, (unsigned int)nbytes,
        h616_getreg(priv, H616_SDXC_BLKSZ),
        h616_getreg(priv, H616_SDXC_BCNTR),
        h616_getreg(priv, H616_SDXC_GCTRL),
        h616_getreg(priv, H616_SDXC_CLKCR),
        h616_getreg(priv, H616_SDXC_WIDTH),
        h616_getreg(priv, H616_SDXC_FTRGL),
        h616_getreg(priv, H616_SDXC_STAS));

  if (g_rx_use_ahb)
    {
  /* AHB/CPU read path.
   *
   * U-Boot's mmc_trans_data_by_cpu does NOT touch the IDMAC at all: it moves
   * the payload with plain CPU reads of the FIFO.  BLKSZ/BCNTR are set by
   * h616_blocksetup() before CMDR (same order as U-Boot).
   *
   * CRITICAL (proven on hardware, mmcdiag u34/u36): the CPU/FIFO data path
   * must be selected *before* the data command is issued.  If GCTRL still
   * says ACCESS_BY_DMA when CMDR is written, the CIU routes the data phase
   * to the (unprogrammed) DMA engine, the FIFO never fills, DATA_OVER never
   * arrives, and the controller is left in a sticky "card data busy" state
   * in which every subsequent data command starves (STAS bit9 set, register
   * writes ignored).  Every read that has ever succeeded on this board had
   * ACCESS_BY_AHB set before CMDR was written.
   */

  h616_modifyreg(priv, H616_SDXC_GCTRL,
                 H616_SDXC_DMA_ENABLE | H616_SDXC_DMA_RESET,
                 H616_SDXC_ACCESS_BY_AHB);

  H616_DIAG("DIAG: recvsetup AHB data path GCTRL=%08" PRIx32 "\n",
        h616_getreg(priv, H616_SDXC_GCTRL));

  (void)nbytes;
    }
  else
    {
  /* Program the IDMAC for this read.  Drop any stale cache lines covering
   * the destination first: the IDMAC writes DRAM directly, and dirty cache
   * lines would otherwise override the data the card transferred.
   */

  up_invalidate_dcache((uintptr_t)buffer, (uintptr_t)buffer + nbytes);
  h616_start_dma(priv, (uintptr_t)buffer, nbytes, false);
    }

  h616_configxfrints(priv, H616_SDXC_DATA_OVER |
                           H616_SDXC_DATA_CRC_ERROR |
                           H616_SDXC_DATA_TIMEOUT |
                           H616_SDXC_FIFO_RUN_ERROR);

  return OK;
}

/* Set up a send (write)
 */

static int h616_sendsetup(struct sdio_dev_s *dev,
                          const uint8_t *buffer, size_t nbytes)
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;

#ifdef H616_SDIO_USEIRQ
  priv->rintr_latch = 0;
#endif

  /* Same descriptor capacity guard as h616_recvsetup.
   */

  if (!g_rx_use_ahb && nbytes > H616_IDMA_MAX_XFER)
    {
      ferr("ERROR: IDMAC transfer %zu bytes exceeds %d descriptor "
           "capacity\n", nbytes, H616_IDMA_MAX_XFER);
      return -E2BIG;
    }

  /* Save the buffer and count for the transfer
   */

  priv->buffer    = (FAR uint8_t *)buffer;

  /* N47/N53: a multi block write that returns while the CIU still holds
   * words in its FIFO leaves the next command mangled on the wire -- the
   * card answered ILLEGAL_COMMAND (R1 = 0x00400900) and the whole write died
   * with
   * -EIO.  Flush the FIFO and the stale latched status before arming a new
   * multi block write.
   *
   * N53: restricted to multi block commands.  Single block writes (what
   * MMCSD_NO_MULTIBLOCK_WRITE leaves us with) never use the auto CMD12 tail
   * that this guards against, and the 1 ms settle delay in here cost more
   * than
   * the write itself: measured 182 KB/s with it, since it ran once per 512
   * byte block.
   *
   * N66+: auto CMD12 is never armed (CMD23 + self-terminating card), so
   * this pre-write flush has no remaining trigger; the post-write flush in
   * h616_eventwait() covers the single block path.
   */

  priv->remaining = nbytes;
  priv->total     = nbytes;
  priv->xfrdir    = true;    /* TX */

  /* Program the IDMAC for this write, then enable the completion
   * interrupts.  The IDMAC moves the data on its own.  h616_start_dma
   * cleans the source buffer out of the data cache before the IDMAC reads
   * it.
   */

  if (g_rx_use_ahb)
    {
  /* AHB/CPU write path -- mirror of h616_recvsetup().
   *
   * U-Boot's mmc_trans_data_by_cpu uses the same recipe for writes: select
   * the CPU/AHB data path and push the payload into the FIFO with plain CPU
   * writes.  The IDMAC write path never started a data phase on this
   * controller (measured with mkfatfs: CMD24 was issued with
   * GCTRL=0x20000020 = ACCESS_BY_DMA|DMA_ENABLE, CBCR stayed 0, the transfer
   * timed out with every subsequent data command starving).  As on the read
   * side, ACCESS_BY_AHB must be selected *before* CMDR is written; the
   * payload itself is pushed by h616_sendfifo() from h616_eventwait().
   */

  h616_modifyreg(priv, H616_SDXC_GCTRL,
                 H616_SDXC_DMA_ENABLE | H616_SDXC_DMA_RESET,
                 H616_SDXC_ACCESS_BY_AHB);

  H616_DIAG("DIAG: sendsetup AHB data path GCTRL=%08" PRIx32 "\n",
        h616_getreg(priv, H616_SDXC_GCTRL));

  (void)nbytes;
    }
  else
    {
  h616_start_dma(priv, (uintptr_t)buffer, nbytes, true);
    }

  h616_configxfrints(priv, H616_SDXC_DATA_OVER |
                           H616_SDXC_DATA_CRC_ERROR |
                           H616_SDXC_DATA_TIMEOUT |
                           H616_SDXC_FIFO_RUN_ERROR);

  return OK;
}

/* Cancel an in-progress transfer
 */

static int h616_cancel(struct sdio_dev_s *dev)
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;

  /* Disable transfer interrupts
   */

  h616_configxfrints(priv, 0);

  /* Signal cancellation
   */

  priv->wkupevent |= SDIOWAIT_TIMEOUT;
  nxsem_post(&priv->waitsem);

  return OK;
}

/* Wait for the response to a command
 */

static int h616_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;
  int32_t timeout;
  uint32_t events;

  /* DIAG: mmcsd also calls WAITRESPONSE for data commands (R1).  Log it so
   * the data stage is visible in the boot log.
   */

  H616_DIAG("DIAG: waitresponse enter cmd=%08" PRIx32 " RINTR=%08" PRIx32
        " IMASK=%08" PRIx32 " STAS=%08" PRIx32 "\n",
        cmd, h616_getreg(priv, H616_SDXC_RINTR),
        h616_getreg(priv, H616_SDXC_IMASK),
        h616_getreg(priv, H616_SDXC_STAS));

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      events  = H616_SDXC_COMMAND_DONE;
      timeout = H616_SDMMC_CMDTIMEOUT;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
    case MMCSD_R7_RESPONSE:
      events  = H616_SDXC_COMMAND_DONE;
      timeout = H616_SDMMC_LONGTIMEOUT;
      break;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R4_RESPONSE:
      events  = H616_SDXC_COMMAND_DONE;
      timeout = H616_SDMMC_CMDTIMEOUT;
      break;

    default:
      return -EINVAL;
    }

  /* The probe's shortened budget applies here too: a command that is not
   * answered must not cost ten times the data timeout on top of it.
   */

  if (timeout > g_sdio_poll_limit)
    {
      timeout = g_sdio_poll_limit;
    }

  /* Wait for command done (or error).  Poll the RAW interrupt status
   * register (RINTR) -- the ISR deliberately leaves command bits set, so
   * this cannot race with interrupt handling (unlike MISTA which depends
   * on IMASK state).
   */

  while (1)
    {
      uint32_t rintr = h616_getreg(priv, H616_SDXC_RINTR);
#ifdef H616_SDIO_USEIRQ
      rintr |= priv->cmdrintr;   /* bits the ISR already cleared */
#endif

      /* Check the error bits FIRST.  RESP_TIMEOUT and COMMAND_DONE latch
       * together when the card does not drive the CMD line (measured
       * RINTR = 0x00000104 at HS400 teardown), and a "done first" loop
       * returned success with RESP0 = 0 -- a fake OK that hid the fact
       * that the card never answered.  A response timeout is never success.
       */

      if ((rintr & (H616_SDXC_RESP_ERROR | H616_SDXC_RESP_CRC_ERROR |
                    H616_SDXC_RESP_TIMEOUT)) != 0)
        {
          _err("SDIO: resp error RINTR=%08" PRIx32 " cmd=%08" PRIx32
               " RESP0=%08" PRIx32 "\n", rintr, cmd,
               h616_getreg(priv, H616_SDXC_RESP0));
          h616_putreg(priv, H616_SDXC_RINTR, 0xffffffff);
          return -EIO;
        }

      if ((rintr & events) != 0)
        {
          break;
        }

      if (--timeout <= 0)
        {
          _err("ERROR: Timeout cmd: %08" PRIx32 " RINTR=%08" PRIx32
               " CMDR=%08" PRIx32 " STAS=%08" PRIx32 "\n",
               cmd, h616_getreg(priv, H616_SDXC_RINTR),
               h616_getreg(priv, H616_SDXC_CMDR),
               h616_getreg(priv, H616_SDXC_STAS));
          h616_putreg(priv, H616_SDXC_RINTR, 0xffffffff);
          return -ETIMEDOUT;
        }

#ifdef H616_SDIO_USEIRQ
      /* After a short hot spin, sleep until the ISR reports.  The raw
       * register is re-read at the top of the loop, so this stays correct
       * even without the interrupt.  A sleep costs a full tick, so account
       * it against the iteration budget.
       */

      else if (timeout < (int32_t)g_sdio_poll_limit - 20000)
        {
          nxsem_tickwait(&priv->cmdsem, 1);
          timeout -= H616_SDIO_POLLS_PER_TICK;
          if (timeout <= 0)
            {
              timeout = 1;
            }
        }
#endif
    }

  /* Command completed -- clear the command interrupt bits
   */

  H616_DIAG("SDIO: waitresponse cmd=%08" PRIx32 " done, RINTR=%08" PRIx32
        " RESP0=%08" PRIx32 "\n",
        cmd, h616_getreg(priv, H616_SDXC_RINTR),
        h616_getreg(priv, H616_SDXC_RESP0));

  h616_putreg(priv, H616_SDXC_RINTR, 0xffffffff);
  return OK;
}

/* Receive a short response (with CRC check) -- R1, R5, R6, R7
 */

static int h616_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t *rshort)
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;
  uint32_t resp;
  volatile int i;

  /* RESP0 latches a few cycles AFTER COMMAND_DONE.  Reading immediately
   * returns the PREVIOUS command's value (mmcdiag proved this: it always
   * reads after a delay and sees the fresh OCR).  Spin long enough for
   * the latch to settle before reading.
   *
   * N51: the historical 100000-iteration spin costs ~0.86ms at this
   * board's calibrated loop rate (CONFIG_BOARD_LOOPSPERMSEC=116524) and
   * is paid by EVERY short-response command -- one CMD17 read was thus
   * capped around 0.7 MB/s in every bus mode, HS400 or not.  The latch
   * delay is a few controller cycles, not milliseconds: 5000 iterations
   * (~43us) keeps a >200x margin over the latch while cutting the
   * single-block penalty by 20x.  Data integrity is re-verified after
   * this change (mount + md5 regression).
   *
   * N77 (2026-11-12): 43us x 2 commands per request was the entire fixed
   * overhead that capped small dd reads at 5.9 MB/s (see PROF notes in
   * this file).  h616_waitresponse() has already polled COMMAND_DONE by
   * the time we get here, and the function call path back through mmcsd
   * covers the latch by itself; the spin is kept only as a belt-and-
   * suspenders margin and shrunk to 200 iterations (~1.7us, still >>100x
   * the latch time).  Verified by mount + ladder byte-compare + dd/cmp
   * soak below.
   */

  for (i = 0; i < 200; i++)
    {
    }

  resp = h616_getreg(priv, H616_SDXC_RESP0);
  *rshort = resp;

  return OK;
}

/* Receive a long response (136-bit) -- R2
 */

static int h616_recvlongcrc(struct sdio_dev_s *dev, uint32_t cmd,
                            uint32_t rlong[4])
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;

  rlong[0] = h616_getreg(priv, H616_SDXC_RESP0);
  rlong[1] = h616_getreg(priv, H616_SDXC_RESP1);
  rlong[2] = h616_getreg(priv, H616_SDXC_RESP2);
  rlong[3] = h616_getreg(priv, H616_SDXC_RESP3);

  return OK;
}

/* Receive a short response without CRC check -- R3, R4
 */

static int h616_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                          uint32_t *rshort)
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;
  int ret = h616_recvshortcrc(dev, cmd, rshort);

  H616_DIAG("SDIO: recvshort cmd=%08" PRIx32 " resp=%08" PRIx32
        " R0..3=%08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32
        " RINTR=%08" PRIx32 "\n",
        cmd, *rshort,
        h616_getreg(priv, H616_SDXC_RESP0),
        h616_getreg(priv, H616_SDXC_RESP1),
        h616_getreg(priv, H616_SDXC_RESP2),
        h616_getreg(priv, H616_SDXC_RESP3),
        h616_getreg(priv, H616_SDXC_RINTR));
  return ret;
}

/* Wait for a set of events.  The wait is driven by the semaphore, which the
 * interrupt handler posts when a waited-for event fires.  (Command responses
 * are handled synchronously in h616_waitresponse, so eventwait here is used
 * for transfer completion.)
 */

static sdio_eventset_t h616_eventwait(struct sdio_dev_s *dev)
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;
  sdio_eventset_t wkupevent;
  FAR uint8_t *bufstart = priv->buffer;
  volatile int32_t timeout = g_sdio_poll_limit;
  /* Poll the transfer to completion instead of waiting on the interrupt
   * semaphore.  This controller has never been observed to deliver its
   * GIC line (mmcdiag, the only implementation that works on this board,
   * is 100% poll driven), and a missed completion here hangs the whole
   * boot before NSH starts.
   *
   * NOTE: in the AHB path do NOT touch the data FIFO from anywhere but the
   * loop below, and in the IDMAC path do not touch it at all.  The IDMAC
   * moves the payload straight into (reads) or out of (writes) the caller's
   * buffer, so a CPU read of the FIFO alongside it would steal bytes out
   * from under the DMA engine and corrupt the transfer.
   */

  if (g_rx_use_ahb)
    {
  /* Keep the CPU/AHB data path selected.  h616_recvsetup() already set this
   * before CMDR was written (see the note there); this write is a no-op
   * safety net for any path that did not go through recvsetup.
   */

  h616_putreg(priv, H616_SDXC_GCTRL,
              (h616_getreg(priv, H616_SDXC_GCTRL) | H616_SDXC_ACCESS_BY_AHB)
              & ~(H616_SDXC_DMA_ENABLE | H616_SDXC_DMA_RESET));
    }

  while (timeout-- > 0)
    {
      uint32_t rintr = *(volatile uint32_t *)(priv->base + H616_SDXC_RINTR);
#ifdef H616_SDIO_USEIRQ
      rintr |= priv->rintr_latch;   /* bits the ISR already cleared */
#endif
      int complete = 0;

      if (g_rx_use_ahb)
        {
      /* Move whatever the card has handed us out of the data FIFO while we
       * wait (the CPU owns the FIFO in this mode; DATA_OVER only arrives
       * once the block is through).
       */

      if (priv->remaining > 0)
        {
          if (priv->xfrdir)
            {
              /* Write: keep pushing the payload into the TX FIFO while the
               * card is clocking it out (never done by the ISR).
               */

              h616_sendfifo(priv);
            }
          else
            {
              h616_recvfifo(priv);
            }
        }

      /* Completion in the CPU/AHB receive path is "the whole block has been
       * read out of the FIFO", *not* the DATA_OVER raw bit.  With an IRQ
       * attached h616_interrupt() clears the data raw bits (and records the
       * event in priv->wkupevent) before this loop can observe them, so
       * polling RINTR alone times out on a transfer that actually completed
       * -- measured on this board: the first working driver read finished
       * with CBCR=0x200 and remaining=0 while RINTR was already back to 0.
       *
       */

      if (!priv->xfrdir && priv->total > 0 && priv->remaining == 0)
        {
          complete = 1;
        }
        }

      if ((rintr & (H616_SDXC_DATA_OVER | H616_SDXC_DATA_CRC_ERROR |
                    H616_SDXC_DATA_TIMEOUT | H616_SDXC_FIFO_RUN_ERROR)) != 0)
        {
          complete = 1;

          /* N48: a multi block write is NOT finished when the last byte has
           * left the FIFO.  That history is now structural: the driver never
           * arms auto CMD12 (see h616_sendcmdhw), the card self-terminates
           * on CMD23, and DATA_OVER is the authoritative completion just
           * like on the single block path.
           */
        }

      /* The ISR may also have recorded the completion (and masked the
       * transfer interrupts) before the raw bit could be seen here.
       */

      if (priv->wkupevent != 0)
        {
          complete = 1;
        }

      if (complete == 0)
        {
#ifdef H616_SDIO_USEIRQ
          /* IDMAC path: the CPU has nothing to do until the descriptor
           * chain finishes - the ISR owns the wakeup.  Sleep on the event
           * semaphore (a full tick per miss) instead of hot-spinning; the
           * raw register is re-checked at the top of the loop, so a lost
           * interrupt still completes via polling.  The AHB path must keep
           * polling: the CPU itself drains the FIFO.
           */

          if (!g_rx_use_ahb)
            {
              nxsem_tickwait(&priv->waitsem, 1);
              timeout -= H616_SDIO_POLLS_PER_TICK;
              if (timeout <= 0)
                {
                  timeout = 1;
                }
            }
#endif

          continue;
        }

      if (!g_sdio_diag_muted)
        {
          H616_DIAG("DIAG: eventwait done RINTR=%08" PRIx32 " remaining=%u"
                " CBCR=%08" PRIx32 " wkupevent=%02x\n", rintr,
                (unsigned int)priv->remaining,
                *(volatile uint32_t *)(priv->base + H616_SDXC_CBCR),
                (unsigned int)priv->wkupevent);

          /* DIAG: full register snapshot + descriptor at the completion
           * point, so the state can be diffed against U-Boot's working
           * 512-byte IDMAC read.
           */

            {
              struct h616_idma_desc_s *d = g_idma_desc;

              H616_DIAG("DIAG: DONE RINTR=%08" PRIx32 " STAS=%08" PRIx32
                    " CBCR=%08" PRIx32 " BBCR=%08" PRIx32 " BLKSZ=%08" PRIx32
                    " BCNTR=%08" PRIx32 "\n",
                    rintr,
                    *(volatile uint32_t *)(priv->base + H616_SDXC_STAS),
                    *(volatile uint32_t *)(priv->base + H616_SDXC_CBCR),
                    *(volatile uint32_t *)(priv->base + H616_SDXC_BBCR),
                    *(volatile uint32_t *)(priv->base + H616_SDXC_BLKSZ),
                    *(volatile uint32_t *)(priv->base + H616_SDXC_BCNTR));
              h616_dbg_card_state(priv, "post-read");
              H616_DIAG("DIAG: DONE GCTRL=%08" PRIx32
                    " WIDTH=%08" PRIx32 " CLKCR=%08" PRIx32
                    " FTRGL=%08" PRIx32 " TMOUT=%08" PRIx32
                    " CCU838=%08" PRIx32 "\n",
                    *(volatile uint32_t *)(priv->base + H616_SDXC_GCTRL),
                    *(volatile uint32_t *)(priv->base + H616_SDXC_WIDTH),
                    *(volatile uint32_t *)(priv->base + H616_SDXC_CLKCR),
                    *(volatile uint32_t *)(priv->base + H616_SDXC_FTRGL),
                    *(volatile uint32_t *)(priv->base + H616_SDXC_TMOUT),
                    getreg32(H616_CCU_ADDR + H616_CCU_SD2_CLK_CFG));

              /* THLDC must be 0x02000005 (read threshold 512 + READ_EN +
               * WRITE_EN) per U-Boot's H616-specific reset ("Needed on
               * H616").  0x144 is the sample-delay register U-Boot writes
               * CAL_DL_SW_EN
               * to on every clock change.  Read the live values here, at the
               * exact moment a data command failed, to see whether they are
               * actually programmed.
               */

              H616_DIAG("DIAG: DONE THLDC=%08" PRIx32 " SAMPDL=%08" PRIx32
                    " FUNS=%08" PRIx32 " A12A=%08" PRIx32 "\n",
                    *(volatile uint32_t *)(priv->base + 0x100),
                    *(volatile uint32_t *)(priv->base + 0x144),
                    *(volatile uint32_t *)(priv->base + H616_SDXC_FUNS),
                    *(volatile uint32_t *)(priv->base + H616_SDXC_A12A));
              H616_DIAG("DIAG: DONE DMAC=%08" PRIx32 " DLBA=%08" PRIx32
                    " IDST=%08" PRIx32 " CHDA=%08" PRIx32 " CBDA=%08" PRIx32
                    "\n",
                    *(volatile uint32_t *)(priv->base + H616_SDXC_DMAC),
                    *(volatile uint32_t *)(priv->base + H616_SDXC_DLBA),
                    *(volatile uint32_t *)(priv->base + H616_SDXC_IDST),
                    *(volatile uint32_t *)(priv->base + H616_SDXC_CHDA),
                    *(volatile uint32_t *)(priv->base + H616_SDXC_CBDA));

              /* CRC registers decide whether the card sent DATA at all:
               * non-zero DAT*_CRC = bytes arrived but failed CRC (sampling
               * or
               * clock problem); all zero = nothing ever came in (card never
               * drove the bus).  0x110 RESP_CRC, 0x130..0x124
               * DAT0..DAT3_CRC,
               * 0x134 CRC_STA.
               */

              H616_DIAG("DIAG: CRC resp=%08" PRIx32 " sta=%08" PRIx32
                    " dat0=%08" PRIx32 " dat1=%08" PRIx32 " dat2=%08" PRIx32
                    " dat3=%08" PRIx32 "\n",
                    *(volatile uint32_t *)(priv->base + 0x110),
                    *(volatile uint32_t *)(priv->base + 0x134),
                    *(volatile uint32_t *)(priv->base + 0x130),
                    *(volatile uint32_t *)(priv->base + 0x12c),
                    *(volatile uint32_t *)(priv->base + 0x128),
                    *(volatile uint32_t *)(priv->base + 0x124));
              (void)d;

              /* Print the payload that actually landed in the caller's
               * buffer (bufstart, NOT priv->buffer: the FIFO drain advances
               * that one to the end of the block).  A payload that does not
               * match the expected data is the only reliable evidence that a
               * bus width or speed change really corrupted the transfer --
               * the CRC registers stay 0 on this controller either way.
               */

              if (priv->total >= 16 && bufstart != NULL)
                {
                  FAR uint32_t *p = (FAR uint32_t *)bufstart;
                  uint32_t n = (uint32_t)priv->total / 4u;
                  uint32_t sum = 0;
                  uint32_t k;

                  for (k = 0; k < n; k++)
                    {
                      sum += p[k];
                    }

                  H616_DIAG("DIAG: PAYLOAD %08" PRIx32 " = %08" PRIx32 " %08"
                        PRIx32 " %08" PRIx32 " %08" PRIx32 " ... %08" PRIx32
                        " %08" PRIx32 " %08" PRIx32 " %08" PRIx32
                        " sum=%08" PRIx32 " total=%u\n",
                        (uint32_t)(uintptr_t)p, p[0], p[1], p[2], p[3],
                        p[n - 4], p[n - 3], p[n - 2], p[n - 1], sum,
                        (unsigned int)priv->total);
                }
            }
        }

      /* Completion bookkeeping must run whether or not the register dump
       * above was printed: with the dump muted this code used to be trapped
       * inside the `if` and every muted transfer ended in the timeout path
       * (measured: CBCR=0x200, remaining=0, "throughput read failed after 0
       * blocks").
       */

      priv->remaining = 0;

      /* The CPU/AHB receive path copies the payload out of the FIFO with
       * ordinary stores, so the cache already holds the new data and no
       * cache maintenance is needed.  The previous call invalidated
       * [buffer+total, buffer+2*total) -- a window *past* the transfer,
       * because the FIFO drain advances priv->buffer -- which silently threw
       * away whatever dirty lines lived next to the buffer.
       */

      if (!g_rx_use_ahb &&
          !priv->xfrdir && priv->buffer != NULL && priv->total > 0)
        {
          up_invalidate_dcache((uintptr_t)priv->buffer,
                               (uintptr_t)priv->buffer + priv->total);
        }

      *(volatile uint32_t *)(priv->base + H616_SDXC_RINTR) = 0xffffffff;

      /* DATA_OVER is the authoritative completion.  A DATA_TIMEOUT bit
       * can latch alongside it on this controller (the CIU's internal
       * counter does not always update for an IDMAC-driven transfer), so
       * only treat the error bits as fatal when DATA_OVER is NOT set.
       */

      if ((priv->wkupevent & SDIOWAIT_ERROR) != 0)
        {
          wkupevent = SDIOWAIT_ERROR;
        }
      else if ((rintr & H616_SDXC_DATA_OVER) != 0)
        {
          wkupevent = SDIOWAIT_TRANSFERDONE;
        }
      else
        {
          wkupevent = ((rintr & (H616_SDXC_DATA_CRC_ERROR |
                                 H616_SDXC_DATA_TIMEOUT |
                                 H616_SDXC_FIFO_RUN_ERROR)) != 0 ?
                       SDIOWAIT_ERROR : SDIOWAIT_TRANSFERDONE);
        }

      priv->wkupevent = 0;
#ifdef H616_SDIO_USEIRQ
      priv->rintr_latch = 0;
#endif

      /* N50: the write is finished in the CIU sense once DATA_OVER is set
       * (the last byte left the FIFO), but the *card* is not: it is still
       * programming and holds DAT0 low.  mmcsd issues CMD13 (READY_FOR_DATA
       * poll) immediately afterwards; a command that arrives while DAT0 is
       * low
       * is refused by the card, which answers the next CMD13 with
       * ILLEGAL_COMMAND (measured R1 = 0x00400900, state = TRAN), and the
       * whole
       * write fails with -EIO.
       *
       * Evidence from the N49 probe, one line per write:
       *   write that was followed by a failing CMD13:
       *     STAS = 01006503  -> bit9 CARD_DATA_BUSY SET
       *   write that was followed by a good CMD13:
       *     STAS = 0000c506  -> bit9 CARD_DATA_BUSY clear
       * STAS bit10 (DATA_FSM_BUSY) is useless here: it is set even between
       * transfers, so N48's "wait for bit9|bit10" spun 200 ms on every write
       * and wedged the CIU.  RINTR bit14 AUTO_COMMAND_DONE never fires on
       * this
       * controller (N49 probe: auto=0 after 20000 polls, RINTR already
       * cleared
       * by the ISR), so U-Boot's "wait for AUTO_COMMAND_DONE" cannot be
       * used.
       *
       * So: wait for DAT0 (bit9) only, polling like U-Boot does for
       * MMC_RSP_BUSY commands.  In the common case it is already clear and
       * this
       * costs a single register read.
       */

      if (priv->xfrdir)
        {
          int32_t spins = 0;
          uint32_t stas;

          for (; ; )
            {
              stas = *(volatile uint32_t *)(priv->base + H616_SDXC_STAS);
              if ((stas & H616_SDXC_CARD_DATA_BUSY) == 0)
                {
                  break;
                }

              if (++spins >= 50000)
                {
                  _err("SDIO: DAT0 still busy 500 ms after write, STAS=%08"
                       PRIx32 " CBCR=%08" PRIx32 "\n", stas,
                       *(volatile uint32_t *)(priv->base + H616_SDXC_CBCR));
                  wkupevent = priv->wkupevent | SDIOWAIT_ERROR;
                  priv->wkupevent = 0;

                  /* The card is still programming and the CIU data FSM is
                   * done; recover so the next command is not refused.
                   */

                  h616_data_recover(priv);
                  return wkupevent;
                }

              up_udelay(10);
            }

          /* U-Boot resets the FIFO after every command (sunxi_mmc_send_cmd,
           * 'out:' label) so no half consumed payload is left behind for the
           * next transfer.
           */

          h616_modifyreg(priv, H616_SDXC_GCTRL, 0, H616_SDXC_FIFO_RESET);
          h616_modifyreg(priv, H616_SDXC_GCTRL, H616_SDXC_FIFO_RESET, 0);
        }
      else
        {
          /* N52: mirror of the write-path FIFO reset for reads.  The read
           * path used to leave the FIFO untouched at completion, and on the
           * HS400 bus a stale FIFO then wedged the CIU: the next CMD17 was
           * written but never started (CMDR still had START set, no
           * COMMAND_DONE and no RESP_TIMEOUT bit ever fired, STAS showed
           * RXWL
           * set with FIFO not empty) and only a power cycle recovered the
           * controller.  DDR52 had the same hygiene gap but never wedged in
           * testing; HS400's tighter timing made it fatal.  Resetting the
           * FIFO at read completion matches U-Boot's per-command discipline.
           *
           */

          h616_modifyreg(priv, H616_SDXC_GCTRL, 0, H616_SDXC_FIFO_RESET);
          h616_modifyreg(priv, H616_SDXC_GCTRL, H616_SDXC_FIFO_RESET, 0);
        }

      /* A failed or timed out data transfer leaves the CIU refusing the next
       * command (HS400 wedges it completely).  Recover here, where every
       * data transfer ends, so the ladder and mmcsd's runtime I/O share one
       * recovery path.
       */

      if ((wkupevent & (SDIOWAIT_ERROR | SDIOWAIT_TIMEOUT)) != 0)
        {
          h616_data_recover(priv);
        }

      return wkupevent;
    }

  _err("DIAG: eventwait TIMEOUT RINTR=%08" PRIx32 " STAS=%08" PRIx32
       " IDST=%08" PRIx32 " DMAC=%08" PRIx32 " DLBA=%08" PRIx32
       " CBCR=%08" PRIx32 " CHDA=%08" PRIx32 " remaining=%u\n",
       *(volatile uint32_t *)(priv->base + H616_SDXC_RINTR),
       *(volatile uint32_t *)(priv->base + H616_SDXC_STAS),
       *(volatile uint32_t *)(priv->base + H616_SDXC_IDST),
       *(volatile uint32_t *)(priv->base + H616_SDXC_DMAC),
       *(volatile uint32_t *)(priv->base + H616_SDXC_DLBA),
       *(volatile uint32_t *)(priv->base + H616_SDXC_CBCR),
       *(volatile uint32_t *)(priv->base + H616_SDXC_CHDA),
       (unsigned int)priv->remaining);

  wkupevent = priv->wkupevent | SDIOWAIT_TIMEOUT;
  priv->wkupevent = 0;

  /* Same recovery as the completed-but-failed path: a transfer that never
   * completed also leaves the data FSM armed.
   */

  h616_data_recover(priv);
  return wkupevent;
}

/* Enable/disable the set of events to wait for.  Sets up the controller
 * interrupt mask for the requested events.  Timeout is not used by the
 * minimal driver (the transfer interrupt always terminates the wait).
 */

static void h616_waitenable(struct sdio_dev_s *dev,
                            sdio_eventset_t eventset, uint32_t timeout)
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;
  uint32_t waitints = 0;

  /* Disable the event-related interrupts first
   */

  h616_configwaitints(priv, 0, 0);

  /* Map the requested events to controller interrupts
   */

  if ((eventset & (SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE)) != 0)
    {
      waitints |= H616_SDXC_COMMAND_DONE;
    }

  if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
    {
      waitints |= H616_SDXC_DATA_OVER;
    }

  /* The wait set must NOT be seeded into wkupevent: mmcsd_eventwait()
   * fails the transfer when the returned set contains any of the failure
   * events it asked for, and it always passes SDIOWAIT_TRANSFERDONE |
   * SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR.  Seeding wkupevent with that set
   * (0x1c) made *every* block transfer look like a timeout+error at the
   * mmcsd layer even when the payload arrived correctly (measured: the
   * EXT_CSD read returned -EIO and the capacity fell back to the legacy
   * CSD value).  Compare imxrt_waitenable(), which passes 0 here.
   */

  h616_configwaitints(priv, waitints, 0);
}

/* Enable callbacks
 */

static void h616_callbackenable(struct sdio_dev_s *dev,
                                sdio_eventset_t eventset)
{
  struct h616_sdhci_priv_s *priv = (struct h616_sdhci_priv_s *)dev;
  priv->cbevents = eventset;
}

/* Register a callback (worker) -- not supported in the minimal driver
 */

static int h616_registercallback(struct sdio_dev_s *dev,
                                 worker_t callback, void *arg)
{
  /* No callback support in the minimal driver
   */

  return OK;
}

/****************************************************************************
 * Public Functions
 ***************************************************************************
 */

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize the SDIO interface for a slot.
 *
 ***************************************************************************
 */

struct sdio_dev_s *sdio_initialize(int slotno)
{
  struct h616_sdhci_priv_s *priv = &g_sdio;

  if (slotno != 0)
    {
      mcerr("ERROR: Only slot 0 (eMMC sdc2) supported\n");
      return NULL;
    }

  /* DIAG: what context are we running in at startup?
   */

  H616_DIAG("DIAG: sdio_initialize context: irq=%d inint=%d cpuid=%d\n",
        up_interrupt_context(), up_interrupt_context() ? 1 :
        (sched_idletask() ? 1 : 0),
        up_cpu_index());

  /* Capabilities: 1/4/8-bit, DMA, eMMC high speed.
   *
   * N59: DMABEFOREWRITE is NOT about DMA here (CONFIG_SDIO_DMA is off; the
   * AHB path moves data through the CPU FIFO) -- it only reorders mmcsd's
   * write sequence from "CMD25 -> BLOCKSETUP -> SENDSETUP" to
   * "BLOCKSETUP -> SENDSETUP -> CMD25".  That order matters: the CIU latches
   * BLKSZ/BCNTR when the data command is issued, and the default order left
   * BCNTR at the previous transfer's value when CMD25 went out (measured:
   * TXCMD cmd=00003459 bcntr=00000200 for an 8 KiB write -- the CIU thought
   * the transfer was one block, moved 512 bytes, fired its auto CMD12 and
   * left 6656 bytes stranded in the buffer, followed by a timeout).
   * U-Boot always programs blksz/bytecnt before the command
   * (sunxi_mmc.c: writel(data->blocksize, blksz); writel(blocks*blocksize,
   * bytecnt); ... writel(cmdval | cmdidx, cmd)), which is the proven-good
   * order on this board.
   */

  priv->caps = SDIO_CAPS_DMASUPPORTED | SDIO_CAPS_4BIT | SDIO_CAPS_8BIT |
#if !H616_LADDER_HS400
               SDIO_CAPS_MMC_HS_MODE |
#endif
               SDIO_CAPS_DMABEFOREWRITE;

  /* The mmcsd layer never calls SDIO_RESET -- bring the controller to a
   * known state here (clock/gate enable + soft reset + ID clock).  Without
   * this the controller may hold stale state and the first CMD1 response
   * reads back wrong (0x900 instead of a ready OCR).  Proven by mmcdiag:
   * reset + CCU clock + UPCLK trigger yields RESP0=0xc0ff8080.
   */

  h616_reset(&priv->dev);

  return &priv->dev;
}

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic to signal a card insertion/removal.
 *   eMMC is non-removable, so this is a no-op.
 *
 ***************************************************************************
 */

void sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot)
{
}

/****************************************************************************
 * Name: sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report write protection.  eMMC is
 *   never write-protected.
 *
 ***************************************************************************
 */

void sdio_wrprotect(struct sdio_dev_s *dev, bool wrprotect)
{
}

#endif /* CONFIG_H616_SDHCI */
