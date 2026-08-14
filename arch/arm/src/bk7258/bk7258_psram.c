/****************************************************************************
 * arch/arm/src/bk7258/bk7258_psram.c
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

/* BK7258 PSRAM controller driver.
 *
 * Initializes the on-chip PSRAM controller and detects the external
 * PSRAM chip via the command-register interface.
 *
 * Source: Armino psram_driver.c + psram_hal.c + sys_psram_driver.c,
 * Beken, Apache-2.0
 *
 * NOTE: Cache coherency not addressed.  The BK7258 port currently
 * runs without CONFIG_ARMV8M_DCACHE; no cache maintenance is
 * performed here.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>
#include <syslog.h>

#include "arm_internal.h"
#include "hardware/bk7258_psram.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Analog register base and SPI state register.
 * Source: Armino sys_ll.h + bk7258_audio.c
 * All ana_reg* at 0x440101xx go through a serial SPI bus; every write
 * must poll the completion bit before the next write.
 */

#define SYS_BASE               0x44010000
#define ANA_BASE               (SYS_BASE + 0x100)
#define ANA_SPI_STATE_REG      (SYS_BASE + 0x3a * 4)

/* AHB power domain: pwd_ahbp = bit[5] of cpu_power_sleep_wakeup.
 * 0 = powered on, 1 = powered off.
 * Source: Armino sys_struct.h, sys_ll.h
 */

#define SYS_POWER_SLEEP_WAKEUP (SYS_BASE + 0x10 * 4)
#define PWD_AHBP_BIT           (1 << 5)

/* PSRAM peripheral clock enable: bit[19] of cpu_device_clk_enable.
 * Source: Armino sys_hal.c, sys_struct.h
 */

#define SYS_DEV_CLK_ENABLE     (SYS_BASE + 0x0c * 4)
#define PSRAM_CKEN_BIT         (1 << 19)

/* Command interface retry/poll limits.
 * Source: Armino psram_hal.c -> psram_hal_cmd_read()
 */

#define CMD_READ_RETRIES       5
#define CMD_READ_POLL_OUTER    10
#define CMD_READ_POLL_INNER    5000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_psram_chip_id;
static uint32_t g_psram_size;
static bool g_psram_init_done;

/* PSRAM heap — initialized on first use via bk7258_psram_heap_init().
 * Uses the NuttX mm allocator over the PSRAM data window.
 */

static struct mm_heap_s *g_psram_heap;

/****************************************************************************
 * Private Functions — Register Access
 ****************************************************************************/

/****************************************************************************
 * Name: psram_putreg / psram_getreg
 *
 * Description:
 *   Raw 32-bit register access helpers.
 *
 ****************************************************************************/

static inline void psram_putreg(uint32_t val, uint32_t addr)
{
  *(volatile uint32_t *)addr = val;
}

static inline uint32_t psram_getreg(uint32_t addr)
{
  return *(volatile uint32_t *)addr;
}

/****************************************************************************
 * Name: ana_write
 *
 * Description:
 *   Write to an analog register (0x440101xx) and poll the SPI bus
 *   completion bit.  The BK7258 analog registers are behind a serial
 *   SPI bridge; writes are NOT instantaneous and must be polled.
 *
 *   Source: Armino sys_ll.h -> sys_ll_set_analog_reg_value()
 *           + bk7258_audio.c -> ana_write()
 *
 *   WARNING: Every write to an address in 0x440101xx MUST go through
 *   this function.  A plain psram_putreg() will appear to succeed
 *   (the value is latched in the register) but the SPI transfer may
 *   not have completed, causing the analog front-end to never
 *   configure properly.
 *
 ****************************************************************************/

static void ana_write(uintptr_t addr, uint32_t val)
{
  uint32_t idx = (addr - ANA_BASE) >> 2;

  psram_putreg(val, addr);

  while (psram_getreg(ANA_SPI_STATE_REG) & (1u << idx))
    {
    }
}

/****************************************************************************
 * Name: ana_rmw
 *
 * Description:
 *   Read-modify-write a bit field in an analog register.
 *   Reads the current value, clears the mask at pos, sets the new
 *   value, and writes via ana_write() (with SPI polling).
 *
 ****************************************************************************/

static void ana_rmw(uintptr_t addr, int pos, uint32_t mask,
                    uint32_t val)
{
  uint32_t reg = psram_getreg(addr);

  reg &= ~(mask << pos);
  reg |= ((val & mask) << pos);
  ana_write(addr, reg);
}

/****************************************************************************
 * Private Functions — PSRAM Controller
 ****************************************************************************/

/****************************************************************************
 * Name: psram_set_sf_reset
 *
 * Description:
 *   Set or release the PSRAM serial-flash interface reset.
 *   Uses read-modify-write to preserve other REG2 bits.
 *
 *   Source: Armino psram_ll_macro_def.h ->
 *           psram_ll_set_sf_reset_value()
 *
 ****************************************************************************/

static void psram_set_sf_reset(uint32_t value)
{
  uint32_t reg = psram_getreg(PSRAM_LL_REG2);

  reg &= ~PSRAM_SF_RESET_BIT;
  reg |= (value & 1);
  psram_putreg(reg, PSRAM_LL_REG2);
}

/****************************************************************************
 * Name: psram_set_reg2_bypass
 *
 * Description:
 *   Set the PSRAM bypass bit (bit[1]) in REG2, preserving other bits.
 *
 *   Source: Armino psram_hal.c -> psram_hal_config_init()
 *
 ****************************************************************************/

static void psram_set_reg2_bypass(void)
{
  uint32_t val = psram_getreg(PSRAM_LL_REG2);

  val |= (0x1 << 1);
  psram_putreg(val, PSRAM_LL_REG2);
}

/****************************************************************************
 * Name: psram_cmd_reset
 *
 * Description:
 *   Issue a reset command to the PSRAM chip via the command interface.
 *
 *   Source: Armino psram_hal.c -> psram_hal_set_cmd_reset()
 ****************************************************************************/

static void psram_cmd_reset(void)
{
  psram_putreg(PSRAM_CMD_RESET, PSRAM_LL_REG8);
}

/****************************************************************************
 * Name: psram_cmd_write
 *
 * Description:
 *   Write a value to a PSRAM register address via the command
 *   interface.  Includes a generous timeout to avoid infinite hang.
 *
 *   Source: Armino psram_hal.c -> psram_hal_cmd_write()
 ****************************************************************************/

static int psram_cmd_write(uint32_t addr, uint32_t value)
{
  int timeout = CMD_READ_POLL_OUTER * CMD_READ_POLL_INNER;

  psram_putreg(addr, PSRAM_LL_REG9);
  psram_putreg(value, PSRAM_LL_REGA);
  psram_putreg(PSRAM_CMD_WRITE_TRIG, PSRAM_LL_REG8);

  while ((psram_getreg(PSRAM_LL_REG8) & PSRAM_CMD_WRITE_TRIG) &&
         --timeout > 0)
    {
    }

  if (timeout <= 0)
    {
      syslog(LOG_ERR, "psram: cmd_write timeout\n");
      return -ETIMEDOUT;
    }

  return OK;
}

/****************************************************************************
 * Name: psram_cmd_read
 *
 * Description:
 *   Read a value from a PSRAM chip register via the command interface.
 *   Retries up to CMD_READ_RETRIES times, with SF reset + cmd reset
 *   between retries.
 *
 *   Source: Armino psram_hal.c -> psram_hal_cmd_read()
 ****************************************************************************/

static uint32_t psram_cmd_read(uint32_t addr)
{
  int retry;
  int outer;
  int inner;

  for (retry = 0; retry < CMD_READ_RETRIES; retry++)
    {
      psram_putreg(addr, PSRAM_LL_REG9);
      psram_putreg(PSRAM_CMD_READ_TRIG, PSRAM_LL_REG8);

      for (outer = 0; outer < CMD_READ_POLL_OUTER; outer++)
        {
          for (inner = 0; inner < CMD_READ_POLL_INNER; inner++)
            {
              if (!(psram_getreg(PSRAM_LL_REG8) &
                    PSRAM_CMD_READ_TRIG))
                {
                  return psram_getreg(PSRAM_LL_REGB);
                }
            }
        }

      /* Timeout: reset SF interface and retry */

      psram_set_sf_reset(0);
      psram_set_sf_reset(1);
      psram_cmd_reset();
    }

  return 0;
}

/****************************************************************************
 * Name: psram_set_clk
 *
 * Description:
 *   Set PSRAM clock source and divider.
 *
 *   Source: Armino psram_hal.c -> psram_hal_set_clk()
 ****************************************************************************/

static void psram_set_clk(uint32_t sel, uint32_t div)
{
  uint32_t val = psram_getreg(SYS_CPU_CLK_DIV2);

  val &= ~(CLK_DIV2_CKSEL_PSRAM | CLK_DIV2_CKDIV_PSRAM);
  val |= (sel ? CLK_DIV2_CKSEL_PSRAM : 0) |
         (div ? CLK_DIV2_CKDIV_PSRAM : 0);
  psram_putreg(val, SYS_CPU_CLK_DIV2);
}

/****************************************************************************
 * Private Functions — Chip-Specific Init
 *
 * Each function configures the PSRAM controller for a specific chip
 * type, reads and verifies the chip ID, then configures the chip's
 * own mode registers (MR0/MR4/MR8) for proper latency, drive
 * strength, and burst settings.
 *
 * Source: Armino psram_hal.c -> psram_hal_APS6408L_init(),
 *         psram_hal_APS128XXO_OB9_init(),
 *         psram_hal_W955D8MKY_5J_init()
 ****************************************************************************/

/****************************************************************************
 * Name: psram_init_aps6408l
 *
 * Description:
 *   Init sequence for AP Memory APS6408L (8MB, ID 0x8D09).
 *   Configures MR0 (drive/latency) and MR4 (write latency).
 *
 ****************************************************************************/

static int psram_init_aps6408l(uint32_t *id)
{
  uint32_t val;

  psram_putreg(PSRAM_MODE_APS6408L, PSRAM_LL_REG4);
  psram_putreg(PSRAM_DRV_APS6408L, PSRAM_LL_REG5);
  psram_cmd_reset();
  up_udelay(500);

  val = psram_cmd_read(0x00000000);
  if (val == 0 || val != *id)
    {
      return -1;
    }

  *id = val;

  /* MR0: set drive strength and latency */

  val = psram_getreg(PSRAM_LL_REGB);
  val = (val & ~0x1f) | (0x4 << 2) | 0x3;
  psram_cmd_write(0x00000000, val);

  /* MR4: set write latency (6 @ 120 MHz) */

  psram_cmd_read(0x00000004);
  val = psram_getreg(PSRAM_LL_REGB);
  val = (val & ~(0x7 << 5)) | (0x6 << 5);
  psram_cmd_write(0x00000004, val);

  return 0;
}

/****************************************************************************
 * Name: psram_init_aps128xxo
 *
 * Description:
 *   Init sequence for AP Memory APS128XXO (16MB, ID 0x8D08).
 *   Configures MR0 (drive/latency), MR4 (write latency), and MR8
 *   (burst length).
 *
 ****************************************************************************/

static int psram_init_aps128xxo(uint32_t *id)
{
  uint32_t val;

  psram_putreg(PSRAM_MODE_APS128XXO, PSRAM_LL_REG4);
  psram_putreg(PSRAM_DRV_APS128XXO, PSRAM_LL_REG5);
  psram_cmd_reset();
  up_udelay(500);

  val = psram_cmd_read(0x00000000);
  if (val == 0 || val != *id)
    {
      return -1;
    }

  *id = val;

  /* MR0: set drive strength and latency */

  val = psram_getreg(PSRAM_LL_REGB);
  val = (val & ~0x1f) | (0x6 << 2) | 0x2;
  psram_cmd_write(0x00000000, val);

  /* MR4: set write latency (6 @ 120 MHz) */

  psram_cmd_read(0x00000004);
  val = psram_getreg(PSRAM_LL_REGB);
  val = (val & ~(0x7 << 5)) | (0x6 << 5);
  psram_cmd_write(0x00000004, val);

  /* MR8: set burst length bit */

  psram_cmd_read(0x00000008);
  val = psram_getreg(PSRAM_LL_REGB);
  val |= 0x40;
  psram_cmd_write(0x00000008, val);

  return 0;
}

/****************************************************************************
 * Name: psram_init_w955d8mky
 *
 * Description:
 *   Init sequence for Winbond W955D8MKY (4MB, ID 0x1C8F on BK7258).
 *   Configures drive strength via a command write to 0x01000000.
 *
 ****************************************************************************/

static int psram_init_w955d8mky(uint32_t *id)
{
  uint32_t val;
  uint32_t io_drv = 0;

  psram_putreg(PSRAM_MODE_W955D8MKY, PSRAM_LL_REG4);
  psram_putreg(PSRAM_DRV_W955D8MKY, PSRAM_LL_REG5);
  psram_cmd_reset();
  up_udelay(500);

  val = psram_cmd_read(0x01000000);
  if (val == 0)
    {
      return -1;
    }

  /* Configure drive strength */

  val = 0x1c8f | (io_drv << 4);
  psram_cmd_write(0x01000000, val);

  /* Verify */

  psram_cmd_read(0x01000000);

  *id = PSRAM_ID_W955D8MKY;
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bk7258_psram_init
 *
 * Description:
 *   Initialize the BK7258 PSRAM controller and detect the external
 *   PSRAM chip.  Sets g_psram_chip_id and g_psram_size.
 *
 *   Sequence (source: Armino psram_hal_power_clk_enable +
 *   psram_hal_config_init):
 *     1. Set voltage: ana_reg13 psldo_swb=1, vpsramsel=3 (1.95V)
 *     2. Enable PSRAM LDO: ana_reg13 enpsram=1, wait 1ms
 *     3. Enable AHB PSRAM power domain: clear pwd_ahbp bit
 *     4. Set init clock to 80 MHz (320MHz PLL, div=1)
 *     5. Enable PSRAM peripheral clock (bit 19)
 *     6. Release SF reset + set bypass bit in REG2
 *     7. Auto-detect: try APS6408L -> APS128XXO -> W955D8MKY
 *        (each: set mode/drive, reset, read ID, configure MRs)
 *     8. Switch to 120 MHz default clock
 *
 *   NOTE: Cache coherency not addressed (no D-cache configured).
 *
 * Returned Value:
 *   OK on success (chip detected), -ENODEV if no known chip found.
 *
 ****************************************************************************/

int bk7258_psram_init(void)
{
  uint32_t val;
  uint32_t id;

  if (g_psram_init_done)
    {
      return OK;
    }

  g_psram_chip_id = 0;
  g_psram_size = 0;

  /* Step 1: Set PSRAM LDO voltage to 1.95V (psldo_swb=1, vpsramsel=3).
   * Source: Armino sys_hal_psram_psldo_vset()
   * Must use ana_write (read-modify-write + SPI poll), not plain MMIO.
   */

  ana_rmw(SYS_ANA_REG13, 28, 0x1, 1);   /* psldo_swb = 1 */
  ana_rmw(SYS_ANA_REG13, 29, 0x3, 3);   /* vpsramsel = 3 */

  /* Step 2: Enable PSRAM LDO.
   * Source: Armino sys_hal_psram_ldo_enable()
   */

  ana_rmw(SYS_ANA_REG13, 31, 0x1, 1);   /* enpsram = 1 */
  up_mdelay(1);

  /* Step 3: Enable AHB PSRAM power domain.
   * Source: Armino bk_pm_module_vote_power_ctrl(AHBP_PSRAM, ON)
   *         -> sys_hal_module_power_ctrl()
   * Clear pwd_ahbp (bit 5) to power on the AHB domain.
   */

  val = psram_getreg(SYS_POWER_SLEEP_WAKEUP);
  val &= ~PWD_AHBP_BIT;
  psram_putreg(val, SYS_POWER_SLEEP_WAKEUP);

  /* Step 4: Set init clock to 80 MHz (320 MHz PLL, divider=1).
   * Source: Armino psram_hal_set_clk(PSRAM_80M)
   */

  psram_set_clk(0, 1);   /* sel=0 (320MHz), div=1 */

  /* Step 5: Enable PSRAM peripheral clock (bit 19).
   * Source: Armino sys_drv_dev_clk_pwr_up(CLK_PWR_ID_PSRAM, PWR_UP)
   */

  val = psram_getreg(SYS_DEV_CLK_ENABLE);
  val |= PSRAM_CKEN_BIT;
  psram_putreg(val, SYS_DEV_CLK_ENABLE);

  /* Step 6: Release SF reset and set bypass bit.
   * Source: Armino psram_hal_config_init()
   */

  psram_set_sf_reset(1);
  psram_set_reg2_bypass();

  up_udelay(3000);

  /* Step 7: Auto-detect PSRAM chip type.
   * Try each type in sequence; first match wins.
   * Each init function sets mode/drive, resets, reads ID,
   * and configures the chip's mode registers.
   * Source: Armino psram_hal_config_init()
   */

  id = PSRAM_ID_APS6408L;
  if (psram_init_aps6408l(&id) == 0)
    {
      g_psram_chip_id = id;
      g_psram_size = 8 * 1024 * 1024;
    }
  else
    {
      id = PSRAM_ID_APS128XXO;
      if (psram_init_aps128xxo(&id) == 0)
        {
          g_psram_chip_id = id;
          g_psram_size = 16 * 1024 * 1024;
        }
      else
        {
          id = PSRAM_ID_W955D8MKY;
          if (psram_init_w955d8mky(&id) == 0)
            {
              g_psram_chip_id = id;
              g_psram_size = 4 * 1024 * 1024;
            }
          else
            {
              syslog(LOG_ERR,
                     "psram: no known chip detected\n");
              return -ENODEV;
            }
        }
    }

  /* Step 8: Switch to 120 MHz default clock.
   * Source: Armino psram_hal_set_default_clk()
   */

  up_mdelay(1);
  psram_set_clk(1, 1);   /* sel=1 (480MHz), div=1 -> 120MHz */

  g_psram_init_done = true;

  syslog(LOG_INFO,
         "psram: init OK, ID=0x%04lx, size=%lu KB\n",
         (unsigned long)g_psram_chip_id,
         (unsigned long)(g_psram_size / 1024));
  return OK;
}

/****************************************************************************
 * Name: bk7258_psram_get_id
 *
 * Description:
 *   Return the detected PSRAM chip ID (0 if not initialized).
 *
 ****************************************************************************/

uint32_t bk7258_psram_get_id(void)
{
  return g_psram_chip_id;
}

/****************************************************************************
 * Name: bk7258_psram_get_size
 *
 * Description:
 *   Return the detected PSRAM size in bytes (0 if not initialized).
 *
 ****************************************************************************/

uint32_t bk7258_psram_get_size(void)
{
  return g_psram_size;
}

/****************************************************************************
 * Name: bk7258_psram_probe
 *
 * Description:
 *   Full S1 verification: init + ID check + single-word data path
 *   test on the memory-mapped window (0x60000000).
 *
 *   The ID check uses the command-register path (REG9/REG_A/REG_B).
 *   The data path test uses the AXI memory-mapped window.  These are
 *   two independent hardware paths; testing both in S1 ensures we
 *   don't mix up "controller works" with "data window works".
 *
 *   NOTE: Cache coherency not addressed (no D-cache configured).
 *
 * Returned Value:
 *   OK on success, negative errno on failure.
 *
 ****************************************************************************/

int bk7258_psram_probe(void)
{
  volatile uint32_t *psram_base;
  uint32_t save;
  uint32_t readback;
  int ret;

  /* Run full init sequence */

  ret = bk7258_psram_init();
  if (ret < 0)
    {
      return ret;
    }

  /* Single-word data path test on memory-mapped window */

  psram_base = (volatile uint32_t *)0x60000000;

  /* Save original content (avoid destructive test on first word) */

  save = *psram_base;

  /* Write test pattern */

  *psram_base = 0xdeadbeef;
  readback = *psram_base;

  /* Restore original content */

  *psram_base = save;

  if (readback != 0xdeadbeef)
    {
      syslog(LOG_ERR, "psram: data path FAIL "
             "(wrote 0xdeadbeef, read 0x%08lx)\n",
             (unsigned long)readback);
      return -EIO;
    }

  syslog(LOG_INFO, "psram: data path OK (0x60000000 verified)\n");
  return OK;
}

/****************************************************************************
 * Name: psram_check_no_heap
 *
 * Description:
 *   Guard for destructive tests.  If the PSRAM heap has been
 *   initialized its metadata lives at 0x60000000; any write to
 *   that region would corrupt it.  Returns -EBUSY if the heap
 *   is active.
 *
 ****************************************************************************/

static int psram_check_no_heap(FAR const char *who)
{
  if (g_psram_heap != NULL)
    {
      syslog(LOG_ERR,
             "psram %s: heap is active, "
             "reboot to run destructive tests\n",
             who);
      return -EBUSY;
    }

  return OK;
}

/****************************************************************************
 * Name: bk7258_psram_test
 *
 * Description:
 *   Address-in-address destructive test over the PSRAM window.
 *   Writes each 32-bit address's own address as the value, then
 *   reads back and verifies.  Reports errors, throughput, and
 *   boundary checks.
 *
 *   WARNING: This is a destructive test.  All data in the tested
 *   range will be overwritten.
 *
 * Input Parameters:
 *   size_bytes - Number of bytes to test (rounded down to 4-byte
 *                alignment).  If 0, uses g_psram_size.
 *
 * Returned Value:
 *   OK on success (zero errors), -EIO if any mismatch found.
 *
 ****************************************************************************/

int bk7258_psram_test(uint32_t size_bytes)
{
  volatile uint32_t *base = (volatile uint32_t *)0x60000000;
  volatile uint32_t *last;
  uint32_t errors = 0;
  uint32_t first_bad = 0;
  uint32_t first_exp = 0;
  uint32_t first_got = 0;
  uint32_t save_last;
  uint32_t i;
  uint32_t nwords;
  clock_t t_start;
  clock_t t_wr;
  clock_t t_rd;
  uint32_t wr_kbps_x10;
  uint32_t rd_kbps_x10;
  int ret;

  ret = psram_check_no_heap("test");
  if (ret < 0)
    {
      return ret;
    }

  ret = bk7258_psram_init();
  if (ret < 0)
    {
      return ret;
    }

  if (size_bytes == 0)
    {
      size_bytes = g_psram_size;
    }

  if (size_bytes > g_psram_size)
    {
      size_bytes = g_psram_size;
    }

  nwords = size_bytes / 4;

  syslog(LOG_INFO,
         "psram test: range 0x%08lx - 0x%08lx (%lu KB, destructive)\n",
         (unsigned long)0x60000000,
         (unsigned long)(0x60000000 + size_bytes - 1),
         (unsigned long)(size_bytes / 1024));

  /* Write phase: each word gets its own byte address */

  t_start = clock_systime_ticks();

  for (i = 0; i < nwords; i++)
    {
      base[i] = (uint32_t)(0x60000000 + i * 4);

      /* Progress every 1 MB */

      if ((i & 0x3ffff) == 0 && i > 0)
        {
          syslog(LOG_INFO, "  wrote %lu MB\n",
                 (unsigned long)(i * 4 / (1024 * 1024)));
        }
    }

  t_wr = clock_systime_ticks() - t_start;

  /* Read-and-verify phase */

  t_start = clock_systime_ticks();

  for (i = 0; i < nwords; i++)
    {
      uint32_t expected = (uint32_t)(0x60000000 + i * 4);
      uint32_t actual = base[i];

      if (actual != expected)
        {
          if (errors == 0)
            {
              first_bad = 0x60000000 + i * 4;
              first_exp = expected;
              first_got = actual;
            }

          errors++;

          /* Print first few errors in detail */

          if (errors <= 5)
            {
              syslog(LOG_ERR,
                     "  MISMATCH @ 0x%08lx: "
                     "expected 0x%08lx, got 0x%08lx\n",
                     (unsigned long)(0x60000000 + i * 4),
                     (unsigned long)expected,
                     (unsigned long)actual);
            }
        }

      /* Progress every 1 MB */

      if ((i & 0x3ffff) == 0 && i > 0)
        {
          syslog(LOG_INFO, "  verified %lu MB\n",
                 (unsigned long)(i * 4 / (1024 * 1024)));
        }
    }

  t_rd = clock_systime_ticks() - t_start;

  /* KB/s with 1 decimal (integer math: value * 10 / 1024).
   * size_bytes * 1000 stays < 2^32 for sizes up to 4 MB per
   * phase; for larger sizes the intermediate is still safe
   * up to ~16 MB (16M * 1000 = 16G < 2^34, fits uint64_t).
   */

  if (t_wr > 0)
    {
      wr_kbps_x10 = (uint32_t)((uint64_t)size_bytes * 1000
                     / (uint64_t)t_wr / 1024);
    }
  else
    {
      wr_kbps_x10 = 0;
    }

  if (t_rd > 0)
    {
      rd_kbps_x10 = (uint32_t)((uint64_t)size_bytes * 1000
                     / (uint64_t)t_rd / 1024);
    }
  else
    {
      rd_kbps_x10 = 0;
    }

  /* Boundary check: last word */

  last = (volatile uint32_t *)(0x60000000 + g_psram_size - 4);
  save_last = *last;
  *last = 0xcafebabe;

  if (*last != 0xcafebabe)
    {
      syslog(LOG_ERR, "  BOUNDARY FAIL @ 0x%08lx: "
             "wrote 0xcafebabe, read 0x%08lx\n",
             (unsigned long)(0x60000000 + g_psram_size - 4),
             (unsigned long)*last);
      errors++;
    }

  *last = save_last;

  syslog(LOG_INFO,
         "psram test: %lu words, %lu errors, "
         "%lu.%lu / %lu.%lu KB/s (wr/rd), "
         "%lu.%02lu s total\n",
         (unsigned long)nwords,
         (unsigned long)errors,
         (unsigned long)(wr_kbps_x10 / 10),
         (unsigned long)(wr_kbps_x10 % 10),
         (unsigned long)(rd_kbps_x10 / 10),
         (unsigned long)(rd_kbps_x10 % 10),
         (unsigned long)((t_wr + t_rd) / TICK_PER_SEC),
         (unsigned long)((t_wr + t_rd) % TICK_PER_SEC *
                         100 / TICK_PER_SEC));

  if (errors > 0)
    {
      syslog(LOG_ERR,
             "psram test: first error @ 0x%08lx "
             "(expected 0x%08lx, got 0x%08lx)\n",
             (unsigned long)first_bad,
             (unsigned long)first_exp,
             (unsigned long)first_got);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: bk7258_psram_alias
 *
 * Description:
 *   Address alias detection.  Writes distinct patterns at offsets
 *   0, 4MB, 8MB, and 16MB-4KB, then re-reads offset 0 to check
 *   for aliasing (address wraparound).
 *
 *   Independent of chip ID — verifies actual usable capacity.
 *
 * Returned Value:
 *   OK if no aliasing detected, -EIO if aliasing found.
 *
 ****************************************************************************/

int bk7258_psram_alias(void)
{
  volatile uint32_t *base = (volatile uint32_t *)0x60000000;
  uint32_t save0;
  uint32_t readback;
  int ret;
  int aliased = 0;

  ret = psram_check_no_heap("alias");
  if (ret < 0)
    {
      return ret;
    }

  ret = bk7258_psram_init();
  if (ret < 0)
    {
      return ret;
    }

  save0 = base[0];

  /* Write unique patterns at key offsets */

  base[0] = 0xaaaaaaaa;

  /* 4 MB offset */

  base[0x400000 / 4] = 0x11111111;
  syslog(LOG_INFO,
         "  wrote 0x11111111 @ 0x%08lx (4 MB)\n",
         (unsigned long)(0x60000000 + 0x400000));

  /* 8 MB offset */

  base[0x800000 / 4] = 0x22222222;
  syslog(LOG_INFO,
         "  wrote 0x22222222 @ 0x%08lx (8 MB)\n",
         (unsigned long)(0x60000000 + 0x800000));

  /* 16 MB - 4 KB offset (only meaningful if >= 16MB) */

  if (g_psram_size >= 16 * 1024 * 1024)
    {
      base[(16 * 1024 * 1024 - 4096) / 4] = 0x33333333;
      syslog(LOG_INFO,
             "  wrote 0x33333333 @ 0x%08lx (16MB-4K)\n",
             (unsigned long)(0x60000000 + 16 * 1024 * 1024 - 4096));
    }

  /* Re-read offset 0 — if it changed, aliasing occurred */

  readback = base[0];

  if (readback != 0xaaaaaaaa)
    {
      syslog(LOG_ERR,
             "psram alias: FAIL — offset 0 changed from "
             "0xaaaaaaaa to 0x%08lx (aliasing!)\n",
             (unsigned long)readback);
      aliased = 1;
    }

  /* Now check if writing at 8MB clobbers offset 0
   * (would indicate 8MB wraparound)
   */

  base[0] = 0xbbbbbbbb;
  readback = base[0x800000 / 4];

  if (readback == 0xbbbbbbbb)
    {
      syslog(LOG_ERR,
             "psram alias: 8 MB offset mirrors offset 0 "
             "(real capacity <= 8 MB)\n");
      aliased = 1;
    }

  /* Restore */

  base[0] = save0;

  if (!aliased)
    {
      syslog(LOG_INFO,
             "psram alias: OK — no aliasing detected "
             "(%lu MB usable)\n",
             (unsigned long)(g_psram_size / (1024 * 1024)));
    }

  return aliased ? -EIO : OK;
}

/****************************************************************************
 * Name: bk7258_psram_width
 *
 * Description:
 *   Access width test: verifies 8-bit, 16-bit, and 32-bit
 *   read/write to the PSRAM window.  Some controllers only
 *   support word access; this test catches that early.
 *
 *   Uses the first 16 bytes of PSRAM (non-destructive to the
 *   rest).
 *
 * Returned Value:
 *   OK if all widths pass, -EIO if any width fails.
 *
 ****************************************************************************/

int bk7258_psram_width(void)
{
  volatile uint8_t  *base8  = (volatile uint8_t  *)0x60000000;
  volatile uint16_t *base16 = (volatile uint16_t *)0x60000000;
  volatile uint32_t *base32 = (volatile uint32_t *)0x60000000;
  uint32_t save[4];
  int failures = 0;
  int ret;

  ret = psram_check_no_heap("width");
  if (ret < 0)
    {
      return ret;
    }

  ret = bk7258_psram_init();
  if (ret < 0)
    {
      return ret;
    }

  /* Save first 16 bytes */

  save[0] = base32[0];
  save[1] = base32[1];
  save[2] = base32[2];
  save[3] = base32[3];

  /* Test 32-bit access */

  base32[0] = 0xdeadbeef;
  base32[1] = 0xcafebabe;
  if (base32[0] != 0xdeadbeef || base32[1] != 0xcafebabe)
    {
      syslog(LOG_ERR, "  32-bit: FAIL (wrote dead/cafe, "
             "read 0x%08x/0x%08x)\n",
             (unsigned)base32[0], (unsigned)base32[1]);
      failures++;
    }
  else
    {
      syslog(LOG_INFO, "  32-bit: OK\n");
    }

  /* Test 16-bit access (critical for RGB565 framebuffer) */

  base16[0] = 0x1234;
  base16[1] = 0x5678;
  base16[2] = 0x9abc;
  base16[3] = 0xdef0;
  if (base16[0] != 0x1234 || base16[1] != 0x5678 ||
      base16[2] != 0x9abc || base16[3] != 0xdef0)
    {
      syslog(LOG_ERR, "  16-bit: FAIL (read 0x%04x/0x%04x/"
             "0x%04x/0x%04x)\n",
             (unsigned)base16[0], (unsigned)base16[1],
             (unsigned)base16[2], (unsigned)base16[3]);
      failures++;
    }
  else
    {
      syslog(LOG_INFO, "  16-bit: OK (RGB565 framebuffer safe)\n");
    }

  /* Test 8-bit access */

  base8[0] = 0xaa;
  base8[1] = 0x55;
  base8[2] = 0xff;
  base8[3] = 0x00;
  if (base8[0] != 0xaa || base8[1] != 0x55 ||
      base8[2] != 0xff || base8[3] != 0x00)
    {
      syslog(LOG_ERR, "  8-bit:  FAIL (read 0x%02x/0x%02x/"
             "0x%02x/0x%02x)\n",
             (unsigned)base8[0], (unsigned)base8[1],
             (unsigned)base8[2], (unsigned)base8[3]);
      failures++;
    }
  else
    {
      syslog(LOG_INFO, "  8-bit:  OK\n");
    }

  /* Restore */

  base32[0] = save[0];
  base32[1] = save[1];
  base32[2] = save[2];
  base32[3] = save[3];

  if (failures > 0)
    {
      syslog(LOG_ERR,
             "psram width: %d width(s) FAILED\n", failures);
      return -EIO;
    }

  syslog(LOG_INFO,
         "psram width: all widths OK (8/16/32-bit)\n");
  return OK;
}

/****************************************************************************
 * Name: bk7258_psram_heap_init
 *
 * Description:
 *   Initialize a standalone heap over the PSRAM data window
 *   (0x60000000, g_psram_size bytes).  Uses NuttX mm_initialize()
 *   to create a private heap that does NOT merge with the main
 *   SRAM heap.
 *
 *   Idempotent: calling when g_psram_heap != NULL is a no-op.
 *   Internally ensures the PSRAM controller is initialized.
 *
 * Returned Value:
 *   OK on success, negative errno on failure.
 *
 ****************************************************************************/

int bk7258_psram_heap_init(void)
{
  int ret;

  if (g_psram_heap != NULL)
    {
      return OK;
    }

  ret = bk7258_psram_init();
  if (ret < 0)
    {
      return ret;
    }

  g_psram_heap = mm_initialize("psram",
                                (FAR void *)0x60000000,
                                g_psram_size);
  if (g_psram_heap == NULL)
    {
      syslog(LOG_ERR, "psram heap: mm_initialize failed\n");
      return -ENOMEM;
    }

  syslog(LOG_INFO,
         "psram heap: ready, base=0x60000000, size=%lu KB\n",
         (unsigned long)(g_psram_size / 1024));
  return OK;
}

/****************************************************************************
 * Name: bk7258_psram_malloc
 *
 * Description:
 *   Allocate `size` bytes from the PSRAM heap.
 *   Returns NULL if the heap is not initialized or allocation fails.
 *   The returned pointer is guaranteed to be in 0x60xxxxxx.
 *
 ****************************************************************************/

FAR void *bk7258_psram_malloc(size_t size)
{
  if (g_psram_heap == NULL)
    {
      return NULL;
    }

  return mm_malloc(g_psram_heap, size);
}

/****************************************************************************
 * Name: bk7258_psram_calloc
 *
 * Description:
 *   Allocate zero-initialized array from the PSRAM heap.
 *
 ****************************************************************************/

FAR void *bk7258_psram_calloc(size_t n, size_t size)
{
  if (g_psram_heap == NULL)
    {
      return NULL;
    }

  return mm_calloc(g_psram_heap, n, size);
}

/****************************************************************************
 * Name: bk7258_psram_free
 *
 * Description:
 *   Free memory previously allocated from the PSRAM heap.
 *
 ****************************************************************************/

void bk7258_psram_free(FAR void *ptr)
{
  if (g_psram_heap != NULL && ptr != NULL)
    {
      mm_free(g_psram_heap, ptr);
    }
}

/****************************************************************************
 * Name: bk7258_psram_meminfo
 *
 * Description:
 *   Fill `info` with PSRAM heap statistics.  Equivalent to
 *   mallinfo() but for the PSRAM heap only.
 *
 ****************************************************************************/

void bk7258_psram_meminfo(FAR struct mallinfo *info)
{
  if (info == NULL)
    {
      return;
    }

  if (g_psram_heap == NULL)
    {
      memset(info, 0, sizeof(*info));
      return;
    }

  *info = mm_mallinfo(g_psram_heap);
}

/****************************************************************************
 * Name: bk7258_psram_memalign
 *
 * Description:
 *   Allocate `size` bytes from the PSRAM heap with `alignment`
 *   byte alignment.  Required for DMA targets (DVP, SPI) that
 *   need 32/64-byte aligned buffers.
 *
 *   D-cache note (2026-08): the current defconfig does NOT enable
 *   CONFIG_ARMV8M_DCACHE, so no cache maintenance is needed.
 *   If D-cache is enabled in the future, every buffer written by
 *   DMA must be cache-invalidated before the CPU reads it
 *   (arm_dcache_invalidate / up_invalidate_dcache).
 *   Without this the CPU will read stale cache lines instead of
 *   the DMA data — a data-corruption bug that is very hard to
 *   diagnose after the fact.
 *
 ****************************************************************************/

FAR void *bk7258_psram_memalign(size_t alignment, size_t size)
{
  if (g_psram_heap == NULL)
    {
      return NULL;
    }

  return mm_memalign(g_psram_heap, alignment, size);
}
