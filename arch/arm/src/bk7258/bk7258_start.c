/****************************************************************************
 * arch/arm/src/bk7258/bk7258_start.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/init.h>
#include <nuttx/syslog/syslog.h>

#include "arm_internal.h"
#include <nuttx/cache.h>

#include "mpu.h"
#include "nvic.h"

#include "hardware/bk7258_memorymap.h"
#include "hardware/bk7258_mbox.h"
#include "hardware/bk7258_psram.h"

#if defined(CONFIG_BK7258_PSRAM) && \
    CONFIG_BK7258_PSRAM_SIZE != BK7258_PSRAM_SIZE
#  error "BK7258 OpenVela and app_ab PSRAM capacities differ"
#endif

int bk7258_syslog_initialize(void);

extern const uint8_t _eronly[];
extern uint8_t _sdata[];
extern uint8_t _edata[];
extern uint8_t _sbss[];
extern uint8_t _ebss[];
extern uint8_t __ram_vectors_load[];
extern uint8_t __ram_vectors_start[];
extern uint8_t __ram_vectors_end[];
extern uint8_t __idle_stack_base[];
extern uint8_t __idle_stack_top[];

const uintptr_t g_idle_topstack = (uintptr_t)__idle_stack_top;

static const struct mpu_region_s g_bk7258_mpu_regions[] =
{
  {
    BK7258_AP_FLASH_BASE,
    BK7258_AP_FLASH_SIZE,
    MPU_RBAR_AP_RORO | MPU_RBAR_SH_NO,
    MPU_RLAR_WRITE_THROUGH
  },
  {
    BK7258_AP_RAM_BASE,
    BK7258_AP_RAM_SIZE,
    MPU_RBAR_XN | MPU_RBAR_AP_RWRW | MPU_RBAR_SH_INNER,
    MPU_RLAR_NONCACHEABLE
  },
  {
    BK7258_CP_RAM_START,
    BK7258_CP_RAM_END - BK7258_CP_RAM_START,
      /* Controller-interface command/event pools and direct-push pbufs carry
       * ownership words which the AP must return to the CP.
       */

    MPU_RBAR_XN | MPU_RBAR_AP_RWRW | MPU_RBAR_SH_INNER,
    MPU_RLAR_NONCACHEABLE
  },
  {
      /* One region for the whole run of SWAP this core writes and the CP
       * reads: the IPC TX frame, the flash service descriptor and the flash
       * payload staging buffer.  Covered as one rather than three because
       * the part's MPU region count is checked against this table at boot
       * and a table that does not fit leaves the AP spinning in wfi with no
       * output.
       *
       * It has to be covered at all: mpu_initialize() below is called with
       * privdefena, so an unmapped address here would still work -- as
       * cacheable Normal memory, per the ARMv8-M default map -- and the CP
       * would then read whatever the AP's cache had not written back yet.
       * Only the I-Cache is on today, which is the only reason the gap that
       * used to sit over the flash descriptor never showed up as corruption.
       */

    BK7258_MB_SHARED_RW_START,
    BK7258_MB_SHARED_RW_SIZE,
    MPU_RBAR_XN | MPU_RBAR_AP_RWRW | MPU_RBAR_SH_INNER,
    MPU_RLAR_NONCACHEABLE
  },
  {
    BK7258_MB_UART_RX_ADDRESS,
    BK7258_MB_UART_CHUNK_SIZE,
    MPU_RBAR_XN | MPU_RBAR_AP_RORO | MPU_RBAR_SH_INNER,
    MPU_RLAR_NONCACHEABLE
  },
  {
    BK7258_MB_UART_TX_ADDRESS,
    BK7258_MB_UART_CHUNK_SIZE,
    MPU_RBAR_XN | MPU_RBAR_AP_RWRW | MPU_RBAR_SH_INNER,
    MPU_RLAR_NONCACHEABLE
  },
#ifdef CONFIG_BK7258_PSRAM
  {
    BK7258_PSRAM_BASE,
    BK7258_PSRAM_SIZE,
      /* Match the BK7258 vendor AP mapping.  The PSRAM controller's normal,
       * non-cacheable window is non-shareable on this interconnect.
       */

    MPU_RBAR_XN | MPU_RBAR_AP_RWRW | MPU_RBAR_SH_NO,
    MPU_RLAR_NONCACHEABLE
  },
#endif
  {
    0x40000000u,
    0x20000000u,
    MPU_RBAR_XN | MPU_RBAR_AP_RWRW | MPU_RBAR_SH_INNER,
    MPU_RLAR_DEVICE
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void __attribute__((noinline, target("general-regs-only")))
bk7258_cpu_private_initialize(bool primary)
{
  const size_t region_count = sizeof(g_bk7258_mpu_regions) /
                              sizeof(g_bk7258_mpu_regions[0]);

  modifyreg32(NVIC_CPACR, 0,
              NVIC_CPACR_CP_FULL(10) | NVIC_CPACR_CP_FULL(11));
  UP_DSB();
  UP_ISB();

  putreg32(0, BK7258_SAU_BASE + 0x08);
  putreg32(0x00000000u, BK7258_SAU_BASE + 0x0c);
  putreg32(0x0fffffe3u, BK7258_SAU_BASE + 0x10);
  putreg32(1, BK7258_SAU_BASE + 0x08);
  putreg32(0x10000000u, BK7258_SAU_BASE + 0x0c);
  putreg32(0xefffffe1u, BK7258_SAU_BASE + 0x10);
  putreg32(1, BK7258_SAU_BASE + 0x00);
  UP_DSB();
  UP_ISB();

  putreg32((uintptr_t)__ram_vectors_start, NVIC_VECTAB);
  UP_DSB();
  UP_ISB();

#ifdef CONFIG_ARM_MPU
  const uint32_t hardware_regions =
    (getreg32(MPU_TYPE) & MPU_TYPE_DREGION_MASK) >>
    MPU_TYPE_DREGION_SHIFT;

  if (hardware_regions < region_count)
    {
      for (; ; )
        {
          __asm__ volatile("wfi");
        }
    }

  mpu_reset();
  mpu_initialize(g_bk7258_mpu_regions, region_count, false, true);

#ifdef CONFIG_ARMV8M_ICACHE
  /* Turn the I-Cache on now that the MPU says the flash region is
   * write-through cacheable.
   *
   * The AP executes in place from QSPI flash at 0x02150000, and nothing
   * used to enable this: setting CONFIG_ARMV8M_ICACHE alone only compiles
   * up_enable_icache() in, it does not call it, and every other ARMv8-M
   * chip in the tree calls it from its own start code.  The cost of not
   * doing so is one flash access per instruction fetch -- measured at
   * roughly 1ms per drawn pixel in the expression renderer, about four
   * orders of magnitude off the same code on a host, and unaffected by
   * raising the core to 480MHz because the core was never the limit.
   *
   * CLIDR reads 0x09200003 on this part, so the cache is really there.
   * Only the I-Cache is enabled: instruction fetch is read-only, so it
   * needs no maintenance against the camera, panel and mailbox DMA that
   * shares AP RAM (which the MPU keeps non-cacheable for that reason).
   */

  up_enable_icache();
#endif
#endif

  (void)primary;
}

static void __attribute__((used, noinline, noreturn,
                           target("general-regs-only")))
bk7258_start(void)
{
  const uint8_t *src;
  uint8_t *dest;

  /* Configure the FPU before code compiled for the hard-float ABI can use
   * it.  arm_fpuconfig() does not depend on BSS-backed synchronization.
   */

  arm_fpuconfig();
  UP_DSB();
  UP_ISB();
  for (src = _eronly, dest = _sdata; dest < _edata; )
    {
      *dest++ = *src++;
    }

  for (dest = _sbss; dest < _ebss; )
    {
      *dest++ = 0;
    }

  src = __ram_vectors_load;
  dest = __ram_vectors_start;
  while (dest < __ram_vectors_end)
    {
      *dest++ = *src++;
    }

  bk7258_cpu_private_initialize(true);
  arm_initialize_stack();

  /* Install the buffered AP log channel before normal NuttX startup. */

  (void)bk7258_syslog_initialize();

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif

  nx_start();
  for (; ; )
    {
    }
}

void __attribute__((naked, noreturn, section(".start_text"))) __start(void)
{
  __asm__ volatile
    (
      "cpsid i\n"
      "movs r0, #0\n"
      "ldr r1, =0x20000000\n"
      "str r0, [r1]\n"
      "ldr r0, =__idle_stack_base\n"
      "msr msplim, r0\n"
      "b bk7258_start\n"
    );
}
