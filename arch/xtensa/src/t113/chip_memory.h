/****************************************************************************
 * arch/xtensa/src/t113/chip_memory.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * T113-S3 HiFi4 DSP memory map (DSP-view physical addresses).
 *
 *   IRAM_STUB  : 0x00400000  4 KB  reset stub
 *   IRAM_VECT  : 0x00401000  2 KB  exception/interrupt vectors
 *   DRAM0      : 0x00420000  128 KB
 *   DRAM1      : 0x00440000  128 KB
 *   DDR cached : 0x37900000  4 MB  bulk firmware
 *                                  (.text/.rodata/.data/.bss/stack/heap)
 *   carveout   : 0x37D00000  1 MB  rptun vrings + rpmsg buffer pool
 *   DDR ncache : 0x17900000  alias of the same physical pages, non-cacheable
 *
 * AP-view of DDR cached:  0x47900000.
 *
 * The 5 MB DDR window is split: the low 4 MB hold the firmware image, the
 * runtime heap and the stack; the top 1 MB is the rptun carveout (the vring
 * control structures and the rpmsg buffer pool, see t113_rsctable.c).  The
 * heap upper bound and the linker DDR region (t113-evb-dsp.ld) must both
 * stop at the carveout base, or the heap (which environ/malloc draws from)
 * collides with the rpmsg shared memory once the AP starts the DSP.
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_T113_CHIP_MEMORY_H
#define __ARCH_XTENSA_SRC_T113_CHIP_MEMORY_H

#define T113_IRAM_STUB_BASE    0x00400000
#define T113_IRAM_STUB_SIZE    0x00001000
#define T113_IRAM_VECT_BASE    0x00401000
#define T113_IRAM_VECT_SIZE    0x00000800

#define T113_DDR_CACHED_BASE   0x37900000
#define T113_DDR_CACHED_SIZE   0x00500000     /* full 5 MB shared window */
#define T113_DDR_NONCACHED_BASE 0x17900000

/* The top 1 MB of the window is the rptun carveout (vrings + rpmsg buffer
 * pool, see t113_rsctable.c CARVEOUT_DA).  Firmware image, heap and stack
 * use only the low 4 MB; nothing the DSP allocates may cross into the
 * carveout or it corrupts the rpmsg shared memory.
 */

#define T113_DDR_CARVEOUT_SIZE 0x00100000
#define T113_DDR_CARVEOUT_BASE \
  (T113_DDR_CACHED_BASE + T113_DDR_CACHED_SIZE - T113_DDR_CARVEOUT_SIZE)

/* Firmware/heap/stack region: the window below the carveout. */

#define T113_DDR_FW_LIMIT      T113_DDR_CARVEOUT_BASE

/* Stack lives at top of the firmware region (just below the carveout).
 * Vectors point _stack_top here.
 */

#define T113_STACK_TOP         T113_DDR_FW_LIMIT

/* UART2 (for serial driver). */

#define T113_UART2_BASE        0x02500800

/* DSP_INTC: per T113-i UserManual V1.4 memory map ("DSP_SYS Related"),
 * the DSP-local interrupt controller occupies 0x01700800..0x01700BFF
 * within the 4 KB DSP_SYS_CFG aperture (DSP_CFG=0x01700000 + 0x800).
 * Fans up to 88 SoC peripheral sources into the HiFi4 core's INT 20.
 * The datasheet does not publish a register-offset table; layout here
 * is derived empirically from baremetal reference firmware that ran
 * on the same silicon.
 */

#define T113_DSP_INTC_BASE     0x01700800

#endif /* __ARCH_XTENSA_SRC_T113_CHIP_MEMORY_H */
