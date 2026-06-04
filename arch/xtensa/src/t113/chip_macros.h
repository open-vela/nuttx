/****************************************************************************
 * arch/xtensa/src/t113/chip_macros.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_T113_CHIP_MACROS_H
#define __ARCH_XTENSA_SRC_T113_CHIP_MACROS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/xtensa/core.h>

/* HANDLER_SECTION names where the upstream Xtensa low-level asm bodies land
 * (xtensa_user_handler.S / xtensa_int_handlers.S / xtensa_context.S /
 * xtensa_panic.S all start with `.section HANDLER_SECTION, "ax"`).  We use
 * a chip-specific section name `.iram_hand.text` so the linker script can
 * place all upstream handler code into a dedicated 122 KB IRAM region
 * (IRAM_HAND at 0x00401800).  Vector slots in IRAM_VECT (0x00401000..)
 * short-jump to bodies in IRAM_HAND -- both are within +/-128 KB so the
 * 'j' instruction reaches.  The literal pool `.iram_hand.literal` is
 * placed before consumers in the SECTIONS rule so backward-only `l32r`
 * loads resolve.
 */

#define HANDLER_SECTION .iram_hand.text

#endif /* __ARCH_XTENSA_SRC_T113_CHIP_MACROS_H */
