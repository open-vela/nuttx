/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_ddr.h
 *
 * DDR Controller register definitions for RK3576
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_DDR_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_DDR_H

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/* DDR controller register offsets */

#define DDR_CONTRL               0x0000   /* DDR control register */
#define DDR_STATUS               0x0004   /* DDR status register */
#define DDR_CFG                  0x0008   /* DDR configuration */
#define DDR_TIMING0              0x0010   /* Timing register 0 */
#define DDR_TIMING1              0x0014   /* Timing register 1 */
#define DDR_TIMING2              0x0018   /* Timing register 2 */
#define DDR_TIMING3              0x001c   /* Timing register 3 */
#define DDR_REFRESH              0x0020   /* Refresh control */
#define DDR_BURST                0x0024   /* Burst control */
#define DDR_MODE                 0x0028   /* Mode register */
#define DDR_ZQ                   0x002c   /* ZQ calibration */
#define DDR_DRIVE                0x0030   /* Drive strength */

/* DDR_CONTRL bits */

#define DDR_CONTRL_ENABLE        (1 << 0)
#define DDR_CONTRL_START         (1 << 1)
#define DDR_CONTRL_RESET         (1 << 2)

/* DDR_STATUS bits */

#define DDR_STATUS_INIT_DONE     (1 << 0)
#define DDR_STATUS_DLL_LOCK      (1 << 1)

#endif
