/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_spiflash.h
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_SPIFLASH_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_SPIFLASH_H

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

#define SF_CTRL                     0x0000
#define SF_ADDR                     0x0004
#define SF_DIR                      0x0008
#define SF_LEN                      0x000c
#define SF_DATA                     0x0010
#define SF_INTEN                    0x0014
#define SF_INTSTS                   0x0018
#define SF_CTRL1                    0x001c
#define SF_XIP_CFG                  0x0020
#define SF_SCAN_CTRL                0x0024
#define SF_IODR                     0x0028
#define SF_IEN                      0x002c
#define SF_STATE                    0x0030
#define SF_ERR                      0x0034
#define SF_FLR                      0x0038

#define SF_CTRL_EN                  (1 << 0)
#define SF_CTRL_DIR_READ            (0 << 1)
#define SF_CTRL_DIR_WRITE           (1 << 1)

#define SF_SR_IS_BUSY               (1 << 0)
#define SF_SR_WIP                   (1 << 1)

#endif
