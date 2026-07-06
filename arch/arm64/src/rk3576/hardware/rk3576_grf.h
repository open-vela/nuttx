/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_grf.h
 *
 * General Register File (GRF) register definitions for RK3576
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_GRF_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_GRF_H

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/* GRF register offsets */

#define GRF_SOC_CON(n)          (0x0000 + (n) * 0x04)
#define GRF_SOC_STATUS(n)       (0x0050 + (n) * 0x04)
#define GRF_CPU_CON             0x0100
#define GRF_CPU_STATUS          0x0120
#define GRF_PERI_CON            0x0200
#define GRF_PERI_STATUS         0x0220
#define GRF_DDRC_CON            0x0300
#define GRF_DDRC_STATUS         0x0320
#define GRF_USB_CON             0x0400
#define GRF_USB_STATUS          0x0420
#define GRF_AUDIO_CON           0x0500
#define GRF_AUDIO_STATUS        0x0520
#define GRF_GPU_CON             0x0600
#define GRF_GPU_STATUS          0x0620
#define GRF_VOP_CON             0x0700
#define GRF_VOP_STATUS          0x0720
#define GRF_HDMI_CON            0x0800
#define GRF_HDMI_STATUS         0x0820
#define GRF_DSI_CON             0x0900
#define GRF_DSI_STATUS          0x0920
#define GRF_ETH_CON             0x0a00
#define GRF_ETH_STATUS          0x0a20
#define GRF_NPU_CON             0x0b00
#define GRF_NPU_STATUS          0x0b20
#define GRF_SECURITY_CON        0x0c00

#endif
