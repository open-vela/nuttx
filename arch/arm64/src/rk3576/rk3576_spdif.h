/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_spdif.h
 *
 * S/PDIF digital audio output driver for RK3576
 ***************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_SPDIF_H
#define __ARCH_ARM64_SRC_RK3576_Rk3576_SPDIF_H

#include <nuttx/config.h>
#include <stdint.h>

#ifndef __ASSEMBLY__



int rk3576_spdif_init(void);
int rk3576_spdif_enable(void);
int rk3576_spdif_disable(void);
int rk3576_spdif_set_samplerate(int rate);
int rk3576_spdif_write(const uint8_t *data, int len);

#endif /* __ASSEMBLY__ */

#endif
