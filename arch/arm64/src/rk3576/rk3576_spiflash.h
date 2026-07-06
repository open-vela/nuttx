/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_spiflash.h
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_SPIFLASH_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_SPIFLASH_H

#include <nuttx/config.h>
#include <stdint.h>

#ifndef __ASSEMBLY__



void rk3576_spiflash_init(void);
int  rk3576_spiflash_read(uint32_t addr, uint8_t *buf, uint32_t len);
int  rk3576_spiflash_write(uint32_t addr, const uint8_t *buf, uint32_t len);
int  rk3576_spiflash_erase(uint32_t addr, uint32_t len);
void rk3576_spiflash_read_id(uint8_t *mfg, uint8_t *type);

#endif /* __ASSEMBLY__ */

#endif
