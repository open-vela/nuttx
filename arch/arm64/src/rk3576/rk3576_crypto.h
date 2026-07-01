/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_crypto.h
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_CRYPTO_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_CRYPTO_H

#include <nuttx/config.h>
#include <stdint.h>

#ifndef __ASSEMBLY__



void rk3576_crypto_init(void);
int  rk3576_crypto_aes_encrypt(int mode, int keybits,
                                const uint8_t *key, const uint8_t *iv,
                                const uint8_t *src, uint8_t *dst, uint32_t len);
int  rk3576_crypto_aes_decrypt(int mode, int keybits,
                                const uint8_t *key, const uint8_t *iv,
                                const uint8_t *src, uint8_t *dst, uint32_t len);
int  rk3576_crypto_sha256(const uint8_t *src, uint32_t len, uint8_t *hash);

#endif /* __ASSEMBLY__ */

#endif
