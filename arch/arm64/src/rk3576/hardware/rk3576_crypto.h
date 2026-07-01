/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_crypto.h
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_CRYPTO_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_CRYPTO_H

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

#define CRYPTO_CTRL                0x0000
#define CRYPTO_INTEN               0x0004
#define CRYPTO_INTSTS              0x0008
#define CRYPTO_LR_KEY_AES          0x0010
#define CRYPTO_BR_KEY_AES          0x0014
#define CRYPTO_AES_CTRL            0x0080
#define CRYPTO_AES_STATUS          0x0084
#define CRYPTO_AES_SRC_ADDR        0x0088
#define CRYPTO_AES_DST_ADDR        0x008c
#define CRYPTO_AES_LEN             0x0090
#define CRYPTO_AES_IV              0x0094
#define CRYPTO_AES_KEY             0x00a0
#define CRYPTO_SHA_CTRL            0x0100
#define CRYPTO_SHA_STATUS          0x0104
#define CRYPTO_SHA_SRC_ADDR        0x0108
#define CRYPTO_SHA_LEN             0x010c
#define CRYPTO_SHA_HASH            0x0110

#define CRYPTO_AES_ENABLE          (1 << 0)
#define CRYPTO_SHA_ENABLE          (1 << 1)

#define CRYPTO_AES_MODE_ECB        (0 << 4)
#define CRYPTO_AES_MODE_CBC        (1 << 4)
#define CRYPTO_AES_MODE_CTR        (2 << 4)

#define CRYPTO_AES_KEY_128         (0 << 6)
#define CRYPTO_AES_KEY_192         (1 << 6)
#define CRYPTO_AES_KEY_256         (2 << 6)

#endif
