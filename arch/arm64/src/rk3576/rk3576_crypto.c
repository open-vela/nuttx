/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_crypto.c - Hardware Crypto driver
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <debug.h>
#include <nuttx/arch.h>
#include "arm64_internal.h"
#include "hardware/rk3576_crypto.h"
#include "rk3576_crypto.h"

static int rk3576_crypto_wait_done(uint32_t flag)
{
  int timeout = 10000;
  while (timeout--)
    {
      uint32_t status = getreg32(RK3576_CRYPTO_ADDR + CRYPTO_AES_STATUS);
      if (status & flag) return OK;
      up_udelay(10);
    }
  return -ETIMEDOUT;
}

void rk3576_crypto_init(void)
{
  putreg32(0, RK3576_CRYPTO_ADDR + CRYPTO_CTRL);
  putreg32(0xffffffff, RK3576_CRYPTO_ADDR + CRYPTO_INTSTS);
  uinfo("Crypto: initialized\n");
}

int rk3576_crypto_aes_encrypt(int mode, int keybits,
                               const uint8_t *key, const uint8_t *iv,
                               const uint8_t *src, uint8_t *dst, uint32_t len)
{
  uint32_t base = RK3576_CRYPTO_ADDR;
  uint32_t ctrl = CRYPTO_AES_ENABLE;

  ctrl |= (mode << 4);
  if (keybits == 256) ctrl |= CRYPTO_AES_KEY_256;
  else if (keybits == 192) ctrl |= CRYPTO_AES_KEY_192;

  putreg32(ctrl, base + CRYPTO_AES_CTRL);
  putreg32(len, base + CRYPTO_AES_LEN);

  if (iv)
    {
      for (int i = 0; i < 4; i++)
        putreg32(((uint32_t)iv[i*4] | (iv[i*4+1] << 8) |
                 (iv[i*4+2] << 16) | (iv[i*4+3] << 24)),
                 base + CRYPTO_AES_IV + i * 4);
    }

  int key_words = keybits / 32;
  for (int i = 0; i < key_words; i++)
    putreg32(((uint32_t)key[i*4] | (key[i*4+1] << 8) |
             (key[i*4+2] << 16) | (key[i*4+3] << 24)),
             base + CRYPTO_AES_KEY + i * 4);

  /* Verify addresses fit in 32-bit DMA address space */

  if ((uintptr_t)src > 0xFFFFFFFF || (uintptr_t)dst > 0xFFFFFFFF)
    {
      return -EINVAL;
    }

  putreg32((uint32_t)(uintptr_t)src, base + CRYPTO_AES_SRC_ADDR);
  putreg32((uint32_t)(uintptr_t)dst, base + CRYPTO_AES_DST_ADDR);
  putreg32(CRYPTO_AES_ENABLE, base + CRYPTO_AES_STATUS);

  return rk3576_crypto_wait_done(CRYPTO_AES_ENABLE);
}

int rk3576_crypto_aes_decrypt(int mode, int keybits,
                               const uint8_t *key, const uint8_t *iv,
                               const uint8_t *src, uint8_t *dst, uint32_t len)
{
  return rk3576_crypto_aes_encrypt(mode | 0x100, keybits, key, iv,
                                    src, dst, len);
}

int rk3576_crypto_sha256(const uint8_t *src, uint32_t len, uint8_t *hash)
{
  uint32_t base = RK3576_CRYPTO_ADDR;
  putreg32(CRYPTO_SHA_ENABLE, base + CRYPTO_SHA_CTRL);
  putreg32(len, base + CRYPTO_SHA_LEN);
  putreg32((uint32_t)(uintptr_t)src, base + CRYPTO_SHA_SRC_ADDR);
  putreg32(CRYPTO_SHA_ENABLE, base + CRYPTO_SHA_STATUS);

  int ret = rk3576_crypto_wait_done(CRYPTO_SHA_ENABLE);
  if (ret < 0) return ret;

  for (int i = 0; i < 8; i++)
    {
      uint32_t word = getreg32(base + CRYPTO_SHA_HASH + i * 4);
      hash[i*4] = word & 0xff;
      hash[i*4+1] = (word >> 8) & 0xff;
      hash[i*4+2] = (word >> 16) & 0xff;
      hash[i*4+3] = (word >> 24) & 0xff;
    }

  return OK;
}
