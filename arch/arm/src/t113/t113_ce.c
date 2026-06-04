/****************************************************************************
 * arch/arm/src/t113/t113_ce.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied.  See the License for the specific language governing
 * permissions and limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <nuttx/crypto/crypto.h>

#include "arm_internal.h"
#include "hardware/t113_ce.h"
#include "hardware/t113_ccu.h"
#include "t113_ccu.h"
#include "t113_ce.h"

#ifdef CONFIG_T113_CE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AES_BLOCK_SIZE    16

/* SHA-256 block/digest sizes */

#define SHA256_BLOCK_SIZE   64
#define SHA256_DIGEST_WORDS 8   /* 256 bits = 8 words */
#define SHA256_DIGEST_SIZE  32

/* TRNG output: 8 words (256 bits) per request */

#define TRNG_OUT_WORDS      8
#define TRNG_OUT_SIZE       (TRNG_OUT_WORDS * 4)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Mutex protecting the CE hardware - only one task at a time */

static mutex_t g_ce_lock = NXMUTEX_INITIALIZER;

/* Task descriptor - aligned to 64 bytes.  Cortex-A7 L1 D-cache line is
 * 32 bytes; we use 64-byte alignment conservatively to cover L2 PL310
 * (64-byte line) so flush/invalidate doesn't affect adjacent data.
 * CE hardware itself only requires word (4-byte) alignment.
 */

static struct t113_ce_task_s g_ce_task
  aligned_data(64);

/* Scratch buffers for key/IV - must be word-aligned */

static uint32_t g_ce_keybuf[8] aligned_data(4);   /* up to 256-bit */
static uint32_t g_ce_ivbuf[8] aligned_data(4);    /* up to 256-bit */

/* Set by t113_ce_initialize(); checked by hot paths via DEBUGASSERT */

static bool g_ce_inited;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_ce_ccu_init
 *
 * Description:
 *   Enable CE module clock, bus gate, reset, and MBUS gate through CCU.
 *
 *   CE clock configuration:
 *     CE_CLK_REG (0x0680): source PLL_PERI(2X)=600MHz, N=1, M=3
 *                           => ce_clk = 600/1/3 = 200 MHz
 *     CE_BGR_REG (0x068C): bit0=gate, bit16=reset de-assert
 *     MBUS_MAT   (0x0804): bit2=CE MBUS gate
 *
 ****************************************************************************/

static void t113_ce_ccu_init(void)
{
  uint32_t clk_cfg;

  /* 1. Enable MBUS clock gate for CE (bit2) */

  t113_ccu_modify(T113_CCU_MBUS_MAT, 0, T113_CCU_MBUS_CE_EN);

  /* 2. Configure CE clock source: PLL_PERI(2X), N=1, M=3 => 200 MHz
   *    bits[26:24]=01 (PLL_PERI2X), bits[9:8]=00 (N=1),
   *    bits[3:0]=0x02 (M=2+1=3), bit31=1 (gate on)
   */

  clk_cfg = (1u << 31) |   /* Clock gate ON */
            (0x01 << 24) | /* PLL_PERI(2X) */
            (0x00 << 8) |  /* N = 1 */
            (0x02 << 0);   /* M = 2+1 = 3 */
  t113_ccu_modify(T113_CCU_CE_CLK, 0xffffffff, clk_cfg);

  /* 3. Enable CE bus gate (bit0) and de-assert reset (bit16) */

  t113_ccu_modify(T113_CCU_CE_BGR, 0, T113_CCU_CE_GATING);
  t113_ccu_modify(T113_CCU_CE_BGR, 0, T113_CCU_CE_RST);

  /* Delay for reset to propagate - Allwinner SoCs need a brief
   * pause after de-asserting peripheral reset before register access.
   */

  up_udelay(1);
  UP_DSB();
  UP_ISB();
}

/****************************************************************************
 * Name: t113_ce_wait_idle
 *
 * Description:
 *   Wait for TLR bit0 to become 0, indicating the CE can accept a new
 *   task descriptor.
 *
 ****************************************************************************/

static int t113_ce_wait_idle(void)
{
  volatile int timeout = CE_POLL_TIMEOUT_US;

  while ((getreg32(T113_CE_TLR) & CE_TLR_LOAD) != 0)
    {
      if (--timeout <= 0)
        {
          crypterr("ERROR: CE TLR busy timeout\n");
          return -ETIMEDOUT;
        }

      up_udelay(1);
    }

  return OK;
}

/****************************************************************************
 * Name: t113_ce_run_task
 *
 * Description:
 *   Submit the global task descriptor to the CE and poll until the
 *   designated channel completes.
 *
 * Input Parameters:
 *   channel - CE channel (0-3)
 *
 * Returned Value:
 *   OK on success, negative errno on error.
 *
 ****************************************************************************/

static int t113_ce_run_task(int channel)
{
  volatile int timeout;
  uint32_t isr;
  uint32_t esr;
  int ret;

  /* Ensure CE is idle */

  ret = t113_ce_wait_idle();
  if (ret < 0)
    {
      return ret;
    }

  /* Flush task descriptor from D-cache so CE DMA sees current data */

  up_flush_dcache((uintptr_t)&g_ce_task,
                  (uintptr_t)&g_ce_task + sizeof(g_ce_task));

  /* Write task descriptor physical address to CE_TDA */

  putreg32((uint32_t)(uintptr_t)&g_ce_task, T113_CE_TDA);

  UP_DSB();

  /* Trigger the task by setting TLR bit0 */

  putreg32(CE_TLR_LOAD, T113_CE_TLR);

  /* Poll CE_ISR for the channel's completion bit */

  timeout = CE_POLL_TIMEOUT_US;
  while (timeout > 0)
    {
      isr = getreg32(T113_CE_ISR);
      if (isr & CE_ISR_CHN_PEND(channel))
        {
          break;
        }

      up_udelay(1);
      timeout--;
    }

  /* Clear the pending bit (W1C) */

  putreg32(CE_ISR_CHN_PEND(channel), T113_CE_ISR);

  if (timeout <= 0)
    {
      crypterr("ERROR: CE task timeout (ch=%d)\n", channel);
      return -ETIMEDOUT;
    }

  /* Check for errors */

  esr = getreg32(T113_CE_ESR);
  if (esr & CE_ESR_CHN_MASK(channel))
    {
      uint32_t err = (esr >> CE_ESR_CHN_SHIFT(channel)) & 0xf;
      crypterr("ERROR: CE error ch=%d esr=0x%02" PRIx32 "\n",
               channel, err);

      /* Clear error bits */

      putreg32(CE_ESR_CHN_MASK(channel), T113_CE_ESR);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: t113_ce_setup_task
 *
 * Description:
 *   Zero the global task descriptor and set common fields.
 *
 ****************************************************************************/

static void t113_ce_setup_task(int channel)
{
  memset(&g_ce_task, 0, sizeof(g_ce_task));
  g_ce_task.task_id = (uint32_t)channel & 0xf;
}

#ifdef CONFIG_CRYPTO_AES
/****************************************************************************
 * Name: aes_cypher
 *
 * Description:
 *   NuttX standard AES cypher interface.  Supports ECB and CBC modes.
 *
 *   All length fields (data_len, src/dst scatter) are in WORDS (/4)
 *   for CE V3.1, except AES-CTS which uses BYTES.
 *
 ****************************************************************************/

int aes_cypher(FAR void *out, FAR const void *in, size_t size,
               FAR const void *iv, FAR const void *key,
               size_t keysize, int mode, int encrypt)
{
  uint32_t sym_ctl;
  uint32_t comm_ctl;
  int ret;

  DEBUGASSERT(g_ce_inited);

  /* Validate block alignment */

  if ((size & (AES_BLOCK_SIZE - 1)) != 0 || size == 0)
    {
      return -EINVAL;
    }

  /* Cache-line hygiene: in/out buffers must be 64-B aligned and size
   * a multiple of 64, otherwise up_invalidate_dcache() rounds to cache
   * lines and would evict neighbouring data when the caller's buffer
   * straddles a line boundary.
   */

  if (((uintptr_t)in & 63) || ((uintptr_t)out & 63) || (size & 63))
    {
      return -EINVAL;
    }

  /* Validate key size */

  switch (keysize)
    {
      case 16:
        sym_ctl = CE_AES_KEYSIZE_128;
        break;

      case 24:
        sym_ctl = CE_AES_KEYSIZE_192;
        break;

      case 32:
        sym_ctl = CE_AES_KEYSIZE_256;
        break;

      default:
        return -EINVAL;
    }

  /* Set algorithm mode */

  switch (mode & AES_MODE_MASK)
    {
      case AES_MODE_ECB:
        sym_ctl |= CE_MODE_ECB;
        break;

      case AES_MODE_CBC:
        sym_ctl |= CE_MODE_CBC;
        break;

      case AES_MODE_CTR:
        sym_ctl |= CE_MODE_CTR | CE_CTR_WIDTH_128;
        break;

      default:
        return -EINVAL;
    }

  /* Use input key (normal mode) */

  sym_ctl |= CE_KEY_SELECT_INPUT;

  /* Common control: AES algorithm */

  comm_ctl = CE_ALG_AES;

  if (!encrypt)
    {
      comm_ctl |= CE_DIR_DECRYPT;
    }

  ret = nxmutex_lock(&g_ce_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Copy key to aligned buffer */

  memcpy(g_ce_keybuf, key, keysize);

  /* Copy IV if present */

  if (iv != NULL)
    {
      memcpy(g_ce_ivbuf, iv, AES_BLOCK_SIZE);
    }

  /* T113 uses flat VA=PA mapping; DMA needs the physical address.  The
   * cast below narrows uintptr_t to uint32_t - assert all addresses
   * fit in 32 bits so the contract is explicit.
   */

  DEBUGASSERT((uintptr_t)g_ce_keybuf < 0x100000000ull);
  DEBUGASSERT((uintptr_t)g_ce_ivbuf  < 0x100000000ull);
  DEBUGASSERT((uintptr_t)in  < 0x100000000ull);
  DEBUGASSERT((uintptr_t)out < 0x100000000ull);

  /* Build task descriptor on channel 0 */

  t113_ce_setup_task(0);
  g_ce_task.common_ctl = comm_ctl;
  g_ce_task.sym_ctl    = sym_ctl;
  g_ce_task.key_addr   = (uint32_t)(uintptr_t)g_ce_keybuf;

  if (iv != NULL)
    {
      g_ce_task.iv_addr = (uint32_t)(uintptr_t)g_ce_ivbuf;
    }

  /* Data length in WORDS for AES (non-CTS) */

  g_ce_task.data_len     = (uint32_t)(size / 4);

  /* Single scatter-gather segment */

  g_ce_task.src[0].addr  = (uint32_t)(uintptr_t)in;
  g_ce_task.src[0].len   = (uint32_t)(size / 4);
  g_ce_task.dst[0].addr  = (uint32_t)(uintptr_t)out;
  g_ce_task.dst[0].len   = (uint32_t)(size / 4);

  /* Flush input data, key and IV from D-cache so CE DMA sees them */

  up_flush_dcache((uintptr_t)g_ce_keybuf,
                  (uintptr_t)g_ce_keybuf + sizeof(g_ce_keybuf));
  if (iv != NULL)
    {
      up_flush_dcache((uintptr_t)g_ce_ivbuf,
                      (uintptr_t)g_ce_ivbuf + sizeof(g_ce_ivbuf));
    }

  up_flush_dcache((uintptr_t)in, (uintptr_t)in + size);

  /* Flush output buffer BEFORE DMA to evict any dirty cache lines.
   * On Cortex-A7, DCIMVAC (invalidate) on a dirty line writes back
   * the stale data first, overwriting the DMA result.  Flushing here
   * ensures the lines are clean before CE DMA writes.
   */

  up_flush_dcache((uintptr_t)out, (uintptr_t)out + size);
  UP_DSB();

  ret = t113_ce_run_task(0);

  /* Invalidate output buffer so CPU reads CE DMA result from DRAM */

  up_invalidate_dcache((uintptr_t)out, (uintptr_t)out + size);

  /* Wipe key / IV material from RAM and their D-cache lines.  These
   * are static globals; leaving sensitive data behind is recoverable
   * via /dev/mem or a core dump.
   */

  explicit_bzero(g_ce_keybuf, sizeof(g_ce_keybuf));
  explicit_bzero(g_ce_ivbuf,  sizeof(g_ce_ivbuf));
  up_flush_dcache((uintptr_t)g_ce_keybuf,
                  (uintptr_t)g_ce_keybuf + sizeof(g_ce_keybuf));
  up_flush_dcache((uintptr_t)g_ce_ivbuf,
                  (uintptr_t)g_ce_ivbuf + sizeof(g_ce_ivbuf));

  nxmutex_unlock(&g_ce_lock);
  return ret;
}
#endif /* CONFIG_CRYPTO_AES */

#ifdef CONFIG_T113_CE_SHA256
/****************************************************************************
 * Name: t113_ce_sha256
 *
 * Description:
 *   Compute SHA-256 digest of the given input data.
 *
 *   The CE requires SHA input to be padded to 64-byte multiples by
 *   software.  This function performs the padding internally.
 *
 * Input Parameters:
 *   in      - Input data
 *   insize  - Input data length in bytes
 *   digest  - Output buffer for 32-byte SHA-256 digest
 *
 * Returned Value:
 *   OK on success; negative errno on failure.
 *
 ****************************************************************************/

int t113_ce_sha256(FAR const void *in, size_t insize,
                   FAR uint8_t *digest)
{
  /* SHA-256 padding: append 0x80, then zeros, then 64-bit big-endian
   * bit count.  Padded length is next multiple of 64 bytes.
   */

  uint64_t bitlen = (uint64_t)insize * 8;
  size_t padded_len;
  FAR uint8_t *padbuf;
  size_t i;
  int ret;

  DEBUGASSERT(g_ce_inited);

  /* Digest buffer must be 32-byte aligned so the full SHA-256 digest
   * sits in a single cache line and up_invalidate_dcache() will not
   * evict neighbouring data.
   */

  if (((uintptr_t)digest & 31) != 0)
    {
      return -EINVAL;
    }

  /* Calculate padded length: insize + 1 (0x80) + padding + 8 (length)
   * must be multiple of 64
   */

  padded_len = ((insize + 1 + 8 + 63) / 64) * 64;

  padbuf = kmm_zalloc(padded_len);
  if (padbuf == NULL)
    {
      return -ENOMEM;
    }

  memcpy(padbuf, in, insize);
  padbuf[insize] = 0x80;

  /* Append bit length as big-endian 64-bit at end of padded buffer */

  for (i = 0; i < 8; i++)
    {
      padbuf[padded_len - 1 - i] = (uint8_t)(bitlen >> (i * 8));
    }

  ret = nxmutex_lock(&g_ce_lock);
  if (ret < 0)
    {
      kmm_free(padbuf);
      return ret;
    }

  /* T113 flat VA=PA mapping - assert DMA addresses fit in 32 bits. */

  DEBUGASSERT((uintptr_t)padbuf < 0x100000000ull);
  DEBUGASSERT((uintptr_t)digest < 0x100000000ull);

  /* Build task descriptor on channel 0 */

  t113_ce_setup_task(0);
  g_ce_task.common_ctl = CE_ALG_SHA256;  /* IV mode = 0 (FIPS constants) */

  /* SHA data length is in WORDS */

  g_ce_task.data_len     = (uint32_t)(padded_len / 4);

  g_ce_task.src[0].addr  = (uint32_t)(uintptr_t)padbuf;
  g_ce_task.src[0].len   = (uint32_t)(padded_len / 4);
  g_ce_task.dst[0].addr  = (uint32_t)(uintptr_t)digest;
  g_ce_task.dst[0].len   = SHA256_DIGEST_WORDS;

  /* Flush padded input from D-cache so CE DMA sees it */

  up_flush_dcache((uintptr_t)padbuf, (uintptr_t)padbuf + padded_len);

  /* Flush digest buffer to evict dirty lines before DMA */

  up_flush_dcache((uintptr_t)digest,
                  (uintptr_t)digest + SHA256_DIGEST_SIZE);
  UP_DSB();

  ret = t113_ce_run_task(0);

  /* Invalidate digest buffer so CPU reads CE DMA result */

  up_invalidate_dcache((uintptr_t)digest,
                       (uintptr_t)digest + SHA256_DIGEST_SIZE);

  nxmutex_unlock(&g_ce_lock);
  kmm_free(padbuf);

  return ret;
}
#endif /* CONFIG_T113_CE_SHA256 */

#ifdef CONFIG_T113_CE_TRNG
/****************************************************************************
 * Name: t113_ce_trng_read
 *
 * Description:
 *   Read true-random bytes from the CE TRNG.  The CE TRNG produces
 *   256 bits (8 words) per request.
 *
 ****************************************************************************/

static ssize_t t113_ce_trng_read(FAR uint8_t *buf, size_t len)
{
  static uint32_t trng_out[TRNG_OUT_WORDS] aligned_data(4);
  size_t total = 0;
  int ret;

  DEBUGASSERT(g_ce_inited);

  while (total < len)
    {
      size_t chunk;

      ret = nxmutex_lock(&g_ce_lock);
      if (ret < 0)
        {
          return (total > 0) ? (ssize_t)total : ret;
        }

      /* Build TRNG task on channel 0 */

      t113_ce_setup_task(0);
      g_ce_task.common_ctl = CE_ALG_TRNG;

      /* TRNG: no source needed; data_len and scatter len in words
       * (CE V3.1, same as AES/SHA).  8 words = 256 bits per request.
       */

      /* T113 flat VA=PA mapping - assert DMA address fits in 32 bits. */

      DEBUGASSERT((uintptr_t)trng_out < 0x100000000ull);

      g_ce_task.data_len     = TRNG_OUT_WORDS;
      g_ce_task.dst[0].addr  = (uint32_t)(uintptr_t)trng_out;
      g_ce_task.dst[0].len   = TRNG_OUT_WORDS;

      /* Flush output buffer before DMA (same Cortex-A7 dirty-line
       * writeback issue as AES - see aes_cypher comment).
       */

      up_flush_dcache((uintptr_t)trng_out,
                      (uintptr_t)trng_out + TRNG_OUT_SIZE);
      UP_DSB();

      ret = t113_ce_run_task(0);
      if (ret < 0)
        {
          nxmutex_unlock(&g_ce_lock);
          return (total > 0) ? (ssize_t)total : ret;
        }

      /* Invalidate D-cache for TRNG output buffer, then copy.
       * Must hold mutex until memcpy completes - trng_out is static
       * and another thread could start a new TRNG task after unlock.
       */

      up_invalidate_dcache((uintptr_t)trng_out,
                           (uintptr_t)trng_out + TRNG_OUT_SIZE);

      chunk = len - total;
      if (chunk > TRNG_OUT_SIZE)
        {
          chunk = TRNG_OUT_SIZE;
        }

      memcpy(buf + total, trng_out, chunk);

      nxmutex_unlock(&g_ce_lock);
      total += chunk;
    }

  return (ssize_t)total;
}

/****************************************************************************
 * /dev/urandom character driver operations for TRNG
 ****************************************************************************/

static ssize_t t113_urandom_read(FAR struct file *filep,
                                 FAR char *buffer, size_t buflen)
{
  return t113_ce_trng_read((FAR uint8_t *)buffer, buflen);
}

static ssize_t t113_urandom_write(FAR struct file *filep,
                                  FAR const char *buffer,
                                  size_t buflen)
{
  /* CE TRNG has no entropy-mixing register; we cannot honor a write.
   * Returning buflen would silently lie to callers that rely on write
   * side-effects (e.g., seeding), so reject per POSIX semantics.
   */

  UNUSED(filep);
  UNUSED(buffer);
  UNUSED(buflen);
  return -EPERM;
}

static const struct file_operations g_urandom_fops =
{
  NULL,                /* open */
  NULL,                /* close */
  t113_urandom_read,   /* read */
  t113_urandom_write,  /* write */
  NULL,                /* seek */
  NULL,                /* ioctl */
  NULL,                /* mmap */
  NULL,                /* truncate */
  NULL,                /* poll */
};

/****************************************************************************
 * Name: devurandom_register
 *
 * Description:
 *   Register /dev/urandom backed by the CE TRNG.
 *   This overrides the weak default in drivers/crypto/dev_urandom.c
 *   when CONFIG_DEV_URANDOM_ARCH is selected.
 *
 ****************************************************************************/

void devurandom_register(void)
{
  register_driver("/dev/urandom", &g_urandom_fops, 0666, NULL);
}

#ifdef CONFIG_DEV_RANDOM
/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Register /dev/random backed by the CE TRNG, same as /dev/urandom.
 *
 ****************************************************************************/

void devrandom_register(void)
{
  register_driver("/dev/random", &g_urandom_fops, 0666, NULL);
}
#endif /* CONFIG_DEV_RANDOM */
#endif /* CONFIG_T113_CE_TRNG */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_ce_initialize
 *
 * Description:
 *   Initialize the T113 Crypto Engine hardware.
 *
 ****************************************************************************/

int t113_ce_initialize(void)
{
  /* Enable clocks and release reset */

  t113_ce_ccu_init();

  g_ce_inited = true;

  cryptinfo("T113 CE initialized\n");

  return OK;
}

#endif /* CONFIG_T113_CE */
