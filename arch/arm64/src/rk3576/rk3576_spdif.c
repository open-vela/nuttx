/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_spdif.c
 *
 * S/PDIF digital audio output driver for RK3576
 ***************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "arm64_internal.h"
#include "hardware/rk3576_memorymap.h"
#include "rk3576_spdif.h"

#ifdef CONFIG_RK3576_SPDIF

#define SPDIF_BASE              RK3576_SPDIF_ADDR

#define SPDIF_CTRL              0x0000
#define SPDIF_SFCR              0x0004
#define SPDIF_INTEN             0x0008
#define SPDIF_INTSTS            0x000c
#define SPDIF_DATA              0x0010
#define SPDIF_FIFO              0x0014
#define SPDIF_DMA_CTRL          0x0018

/* SPDIF_CTRL bits */

#define SPDIF_CTRL_EN           (1 << 0)
#define SPDIF_CTRL_TX_MODE      (1 << 1)

/* SPDIF_SFCR bits */

#define SPDIF_SFCR_SRC_SEL_SHIFT 0
#define SPDIF_SFCR_WS_POL       (1 << 4)
#define SPDIF_SFCR_CHN_MODE_SHIFT 5

/* SPDIF_INTEN bits */

#define SPDIF_INTEN_TX_EMPTY    (1 << 0)
#define SPDIF_INTEN_TX_FULL     (1 << 1)

struct rk3576_spdif_s
{
  bool enabled;
  int sample_rate;
};

static struct rk3576_spdif_s g_spdif;

int rk3576_spdif_init(void)
{
  /* Power down SPDIF */

  putreg32(0, SPDIF_BASE + SPDIF_CTRL);

  memset(&g_spdif, 0, sizeof(g_spdif));

  ginfo("SPDIF: initialized\n");
  return OK;
}

int rk3576_spdif_enable(void)
{
  putreg32(SPDIF_CTRL_EN | SPDIF_CTRL_TX_MODE,
           SPDIF_BASE + SPDIF_CTRL);

  g_spdif.enabled = true;

  ginfo("SPDIF: enabled\n");
  return OK;
}

int rk3576_spdif_disable(void)
{
  putreg32(0, SPDIF_BASE + SPDIF_CTRL);

  g_spdif.enabled = false;

  ginfo("SPDIF: disabled\n");
  return OK;
}

int rk3576_spdif_set_samplerate(int rate)
{
  uint32_t sfcr = 0;

  /* Configure source clock based on sample rate */

  switch (rate)
    {
    case 32000:
      sfcr = (0 << SPDIF_SFCR_SRC_SEL_SHIFT);
      break;
    case 44100:
      sfcr = (1 << SPDIF_SFCR_SRC_SEL_SHIFT);
      break;
    case 48000:
      sfcr = (2 << SPDIF_SFCR_SRC_SEL_SHIFT);
      break;
    case 96000:
      sfcr = (3 << SPDIF_SFCR_SRC_SEL_SHIFT);
      break;
    default:
      sfcr = (2 << SPDIF_SFCR_SRC_SEL_SHIFT);
      rate = 48000;
      break;
    }

  putreg32(sfcr, SPDIF_BASE + SPDIF_SFCR);

  g_spdif.sample_rate = rate;

  ginfo("SPDIF: sample rate %d Hz\n", rate);
  return OK;
}

int rk3576_spdif_write(const uint8_t *data, int len)
{
  if (!g_spdif.enabled)
    {
      return -EACCES;
    }

  /* Write data to SPDIF FIFO */

  const uint32_t *src = (const uint32_t *)data;
  int words = len / 4;

  for (int i = 0; i < words; i++)
    {
      putreg32(src[i], SPDIF_BASE + SPDIF_DATA);
    }

  return len;
}

#endif /* CONFIG_RK3576_SPDIF */
