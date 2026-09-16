#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timers/watchdog.h>

#include "loongarch_internal.h"
#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_wdt.h"

#if defined(CONFIG_LS2K0300_WDT)

#define LS2K0300_WDT_CLOCK_FREQ 200000000UL

struct ls2k0300_wdt_priv_s
{
  struct watchdog_lowerhalf_s lower;
  uintptr_t base;
  uint32_t clock_freq;
  uint32_t timeout;
  uint32_t max_hw_heatbeat_ms;
  bool started;
};

static uint32_t wdt_getreg(FAR struct ls2k0300_wdt_priv_s *priv,
                           uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static void wdt_putreg(FAR struct ls2k0300_wdt_priv_s *priv,
                       uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static int ls2k0300_wdt_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct ls2k0300_wdt_priv_s *priv =
      (FAR struct ls2k0300_wdt_priv_s *)lower;

  wdt_putreg(priv, LS2K0300_WDT_RST_CTR, WDT_RST_CTR_ENABLE);
  wdt_putreg(priv, LS2K0300_WDT_CR, WDT_CR_RESET);
  priv->started = true;

  return OK;
}

static int ls2k0300_wdt_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct ls2k0300_wdt_priv_s *priv =
      (FAR struct ls2k0300_wdt_priv_s *)lower;

  wdt_putreg(priv, LS2K0300_WDT_RST_CTR, 0);
  priv->started = false;

  return OK;
}

static int ls2k0300_wdt_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct ls2k0300_wdt_priv_s *priv =
      (FAR struct ls2k0300_wdt_priv_s *)lower;

  wdt_putreg(priv, LS2K0300_WDT_CR, WDT_CR_RESET);

  return OK;
}

static int ls2k0300_wdt_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                                 FAR struct watchdog_status_s *status)
{
  FAR struct ls2k0300_wdt_priv_s *priv =
      (FAR struct ls2k0300_wdt_priv_s *)lower;
  uint32_t rst_ctr;

  rst_ctr = wdt_getreg(priv, LS2K0300_WDT_RST_CTR);

  status->flags = 0;
  if (rst_ctr & WDT_RST_CTR_ENABLE)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  status->flags |= WDFLAGS_RESET;
  status->timeout = priv->timeout;

  return OK;
}

static int ls2k0300_wdt_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                                  uint32_t timeout)
{
  FAR struct ls2k0300_wdt_priv_s *priv =
      (FAR struct ls2k0300_wdt_priv_s *)lower;
  uint64_t counts64;
  uint32_t counts;

  if (timeout < 1)
    {
      return -EINVAL;
    }

  if (timeout > priv->max_hw_heatbeat_ms)
    {
      timeout = priv->max_hw_heatbeat_ms;
    }

  counts64 = (uint64_t)priv->clock_freq * timeout / 1000;
  if (counts64 > UINT32_MAX)
    {
      return -EINVAL;
    }

  counts = (uint32_t)counts64;
  wdt_putreg(priv, LS2K0300_WDT_TIMER, counts);

  priv->timeout = timeout;

  return OK;
}

static const struct watchdog_ops_s g_wdt_ops =
{
  ls2k0300_wdt_start,
  ls2k0300_wdt_stop,
  ls2k0300_wdt_keepalive,
  ls2k0300_wdt_getstatus,
  ls2k0300_wdt_settimeout,
  NULL,
  NULL
};

static struct ls2k0300_wdt_priv_s g_wdt_priv =
{
  .lower = { .ops = &g_wdt_ops },
  .base = LS2K0300_WDT_BASE,
  .clock_freq = LS2K0300_WDT_CLOCK_FREQ,
  .timeout = 30000,
  .max_hw_heatbeat_ms = UINT32_MAX / LS2K0300_WDT_CLOCK_FREQ * 1000,
  .started = false,
};

int ls2k0300_wdt_initialize(void)
{
  FAR struct watchdog_lowerhalf_s *lower = &g_wdt_priv.lower;
  FAR void *handle;
  int ret;

  wdt_putreg(&g_wdt_priv, LS2K0300_WDT_RST_CTR, 0);

  ret = ls2k0300_wdt_settimeout(lower, g_wdt_priv.timeout);
  if (ret < 0)
    {
      return ret;
    }

  handle = watchdog_register("/dev/watchdog0", lower);
  if (handle == NULL)
    {
      return -EIO;
    }

  return OK;
}

#endif
