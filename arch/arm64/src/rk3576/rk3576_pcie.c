/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_pcie.c
 *
 * PCIe controller driver for RK3576
 * Supports 2 PCIe lanes (Gen2 x1)
 ***************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "arm64_internal.h"
#include "hardware/rk3576_memorymap.h"
#include "rk3576_pcie.h"

#ifdef CONFIG_RK3576_PCIE

#define RK3576_PCIE0_ADDR       0xf8000000
#define RK3576_PCIE1_ADDR       0xf9000000

/* PCIe register offsets */

#define PCIE_RC_VENDOR          0x0040
#define PCIE_RC_STATUS          0x0044
#define PCIE_RC_DLINK           0x0050
#define PCIE_ATU_CTRL           0x0900
#define PCIE_ATU_VIEWPORT      0x0900
#define PCIE_ATU_OUT_CTRL1      0x0904
#define PCIE_ATU_OUT_CTRL2      0x0908
#define PCIE_ATU_OUT_LOWER      0x090c
#define PCIE_ATU_OUT_UPPER      0x0910
#define PCIE_ATU_IN_LIMIT       0x914
#define PCIE_ATU_IN_TARGET_LOW  0x918
#define PCIE_ATU_IN_TARGET_HIGH 0x91c

/* RC STATUS bits */

#define PCIE_RC_STATUS_LINK_UP  (1 << 0)

/* ATU Control bits */

#define PCIE_ATU_ENABLE         (1 << 0)
#define PCIE_ATU_TYPE_CONFIG    (0 << 1)
#define PCIE_ATU_TYPE_IO        (2 << 1)
#define PCIE_ATU_TYPE_MEM       (0 << 1)

struct rk3576_pcie_s
{
  bool enabled;
  bool link_up;
};

static struct rk3576_pcie_s g_pcie[2];

int rk3576_pcie_init(int pcie)
{
  if (pcie < 0 || pcie > 1)
    {
      return -EINVAL;
    }

  memset(&g_pcie[pcie], 0, sizeof(g_pcie[pcie]));

  ginfo("PCIe%d: initialized\n", pcie);
  return OK;
}

int rk3576_pcie_enable(int pcie)
{
  uint32_t base;

  if (pcie < 0 || pcie > 1)
    {
      return -EINVAL;
    }

  base = (pcie == 0) ? RK3576_PCIE0_ADDR : RK3576_PCIE1_ADDR;

  /* Enable link */

  uint32_t dlink = getreg32(base + PCIE_RC_DLINK);
  dlink |= (1 << 0);  /* Start link training */
  putreg32(dlink, base + PCIE_RC_DLINK);

  /* Wait for link up */

  int timeout = 100;
  while (timeout--)
    {
      if (getreg32(base + PCIE_RC_STATUS) & PCIE_RC_STATUS_LINK_UP)
        {
          g_pcie[pcie].link_up = true;
          break;
        }

      up_mdelay(10);
    }

  g_pcie[pcie].enabled = true;

  ginfo("PCIe%d: enabled (link=%s)\n", pcie,
        g_pcie[pcie].link_up ? "up" : "down");
  return OK;
}

int rk3576_pcie_disable(int pcie)
{
  if (pcie < 0 || pcie > 1)
    {
      return -EINVAL;
    }

  g_pcie[pcie].enabled = false;
  g_pcie[pcie].link_up = false;

  ginfo("PCIe%d: disabled\n", pcie);
  return OK;
}

bool rk3576_pcie_is_link_up(int pcie)
{
  if (pcie < 0 || pcie > 1)
    {
      return false;
    }

  return g_pcie[pcie].link_up;
}

uint32_t rk3576_pcie_read_config(int pcie, int bus, int dev, int func, int reg)
{
  uint32_t base;
  uint32_t addr;
  uint32_t viewport;

  if (pcie < 0 || pcie > 1)
    {
      return 0xffffffff;
    }

  base = (pcie == 0) ? RK3576_PCIE0_ADDR : RK3576_PCIE1_ADDR;

  /* Setup ATU for config space access */

  viewport = (bus << 24) | (dev << 19) | (func << 16) | (reg & 0xfc);
  putreg32(viewport, base + PCIE_ATU_VIEWPORT);
  putreg32(PCIE_ATU_ENABLE | PCIE_ATU_TYPE_CONFIG, base + PCIE_ATU_CTRL);

  /* Read from config space */

  uint32_t offset = (bus << 20) | (dev << 15) | (func << 12) | (reg & 0xfff);
  addr = base + 0x10000 + offset;

  return getreg32(addr);
}

void rk3576_pcie_write_config(int pcie, int bus, int dev, int func,
                                int reg, uint32_t val)
{
  uint32_t base;
  uint32_t viewport;
  uint32_t addr;

  if (pcie < 0 || pcie > 1)
    {
      return;
    }

  base = (pcie == 0) ? RK3576_PCIE0_ADDR : RK3576_PCIE1_ADDR;

  viewport = (bus << 24) | (dev << 19) | (func << 16) | (reg & 0xfc);
  putreg32(viewport, base + PCIE_ATU_VIEWPORT);
  putreg32(PCIE_ATU_ENABLE | PCIE_ATU_TYPE_CONFIG, base + PCIE_ATU_CTRL);

  uint32_t offset = (bus << 20) | (dev << 15) | (func << 12) | (reg & 0xfff);
  addr = base + 0x10000 + offset;

  putreg32(val, addr);
}

#endif /* CONFIG_RK3576_PCIE */
