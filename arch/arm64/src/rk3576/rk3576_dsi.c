/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_dsi.c
 *
 * MIPI DSI Host controller driver for RK3576
 *
 * Supports:
 * - DSI video mode (DPI interface)
 * - DSI command mode (DCS read/write)
 * - DPHY configuration
 * - Panel initialization sequence
 *
 ***************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "arm64_internal.h"
#include "hardware/rk3576_dsi.h"
#include "rk3576_dsi.h"

#ifdef CONFIG_RK3576_DSI

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

#define DSI_DPHY_LANES_2           2
#define DSI_DPHY_LANES_4           4

const uint32_t g_dsi_base[RK3576_DSI_COUNT] =
{
  RK3576_DSI0_ADDR,
  RK3576_DSI1_ADDR,
};

/***************************************************************************
 * Private Types
 ***************************************************************************/

struct rk3576_dsi_s
{
  bool enabled;
  int lanes;
  int color_fmt;
  bool cmd_mode;
};

/***************************************************************************
 * Private Data
 ***************************************************************************/

static struct rk3576_dsi_s g_dsi[RK3576_DSI_COUNT];

/***************************************************************************
 * Private Functions
 ***************************************************************************/

static int rk3576_dsi_phy_wait_lock(uint32_t base)
{
  int timeout;

  for (timeout = 0; timeout < DSI_PHY_LOCK_TIMEOUT_MS; timeout++)
    {
      if (getreg32(base + DSI_PHY_STATUS) & DSI_PHY_STATUS_LOCK)
        {
          return OK;
        }

      up_mdelay(1);
    }

  _err("DSI: PHY lock timeout\n");
  return -ETIMEDOUT;
}

static int rk3576_dsi_wait_cmd_done(uint32_t base)
{
  int timeout;

  for (timeout = 0; timeout < DSI_TIMEOUT_MS; timeout++)
    {
      uint32_t status = getreg32(base + DSI_INT_ST0);

      if (status & DSI_INT_ST0_CMD_PKT_RCVD)
        {
          putreg32(DSI_INT_ST0_CMD_PKT_RCVD, base + DSI_INT_CLR0);
          return OK;
        }

      up_mdelay(1);
    }

  _err("DSI: command done timeout\n");
  return -ETIMEDOUT;
}

static void rk3576_dphy_test_write(uint32_t base, uint32_t addr, uint8_t data)
  __attribute__((unused));

static uint8_t rk3576_dphy_test_read(uint32_t base, uint32_t addr)
  __attribute__((unused));

static void rk3576_dphy_test_write(uint32_t base, uint32_t addr, uint8_t data)
{
  /* Set test mode: write */

  putreg32(addr | DSI_PHY_TST_WRITE_EN,
           base + DSI_PHY_TST_CTRL1);
  putreg32((1 << 1), base + DSI_PHY_TST_CTRL0);
  putreg32((0 << 1), base + DSI_PHY_TST_CTRL0);

  /* Write data */

  putreg32(data | DSI_PHY_TST_WRITE_EN,
           base + DSI_PHY_TST_CTRL1);
  putreg32((1 << 1), base + DSI_PHY_TST_CTRL0);
  putreg32((0 << 1), base + DSI_PHY_TST_CTRL0);
}

static uint8_t rk3576_dphy_test_read(uint32_t base, uint32_t addr)
{
  /* Set test mode: read */

  putreg32(addr | DSI_PHY_TST_WRITE_EN,
           base + DSI_PHY_TST_CTRL1);
  putreg32((1 << 1), base + DSI_PHY_TST_CTRL0);
  putreg32((0 << 1), base + DSI_PHY_TST_CTRL0);

  /* Read data */

  putreg32(0, base + DSI_PHY_TST_CTRL1);
  putreg32((1 << 0), base + DSI_PHY_TST_CTRL0);
  putreg32((0 << 0), base + DSI_PHY_TST_CTRL0);

  return (uint8_t)(getreg32(base + DSI_PHY_TST_CTRL1) & 0xff);
}

static void rk3576_dphy_init(uint32_t base, int lanes)
{
  /* Enable DPHY */

  putreg32(DSI_PHY_RSTZ_SHUTDOWNZ | DSI_PHY_RSTZ_RSTZ | DSI_PHY_RSTZ_ENABLE,
           base + DSI_PHY_RSTZ);

  /* Configure lane count */

  uint32_t clkm_cfg = getreg32(base + DSI_CLKM_CFG);
  clkm_cfg &= ~DSI_CLKM_CFG_LANE_EN_MASK;
  clkm_cfg |= ((lanes - 1) << DSI_CLKM_CFG_LANE_EN_SHIFT);
  putreg32(clkm_cfg, base + DSI_CLKM_CFG);

  /* Wait for PHY lock */

  rk3576_dsi_phy_wait_lock(base);
}

static void rk3576_dphy_set_timing(uint32_t base, int lanes)
{
  /* DPHY timing parameters for 1.5Gbps/lane */

  uint32_t tx_timing = (0x02 << DSI_DPHY_TX_TLPX_CTL_SHIFT) |
                       (0x0a << DSI_DPHY_TX_THS_ZERO_CTL_SHIFT) |
                       (0x08 << DSI_DPHY_TX_THS_TRAIL_CTL_SHIFT) |
                       (0x02 << DSI_DPHY_TX_THS_EXIT_CTL_SHIFT);

  uint32_t rx_timing = (0x02 << DSI_DPHY_RX_TLPX_CTL_SHIFT) |
                       (0x0a << DSI_DPHY_RX_THS_ZERO_CTL_SHIFT) |
                       (0x08 << DSI_DPHY_RX_THS_TRAIL_CTL_SHIFT) |
                       (0x02 << DSI_DPHY_RX_THS_EXIT_CTL_SHIFT);

  putreg32(tx_timing, base + DSI_DPHY_TX_TIMING);
  putreg32(rx_timing, base + DSI_DPHY_RX_TIMING);

  UNUSED(lanes);
}

/***************************************************************************
 * Public Functions
 ***************************************************************************/

int rk3576_dsi_init(int dsi)
{
  uint32_t base;

  if (dsi < 0 || dsi >= RK3576_DSI_COUNT)
    {
      return -EINVAL;
    }

  base = g_dsi_base[dsi];

  /* Power down DSI */

  putreg32(DSI_PWR_UP_SHUTDOWN, base + DSI_PWR_UP);

  /* Clear all interrupts */

  putreg32(0xffffffff, base + DSI_INT_CLR0);
  putreg32(0xffffffff, base + DSI_INT_CLR1);

  /* Disable all interrupts */

  putreg32(0, base + DSI_INT_MASK0);
  putreg32(0, base + DSI_INT_MASK1);

  /* Reset PHY */

  putreg32(0, base + DSI_PHY_RSTZ);

  /* Initialize state */

  g_dsi[dsi].enabled = false;
  g_dsi[dsi].lanes = DSI_DPHY_LANES_4;
  g_dsi[dsi].color_fmt = DSI_DPI_24BIT_888;
  g_dsi[dsi].cmd_mode = false;

  uinfo("DSI%d: initialized\n", dsi);
  return OK;
}

int rk3576_dsi_enable(int dsi)
{
  uint32_t base;

  if (dsi < 0 || dsi >= RK3576_DSI_COUNT)
    {
      return -EINVAL;
    }

  base = g_dsi_base[dsi];

  /* Initialize DPHY */

  rk3576_dphy_init(base, g_dsi[dsi].lanes);
  rk3576_dphy_set_timing(base, g_dsi[dsi].lanes);

  /* Configure color coding */

  putreg32(g_dsi[dsi].color_fmt, base + DSI_DPI_COLOR_CODING);

  /* Enable DWC config */

  putreg32(DSI_DWC_CONFIG_EN, base + DSI_DWC_CONFIG);

  /* Power up DSI */

  putreg32(DSI_PWR_UP_POWERUP, base + DSI_PWR_UP);

  g_dsi[dsi].enabled = true;

  uinfo("DSI%d: enabled (lanes=%d, color=%d)\n",
        dsi, g_dsi[dsi].lanes, g_dsi[dsi].color_fmt);
  return OK;
}

int rk3576_dsi_disable(int dsi)
{
  uint32_t base;

  if (dsi < 0 || dsi >= RK3576_DSI_COUNT)
    {
      return -EINVAL;
    }

  base = g_dsi_base[dsi];

  /* Power down */

  putreg32(DSI_PWR_UP_SHUTDOWN, base + DSI_PWR_UP);

  /* Disable PHY */

  putreg32(0, base + DSI_PHY_RSTZ);

  g_dsi[dsi].enabled = false;

  uinfo("DSI%d: disabled\n", dsi);
  return OK;
}

int rk3576_dsi_set_lanes(int dsi, int lanes)
{
  if (dsi < 0 || dsi >= RK3576_DSI_COUNT)
    {
      return -EINVAL;
    }

  if (lanes != DSI_DPHY_LANES_2 && lanes != DSI_DPHY_LANES_4)
    {
      return -EINVAL;
    }

  g_dsi[dsi].lanes = lanes;
  return OK;
}

int rk3576_dsi_set_color(int dsi, int color_fmt)
{
  if (dsi < 0 || dsi >= RK3576_DSI_COUNT)
    {
      return -EINVAL;
    }

  g_dsi[dsi].color_fmt = color_fmt;
  return OK;
}

int rk3576_dsi_set_cmd_mode(int dsi, bool enable)
{
  if (dsi < 0 || dsi >= RK3576_DSI_COUNT)
    {
      return -EINVAL;
    }

  g_dsi[dsi].cmd_mode = enable;

  uint32_t base = g_dsi_base[dsi];

  if (enable)
    {
      putreg32(DSI_DPI_CMD_MODE_EN, base + DSI_DPI_CMD_MODE);
    }
  else
    {
      putreg32(0, base + DSI_DPI_CMD_MODE);
    }

  return OK;
}

int rk3576_dsi_set_video_timing(int dsi, int hfp, int hbp, int hsa,
                                  int vfp, int vbp, int vsa, int vact)
{
  uint32_t base;

  if (dsi < 0 || dsi >= RK3576_DSI_COUNT)
    {
      return -EINVAL;
    }

  base = g_dsi_base[dsi];

  putreg32(hfp, base + DSI_DPI_HFP);
  putreg32(hbp, base + DSI_DPI_HBP);
  putreg32(hsa, base + DSI_DPI_HSA);
  putreg32(vfp, base + DSI_DPI_VFP);
  putreg32(vbp, base + DSI_DPI_VBP);
  putreg32(vsa, base + DSI_DPI_VSA);
  putreg32(vact, base + DSI_DPI_VACT);

  uinfo("DSI%d: video timing H=%d+%d+%d+%d V=%d+%d+%d+%d\n",
        dsi, hsa, hbp, hfp, (hsa + hbp + hfp),
        vsa, vbp, vfp, (vsa + vbp + vfp));

  return OK;
}

int rk3576_dsi_set_polarity(int dsi, bool hsync_pol, bool vsync_pol,
                              bool dataen_pol, bool dotclk_pol)
{
  uint32_t base;
  uint32_t pol = 0;

  if (dsi < 0 || dsi >= RK3576_DSI_COUNT)
    {
      return -EINVAL;
    }

  base = g_dsi_base[dsi];

  if (hsync_pol)
    {
      pol |= DSI_DPI_CFG_POL_HSYNC;
    }

  if (vsync_pol)
    {
      pol |= DSI_DPI_CFG_POL_VSYNC;
    }

  if (dataen_pol)
    {
      pol |= DSI_DPI_CFG_POL_DATAEN;
    }

  if (dotclk_pol)
    {
      pol |= DSI_DPI_CFG_POL_DOTCLK;
    }

  putreg32(pol, base + DSI_DPI_CFG_POL);
  return OK;
}

int rk3576_dsi_dcs_write(int dsi, uint8_t cmd, const uint8_t *params,
                           int len)
{
  uint32_t base;
  uint32_t hdr;
  int ret;

  if (dsi < 0 || dsi >= RK3576_DSI_COUNT)
    {
      return -EINVAL;
    }

  base = g_dsi_base[dsi];

  /* Build header word */

  if (len == 0)
    {
      /* DCS short write, no parameter */

      hdr = (DSI_DCS_SHORT_WRITE << 6) | (0 << 16) |
            (0 << 8) | cmd;
    }
  else if (len == 1)
    {
      /* DCS short write, 1 parameter */

      hdr = (DSI_DCS_SHORT_WRITE_PARAM << 6) | (1 << 16) |
            (params[0] << 8) | cmd;
    }
  else
    {
      /* DCS long write */

      hdr = (DSI_GEN_LONG_WRITE << 6) | ((len + 1) << 16) |
            (0 << 8) | cmd;
    }

  /* Send payload bytes for long write */

  if (len > 1)
    {
      for (int i = 0; i < len; i++)
        {
          putreg32(params[i], base + DSI_SINGLE_CMD_TX_PLD);
        }
    }

  /* Send header */

  putreg32(hdr, base + DSI_SINGLE_CMD_TX_HDR);

  /* Wait for command done */

  ret = rk3576_dsi_wait_cmd_done(base);
  return ret;
}

int rk3576_dcs_read(int dsi, uint8_t cmd, uint8_t *data, int len)
{
  uint32_t base;
  uint32_t hdr;
  int ret;

  if (dsi < 0 || dsi >= RK3576_DSI_COUNT || data == NULL || len < 1)
    {
      return -EINVAL;
    }

  base = g_dsi_base[dsi];

  /* Build read header */

  hdr = (DSI_DCS_READ << 6) | (len << 16) | (0 << 8) | cmd;
  putreg32(hdr, base + DSI_SINGLE_CMD_RX_HDR);

  /* Wait for response */

  ret = rk3576_dsi_wait_cmd_done(base);
  if (ret < 0)
    {
      return ret;
    }

  /* Read payload */

  for (int i = 0; i < len; i++)
    {
      data[i] = (uint8_t)(getreg32(base + DSI_SINGLE_CMD_RX_PLD) & 0xff);
    }

  return OK;
}

int rk3576_dsi_panel_init(int dsi, const struct rk3576_dsi_panel_s *panel)
{
  int ret;

  if (dsi < 0 || dsi >= RK3576_DSI_COUNT || panel == NULL)
    {
      return -EINVAL;
    }

  /* Set lanes and color format from panel info */

  ret = rk3576_dsi_set_lanes(dsi, panel->lanes);
  if (ret < 0)
    {
      return ret;
    }

  ret = rk3576_dsi_set_color(dsi, panel->color_fmt);
  if (ret < 0)
    {
      return ret;
    }

  /* Set video timing */

  ret = rk3576_dsi_set_video_timing(dsi,
           panel->hfp, panel->hbp, panel->hsa,
           panel->vfp, panel->vbp, panel->vsa,
           panel->vact);
  if (ret < 0)
    {
      return ret;
    }

  /* Set signal polarity */

  ret = rk3576_dsi_set_polarity(dsi,
           panel->hsync_pol, panel->vsync_pol,
           panel->dataen_pol, panel->dotclk_pol);
  if (ret < 0)
    {
      return ret;
    }

  /* Send panel init sequence */

  if (panel->init_cmds != NULL && panel->init_cmd_count > 0)
    {
      for (int i = 0; i < panel->init_cmd_count; i++)
        {
          const struct rk3576_dsi_cmd_s *cmd = &panel->init_cmds[i];

          if (cmd->delay_ms > 0)
            {
              up_mdelay(cmd->delay_ms);
            }

          ret = rk3576_dsi_dcs_write(dsi, cmd->cmd,
                                       cmd->params, cmd->param_len);
          if (ret < 0)
            {
              _err("DSI%d: panel init cmd %d failed: %d\n", dsi, i, ret);
              return ret;
            }
        }
    }

  ginfo("DSI%d: panel initialized (%dx%d, %d lanes)\n",
        dsi, panel->hact, panel->vact, panel->lanes);
  return OK;
}

#endif /* CONFIG_RK3576_DSI */
