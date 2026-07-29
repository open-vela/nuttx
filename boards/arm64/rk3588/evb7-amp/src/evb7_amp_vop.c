/****************************************************************************
 * boards/arm64/rk3588/evb7-amp/src/evb7_amp_vop.c
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
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* Scanning out of this core's framebuffer by writing the VOP's Esmart3 window
 * directly, while Linux keeps owning everything else about the display.
 *
 * Why this is possible at all, and where the boundary is
 * ----------------------------------------------------------------------------
 * Lighting this panel is about 13000 lines of Linux: a DSI D-PHY PLL, the MIPI
 * host controller, the panel's own initialisation sequence over DCS, the video
 * port timing generator and four clock trees feeding it. None of that is worth
 * reimplementing here, and none of it needs to be: it is all per-video-port
 * state, set once at modeset, and it stays set.
 *
 * What is per-window is small: where the pixels are, how big they are, what
 * format they are in, and a commit bit. Those are the registers below.
 *
 * The dts marks Esmart3 as vp3's `rockchip,reserved-plane`. On RK3588 that
 * property does exactly one thing that matters (rockchip_drm_vop2.c:1961): it
 * switches Linux's commits on vp3 from the per-port cfg_done register to the
 * per-window one, and the bitmask it writes there only ever contains windows
 * Linux itself touched this commit (rockchip_drm_vop2.c:2307). So register
 * writes made from this core sit pending in the VOP's shadow registers until
 * this core commits them, instead of being latched by whatever frame Linux
 * happens to be pushing. That is the whole mechanism.
 *
 * It does *not* do the other things the same property does on RK3576: the
 * reserved-plane layer routing at rockchip_drm_vop2.c:12580 lives in
 * vop3_setup_layer_sel_for_vp(), and RK3588 is not VOP3 (is_vop3() returns
 * false for it at rockchip_drm_vop2.c:1260), so RK3588 goes through
 * vop2_setup_layer_mixer_for_vp() which has no notion of a reserved plane.
 *
 * That turns out not to matter, because Esmart3 is still listed in vp3's
 * plane-mask, and the pieces that would otherwise be missing all fall out of
 * that:
 *
 *   - vop2_layer_map_initial() walks win_mask (== plane_mask) at probe and
 *     assigns layers in order. vp0..vp2 take two windows each, so vp3 gets
 *     layers 6 and 7, and Esmart3 - being the higher phys_id - lands on layer 7,
 *     the topmost. It also sets win_vp_id[Esmart3] = 3 there.
 *   - vop2_calc_bg_ovl_and_port_mux() sizes each port's slice of the mixer
 *     chain from hweight32(win_mask), not from how many windows Linux is
 *     actually using, and the last video port is hardcoded to take everything
 *     remaining. So layer 7 belongs to vp3's blend chain.
 *   - vop2_setup_alpha()'s tail loop runs to hweight32(vp->win_mask) and
 *     configures the mixer for layer 7 even though Linux has no plane there.
 *     The values it writes are, for a source with no alpha channel, the same
 *     ones it would write for a real XRGB8888 plane - Cd = Cs + (1-As)*Cd with
 *     As pinned to the global 0xff, i.e. an opaque overwrite.
 *
 * None of those three are re-derived on every commit in a way that would clobber
 * us, so the mixer, the layer routing and the port mux are left alone here.
 *
 * Why every register is rewritten on every flip
 * ----------------------------------------------------------------------------
 * Because Esmart3 stays in win_mask, vop2_disable_all_planes_for_crtc() walks
 * it and will switch the window off - it does not skip reserved planes. That
 * only happens on a modeset or a CRTC shutdown, so it cannot happen mid-stream,
 * but it can happen after this core has already taken over (a second run of the
 * host-side program, say). Rewriting ten device registers costs a handful of
 * nanoseconds against a frame time measured in tens of milliseconds, so the
 * window is simply reprogrammed each time rather than tracking whether Linux
 * has interfered.
 *
 * Addresses are physical
 * ----------------------------------------------------------------------------
 * The window's address register is fed straight to the VOP's AXI master, so it
 * has to be a physical address - there is no IOMMU in the path. That is why the
 * dts disables vop_mmu: with the IOMMU present Linux hands the hardware IOVAs,
 * and the first IOVA it allocates is zero, which is what
 * /sys/kernel/debug/dri/0/summary showed before the change. With it gone,
 * rockchip_gem_create_object() forces contiguous CMA allocations and programs
 * dma_handle (rockchip_drm_gem.c:455), so both sides now speak the same
 * addresses. This core's mapping is flat, so a pointer into the shared area is
 * already the physical address the VOP needs.
 *
 * The caller is responsible for the frame having reached DRAM before the
 * address is committed. The framebuffer this drives lives in ordinary cacheable
 * RAM (see evb7_amp_fb.c for why), so that means a cache clean; this file does
 * not do it, because it does not know which rows were touched and cleaning the
 * whole buffer on every flip would undo the point of tracking damage.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>
#include <unistd.h>

#include "arm64_arch.h"

#include "evb7_amp.h"
#include "evb7_amp_shm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VOP2_BASE               0xfdd90000ul

/* RK3588_SYS_WIN_REG_CFG_DONE (rockchip_vop_reg.h:1067). Declared as
 * VOP_REG_MASK(..., 0xffffffff, 0), and VOP_REG_MASK means write_mask, so
 * vop2_mask_write() turns a value v into (v & mask) | (mask << 16) - with a
 * 32-bit mask the upper half is 0xffff0000 (rockchip_drm_vop2.c:1224). The
 * low half selects which windows to latch; Esmart3's reg_done_bit is 7
 * (rockchip_vop2_reg.c:4732).
 */

#define VOP2_WIN_CFG_DONE       (VOP2_BASE + 0x00c)
#define ESMART3_REG_DONE_BIT    7
#define VOP2_CFG_DONE_VAL       (0xffff0000u | (1u << ESMART3_REG_DONE_BIT))

/* Esmart3's register block base is 0x600 (rockchip_vop2_reg.c:4715); the
 * offsets below are the RK3568_ESMART0_* ones from rockchip_vop_reg.h, which
 * every Esmart window shares.
 */

#define ESMART3_BASE            (VOP2_BASE + 0x600)

#define ESMART_CTRL0            (ESMART3_BASE + 0x1800)
#define ESMART_CTRL1            (ESMART3_BASE + 0x1804)
#define ESMART_AXI_CTRL         (ESMART3_BASE + 0x1808)
#define ESMART_R0_CTRL          (ESMART3_BASE + 0x1810)
#define ESMART_R0_YRGB_MST      (ESMART3_BASE + 0x1814)
#define ESMART_R0_VIR           (ESMART3_BASE + 0x181c)
#define ESMART_R0_ACT_INFO      (ESMART3_BASE + 0x1820)
#define ESMART_R0_DSP_INFO      (ESMART3_BASE + 0x1824)
#define ESMART_R0_DSP_ST        (ESMART3_BASE + 0x1828)
#define ESMART_R0_SCL_CTRL      (ESMART3_BASE + 0x1830)
#define ESMART_R0_SCL_FAC_YRGB  (ESMART3_BASE + 0x1834)

/* Field positions, all from rk3568_esmart_win_data / rk3568_esmart_win_scl
 * (rockchip_vop2_reg.c:2833 and :3099).
 */

#define CTRL0_Y2R_EN            (1u << 0)
#define CTRL0_R2Y_EN            (1u << 1)
#define CTRL0_CSC_MODE_MASK     (3u << 2)
#define CTRL0_CSC_13BIT_EN      (1u << 16)
#define CTRL0_CSC_Y2R_PATH_SEL  (1u << 24)

#define CTRL0_CSC_MASK          (CTRL0_Y2R_EN | CTRL0_R2Y_EN | \
                                 CTRL0_CSC_MODE_MASK | CTRL0_CSC_13BIT_EN | \
                                 CTRL0_CSC_Y2R_PATH_SEL)

#define CTRL1_AXI_YRGB_ID_MASK  (0x1fu << 4)
#define CTRL1_AXI_YRGB_ID(v)    (((v) & 0x1fu) << 4)
#define CTRL1_AXI_UV_ID_MASK    (0x1fu << 12)
#define CTRL1_AXI_UV_ID(v)      (((v) & 0x1fu) << 12)
#define CTRL1_YMIRROR           (1u << 31)

#define AXI_CTRL_AXI_ID_MASK    (1u << 1)
#define AXI_CTRL_AXI_ID(v)      (((v) & 1u) << 1)

#define R0_CTRL_ENABLE          (1u << 0)
#define R0_CTRL_FORMAT_MASK     (0x1fu << 1)
#define R0_CTRL_FORMAT(v)       (((v) & 0x1fu) << 1)
#define R0_CTRL_ARGB1555        (1u << 7)
#define R0_CTRL_VSD_GT2         (1u << 8)
#define R0_CTRL_VSD_GT4         (1u << 9)
#define R0_CTRL_RB_SWAP         (1u << 14)
#define R0_CTRL_UV_SWAP         (1u << 16)
#define R0_CTRL_RG_SWAP         (1u << 18)

#define VIR_YRGB_MASK           0xffffu

/* VOP2_FMT_ARGB8888 == 0 (rockchip_drm_vop2.c:174), which is what
 * vop2_convert_format() returns for both XRGB8888 and ARGB8888. rb_swap stays
 * clear for those two (vop2_win_rb_swap(), rockchip_drm_vop2.c:2586).
 */

#define VOP2_FMT_ARGB8888       0

/* SCL_CTRL, low byte: hor mode / hor filter / ver mode / ver filter, two bits
 * each. SCALE_UP is 1 (enum scale_mode). Esmart3's scale-up filters are BIC
 * horizontally and BIL vertically (rockchip_vop2_reg.c:4717), which is where
 * the hardware earns its keep over the nearest-neighbour scaling the host side
 * was doing in software.
 */

#define SCALE_UP                1
#define SCALE_UP_BIL            1
#define SCALE_UP_BIC            2

#define SCL_CTRL_YRGB_MASK      0xffu
#define SCL_CTRL_YRGB_UP        (SCALE_UP | (SCALE_UP_BIC << 2) | \
                                 (SCALE_UP << 4) | (SCALE_UP_BIL << 6))

/* The panel Linux has already brought up on vp3. Not read back from anywhere:
 * the mode is fixed by the dts panel node, and a mismatch shows up
 * immediately as a wrongly scaled picture rather than as silent corruption.
 */

#define PANEL_WIDTH             1080
#define PANEL_HEIGHT            1920

/* Two frames at 60Hz, in 1ms steps. See evb7_amp_vop_wait_latch(). */

#define VOP2_LATCH_POLL_US      1000
#define VOP2_LATCH_POLLS        40

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_taken_over;
static unsigned long g_flips;
static unsigned long g_latch_timeouts;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: amp_vop_scale_factor
 *
 * Description:
 *   The scale-up factor the VOP wants, which is (src-1)/(dst-1) in 16.16 fixed
 *   point, rounded so that the last output pixel still reads inside the source.
 *
 *   Copied from vop2_scale_factor() (rockchip_drm_vop2.c:3116) including the
 *   decrement loop: integer division can leave the factor one LSB too large,
 *   and stepping it down until the check passes is what the driver does rather
 *   than reasoning about the rounding. Only the scale-up form is needed here -
 *   this core's framebuffer is always smaller than the panel - and the caller
 *   asserts that by construction.
 *
 ****************************************************************************/

static uint16_t amp_vop_scale_factor(uint32_t src, uint32_t dst)
{
  uint32_t fac;
  int i;

  if (src == dst)
    {
      return 0;
    }

  if (dst < 2)
    {
      dst = 2;
    }

  fac = ((src - 1) << 16) / (dst - 1);

  for (i = 0; i < 100; i++)
    {
      if ((uint32_t)(((uint64_t)fac * (dst - 1)) >> 16) < src - 1)
        {
          break;
        }

      fac--;
    }

  return (uint16_t)fac;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: evb7_amp_vop_flip
 *
 * Description:
 *   Point Esmart3 at a frame and commit it.
 *
 *   Every register is written every time, for the reason in the file comment:
 *   Linux can disable this window behind our back on a modeset, and rewriting
 *   the lot is cheaper than detecting that.
 *
 *   The commit is a single write to the per-window cfg_done register. Nothing
 *   here waits for it - the VOP latches shadow registers at the next vertical
 *   blank on its own, and blocking a caller for up to one frame to observe
 *   that would cost more than the tearing it prevents. There is one buffer, so
 *   tearing is not avoidable by waiting anyway.
 *
 * Input Parameters:
 *   phys - physical address of the frame, which for this core's flat mapping
 *          is just the pointer.
 *
 ****************************************************************************/

void evb7_amp_vop_flip(uintptr_t phys)
{
  uint32_t fac_x = amp_vop_scale_factor(AMP_SHM_WIDTH, PANEL_WIDTH);
  uint32_t fac_y = amp_vop_scale_factor(AMP_SHM_HEIGHT, PANEL_HEIGHT);

  /* Colour space conversion off: an RGB source going to an RGB output. Linux
   * never touches Esmart3's CTRL0 once it stops using the window, so this is
   * the only thing that keeps it correct.
   */

  modreg32(0, CTRL0_CSC_MASK, ESMART_CTRL0);

  /* AXI read IDs. Linux sets these from rk3588_vop2_win_cfg_axi(), but only
   * from inside the plane update path (rockchip_drm_vop2.c:7139) - so for a
   * window Linux no longer updates, nobody sets them but us. Esmart3 sits on
   * axi1 with yrgb/uv ids 0x0c/0x0d (rockchip_vop2_reg.c:4728).
   */

  modreg32(CTRL1_AXI_YRGB_ID(0x0c) | CTRL1_AXI_UV_ID(0x0d),
           CTRL1_AXI_YRGB_ID_MASK | CTRL1_AXI_UV_ID_MASK | CTRL1_YMIRROR,
           ESMART_CTRL1);

  modreg32(AXI_CTRL_AXI_ID(1), AXI_CTRL_AXI_ID_MASK, ESMART_AXI_CTRL);

  /* Geometry. act_info is the source rectangle, dsp_info the destination, both
   * as (height-1)<<16 | (width-1), and dsp_st the destination origin -
   * unlike the older VOP there is no back porch to add here
   * (rockchip_drm_vop2.c:7103..7108). vir is the source stride in 4-byte
   * words.
   */

  putreg32(((AMP_SHM_HEIGHT - 1) << 16) | ((AMP_SHM_WIDTH - 1) & 0xffff),
           ESMART_R0_ACT_INFO);
  putreg32(((PANEL_HEIGHT - 1) << 16) | ((PANEL_WIDTH - 1) & 0xffff),
           ESMART_R0_DSP_INFO);
  putreg32(0, ESMART_R0_DSP_ST);

  modreg32(AMP_SHM_STRIDE / 4, VIR_YRGB_MASK, ESMART_R0_VIR);

  /* Hardware scaling from this core's buffer up to the panel. */

  modreg32(SCL_CTRL_YRGB_UP, SCL_CTRL_YRGB_MASK, ESMART_R0_SCL_CTRL);
  putreg32((fac_y << 16) | (fac_x & 0xffff), ESMART_R0_SCL_FAC_YRGB);

  /* Where the pixels are. */

  putreg32((uint32_t)phys, ESMART_R0_YRGB_MST);

  /* Format and enable last, so the window never goes live against a
   * half-written configuration. The swaps are cleared explicitly rather than
   * assumed: nothing else writes this register now, and a stale rb_swap from
   * whatever ran before would show up as blue-and-red-reversed output.
   */

  modreg32(R0_CTRL_ENABLE | R0_CTRL_FORMAT(VOP2_FMT_ARGB8888),
           R0_CTRL_ENABLE | R0_CTRL_FORMAT_MASK | R0_CTRL_ARGB1555 |
           R0_CTRL_VSD_GT2 | R0_CTRL_VSD_GT4 | R0_CTRL_RB_SWAP |
           R0_CTRL_UV_SWAP | R0_CTRL_RG_SWAP,
           ESMART_R0_CTRL);

  putreg32(VOP2_CFG_DONE_VAL, VOP2_WIN_CFG_DONE);

  g_flips++;
}

/****************************************************************************
 * Name: evb7_amp_vop_wait_latch
 *
 * Description:
 *   Wait until the VOP is actually scanning out the given frame.
 *
 *   This is what makes double buffering worth having. Writing the address and
 *   the commit bit only queues the change; the hardware promotes it at the next
 *   vertical blank. Until that happens the previous buffer is still being read,
 *   so an application that starts drawing into it straight away is drawing into
 *   the frame on screen - the tearing double buffering was supposed to remove.
 *
 *   There is no vsync interrupt to wait on: the VOP's interrupt belongs to
 *   Linux and is shared with the IOMMU's, so it cannot be handed to this core
 *   while Linux owns the other three video ports. What is available is the
 *   address register itself. A read returns the live copy, so it reads back as
 *   the new address only once the promotion has happened - which makes it a
 *   direct, if polled, vsync.
 *
 *   The timeout matters more than the polling interval. If Linux has taken the
 *   video port down, nothing will ever latch, and a driver that waits forever
 *   for that turns a blank screen into a hung application. Two frames is long
 *   enough that a healthy pipeline never reaches it.
 *
 * Input Parameters:
 *   phys - the address passed to the matching evb7_amp_vop_flip().
 *
 * Returned Value:
 *   true if the frame is on screen, false if it timed out - in which case the
 *   caller should carry on rather than retry, because the likely cause is that
 *   there is nothing scanning at all.
 *
 ****************************************************************************/

bool evb7_amp_vop_wait_latch(uintptr_t phys)
{
  int i;

  for (i = 0; i < VOP2_LATCH_POLLS; i++)
    {
      if (getreg32(ESMART_R0_YRGB_MST) == (uint32_t)phys)
        {
          return true;
        }

      usleep(VOP2_LATCH_POLL_US);
    }

  g_latch_timeouts++;
  return false;
}

/****************************************************************************
 * Name: evb7_amp_vop_takeover
 *
 * Description:
 *   Start driving Esmart3 from this core, and report enough of the state to
 *   tell a working takeover from a silent one.
 *
 *   Deliberately not called from bringup. Linux's modeset on vp3 happens when
 *   the host-side program opens the card, which is well after this core has
 *   booted, and a modeset disables every window in vp3's mask - including this
 *   one. So the takeover has to be requested after the panel is up, not
 *   before.
 *
 * Input Parameters:
 *   phys - physical address of the first frame to show.
 *
 * Returned Value:
 *   OK, or -EALREADY if the takeover has already happened.
 *
 ****************************************************************************/

int evb7_amp_vop_takeover(uintptr_t phys)
{
  if (g_taken_over)
    {
      return -EALREADY;
    }

  evb7_amp_vop_flip(phys);
  g_taken_over = true;

  syslog(LOG_INFO,
         "[AMP] vop: Esmart3 driven from here, %ux%u -> %ux%u, "
         "addr 0x%08lx, wrote scl 0x%08lx, cfg_done 0x%08lx\n",
         AMP_SHM_WIDTH, AMP_SHM_HEIGHT, PANEL_WIDTH, PANEL_HEIGHT,
         (unsigned long)phys,
         (unsigned long)((amp_vop_scale_factor(AMP_SHM_HEIGHT,
                                               PANEL_HEIGHT) << 16) |
                         amp_vop_scale_factor(AMP_SHM_WIDTH, PANEL_WIDTH)),
         (unsigned long)VOP2_CFG_DONE_VAL);

  /* Read the window back, but only after a frame has gone by.
   *
   * Writes to these registers land in the VOP's shadow copy and the hardware
   * promotes them to the live copy at the next vertical blank, which is what
   * the cfg_done above asked for. A read returns the *live* copy. So reading
   * immediately after writing returns the previous values, not the ones just
   * written - which on a window nobody has used yet means reset values, and
   * reset values read exactly like a write that never arrived.
   *
   * That is a genuinely misleading readback, so it is worth the wait: one
   * frame at 60Hz is 16.7ms, this runs once, and it turns the readback from
   * something that always looks broken into something that distinguishes
   * "the writes landed and were latched" from "the writes went nowhere".
   */

  usleep(30000);

  syslog(LOG_INFO,
         "[AMP] vop: live r0_ctrl 0x%08lx mst 0x%08lx vir 0x%08lx "
         "act 0x%08lx dsp 0x%08lx scl 0x%08lx\n",
         (unsigned long)getreg32(ESMART_R0_CTRL),
         (unsigned long)getreg32(ESMART_R0_YRGB_MST),
         (unsigned long)getreg32(ESMART_R0_VIR),
         (unsigned long)getreg32(ESMART_R0_ACT_INFO),
         (unsigned long)getreg32(ESMART_R0_DSP_INFO),
         (unsigned long)getreg32(ESMART_R0_SCL_FAC_YRGB));

  return OK;
}

/****************************************************************************
 * Name: evb7_amp_vop_active
 *
 * Description:
 *   Whether the framebuffer flush path should be driving the window instead of
 *   handing the frame to Linux.
 *
 ****************************************************************************/

bool evb7_amp_vop_active(void)
{
  return g_taken_over;
}

/****************************************************************************
 * Name: evb7_amp_vop_flips / evb7_amp_vop_latch_timeouts
 *
 * Description:
 *   Frames committed, and how many of them were never seen to reach the
 *   screen. The second number is the interesting one: a non-zero count means
 *   the pipeline is not actually synchronised to the panel, so double buffering
 *   is not protecting against tearing even though it looks like it should be.
 *
 ****************************************************************************/

unsigned long evb7_amp_vop_flips(void)
{
  return g_flips;
}

unsigned long evb7_amp_vop_latch_timeouts(void)
{
  return g_latch_timeouts;
}
