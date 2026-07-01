/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_dsi.h
 *
 * MIPI DSI Host controller register definitions for RK3576
 * Based on Synopsys DesignWare DSI Host (DWC_mipi_dsi)
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_DSI_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_DSI_H

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/* DSI controller count */

#define RK3576_DSI_COUNT           2

/* DSI register offsets (DWC_mipi_dsi) */

#define DSI_VERSION                0x0000
#define DSI_PWR_UP                 0x0004
#define DSI_CLKM_CFG              0x0008
#define DSI_DPHY_IF_CFG            0x000c
#define DSI_DPHY_TX_TIMING         0x0010
#define DSI_DPHY_RX_TIMING         0x0014
#define DSI_CLK_STATUS             0x0018
#define DSI_INT_ST0                0x001c
#define DSI_INT_ST1                0x0020
#define DSI_INT_MASK0              0x0024
#define DSI_INT_MASK1              0x0028
#define DSI_INT_CLR0               0x002c
#define DSI_INT_CLR1               0x0030
#define DSI_FIFO_RD                0x0034
#define DSI_FIFO_PD                0x0038
#define DSI_LP_CMD_TX              0x003c
#define DSI_LP_CMD_RX              0x0040
#define DSI_SINGLE_CMD_TX          0x0044
#define DSI_SINGLE_CMD_TX_HDR      0x0044
#define DSI_SINGLE_CMD_TX_PLD      0x0048
#define DSI_SINGLE_CMD_RX          0x004c
#define DSI_SINGLE_CMD_RX_HDR      0x004c
#define DSI_SINGLE_CMD_RX_PLD      0x0050
#define DSI_DWC_CONFIG             0x0054
#define DSI_DPI_CFG                0x0058
#define DSI_DPI_VID                0x005c
#define DSI_DPI_VCID               0x0060
#define DSI_DPI_COLOR_CODING       0x0064
#define DSI_DPI_CFG_POL            0x0068
#define DSI_DPI_HFP                0x006c
#define DSI_DPI_HBP                0x0070
#define DSI_DPI_HSA                0x0074
#define DSI_DPI_VFP                0x0078
#define DSI_DPI_VBP                0x007c
#define DSI_DPI_VSA                0x0080
#define DSI_DPI_VACT               0x0084
#define DSI_DPI_CMD_MODE           0x0088
#define DSI_PHY_RSTZ               0x00a0
#define DSI_PHY_TX_TRIGGER         0x00a4
#define DSI_PHY_STATUS             0x00b0
#define DSI_PHY_TST_CFG0           0x00b4
#define DSI_PHY_TST_CFG1           0x00b8
#define DSI_PHY_TST_CFG2           0x00bc

/* DSI_PWR_UP bits */

#define DSI_PWR_UP_SHUTDOWN        (0 << 0)
#define DSI_PWR_UP_POWERUP         (1 << 0)

/* DSI_CLKM_CFG bits */

#define DSI_CLKM_CFG_LANE_EN_SHIFT 0
#define DSI_CLKM_CFG_LANE_EN_MASK  (0xf << 0)
#define DSI_CLKM_CFG_DIV_SHIFT     8
#define DSI_CLKM_CFG_DIV_MASK      (0xff << 8)

/* DSI_DPHY_IF_CFG bits */

#define DSI_DPHY_IF_CFG_LPRX_T_SEL_SHIFT  0
#define DSI_DPHY_IF_CFG_HSTX_T_SEL_SHIFT  16
#define DSI_DPHY_IF_CFG_RX_ULPS_SHIFT     24
#define DSI_DPHY_IF_CFG_RX_ULPS_MASK      (0xf << 24)

/* DSI_DPHY_TX_TIMING bits */

#define DSI_DPHY_TX_TLPX_CTL_SHIFT  0
#define DSI_DPHY_TX_THS_ZERO_CTL_SHIFT 8
#define DSI_DPHY_TX_THS_TRAIL_CTL_SHIFT 16
#define DSI_DPHY_TX_THS_EXIT_CTL_SHIFT 24

/* DSI_DPHY_RX_TIMING bits */

#define DSI_DPHY_RX_TLPX_CTL_SHIFT  0
#define DSI_DPHY_RX_THS_ZERO_CTL_SHIFT 8
#define DSI_DPHY_RX_THS_TRAIL_CTL_SHIFT 16
#define DSI_DPHY_RX_THS_EXIT_CTL_SHIFT 24

/* DSI_CLK_STATUS bits */

#define DSI_CLK_STATUS_PHY_LOCK    (1 << 0)
#define DSI_CLK_STATUS_CLK_RDY     (1 << 1)

/* DSI_INT_ST0 bits */

#define DSI_INT_ST0_DPHY_START     (1 << 0)
#define DSI_INT_ST0_DPHY_END       (1 << 1)
#define DSI_INT_ST0_CMD_PKT_RCVD   (1 << 6)
#define DSI_INT_ST0_PLD_RCV        (1 << 8)
#define DSI_INT_ST0_PLD_SND        (1 << 9)
#define DSI_INT_ST0_CRC_ERR        (1 << 10)
#define DSI_INT_ST0_EOT_ERR        (1 << 11)
#define DSI_INT_ST0_LP_CMD_ERR     (1 << 12)
#define DSI_INT_ST0_PHY_PH_ERR     (1 << 13)

/* DSI_INT_ST1 bits */

#define DSI_INT_ST1_CMD_DONE       (1 << 0)
#define DSI_INT_ST1_CMD_Q_EMPTY    (1 << 1)
#define DSI_INT_ST1_TX_FIFO_FULL   (1 << 2)

/* DSI_LP_CMD_TX bits */

#define DSI_LP_CMD_TX_EN           (1 << 0)
#define DSI_LP_CMD_TX_RESET        (1 << 1)
#define DSI_LP_CMD_TX_OVERFLOW     (1 << 2)

/* DSI_SINGLE_CMD_TX bits */

#define DSI_SINGLE_CMD_TX_EN       (1 << 0)
#define DSI_SINGLE_CMD_TX_START    (1 << 1)
#define DSI_SINGLE_CMD_TX_DONE     (1 << 2)
#define DSI_SINGLE_CMD_TX_ERROR    (1 << 3)

/* DSI_DWC_CONFIG bits */

#define DSI_DWC_CONFIG_EN          (1 << 0)

/* DSI_DPI_CFG bits */

#define DSI_DPI_CFG_COLORMODE_SHIFT 0
#define DSI_DPI_CFG_COLORMODE_MASK  (0xf << 0)
#define DSI_DPI_CFG_SHUTDWN        (1 << 4)

/* DSI_DPI_COLOR_CODING bits */

#define DSI_DPI_16BIT_565          0x00
#define DSI_DPI_16BIT_565_TRIM     0x01
#define DSI_DPI_18BIT_666          0x02
#define DSI_DPI_18BIT_666_PACKED   0x03
#define DSI_DPI_24BIT_888          0x04

/* DSI_DPI_CFG_POL bits */

#define DSI_DPI_CFG_POL_HSYNC      (1 << 0)
#define DSI_DPI_CFG_POL_VSYNC      (1 << 1)
#define DSI_DPI_CFG_POL_DATAEN     (1 << 2)
#define DSI_DPI_CFG_POL_DOTCLK     (1 << 3)

/* DSI_DPI_CMD_MODE bits */

#define DSI_DPI_CMD_MODE_EN        (1 << 0)
#define DSI_DPI_CMD_MODE_TRIGGER   (1 << 1)

/* DSI_PHY_RSTZ bits */

#define DSI_PHY_RSTZ_SHUTDOWNZ     (1 << 0)
#define DSI_PHY_RSTZ_RSTZ          (1 << 1)
#define DSI_PHY_RSTZ_ENABLE        (1 << 2)

/* DSI_PHY_STATUS bits */

#define DSI_PHY_STATUS_LOCK        (1 << 0)
#define DSI_PHY_STATUS_DIRECTION   (1 << 1)
#define DSI_PHY_STATUS_RXSTOP      (1 << 2)
#define DSI_PHY_STATUS_RXULPSESC   (1 << 3)
#define DSI_PHY_STATUS_RXULPSCLK   (1 << 4)

/* DSI_PHY_TST_CFG0 bits */

#define DSI_PHY_TST_CFG0_CLRZ      (1 << 0)
#define DSI_PHY_TST_CFG0_EN        (1 << 1)
#define DSI_PHY_TST_CFG0_WDATA_SHIFT 2

/* DSI PHY test interface write sequence */

#define DSI_PHY_TST_CTRL0          DSI_PHY_TST_CFG0
#define DSI_PHY_TST_CTRL1          DSI_PHY_TST_CFG1
#define DSI_PHY_TST_WRITE_EN       (1 << 16)
#define DSI_PHY_TST_WRITE_DATA_SHIFT 0

/* DSI PHY register addresses for test interface */

#define DSI_PHY_TEST_CTRL0         0x10010
#define DSI_PHY_TEST_CTRL1         0x10011
#define DSI_PHY_TX_TRIGGERS        0x1001f

/* DSI LP command types */

#define DSI_DCS_SHORT_WRITE        0x05
#define DSI_DCS_SHORT_WRITE_PARAM  0x15
#define DSI_DCS_READ               0x06
#define DSI_GEN_SHORT_WRITE        0x03
#define DSI_GEN_SHORT_WRITE_PARAM  0x13
#define DSI_GEN_LONG_WRITE         0x29
#define DSI_GEN_LONG_READ          0x14

/* DCS command definitions */

#define DSI_DCS_NOP                0x00
#define DSI_DCS_SOFT_RESET         0x01
#define DSI_DCS_GET_DISPLAY_MODE   0x0a
#define DSI_DCS_GET_POWER_MODE     0x0b
#define DSI_DCS_SET_DISPLAY_ON     0x29
#define DSI_DCS_SET_DISPLAY_OFF    0x28
#define DSI_DCS_SET_COLUMN_ADDR    0x2a
#define DSI_DCS_SET_PAGE_ADDR      0x2b
#define DSI_DCS_WRITE_MEMORY_START 0x2c
#define DSI_DCS_SET_TEAR_ON        0x35
#define DSI_DCS_SET_ADDRESS_MODE   0x36
#define DSI_DCS_SET_PIXEL_FORMAT   0x3a
#define DSI_DCS_WRITE_CTRL_LUT     0xb1
#define DSI_DCS_SET_TEAR_SCANLINE  0x44

/* Timeouts */

#define DSI_TIMEOUT_MS             100
#define DSI_PHY_LOCK_TIMEOUT_MS    10

#ifndef __ASSEMBLY__

extern const uint32_t g_dsi_base[RK3576_DSI_COUNT];

#endif /* __ASSEMBLY__ */

#endif
