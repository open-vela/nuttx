/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_mipi_dsi_regonly.c
 *
 * Pure register-based MIPI-DSI driver for ESP32-P4 (EK79007 panel).
 * No HAL dependencies - all operations via volatile register access.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>
#include <syslog.h>
#include <nuttx/kmalloc.h>

#include "esp_mipi_dsi.h"

/****************************************************************************
 * Pre-processor Definitions - Base Addresses
 ****************************************************************************/

/* HP Periph base addresses */
#define HPPERIPH0_BASE            0x50000000
#define HPPERIPH1_BASE            0x500C0000
#define LPAON_BASE                0x50110000

/* DSI Host controller */
#define DSI_HOST_BASE             (HPPERIPH0_BASE + 0xA0000)  /* 0x500A0000 */
/* DSI Bridge */
#define DSI_BRG_BASE              (HPPERIPH0_BASE + 0xA0800)  /* 0x500A0800 */
/* HP_SYS_CLKRST */
#define HP_SYS_CLKRST_BASE       (HPPERIPH1_BASE + 0x26000)  /* 0x500E6000 */
/* PMU */
#define PMU_BASE                  (LPAON_BASE + 0x5000)       /* 0x50115000 */
/* GPIO */
#define GPIO_BASE                 (HPPERIPH1_BASE + 0x20000)  /* 0x500E0000 */
/* IO MUX */
#define IO_MUX_BASE               (HPPERIPH1_BASE + 0x21000)  /* 0x500E1000 */

/****************************************************************************
 * Pre-processor Definitions - HP_SYS_CLKRST Registers
 ****************************************************************************/

/* SOC_CLK_CTRL1: offset 0x18 from HP_SYS_CLKRST_BASE */
#define CLKRST_SOC_CLK_CTRL1     (HP_SYS_CLKRST_BASE + 0x18)
#define CLKRST_SOC_CLK_CTRL1_DSI_SYS_CLK_EN  (1 << 12)

/* PERI_CLK_CTRL02: offset 0x38 - contains DPHY_CLK_SRC_SEL [31:30] */
#define CLKRST_PERI_CLK_CTRL02   (HP_SYS_CLKRST_BASE + 0x38)
#define CLKRST_DPHY_CLK_SRC_SEL_S  30
#define CLKRST_DPHY_CLK_SRC_SEL_M  (0x3 << 30)

/* PERI_CLK_CTRL03: offset 0x3c */
#define CLKRST_PERI_CLK_CTRL03   (HP_SYS_CLKRST_BASE + 0x3c)
#define CLKRST_DPHY_CFG_CLK_EN   (1 << 0)
#define CLKRST_DPHY_PLL_REFCLK_EN (1 << 1)
#define CLKRST_DPICLK_SRC_SEL_S  5
#define CLKRST_DPICLK_SRC_SEL_M  (0x3 << 5)
#define CLKRST_DPICLK_EN         (1 << 7)
#define CLKRST_DPICLK_DIV_NUM_S  8
#define CLKRST_DPICLK_DIV_NUM_M  (0xFF << 8)

/* HP_RST_EN1: offset 0xc4 */
#define CLKRST_HP_RST_EN1        (HP_SYS_CLKRST_BASE + 0xc4)
#define CLKRST_RST_EN_DSI_BRG    (1 << 26)

/****************************************************************************
 * Pre-processor Definitions - DSI Host Registers
 ****************************************************************************/

#define HOST_VERSION              (DSI_HOST_BASE + 0x00)
#define HOST_PWR_UP               (DSI_HOST_BASE + 0x04)
#define HOST_CLKMGR_CFG           (DSI_HOST_BASE + 0x08)
#define HOST_DPI_VCID             (DSI_HOST_BASE + 0x0C)
#define HOST_DPI_COLOR_CODING     (DSI_HOST_BASE + 0x10)
#define HOST_DPI_CFG_POL          (DSI_HOST_BASE + 0x14)
#define HOST_DPI_LP_CMD_TIM       (DSI_HOST_BASE + 0x18)
#define HOST_PCKHDL_CFG           (DSI_HOST_BASE + 0x2C)
#define HOST_GEN_VCID             (DSI_HOST_BASE + 0x30)
#define HOST_MODE_CFG             (DSI_HOST_BASE + 0x34)
#define HOST_VID_MODE_CFG         (DSI_HOST_BASE + 0x38)
#define HOST_VID_PKT_SIZE         (DSI_HOST_BASE + 0x3C)
#define HOST_VID_NUM_CHUNKS       (DSI_HOST_BASE + 0x40)
#define HOST_VID_NULL_SIZE        (DSI_HOST_BASE + 0x44)
#define HOST_VID_HSA_TIME         (DSI_HOST_BASE + 0x48)
#define HOST_VID_HBP_TIME         (DSI_HOST_BASE + 0x4C)
#define HOST_VID_HLINE_TIME       (DSI_HOST_BASE + 0x50)
#define HOST_VID_VSA_LINES        (DSI_HOST_BASE + 0x54)
#define HOST_VID_VBP_LINES        (DSI_HOST_BASE + 0x58)
#define HOST_VID_VFP_LINES        (DSI_HOST_BASE + 0x5C)
#define HOST_VID_VACTIVE_LINES    (DSI_HOST_BASE + 0x60)
#define HOST_CMD_MODE_CFG         (DSI_HOST_BASE + 0x68)
#define HOST_GEN_HDR              (DSI_HOST_BASE + 0x6C)
#define HOST_GEN_PLD_DATA         (DSI_HOST_BASE + 0x70)
#define HOST_CMD_PKT_STATUS       (DSI_HOST_BASE + 0x74)
#define HOST_TO_CNT_CFG           (DSI_HOST_BASE + 0x78)
#define HOST_HS_RD_TO_CNT         (DSI_HOST_BASE + 0x7C)
#define HOST_LP_RD_TO_CNT         (DSI_HOST_BASE + 0x80)
#define HOST_HS_WR_TO_CNT         (DSI_HOST_BASE + 0x84)
#define HOST_LP_WR_TO_CNT         (DSI_HOST_BASE + 0x88)
#define HOST_BTA_TO_CNT           (DSI_HOST_BASE + 0x8C)
#define HOST_LPCLK_CTRL           (DSI_HOST_BASE + 0x94)
#define HOST_PHY_TMR_LPCLK_CFG   (DSI_HOST_BASE + 0x98)
#define HOST_PHY_TMR_CFG          (DSI_HOST_BASE + 0x9C)
#define HOST_PHY_RSTZ             (DSI_HOST_BASE + 0xA0)
#define HOST_PHY_IF_CFG           (DSI_HOST_BASE + 0xA4)
#define HOST_PHY_STATUS           (DSI_HOST_BASE + 0xB0)
#define HOST_PHY_TST_CTRL0        (DSI_HOST_BASE + 0xB4)
#define HOST_PHY_TST_CTRL1        (DSI_HOST_BASE + 0xB8)

/* PHY_RSTZ bits */
#define PHY_SHUTDOWNZ             (1 << 0)
#define PHY_RSTZ                  (1 << 1)
#define PHY_ENABLECLK             (1 << 2)
#define PHY_FORCEPLL              (1 << 3)

/* PHY_IF_CFG bits */
#define PHY_N_LANES_S             0
#define PHY_N_LANES_M             0x03
#define PHY_STOP_WAIT_TIME_S      8
#define PHY_STOP_WAIT_TIME_M      (0xFF << 8)

/* PHY_STATUS bits */
#define PHY_LOCK                  (1 << 0)
#define PHY_STOPSTATECLKLANE      (1 << 2)
#define PHY_STOPSTATE0LANE        (1 << 4)
#define PHY_STOPSTATE1LANE        (1 << 7)

/* PHY_TST_CTRL0 bits */
#define PHY_TESTCLR               (1 << 0)
#define PHY_TESTCLK               (1 << 1)

/* PHY_TST_CTRL1 bits */
#define PHY_TESTDIN_S             0
#define PHY_TESTDIN_M             0xFF
#define PHY_TESTDOUT_S            8
#define PHY_TESTEN                (1 << 16)

/* CMD_PKT_STATUS bits */
#define GEN_CMD_FULL              (1 << 1)
#define GEN_PLD_W_FULL            (1 << 3)

/* MODE_CFG bits */
#define MODE_CFG_CMD_MODE         0   /* command mode */
#define MODE_CFG_VID_MODE         1   /* video mode */

/* VID_MODE_CFG bits */
#define VID_MODE_TYPE_BURST       0x02  /* burst mode with sync pulses */
#define LP_VSA_EN                 (1 << 8)
#define LP_VBP_EN                 (1 << 9)
#define LP_VFP_EN                 (1 << 10)
#define LP_VACT_EN                (1 << 11)
#define LP_HBP_EN                 (1 << 12)
#define LP_HFP_EN                 (1 << 13)
#define FRAME_BTA_ACK_EN          (1 << 14)
#define LP_CMD_EN                 (1 << 15)

/* LPCLK_CTRL bits */
#define PHY_TXREQUESTCLKHS        (1 << 0)
#define AUTO_CLKLANE_CTRL         (1 << 1)

/* PCKHDL_CFG bits */
#define BTA_EN                    (1 << 2)
#define CRC_RX_EN                 (1 << 4)
#define ECC_RX_EN                 (1 << 5)
#define EOTP_TX_EN                (1 << 6)

/* DPI color coding values (from DW DSI Host spec) */
#define COLOR_CODE_RGB565         0x0
#define COLOR_CODE_RGB666         0x3
#define COLOR_CODE_RGB888         0x5

/****************************************************************************
 * Pre-processor Definitions - DSI Bridge Registers
 ****************************************************************************/

#define BRG_CLK_EN                (DSI_BRG_BASE + 0x00)
#define BRG_EN                    (DSI_BRG_BASE + 0x04)
#define BRG_DMA_REQ_CFG           (DSI_BRG_BASE + 0x08)
#define BRG_RAW_NUM_CFG           (DSI_BRG_BASE + 0x0C)
#define BRG_RAW_BUF_CREDIT_CTL   (DSI_BRG_BASE + 0x10)
#define BRG_PIXEL_TYPE            (DSI_BRG_BASE + 0x18)
#define BRG_DMA_BLOCK_INTERVAL    (DSI_BRG_BASE + 0x1C)
#define BRG_DMA_REQ_INTERVAL      (DSI_BRG_BASE + 0x20)
#define BRG_DPI_LCD_CTL           (DSI_BRG_BASE + 0x24)
#define BRG_DPI_V_CFG0            (DSI_BRG_BASE + 0x30)
#define BRG_DPI_V_CFG1            (DSI_BRG_BASE + 0x34)
#define BRG_DPI_H_CFG0            (DSI_BRG_BASE + 0x38)
#define BRG_DPI_H_CFG1            (DSI_BRG_BASE + 0x3C)
#define BRG_DPI_MISC_CONFIG       (DSI_BRG_BASE + 0x40)
#define BRG_DPI_CONFIG_UPDATE     (DSI_BRG_BASE + 0x44)
#define BRG_INT_ENA               (DSI_BRG_BASE + 0x50)
#define BRG_INT_CLR               (DSI_BRG_BASE + 0x54)
#define BRG_BLK_RAW_NUM_CFG      (DSI_BRG_BASE + 0x68)
#define BRG_DMA_FRAME_INTERVAL    (DSI_BRG_BASE + 0x6C)
#define BRG_HOST_CTRL             (DSI_BRG_BASE + 0x80)

/****************************************************************************
 * Pre-processor Definitions - PMU/LDO Registers
 ****************************************************************************/

/* VO3 LDO: PMU_EXT_LDO_P0_0P2A (index 1 in ext_ldo array)
 * From pmu_reg.h: offset 0x1c0 for control, 0x1c4 for analog
 */
#define PMU_EXT_LDO_VO3           (PMU_BASE + 0x1c0)
#define PMU_EXT_LDO_VO3_ANA       (PMU_BASE + 0x1c4)

/* PMU_EXT_LDO_P0_0P2A_REG bit positions (from pmu_reg.h) */
#define LDO_FORCE_TIEH_SEL        (1 << 7)
#define LDO_XPD                   (1 << 8)
#define LDO_TIEH_SEL_S            9
#define LDO_TIEH_SEL_M            (0x7 << 9)
#define LDO_TIEH_POS_EN           (1 << 12)
#define LDO_TIEH_NEG_EN           (1 << 13)
#define LDO_TIEH                  (1 << 14)

/* PMU_EXT_LDO_P0_0P2A_ANA_REG bit positions */
#define LDO_ANA_MUL_S             23
#define LDO_ANA_MUL_M             (0x7 << 23)
#define LDO_ANA_DREF_S            28
#define LDO_ANA_DREF_M            (0xF << 28)

/****************************************************************************
 * Pre-processor Definitions - GPIO Registers
 ****************************************************************************/

/* GPIO OUT register (for GPIO < 32) */
#define GPIO_OUT_W1TS             (GPIO_BASE + 0x08)
#define GPIO_OUT_W1TC             (GPIO_BASE + 0x0C)
#define GPIO_ENABLE_W1TS          (GPIO_BASE + 0x24)

/* GPIO function select (IO_MUX): each pin has a 4-byte register */
#define IO_MUX_GPIOn(n)           (IO_MUX_BASE + 0x04 + (n) * 4)

/****************************************************************************
 * Pre-processor Definitions - Panel Parameters
 ****************************************************************************/

#define PANEL_RESET_GPIO          27
#define BACKLIGHT_GPIO            26

/* PHY PLL config for 1000 Mbps from 40 MHz XTAL:
 * f_vco = M/N * f_ref = 50/2 * 40 = 1000 MHz
 */
#define PHY_PLL_N                 2
#define PHY_PLL_M                 50

/* DPI clock: 240 MHz / 5 = 48 MHz */
#define DPI_CLK_DIV               5

/* Lane byte clock = lane_bit_rate / 8 = 125 MHz
 * dpi2lane ratio = 125 / 48 ≈ 2.6
 * For timing conversion: multiply DPI pixel counts by this ratio
 */
#define LANE_BYTE_CLK_MHZ        125
#define DPI_TO_LANE_RATIO_X10    26  /* 2.6 * 10 */

/* Escape clock divider: lane_byte_clk / esc_clk ≈ 125/7 ≈ 18 */
#define ESC_CLK_DIV               18
/* Timeout clock divider: lane_byte_clk / to_clk ≈ 125/12.5 = 10 */
#define TO_CLK_DIV                13

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t *g_framebuffer = NULL;

/****************************************************************************
 * Private Functions - Register Access
 ****************************************************************************/

static inline void reg_write(uint32_t addr, uint32_t val)
{
  *(volatile uint32_t *)addr = val;
}

static inline uint32_t reg_read(uint32_t addr)
{
  return *(volatile uint32_t *)addr;
}

static inline void reg_set_bits(uint32_t addr, uint32_t bits)
{
  uint32_t val = reg_read(addr);
  val |= bits;
  reg_write(addr, val);
}

static inline void reg_clr_bits(uint32_t addr, uint32_t bits)
{
  uint32_t val = reg_read(addr);
  val &= ~bits;
  reg_write(addr, val);
}

static inline void reg_set_field(uint32_t addr, uint32_t mask,
                                 uint32_t shift, uint32_t val)
{
  uint32_t reg = reg_read(addr);
  reg &= ~mask;
  reg |= (val << shift) & mask;
  reg_write(addr, reg);
}

static void delay_us(uint32_t us)
{
  volatile uint32_t count = us * 40;  /* rough delay at ~400MHz */
  while (count--)
    {
      __asm__ volatile("nop");
    }
}

static void delay_ms(uint32_t ms)
{
  while (ms--)
    {
      delay_us(1000);
    }
}

/****************************************************************************
 * Private Functions - GPIO
 ****************************************************************************/

static void gpio_set_output(uint32_t gpio_num)
{
  /* Enable output */
  reg_write(GPIO_ENABLE_W1TS, 1 << gpio_num);

  /* Configure IO MUX: function 1 (GPIO), output enable */
  uint32_t mux_reg = IO_MUX_GPIOn(gpio_num);
  uint32_t val = reg_read(mux_reg);
  val &= ~(0x7 << 12);  /* Clear func_sel [14:12] */
  val |= (1 << 12);     /* func_sel = 1 (GPIO) */
  reg_write(mux_reg, val);
}

static void gpio_set_high(uint32_t gpio_num)
{
  reg_write(GPIO_OUT_W1TS, 1 << gpio_num);
}

static void gpio_set_low(uint32_t gpio_num)
{
  reg_write(GPIO_OUT_W1TC, 1 << gpio_num);
}

/****************************************************************************
 * Private Functions - PHY Test Interface
 ****************************************************************************/

static void phy_write_reg(uint8_t addr, uint8_t data)
{
  /* Step 1: Set test code address */
  reg_write(HOST_PHY_TST_CTRL0, 0);                   /* testclk=0, testclr=0 */
  reg_write(HOST_PHY_TST_CTRL1, PHY_TESTEN | addr);   /* testen=1, testdin=addr */
  reg_write(HOST_PHY_TST_CTRL0, PHY_TESTCLK);         /* testclk=1 (rising edge) */
  reg_write(HOST_PHY_TST_CTRL0, 0);                   /* testclk=0 */

  /* Step 2: Write data */
  reg_write(HOST_PHY_TST_CTRL1, data);                /* testen=0, testdin=data */
  reg_write(HOST_PHY_TST_CTRL0, PHY_TESTCLK);         /* testclk=1 (latch) */
  reg_write(HOST_PHY_TST_CTRL0, 0);                   /* testclk=0 */
}

/****************************************************************************
 * Private Functions - DCS Command
 ****************************************************************************/

static void dsi_dcs_write_short(uint8_t cmd, uint8_t param, bool has_param)
{
  uint32_t timeout = 10000;

  /* Wait for command FIFO not full */
  while ((reg_read(HOST_CMD_PKT_STATUS) & GEN_CMD_FULL) && timeout--)
    {
      delay_us(1);
    }

  if (has_param)
    {
      /* DCS short write with 1 param: data_type = 0x15 */
      reg_write(HOST_GEN_HDR, (param << 16) | (cmd << 8) | 0x15);
    }
  else
    {
      /* DCS short write no param: data_type = 0x05 */
      reg_write(HOST_GEN_HDR, (cmd << 8) | 0x05);
    }

  delay_us(100);
}

/****************************************************************************
 * Private Functions - Initialization Phases
 ****************************************************************************/

static void phase0_ldo_enable(void)
{
  syslog(LOG_INFO, "[DSI] Phase 0: LDO enable\n");

  /* For VO3 (PMU_EXT_LDO_P0_0P2A):
   * Set force_tieh_sel=1 (software control)
   * Set tieh=0 (use Vref*mul, not 3.3V rail)
   * Set xpd=1 (enable LDO)
   * Set tieh_pos_en=1 (power on delay)
   *
   * ANA register: dref=10, mul=5 for ~2.5V
   * (default dref=10 per pmu_reg.h)
   */

  uint32_t ldo_reg = reg_read(PMU_EXT_LDO_VO3);
  ldo_reg |= LDO_FORCE_TIEH_SEL;   /* bit 7: software control */
  ldo_reg &= ~LDO_TIEH_SEL_M;      /* clear tieh_sel */
  ldo_reg &= ~LDO_TIEH;            /* bit 14: use Vref*mul, not 3.3V */
  ldo_reg |= LDO_TIEH_POS_EN;      /* bit 12: power on delay */
  ldo_reg |= LDO_XPD;              /* bit 8: enable */
  reg_write(PMU_EXT_LDO_VO3, ldo_reg);

  /* Set analog parameters: dref=10 [31:28], mul=5 [25:23] for ~2.5V */
  uint32_t ana_reg = reg_read(PMU_EXT_LDO_VO3_ANA);
  ana_reg &= ~LDO_ANA_DREF_M;
  ana_reg |= (10 << LDO_ANA_DREF_S);
  ana_reg &= ~LDO_ANA_MUL_M;
  ana_reg |= (5 << LDO_ANA_MUL_S);
  reg_write(PMU_EXT_LDO_VO3_ANA, ana_reg);

  delay_ms(5);  /* Wait for LDO to stabilize */
}

static void phase1_clock_enable(void)
{
  syslog(LOG_INFO, "[DSI] Phase 1: Clock enable\n");

  /* 1. Enable DSI system bus clock (SOC_CLK_CTRL1 bit 12) */
  reg_set_bits(CLKRST_SOC_CLK_CTRL1, CLKRST_SOC_CLK_CTRL1_DSI_SYS_CLK_EN);

  /* 2. Reset DSI Bridge (assert then deassert) */
  reg_set_bits(CLKRST_HP_RST_EN1, CLKRST_RST_EN_DSI_BRG);
  delay_us(10);
  reg_clr_bits(CLKRST_HP_RST_EN1, CLKRST_RST_EN_DSI_BRG);

  /* 3. Set PHY DPHY clock source to XTAL (value 0) in PERI_CLK_CTRL02 [31:30] */
  reg_clr_bits(CLKRST_PERI_CLK_CTRL02, CLKRST_DPHY_CLK_SRC_SEL_M);

  /* 4. Enable PHY config clock + PLL reference clock in PERI_CLK_CTRL03 */
  reg_set_bits(CLKRST_PERI_CLK_CTRL03,
               CLKRST_DPHY_CFG_CLK_EN | CLKRST_DPHY_PLL_REFCLK_EN);

  /* 5. Configure DPI clock: source = PLL_F240M (value 0), div = 5-1 = 4 */
  uint32_t ctrl03 = reg_read(CLKRST_PERI_CLK_CTRL03);
  ctrl03 &= ~CLKRST_DPICLK_SRC_SEL_M;      /* source = 0 (PLL_F240M) */
  ctrl03 &= ~CLKRST_DPICLK_DIV_NUM_M;
  ctrl03 |= ((DPI_CLK_DIV - 1) << CLKRST_DPICLK_DIV_NUM_S);
  ctrl03 |= CLKRST_DPICLK_EN;
  reg_write(CLKRST_PERI_CLK_CTRL03, ctrl03);

  delay_us(100);
}

static void phase2_phy_init(void)
{
  syslog(LOG_INFO, "[DSI] Phase 2: PHY init\n");

  /* 1. Power down host first */
  reg_write(HOST_PWR_UP, 0);

  /* 2. Configure number of lanes (N_LANES = num_lanes - 1 = 1) */
  uint32_t if_cfg = (ESP_DSI_NUM_DATA_LANES - 1) & PHY_N_LANES_M;
  if_cfg |= (0x3F << PHY_STOP_WAIT_TIME_S);  /* stop wait time */
  reg_write(HOST_PHY_IF_CFG, if_cfg);

  /* 3. PHY reset sequence: clear all first */
  reg_write(HOST_PHY_RSTZ, 0);
  delay_us(10);

  /* 4. Clear PHY test interface */
  reg_write(HOST_PHY_TST_CTRL0, PHY_TESTCLR);
  delay_us(10);
  reg_write(HOST_PHY_TST_CTRL0, 0);
  delay_us(10);

  /* 5. Configure PHY PLL via test interface
   * For 1000 Mbps:
   *   hs_freq_sel for 1000 Mbps range: 0x49 (per DW DPHY databook)
   *   reg[0x44] = hs_freq_sel << 1
   *   reg[0x19] = 0x30 (use N/M values)
   *   reg[0x17] = N - 1 = 1
   *   reg[0x18] = (M-1) & 0x1F = 49 & 0x1F = 17
   *   then  reg[0x18] = 0x80 | ((M-1)>>5 & 0x0F) = 0x80 | 1 = 0x81
   */
  phy_write_reg(0x44, 0x49 << 1);  /* HS frequency range: 1000 Mbps */
  phy_write_reg(0x19, 0x30);       /* Use PLL N/M configuration */
  phy_write_reg(0x17, PHY_PLL_N - 1);               /* N-1 = 1 */
  phy_write_reg(0x18, (PHY_PLL_M - 1) & 0x1F);      /* M low bits = 17 */
  phy_write_reg(0x18, 0x80 | (((PHY_PLL_M - 1) >> 5) & 0x0F)); /* M high */

  /* 6. PHY power up sequence */
  reg_write(HOST_PHY_RSTZ, PHY_SHUTDOWNZ);
  delay_us(10);
  reg_write(HOST_PHY_RSTZ, PHY_SHUTDOWNZ | PHY_RSTZ);
  delay_us(10);
  reg_write(HOST_PHY_RSTZ, PHY_SHUTDOWNZ | PHY_RSTZ | PHY_ENABLECLK);
  delay_us(10);
  reg_write(HOST_PHY_RSTZ, PHY_SHUTDOWNZ | PHY_RSTZ |
            PHY_ENABLECLK | PHY_FORCEPLL);

  /* 7. Wait for PLL lock (with timeout) */
  uint32_t timeout = 5000;
  while (!(reg_read(HOST_PHY_STATUS) & PHY_LOCK) && timeout--)
    {
      delay_us(1);
    }

  if (timeout == 0)
    {
      syslog(LOG_ERR, "[DSI] ERROR: PHY PLL lock timeout!\n");
    }
  else
    {
      syslog(LOG_INFO, "[DSI] PHY PLL locked\n");
    }

  /* 8. Wait for lanes to enter stop state */
  timeout = 100000;
  uint32_t stop_mask = PHY_STOPSTATECLKLANE | PHY_STOPSTATE0LANE;
  if (ESP_DSI_NUM_DATA_LANES > 1)
    {
      stop_mask |= PHY_STOPSTATE1LANE;
    }

  while ((reg_read(HOST_PHY_STATUS) & stop_mask) != stop_mask && timeout--)
    {
      delay_us(1);
    }

  if (timeout == 0)
    {
      syslog(LOG_WARNING, "[DSI] Warning: lanes stop state timeout "
             "(status=0x%08lx)\n", (unsigned long)reg_read(HOST_PHY_STATUS));
    }

  /* 9. Power up host */
  reg_write(HOST_PWR_UP, 1);
}

static void phase3_host_config(void)
{
  syslog(LOG_INFO, "[DSI] Phase 3: Host config (command mode)\n");

  /* 1. Enter command mode */
  reg_write(HOST_MODE_CFG, MODE_CFG_CMD_MODE);

  /* 2. Set clock dividers */
  uint32_t clkmgr = (ESC_CLK_DIV & 0xFF) | ((TO_CLK_DIV & 0xFF) << 8);
  reg_write(HOST_CLKMGR_CFG, clkmgr);

  /* 3. Set LP clock control: auto clock lane */
  reg_write(HOST_LPCLK_CTRL, AUTO_CLKLANE_CTRL);

  /* 4. Set timeouts to 0 (disabled) */
  reg_write(HOST_TO_CNT_CFG, 0);
  reg_write(HOST_HS_RD_TO_CNT, 0);
  reg_write(HOST_LP_RD_TO_CNT, 0);
  reg_write(HOST_HS_WR_TO_CNT, 0);
  reg_write(HOST_LP_WR_TO_CNT, 0);
  reg_write(HOST_BTA_TO_CNT, 0);

  /* 5. PHY timing config:
   * LP clock period: data_hs2lp=50, data_lp2hs=104
   * phy_tmr_lpclk_cfg: clk_hs2lp=46, clk_lp2hs=128
   */
  reg_write(HOST_PHY_TMR_CFG, (50 & 0x3FF) | ((104 & 0x3FF) << 16));
  reg_write(HOST_PHY_TMR_LPCLK_CFG, (46 & 0x3FF) | ((128 & 0x3FF) << 16));

  /* 6. Enable CRC, ECC, EOTP */
  reg_write(HOST_PCKHDL_CFG, BTA_EN | CRC_RX_EN | ECC_RX_EN | EOTP_TX_EN);

  /* 7. Set DPI virtual channel to 0 */
  reg_write(HOST_DPI_VCID, 0);

  /* 8. Set generic interface virtual channel to 0 */
  reg_write(HOST_GEN_VCID, 0);
}

static void phase3_5_panel_reset(void)
{
  syslog(LOG_INFO, "[DSI] Phase 3.5: Panel reset (GPIO %d)\n",
         PANEL_RESET_GPIO);

  gpio_set_output(PANEL_RESET_GPIO);

  /* Reset pulse: low -> high -> low -> high */
  gpio_set_high(PANEL_RESET_GPIO);
  delay_ms(10);
  gpio_set_low(PANEL_RESET_GPIO);
  delay_ms(10);
  gpio_set_high(PANEL_RESET_GPIO);
  delay_ms(120);  /* Wait for panel to come out of reset */
}

static void phase4_panel_init(void)
{
  syslog(LOG_INFO, "[DSI] Phase 4: Panel init commands (EK79007)\n");

  /* EK79007 initialization sequence via DCS:
   * 1. Configure 2 data lanes: cmd 0xB2, param 0x10
   * 2. Vendor specific registers
   * 3. Sleep Out command
   */

  dsi_dcs_write_short(0xB2, 0x10, true);  /* 2-lane mode */
  dsi_dcs_write_short(0x80, 0x8B, true);
  dsi_dcs_write_short(0x81, 0x78, true);
  dsi_dcs_write_short(0x82, 0x84, true);
  dsi_dcs_write_short(0x83, 0x88, true);
  dsi_dcs_write_short(0x84, 0xA8, true);
  dsi_dcs_write_short(0x85, 0xE3, true);
  dsi_dcs_write_short(0x86, 0x88, true);

  /* Sleep Out */
  dsi_dcs_write_short(0x11, 0x00, false);
  delay_ms(120);  /* Required delay after sleep out */

  /* Display On */
  dsi_dcs_write_short(0x29, 0x00, false);
  delay_ms(20);

  syslog(LOG_INFO, "[DSI] Panel init commands sent\n");
}

static void phase5_dpi_config(void)
{
  syslog(LOG_INFO, "[DSI] Phase 5: DPI config\n");

  /* Convert horizontal timings from DPI pixel clock to lane byte clock:
   * ratio = lane_byte_clk / dpi_clk = 125/48 ≈ 2.6
   * For integer math: multiply by 26 then divide by 10
   */
  uint32_t hsw_bytes = (ESP_DSI_HSYNC * DPI_TO_LANE_RATIO_X10 + 5) / 10;
  uint32_t hbp_bytes = (ESP_DSI_HBP * DPI_TO_LANE_RATIO_X10 + 5) / 10;
  uint32_t hline_bytes = ((ESP_DSI_HSYNC + ESP_DSI_HBP + ESP_DSI_HRES +
                           ESP_DSI_HFP) * DPI_TO_LANE_RATIO_X10 + 5) / 10;

  /* Set color coding: RGB565 = 0x0 */
  reg_write(HOST_DPI_COLOR_CODING, COLOR_CODE_RGB565);

  /* Set polarity: all active high (write 0) */
  reg_write(HOST_DPI_CFG_POL, 0);

  /* Video mode configuration */
  uint32_t vid_mode_cfg = VID_MODE_TYPE_BURST;
  vid_mode_cfg |= LP_VSA_EN | LP_VBP_EN | LP_VFP_EN | LP_VACT_EN;
  vid_mode_cfg |= LP_HBP_EN | LP_HFP_EN;
  vid_mode_cfg |= FRAME_BTA_ACK_EN | LP_CMD_EN;
  reg_write(HOST_VID_MODE_CFG, vid_mode_cfg);

  /* Set packet size = horizontal resolution */
  reg_write(HOST_VID_PKT_SIZE, ESP_DSI_HRES);
  reg_write(HOST_VID_NUM_CHUNKS, 0);
  reg_write(HOST_VID_NULL_SIZE, 0);

  /* Horizontal timing in lane byte clock cycles */
  reg_write(HOST_VID_HSA_TIME, hsw_bytes);
  reg_write(HOST_VID_HBP_TIME, hbp_bytes);
  reg_write(HOST_VID_HLINE_TIME, hline_bytes);

  /* Vertical timing in lines (no clock domain conversion) */
  reg_write(HOST_VID_VSA_LINES, ESP_DSI_VSYNC);
  reg_write(HOST_VID_VBP_LINES, ESP_DSI_VBP);
  reg_write(HOST_VID_VFP_LINES, ESP_DSI_VFP);
  reg_write(HOST_VID_VACTIVE_LINES, ESP_DSI_VRES);

  syslog(LOG_INFO, "[DSI] DPI timing: HSA=%lu HBP=%lu HLINE=%lu\n",
         (unsigned long)hsw_bytes, (unsigned long)hbp_bytes,
         (unsigned long)hline_bytes);
}

static void phase5_bridge_config(void)
{
  syslog(LOG_INFO, "[DSI] Phase 5b: Bridge config\n");

  /* 1. Enable bridge clock */
  reg_write(BRG_CLK_EN, 1);

  /* 2. Set pixel type for RGB565 input + RGB565 output:
   * raw_type[3:0]=2 (RGB565), data_in_type[6]=0, dpi_type[10:7]=2 (RGB565)
   */
  reg_write(BRG_PIXEL_TYPE, (2 << 0) | (2 << 7));

  /* 3. Set total raw pixel bits (in 64-bit words) and trigger reload
   * total_pixel_bits = H * V * BPP = 1024 * 600 * 16 = 9,830,400
   * raw_num_total = total_pixel_bits / 64 = 153,600
   */
  uint32_t total_pixel_bits = (uint32_t)ESP_DSI_HRES * ESP_DSI_VRES *
                              ESP_DSI_FB_BPP;
  uint32_t raw_num_total = (total_pixel_bits + 63) / 64;
  uint32_t unalign = (total_pixel_bits % 64) ? 1 : 0;
  reg_write(BRG_RAW_NUM_CFG, (raw_num_total & 0x3FFFFF) |
            (unalign << 22) | (1u << 31));

  /* 4. Set underrun discard count (in dpi_misc_config, bits[15:4]) */
  uint32_t misc = reg_read(BRG_DPI_MISC_CONFIG);
  misc &= ~(0xFFF << 4);
  misc |= ((ESP_DSI_HRES & 0xFFF) << 4);
  reg_write(BRG_DPI_MISC_CONFIG, misc);

  /* 5. DMA configuration: burst_len=256 */
  reg_write(BRG_DMA_REQ_CFG, 256);

  /* 6. Set block interval: enable interval, auto reload raw_num_total */
  reg_write(BRG_DMA_BLOCK_INTERVAL, (9 << 0) |     /* block_slot = 9 */
            (9 << 10) |                              /* block_interval = 9 */
            (1 << 28) |                              /* raw_num_total_auto_reload */
            (1 << 29));                              /* block_interval_en */
  reg_write(BRG_DMA_REQ_INTERVAL, 1);

  /* 7. DPI LCD control: clear (dpishutdn=0, dpicolorm=0) */
  reg_write(BRG_DPI_LCD_CTL, 0);

  /* 8. Vertical timing in bridge (per hw_ver3 struct):
   *    DPI_V_CFG0: vtotal[11:0] | vdisp[27:16]
   *    DPI_V_CFG1: vbank(=vbp)[11:0] | vsync[27:16]
   */
  uint32_t vtotal = ESP_DSI_VSYNC + ESP_DSI_VBP + ESP_DSI_VRES + ESP_DSI_VFP;
  reg_write(BRG_DPI_V_CFG0,
            (vtotal & 0xFFF) | ((ESP_DSI_VRES & 0xFFF) << 16));
  reg_write(BRG_DPI_V_CFG1,
            (ESP_DSI_VBP & 0xFFF) | ((ESP_DSI_VSYNC & 0xFFF) << 16));

  /* 9. Horizontal timing in bridge:
   *    DPI_H_CFG0: htotal[11:0] | hdisp[27:16]
   *    DPI_H_CFG1: hbank(=hbp)[11:0] | hsync[27:16]
   */
  uint32_t htotal = ESP_DSI_HSYNC + ESP_DSI_HBP + ESP_DSI_HRES + ESP_DSI_HFP;
  reg_write(BRG_DPI_H_CFG0,
            (htotal & 0xFFF) | ((ESP_DSI_HRES & 0xFFF) << 16));
  reg_write(BRG_DPI_H_CFG1,
            (ESP_DSI_HBP & 0xFFF) | ((ESP_DSI_HSYNC & 0xFFF) << 16));

  syslog(LOG_INFO, "[DSI] Bridge timing: vtotal=%lu vdisp=%d htotal=%lu hdisp=%d\n",
         (unsigned long)vtotal, ESP_DSI_VRES,
         (unsigned long)htotal, ESP_DSI_HRES);

  /* 10. Enable bridge */
  reg_write(BRG_EN, 1);

  /* 11. Update DPI config */
  reg_write(BRG_DPI_CONFIG_UPDATE, 1);
}

static int phase6_fb_alloc(void)
{
  syslog(LOG_INFO, "[DSI] Phase 6: Framebuffer alloc (%d bytes)\n",
         ESP_DSI_FB_SIZE);

  /* Allocate framebuffer aligned to 64 bytes (cache line) */
  g_framebuffer = (uint8_t *)kmm_memalign(64, ESP_DSI_FB_SIZE);
  if (g_framebuffer == NULL)
    {
      g_framebuffer = (uint8_t *)kmm_memalign(64, 4096);
      if (g_framebuffer == NULL) {
        syslog(LOG_ERR, "[DSI] ERROR: Even 4KB alloc failed!\n");
        return -ENOMEM;
      }
      memset(g_framebuffer, 0xFF, 4096);
      syslog(LOG_WARNING, "[DSI] Using 4KB test FB at %p\n", g_framebuffer);
      return 0;
      return -ENOMEM;
    }

  /* Fill with red (RGB565 = 0xF800) for visual verification */
  {
    uint16_t *fb16 = (uint16_t *)g_framebuffer;
    for (int i = 0; i < ESP_DSI_HRES * ESP_DSI_VRES; i++)
      {
        fb16[i] = 0xF800;
      }
  }

  syslog(LOG_INFO, "[DSI] Framebuffer at %p\n", g_framebuffer);
  return 0;
}

static void phase7_video_mode(void)
{
  syslog(LOG_INFO, "[DSI] Phase 7: Switch to video mode\n");

  /* 1. Enable continuous HS clock (required for video mode) */
  reg_write(HOST_LPCLK_CTRL, PHY_TXREQUESTCLKHS | AUTO_CLKLANE_CTRL);

  /* 2. Switch host to video mode */
  reg_write(HOST_MODE_CFG, MODE_CFG_VID_MODE);

  /* 3. Enable DPI output in bridge: dpi_misc_config bit0 = dpi_en */
  reg_set_bits(BRG_DPI_MISC_CONFIG, 1);  /* bit 0: dpi_en */

  /* 4. Update bridge config */
  reg_write(BRG_DPI_CONFIG_UPDATE, 1);

  syslog(LOG_INFO, "[DSI] Video mode active\n");
}

static void phase7_5_backlight(void)
{
  syslog(LOG_INFO, "[DSI] Phase 7.5: Backlight (GPIO %d)\n", BACKLIGHT_GPIO);

  gpio_set_output(BACKLIGHT_GPIO);
  gpio_set_high(BACKLIGHT_GPIO);
}

/****************************************************************************
 * Pre-processor Definitions - DW-GDMA (for DSI framebuffer DMA)
 ****************************************************************************/

#define DW_GDMA_BASE              0x50081000
#define DW_GDMA_CH0_BASE          (DW_GDMA_BASE + 0x100)
#define MIPI_DSI_BRG_MEM_BASE     0x50105000

/* DW-GDMA global register offsets */
#define DMAC_CFG0_OFF             0x010
#define DMAC_CHEN0_OFF            0x018
#define DMAC_RESET0_OFF           0x058

/* DW-GDMA channel register offsets */
#define CH_CFG0_OFF               0x020
#define CH_CFG1_OFF               0x024
#define CH_LLP0_OFF               0x028
#define CH_LLP1_OFF               0x02C
#define CH_INTSTATUS_ENA0_OFF     0x080
#define CH_INTSIGNAL_ENA0_OFF     0x090
#define CH_INTCLEAR0_OFF          0x098

/* HP_SYS_CLKRST for DMA clock */
#define GDMA_CPU_CLK_EN_BIT       (1 << 13)
#define GDMA_SYS_CLK_EN_BIT       (1 << 5)
#define RST_EN_GDMA_BIT           (1 << 21)
#define CLKRST_HP_RST_EN0_REG    (HP_SYS_CLKRST_BASE + 0xC0)

/****************************************************************************
 * Private Types - DW-GDMA LLI
 ****************************************************************************/

struct dw_gdma_lli_s
{
  uint32_t sar_lo;        /* 0x00 */
  uint32_t sar_hi;        /* 0x04 */
  uint32_t dar_lo;        /* 0x08 */
  uint32_t dar_hi;        /* 0x0C */
  uint32_t block_ts;      /* 0x10 */
  uint32_t reserved_14;   /* 0x14 */
  uint32_t llp_lo;        /* 0x18: next LLI pointer (before CTL!) */
  uint32_t llp_hi;        /* 0x1C */
  uint32_t ctl_lo;        /* 0x20: control low */
  uint32_t ctl_hi;        /* 0x24: control high */
  uint32_t sstat;         /* 0x28 */
  uint32_t dstat;         /* 0x2C */
  uint32_t status_lo;     /* 0x30 */
  uint32_t status_hi;     /* 0x34 */
  uint32_t reserved_38;   /* 0x38 */
  uint32_t reserved_3c;   /* 0x3C */
} __attribute__((aligned(64)));

static struct dw_gdma_lli_s g_dma_lli __attribute__((aligned(64)));

/* DMA uses the original (bus-accessible) address to read LLI, not the non-cached CPU address */
#define LLI_DMA_ADDR              ((uint32_t)&g_dma_lli)

/****************************************************************************
 * Private Functions - DMA
 ****************************************************************************/

static void phase8_dma_start(void)
{
  syslog(LOG_INFO, "[DSI] Phase 8: DMA start\n");

  if (g_framebuffer == NULL)
    {
      syslog(LOG_ERR, "[DSI] ERROR: no framebuffer for DMA\n");
      return;
    }

  uint32_t fb_size = ESP_DSI_FB_SIZE;
  uint32_t transfer_items = fb_size / 8;  /* 64-bit words */

  /* Step 1: Enable DW-GDMA clock */

  reg_set_bits(HP_SYS_CLKRST_BASE + 0x14, GDMA_CPU_CLK_EN_BIT);
  reg_set_bits(HP_SYS_CLKRST_BASE + 0x18, GDMA_SYS_CLK_EN_BIT);

  /* Step 2: Reset DW-GDMA via HP_RST_EN0 */

  reg_set_bits(CLKRST_HP_RST_EN0_REG, RST_EN_GDMA_BIT);
  delay_us(10);
  reg_clr_bits(CLKRST_HP_RST_EN0_REG, RST_EN_GDMA_BIT);
  delay_us(10);

  /* Step 3: Software reset DMA controller */

  reg_write(DW_GDMA_BASE + DMAC_RESET0_OFF, 1);
  while (reg_read(DW_GDMA_BASE + DMAC_RESET0_OFF) & 1)
    {
    }

  /* Step 4: Enable DMA controller + global interrupt */

  reg_write(DW_GDMA_BASE + DMAC_CFG0_OFF, 0x03);

  /* Step 4.5: Configure DSI Bridge for DMA flow control */

  reg_write(DSI_BRG_BASE + 0x88, (1 << 4));  /* flow_ctrl=0(DMA), multiblk=1 */
  reg_write(DSI_BRG_BASE + 0x8C, 1024 - 256);  /* empty threshold = 768 */

  /* Update bridge config after flow control change */
  reg_write(DSI_BRG_BASE + 0x44, 1);  /* BRG_DPI_CONFIG_UPDATE */

  /* Step 5: Setup linked-list item (circular: points to itself) */

  uint32_t ctl_lo = (1 << 0)  |  /* sms = master1 (memory) */
                    (0 << 2)  |  /* dms = master0 (DSI) */
                    (0 << 4)  |  /* sinc = increment */
                    (1 << 6)  |  /* dinc = fixed */
                    (3 << 8)  |  /* src_tr_width = 64-bit */
                    (3 << 11) |  /* dst_tr_width = 64-bit */
                    (8 << 14) |  /* src_msize = 512 items */
                    (7 << 18);   /* dst_msize = 256 items */

  uint32_t ctl_hi = (1 << 6)  |  /* arlen_en */
                    (15 << 7) |  /* arlen = 16-1 */
                    (1 << 15) |  /* awlen_en */
                    (15 << 16)|  /* awlen = 16-1 */
                    (1 << 26) |  /* ioc_blktfr */
                    (1u << 31);  /* shadowreg_or_lli_valid=1, lli_last=0 (circular) */

  /* Fill LLI structure via non-cached address for DMA coherency.
   * CPU writes go directly to memory, bypassing cache.
   * DMA reads the cached address (which maps to same physical memory).
   */

  volatile struct dw_gdma_lli_s *lli_nc =
    (volatile struct dw_gdma_lli_s *)((uint32_t)&g_dma_lli + 0x40000000);
  lli_nc->sar_lo = (uint32_t)g_framebuffer;
  lli_nc->sar_hi = 0;
  lli_nc->dar_lo = MIPI_DSI_BRG_MEM_BASE;
  lli_nc->dar_hi = 0;
  lli_nc->block_ts = transfer_items - 1;
  lli_nc->reserved_14 = 0;
  lli_nc->llp_lo = LLI_DMA_ADDR;  /* Point to self (cached addr) for circular */
  lli_nc->llp_hi = 0;
  lli_nc->ctl_lo = ctl_lo;
  lli_nc->ctl_hi = ctl_hi;
  lli_nc->sstat = 0;
  lli_nc->dstat = 0;
  lli_nc->status_lo = 0;
  lli_nc->status_hi = 0;
  lli_nc->reserved_38 = 0;
  lli_nc->reserved_3c = 0;

  /* Step 6: Configure channel registers for linked-list mode */

  /* CFG0: src_multblk_type=3 (linked-list), dst_multblk_type=3 (linked-list) */

  reg_write(DW_GDMA_CH0_BASE + CH_CFG0_OFF, (3 << 0) | (3 << 2));

  /* CFG1: M2P, HW handshake for DSI, highest priority */

  reg_write(DW_GDMA_CH0_BASE + CH_CFG1_OFF,
            (1 << 0)  |  /* tt_fc = M2P_DMAC */
            (1 << 3)  |  /* hs_sel_src = SW (memory side) */
            (0 << 4)  |  /* hs_sel_dst = HW (DSI bridge) */
            (0 << 12) |  /* dst_per = DSI (periph index 0) */
            (7 << 17));  /* ch_prior = 7 (highest) */

  /* Step 7: Set LLP to point to our LLI (non-cached address) */

  reg_write(DW_GDMA_CH0_BASE + CH_LLP0_OFF, LLI_DMA_ADDR);
  reg_write(DW_GDMA_CH0_BASE + CH_LLP1_OFF, 0);

  /* Step 8: Clear pending interrupts and enable */

  reg_write(DW_GDMA_CH0_BASE + CH_INTCLEAR0_OFF, 0xFFFFFFFF);
  reg_write(DW_GDMA_CH0_BASE + CH_INTSTATUS_ENA0_OFF, 0x03);

  /* Step 9: Enable channel 0 (write-enable mode) */

  reg_write(DW_GDMA_BASE + DMAC_CHEN0_OFF, (1 << 0) | (1 << 8));

  syslog(LOG_INFO, "[DSI] DMA started (linked-list circular): fb=%p → DSI Bridge, %lu words\n",
         g_framebuffer, (unsigned long)transfer_items);
  syslog(LOG_INFO, "[DSI] LLI at %p (DMA addr: 0x%08lx)\n",
         &g_dma_lli, (unsigned long)LLI_DMA_ADDR);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_mipi_dsi_initialize
 ****************************************************************************/

int esp_mipi_dsi_initialize(void)
{
  int ret;

  syslog(LOG_INFO, "[DSI] === MIPI-DSI Register-Only Init Start ===\n");
  syslog(LOG_INFO, "[DSI] Panel: EK79007 1024x600 RGB565\n");
  syslog(LOG_INFO, "[DSI] Lanes: %d, Bitrate: %d Mbps, DPI: %d MHz\n",
         ESP_DSI_NUM_DATA_LANES, ESP_DSI_LANE_BITRATE_MBPS,
         ESP_DSI_DPI_CLK_MHZ);

  /* Phase 0: LDO power enable */
  phase0_ldo_enable();

  /* Phase 1: Clock enable */
  phase1_clock_enable();

  /* Phase 2: PHY init + PLL */
  phase2_phy_init();

  /* Phase 3: Host controller config (command mode) */
  phase3_host_config();

  /* Phase 3.5: Panel hardware reset */
  phase3_5_panel_reset();

  /* Phase 4: Send panel init commands */
  phase4_panel_init();

  /* Phase 5: DPI configuration */
  phase5_dpi_config();
  phase5_bridge_config();

  /* Phase 6: Allocate framebuffer */
  ret = phase6_fb_alloc();
  if (ret < 0)
    {
      return ret;
    }

  /* Phase 7.5: Backlight on */
  phase7_5_backlight();

  /* Phase 8: Start DMA BEFORE enabling video mode
   * (Bridge needs DMA ready before it starts consuming data)
   */
  phase8_dma_start();

  /* Phase 7: Switch to video mode (AFTER DMA is ready) */
  phase7_video_mode();

  syslog(LOG_INFO, "[DSI] === MIPI-DSI Init Complete ===\n");
  return 0;
}

/****************************************************************************
 * Name: esp_mipi_dsi_get_fb
 ****************************************************************************/

uint8_t *esp_mipi_dsi_get_fb(void)
{
  return g_framebuffer;
}
