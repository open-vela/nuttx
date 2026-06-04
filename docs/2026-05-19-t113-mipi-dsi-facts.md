# T113 MIPI DSI 真缺口钉死记录

Target: R528 HMI EVB4, GC9503CV 480x800 2-lane MIPI DSI (X4B配置)
Date: 2026-05-19

---

## 1. DE 基址 (D1)

| Block          | Physical Base  | Size   | Source |
|----------------|----------------|--------|--------|
| DE (DE0)       | 0x05000000     | 4 MB   | manual §2.2 Memory Map (0x0500_0000–0x053F_FFFF); vendor: `sun8iw20.c` g_reg_base[0] |
| display_if_top | 0x05460000     | 64 KB  | vendor: `sun8iw20.c` g_reg_base[1] |
| TCON-LCD0      | 0x05461000     | 4 KB   | vendor: `sun8iw20.c` g_reg_base[2]; manual §2.2 (0x0546_1000–0x0546_1FFF) |
| DSI0           | 0x05450000     | —      | vendor: `sun8iw20.c` g_reg_base[4]; manual §5.4 |

DE 顶级基址 (DE0) = **0x05000000**。其下子模块偏移由 DE 内部寄存器映射决定，不在本表范围。

---

## 2. CCU clock / reset ID 表 (D2)

CCU 基址: 0x02001000 (manual §3.3; vendor: `ccu-sun8iw20.h` SUNXI_CCU_BASE)

所有偏移均相对于 CCU 基址。

> 位字段表示约定: `bit[hi:lo]` 为 ARM 风格的高位:低位闭区间(MSB-first); 部分注解中出现的 `(start, width)` 来自 vendor `SUNXI_CCU_M_WITH_MUX_GATE(..., start_bit, width, ...)` 宏调用约定，二者等价。

### Clock IDs

| NuttX 常量名 (拟) | vendor CLK_ID | CCU 偏移 | 功能位 | Parent MUX 选项 | 典型 parent | 分频字段 | Source |
|---|---|---|---|---|---|---|---|
| T113_CLK_DE | CLK_DE0 = 29 | 0x0600 | bit31: gate, bit[26:24]: mux, bit[4:0]: M | 000=PLL_PERIPH0_2X, 001=PLL_VIDEO0_4X, 010=PLL_VIDEO1_4X, 011=PLL_AUDIO1_DIV2 | PLL_PERIPH0_2X | M = FACTOR_M+1 (bits[4:0]) | manual §3.3.6.38; vendor `ccu-sun8iw20.c`:256 |
| T113_CLK_DE_BUS | CLK_BUS_DE0 = 30 | 0x060C | bit0: gating | PSI/AHB (fixed) | — | — | manual §3.3.6.39; vendor `ccu-sun8iw20.c`:262 |
| T113_CLK_DPSS_TOP | CLK_BUS_DPSS_TOP0 = 108 | 0x0ABC | bit0: gating | PSI/AHB (fixed) | — | — | manual §3.3.6.89; vendor `ccu-sun8iw20.c`:553 |
| T113_CLK_DSI | CLK_MIPI_DSI = 113 | 0x0B24 | bit31: gate, bit[26:24]: mux, bit[3:0]: M | 000=HOSC, 001=PLL_PERIPH0(1X), 010=PLL_VIDEO0_2X, 011=PLL_VIDEO1_2X, 100=PLL_AUDIO1_DIV2 | PLL_PERIPH0 | M = FACTOR_M+1 (bits[3:0]) | manual §3.3.6.90; vendor `ccu-sun8iw20.c`:572 |
| T113_CLK_DSI_BUS | CLK_BUS_MIPI_DSI = 114 | 0x0B4C | bit0: gating | PSI/AHB (fixed) | — | — | manual §3.3.6.91; vendor `ccu-sun8iw20.c`:580 |
| T113_CLK_TCON_LCD0 | CLK_TCON_LCD0 = 115 | 0x0B60 | bit31: gate, bit[26:24]: mux, bit[9:8]: N, bit[3:0]: M | 000=PLL_VIDEO0(1X), 001=PLL_VIDEO0(4X), 010=PLL_VIDEO1(1X), 011=PLL_VIDEO1(4X), 100=PLL_PERI(2X), 101=PLL_AUDIO1_DIV2 | PLL_VIDEO0_4X | M = FACTOR_M+1 (bits[3:0]); N = 1/2/4/8 (bits[9:8]) | manual §3.3.6.92; vendor `ccu-sun8iw20.c`:587 |
| T113_CLK_TCON_LCD0_BUS | CLK_BUS_TCON_LCD0 = 116 | 0x0B7C | bit0: gating | PSI/AHB (fixed) | — | — | manual §3.3.6.93; vendor `ccu-sun8iw20.c`:595 |

### Reset IDs

| NuttX 常量名 (拟) | vendor RST_ID | CCU 偏移 | 功能位 | Source |
|---|---|---|---|---|
| T113_RST_DE | RST_BUS_DE0 = 1 | 0x060C | bit16: 0=assert, 1=de-assert | manual §3.3.6.39; vendor `rst-sun8iw20.h`:10 |
| T113_RST_DPSS_TOP | RST_BUS_DPSS_TOP0 = 50 | 0x0ABC | bit16: 0=assert, 1=de-assert | manual §3.3.6.89; vendor `rst-sun8iw20.h`:57 |
| T113_RST_DSI | RST_BUS_MIPI_DSI = 53 | 0x0B4C | bit16: 0=assert, 1=de-assert | manual §3.3.6.91; vendor `rst-sun8iw20.h`:60 |
| T113_RST_TCON_LCD0 | RST_BUS_TCON_LCD0 = 54 | 0x0B7C | bit16: 0=assert, 1=de-assert | manual §3.3.6.93; vendor `rst-sun8iw20.h`:61 |

注: 原计划要求 T113_CLK_DSI_PHY 独立时钟 ID — 查 ccu-sun8iw20.h 和 sun8iw20.c 确认不存在独立 DSI PHY LDO/PLL 时钟 ID；PHY PLL 由 DPHY 自身内部 PLL 寄存器控制（见第3节），无 CCU 时钟门控。

---

## 3. DSI PHY 时钟分频公式与 X4B 代入值 (D3)

### 公式

vendor 算法来源: `de_dsi.c::dsi_comb_dphy_pll_set()` (line 667)

```
frq = pixel_clk_MHz * bits_per_pixel / lanes   [MHz/lane, 即原始 bit rate]

# 找最小 m 使 VCO = frq * m > 1300 MHz (VCO target = 1.3 GHz):
m = ceil(1300 / frq)    # vendor 算法直接 break,无显式 upper-bound clamp;X4B 自然落在 m=4

frq_p = frq * m                    # VCO 频率 (Hz)
n = floor(frq_p / 24_000_000)      # PLL 倍频系数 N (整数截断)

# DPHY PLL 寄存器:
#   dphy_pll_reg0.n   = n
#   dphy_pll_reg0.p   = 0  (div_p = 0, P = div_p+1 = 1, 固定)
#   dphy_pll_reg0.m0  = m-1
#   dphy_pll_reg0.m1  = 2  (固定, 仅用于 LP 时钟)

HS_clk_per_lane = 24_MHz * n / (p+1) / (m0+1)   [MHz/lane]
```

### X4B 代入 (pixel_clk=31 MHz, RGB888=24 bpp, lanes=2)

```
frq = 31 * 24 / 2 = 372 MHz/lane

# VCO search: m=1 → 372 MHz < 1300; m=2 → 744 < 1300; m=3 → 1116 < 1300; m=4 → 1488 > 1300
m = 4

frq_p = 372 * 4 = 1488 MHz
n = floor(1488 / 24) = 62

# 寄存器写入值:
p   = 0   → P  = 1
m0  = 3   → M0 = 4
m1  = 2   (固定)
n   = 62

HS_clk = 24 * 62 / 1 / 4 = 372 MHz/lane = 372 Mbps/lane
```

**结论**: X4B 目标 HS 时钟 = **372 Mbps/lane** (2 lane 总带宽 744 Mbps)

注: 计划文档中写 "465 Mbps/lane (overhead 1.25)" 是近似估算。vendor 算法不含 overhead 系数，直接用 pixel_clk * bpp / lanes 算原始 bit rate。实际 overhead 由 DSI protocol 包头/消隐期承担，不体现在 PHY PLL 配置中。**以 vendor 算法为准：PHY = 372 Mbps/lane。**

---

## 4. IRQ 号

中断号直接引用 manual §3.8 Interrupt Table（GIC interrupt ID = NuttX IRQ number，与现有 t113_usb.h 定义一致：`T113_IRQ_USB1_EHCI = 65` = GIC ID 65 = SPI 33+32）。

| 模块 | GIC 中断号 (= NuttX IRQ) | GIC SPI 号 | Source |
|---|---|---|---|
| DE | 119 | SPI 87 | manual §3.8 interrupt table line 119 |
| TCON-LCD0 (LCD) | 122 | SPI 90 | manual §3.8 line 122; vendor `sun8iw20.c` g_irq_no[0]=122 |
| TCON-TV | 123 | SPI 91 | manual §3.8 line 123 |
| DSI | 124 | SPI 92 | manual §3.8 line 124; vendor `sun8iw20.c` g_irq_no[2]=124 |

DE 无独立 IRQ 入口在 vendor display subsystem 代码中使用，但 GIC 表显示 IRQ 119 已分配给 DE。NuttX driver 初期可轮询，无需强依赖 DE IRQ。

---

## 参考文件索引

- `~/projects/t113/T113-s3_user_manual_v1.1.md` — §2.2 Memory Map, §3.3 CCU, §3.8 Interrupt Table, §5.4 MIPI DSI
- `vendor: .../disp2/soc/sun8iw20.c` — g_reg_base[], g_irq_no[], g_clk_no[] for T113-S3 (sun8iw20)
- `vendor: .../ccmu/sunxi-ng/ccu-sun8iw20.h` — CLK_* ID 定义
- `vendor: .../ccmu/sunxi-ng/rst-sun8iw20.h` — RST_BUS_* ID 定义
- `vendor: .../ccmu/sunxi-ng/ccu-sun8iw20.c` — 各 clock 的 CCU 寄存器偏移和 bit 字段
- `vendor: .../disp2/disp/de/lowlevel_v2x/de_dsi.c::dsi_comb_dphy_pll_set()` — DPHY PLL 算法
