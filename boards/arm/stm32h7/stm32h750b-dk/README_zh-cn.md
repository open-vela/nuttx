# STM32H750B-DK 开发板 openvela 适配

[ [English](README.md) | 简体中文 ]

## 简介

本目录提供 **STMicroelectronics STM32H750B-DK Discovery Kit** 在
`dev-ai-contest-2026` 大赛分支上的 openvela 适配。

STM32H750B-DK 搭载 STM32H750XBH6 MCU(Cortex-M7,主频最高 480 MHz,
片内 128 KB Flash,1 MB SRAM,带单/双精度 FPU)。它是 H750 价值
线的官方 Discovery 板 — H750 系列的设计定位是"基于外部存储的高
性能 Cortex-M7",所以这块板把片内小 Flash 与 **256 Mbit Quad-SPI
NOR**、**128 MB 外部 SDRAM**、**8 GB 板载 eMMC**、**4.3" 480x272
RGB 触摸屏**(FT5x06)、**音频编解码器**(WM8994)和 **USB OTG HS**
凑齐 — 是 openvela 跑图形/LVGL 类 demo 的标准目标板。

板卡硬件细节、原理图与官方上手指南请见 ST 上游文档:

- [STM32H750B-DK 产品页](https://www.st.com/en/evaluation-tools/stm32h750b-dk.html)
- [UM2611 — STM32H750B-DK 用户手册](https://www.st.com/resource/en/user_manual/um2611-discovery-kit-with-stm32h750xb-mcu-stmicroelectronics.pdf)
- [RM0433 — STM32H750 参考手册](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

> 📘 **用户教程**
>
> 完整图文部署教程(含 STM32CubeProgrammer 截图、Minicom 串口配置、
> LVGL demo 跑通过程)见 openvela 文档站:
>
> [`docs/zh-cn/quickstart/development_board/STM32H750.md`](https://github.com/open-vela/docs/blob/dev/zh-cn/quickstart/development_board/STM32H750.md)
>
> 本 README 聚焦**板级开发参考信息**(时钟树、GPIO 引脚、defconfig
> 说明、烧录步骤、已知限制)。

## 目录结构

```
boards/arm/stm32h7/stm32h750b-dk/
├── Kconfig                 # 板级 Kconfig
├── CMakeLists.txt          # CMake 入口
├── README.md / README_zh-cn.md
├── include/board.h         # 板级时钟 / LED / GPIO 定义
├── scripts/                # 链接脚本 + Make.defs
│   ├── flash.ld            # CM7 Flash 布局
│   ├── flash_m4.ld         # CM4 Flash 布局(目前未启用)
│   ├── memory.ld
│   ├── user-space.ld
│   ├── gnu-elf.ld
│   └── Make.defs
├── src/                    # 板级 bring-up + 驱动
│   ├── stm32_appinitialize.c
│   ├── stm32_autoleds.c    # 自动 LED 状态指示
│   ├── stm32_userleds.c    # /dev/userleds 驱动
│   ├── stm32_boot.c        # board_early/late_initialize
│   ├── stm32_bringup.c     # 外设注册入口
│   ├── stm32_lcd.c         # LTDC + LCD panel 初始化
│   ├── stm32_ft5x06.c      # FT5x06 电容触摸接线
│   ├── stm32_usb.c         # USB OTG HS host bring-up
│   ├── stm32_ostest.c      # ostest 入口
│   ├── stm32_reset.c       # 板级 reset 钩子
│   ├── stm32h750b-dk.h     # 板级私有头文件
│   ├── CMakeLists.txt
│   └── Makefile
└── configs/
    └── lvgl/defconfig      # 默认 LVGL demo 配置
```

芯片家族代码(`arch/arm/src/stm32h7/`)与 H7 家族其它成员
(H743 / H753 / H745 / H7B3)共享 — 本目录只拥有上述板级文件。

## 硬件概览

| 模块         | 元件                       | 备注                                         |
| ------------ | -------------------------- | -------------------------------------------- |
| MCU          | STM32H750XBH6              | Cortex-M7,默认 400 MHz(支持 480 MHz)      |
| 片内 Flash   | 128 KB                     | bootloader / 向量表                          |
| 片内 SRAM    | ~1 MB 总容量               | DTCM + AXI SRAM + SRAM1/2/3/4                |
| 外部 SDRAM   | 128 MB IS42S32800J         | 通过 FMC bank 1,作为 LCD 帧缓冲             |
| 外部 NOR     | 256 Mbit MT25QL512ABB      | Quad-SPI(默认未挂为 MTD)                   |
| eMMC         | 8 GB                       | SDMMC 接口(默认未启用)                     |
| microSD      | 卡槽                       | SDMMC 接口(默认未启用)                     |
| LCD          | 4.3" 480x272 RGB           | LTDC,默认 RGB-565                           |
| 触摸         | FT5x06 电容屏              | I2C4 + INT 中断                              |
| 音频         | WM8994                     | I2C + SAI(尚未驱动)                        |
| 以太网 PHY   | LAN8742A                   | RMII(默认配置未启用)                       |
| USB          | OTG HS Micro-AB            | host bring-up 由 `stm32_usbhost` 完成        |
| 调试器       | ST-LINK/V3E + VCP          | `/dev/ttyACM0` @ 115200 8N1                  |
| LED          | LD1 / LD2 / LD3 / LD4      | 绿 / 橙 / 红 / 蓝                            |
| 按键         | USER (B1) + WAKE-UP + RESET| 用户输入 + 唤醒源 + 硬件复位                 |
| 晶振         | 25 MHz HSE + 32.768 kHz LSE| HSE 驱动 PLL1/2/3                            |

## 时钟树

默认 board.h 用板载 25 MHz HSE 晶振驱动 PLL1:

| 阶段                | 取值              |
| ------------------- | ----------------- |
| HSE 输入            | 25 MHz            |
| PLL1 输入 M1        | 25 / 5 = 5 MHz    |
| PLL1 N1             | x 160 = 800 MHz   |
| PLL1 P1 (SYSCLK)    | / 2 = **400 MHz** |
| PLL1 Q1 (USB/SDMMC) | / 4 = 200 MHz     |

得到的总线频率:

| 总线             | 频率      |
| ---------------- | --------- |
| SYSCLK / CPUCLK  | 400 MHz   |
| AXI / HCLK       | 200 MHz   |
| APB1 / APB2      | 100 MHz   |
| APB3             | 100 MHz   |
| APB4             | 50 MHz    |

> MCU 标称最高 480 MHz;本板默认配置取保守的 400 MHz。

## 已支持外设(lvgl defconfig)

| 外设         | NuttX 节点          | 说明                                         |
| ------------ | ------------------- | -------------------------------------------- |
| USART3 VCP   | `/dev/ttyS0` (cons) | 通过 ST-LINK 虚拟串口的控制台                |
| LED          | autoleds(内核)    | LD1..LD4 用作系统状态指示                    |
| 帧缓冲       | `/dev/fb0`          | 480x272 RGB-565,缓冲区位于外部 SDRAM         |
| 触摸         | `/dev/input0`       | FT5x06 单点触摸                              |
| USB Host     | 视设备而定          | OTG HS host(U 盘 / Hub 等)                 |
| RAM MTD      | `/dev/rammtd`       | RAM 模拟的 MTD,用于测试                     |

## GPIO 引脚映射(lvgl defconfig)

| 功能              | 引脚  | 备注                               |
| ----------------- | ----- | ---------------------------------- |
| USART3_TX         | PB10  | 控制台 TX(到 ST-LINK VCP)        |
| USART3_RX         | PB11  | 控制台 RX(来自 ST-LINK VCP)      |
| I2C4_SCL          | PD12  | FT5x06 + WM8994                    |
| I2C4_SDA          | PD13  | FT5x06 + WM8994                    |
| LD1(绿)          | PI12  | autoled                            |
| LD2(橙)          | PI13  | autoled                            |
| LD3(红)          | PI14  | autoled                            |
| LD4(蓝)          | PI15  | autoled                            |
| USER 按键(B1)   | PC13  | 用户按键                           |
| LCD R0..R7        | PI15..|                                    |
| LCD G0..G7        | PJ7..PK2 |                                 |
| LCD B0..B7        | PJ12..PK7 |                                |
| LCD HSYNC/VSYNC   | PI10/PI9  |                                |
| LCD DCLK / DE     | PI14/PK7  |                                |
| LCD 背光使能      | PK0   |                                    |
| FT5x06 INT        | PK7   | 触摸中断                           |

> LCD 完整引脚列表在 `include/board.h` 中,上表为速查精简版。

## 编译

```bash
cd nuttx
./tools/configure.sh stm32h750b-dk:lvgl
make -j$(nproc)
```

输出文件:

- `nuttx`     — 带调试符号的 ELF
- `nuttx.bin` — 烧到 `0x08000000` 的原始 bin
- `nuttx.hex` — STM32CubeProgrammer 用的 Intel HEX

### 可用 defconfig

| Defconfig | 用途                                                         |
| --------- | ------------------------------------------------------------ |
| `lvgl`    | 在板载 LCD 上跑 LVGL 图形 demo,带触摸输入。当前默认且唯一配置。|

## 烧录

### 方案 A — STM32CubeProgrammer CLI(推荐)

```bash
STM32_Programmer_CLI -c port=SWD mode=UR reset=HWrst freq=4000 \
  -e all -w nuttx.hex -v
```

`-e all` 全片擦除(开发阶段安全);如想保留旧数据可去掉。

### 方案 B — STM32CubeProgrammer GUI

完整图文步骤(Erase & Programming 标签、Skip flash erase、Start
Programming 等)请见
[用户教程](https://github.com/open-vela/docs/blob/dev/zh-cn/quickstart/development_board/STM32H750.md)。

### 方案 C — OpenOCD

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
  -c "program nuttx.hex verify reset exit"
```

## 首次启动与冒烟测试

1. 打开 115200 8N1 串口终端(Linux 上 `/dev/ttyACM0`):

   ```bash
   sudo minicom -D /dev/ttyACM0 -b 115200
   # 在 Minicom 配置里关掉硬件/软件流控
   ```

2. 按板上黑色 RESET 键,应看到 NSH banner:

   ```
   NuttShell (NSH) NuttX-x.y.z
   nsh>
   ```

3. 查看设备节点:

   ```
   nsh> ls /dev
     console   fb0       input0    null      ttyS0     userleds  ...
   ```

4. 跑 LVGL demo:

   ```
   nsh> lvgldemo
   ```

   widgets demo 会出现在 LCD 上;触摸输入由 FT5x06 驱动通过
   `/dev/input0` 处理。

5. 其它快速测试:

   ```
   nsh> ostest               # OS API + FPU 回归测试
   nsh> ls /dev/input0       # 触摸输入设备
   nsh> cat /proc/uptime     # procfs 健全性测试
   ```

## 定制配置

```bash
make menuconfig            # 交互式
make savedefconfig         # 写回 configs/lvgl/defconfig
```

本板常用开关:

| 符号                            | 效果                              |
| ------------------------------- | --------------------------------- |
| `CONFIG_GRAPHICS_LVGL`          | LVGL 图形库                       |
| `CONFIG_LV_USE_DEMO_WIDGETS`    | 启用 LVGL widgets demo            |
| `CONFIG_INPUT_FT5X06`           | FT5x06 电容触摸驱动               |
| `CONFIG_STM32H7_LTDC`           | LCD-TFT 控制器                    |
| `CONFIG_STM32H7_LTDC_FB_BASE`   | 帧缓冲基地址(SDRAM)             |
| `CONFIG_STM32H7_LTDC_FB_SIZE`   | 帧缓冲大小(字节)                |
| `CONFIG_STM32H7_OTGFS`          | USB OTG host                      |
| `CONFIG_FB_DOUBLE_BUFFER`       | 启用双缓冲 LCD 输出               |

上面链接的用户教程详细说明了如何把 LCD 切到双缓冲模式获得更顺
畅的滚动效果。

## 调试

用 OpenOCD 启动一个 GDB server 配合板载 ST-LINK:

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg
```

然后另开一个终端:

```bash
arm-none-eabi-gdb nuttx \
  -ex "target extended-remote :3333" \
  -ex "monitor reset halt"
```

## 已知限制

下列项默认配置**未**启用,作为后续增强项:

- **以太网(LAN8742A)** — `lvgl` defconfig 中未启用。板上有以太网
  硬件,但默认未带 NSH 网络配置。
- **音频编解码器(WM8994)** — 未驱动。I2C 总线和 SAI 引脚已布通,
  但树内目前没有该 codec 驱动。
- **eMMC + microSD(SDMMC)** — 默认配置未跑通。可通过
  `STM32H7_SDMMC*` Kconfig 选项开启。
- **Quad-SPI 外部 NOR(MT25QL512ABB)** — 默认未挂为 NuttX MTD 设备。
- **USB OTG HS device 模式** — 当前只跑通 host;device 模式可配但
  默认未启用。

## License

Apache License 2.0,见每个源文件顶部的 SPDX 头。本板适配许可与
NuttX RTOS 上游完全一致。
