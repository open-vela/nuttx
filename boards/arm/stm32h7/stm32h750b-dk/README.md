# STM32H750B-DK Board Support for openvela

[ English | [简体中文](README_zh-cn.md) ]

## Introduction

This directory provides openvela support for the **STMicroelectronics
STM32H750B-DK Discovery Kit** on the `dev-ai-contest-2026` branch.

The STM32H750B-DK hosts an STM32H750XBH6 MCU (Cortex-M7 @ up to
480 MHz, 128 KB on-die Flash, 1 MB SRAM, single + double precision
FPU). It is the official Discovery board for the value-line **H750**
family — the design intent of H750 is "high-performance Cortex-M7 with
external storage", so the board pairs the small on-die flash with a
**256 Mbit Quad-SPI NOR**, **128 MB external SDRAM**, **8 GB on-board
eMMC**, a **4.3" 480x272 RGB LCD** with capacitive touch (FT5x06), an
**audio codec (WM8994)** and **USB OTG HS** — making it the canonical
target for graphics-heavy / LVGL demos in openvela.

For board hardware details, schematics and the official getting-started
guide, see the upstream STMicroelectronics documentation:

- [STM32H750B-DK product page](https://www.st.com/en/evaluation-tools/stm32h750b-dk.html)
- [UM2611 — STM32H750B-DK user manual](https://www.st.com/resource/en/user_manual/um2611-discovery-kit-with-stm32h750xb-mcu-stmicroelectronics.pdf)
- [RM0433 — STM32H750 reference manual](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

> 📘 **End-user tutorial**
>
> A full step-by-step deployment tutorial (with STM32CubeProgrammer
> screenshots, Minicom serial console setup, and how to run the LVGL
> demo) lives in the openvela documentation site:
>
> [`docs/zh-cn/quickstart/development_board/STM32H750.md`](https://github.com/open-vela/docs/blob/dev/zh-cn/quickstart/development_board/STM32H750.md)
>
> This README focuses on **board-level reference information** for
> developers (clock tree, GPIO pinout, defconfig description, flashing
> recipes, known limitations).

## Directory Structure

```
boards/arm/stm32h7/stm32h750b-dk/
├── Kconfig                 # board-level Kconfig
├── CMakeLists.txt          # CMake glue
├── README.md / README_zh-cn.md
├── include/board.h         # clock / LED / GPIO board definitions
├── scripts/                # linker scripts + Make.defs
│   ├── flash.ld            # CM7 flash layout
│   ├── flash_m4.ld         # CM4 flash layout (currently unused)
│   ├── memory.ld
│   ├── user-space.ld
│   ├── gnu-elf.ld
│   └── Make.defs
├── src/                    # board bring-up + user drivers
│   ├── stm32_appinitialize.c
│   ├── stm32_autoleds.c    # auto LED status indication
│   ├── stm32_userleds.c    # /dev/userleds driver
│   ├── stm32_boot.c        # board_early_initialize / board_late_initialize
│   ├── stm32_bringup.c     # peripheral registration
│   ├── stm32_lcd.c         # LTDC + LCD panel init
│   ├── stm32_ft5x06.c      # FT5x06 capacitive touch wiring
│   ├── stm32_usb.c         # USB OTG HS host bring-up
│   ├── stm32_ostest.c      # ostest entry point
│   ├── stm32_reset.c       # board reset hook
│   ├── stm32h750b-dk.h     # private board header
│   ├── CMakeLists.txt
│   └── Makefile
└── configs/
    └── lvgl/defconfig      # default LVGL demo configuration
```

The chip family code (`arch/arm/src/stm32h7/`) is shared with the rest
of the H7 family (H743 / H753 / H745 / H7B3) — this board only owns the
files listed above.

## Hardware Overview

| Block          | Component                  | Notes                                    |
| -------------- | -------------------------- | ---------------------------------------- |
| MCU            | STM32H750XBH6              | Cortex-M7 @ 400 MHz default (480 capable)|
| On-die Flash   | 128 KB                     | bootloader / vector table                |
| On-die SRAM    | ~1 MB total                | DTCM + AXI SRAM + SRAM1/2/3/4            |
| External SDRAM | 128 MB IS42S32800J         | via FMC bank 1, used as LCD framebuffer  |
| External NOR   | 256 Mbit MT25QL512ABB      | Quad-SPI (not exposed as MTD by default) |
| eMMC           | 8 GB                       | SDMMC interface (not exercised)          |
| microSD        | slot                       | SDMMC interface (not exercised)          |
| LCD            | 4.3" 480x272 RGB           | LTDC, RGB-565 in default                 |
| Touch          | FT5x06 capacitive          | I2C4 + INT line                          |
| Audio codec    | WM8994                     | I2C + SAI (not driven yet)               |
| Ethernet PHY   | LAN8742A                   | RMII (not enabled in default config)     |
| USB            | OTG HS Micro-AB            | host bring-up via `stm32_usbhost`        |
| Debugger       | ST-LINK/V3E + VCP          | `/dev/ttyACM0` @ 115200 8N1              |
| LEDs           | LD1 / LD2 / LD3 / LD4      | green / orange / red / blue              |
| Buttons        | USER (B1) + WAKE-UP + RESET| user input + wake source + HW reset      |
| Crystals       | 25 MHz HSE + 32.768 kHz LSE| HSE drives PLL1/2/3                      |

## Clock Tree

The default board.h drives PLL1 from the on-board 25 MHz HSE crystal:

| Stage              | Value             |
| ------------------ | ----------------- |
| HSE input          | 25 MHz            |
| PLL1 input M1      | 25 / 5 = 5 MHz    |
| PLL1 N1            | x 160 = 800 MHz   |
| PLL1 P1 (SYSCLK)   | / 2 = **400 MHz** |
| PLL1 Q1 (USB/SDMMC)| / 4 = 200 MHz     |

Resulting bus frequencies:

| Bus              | Frequency |
| ---------------- | --------- |
| SYSCLK / CPUCLK  | 400 MHz   |
| AXI / HCLK       | 200 MHz   |
| APB1 / APB2      | 100 MHz   |
| APB3             | 100 MHz   |
| APB4             | 50 MHz    |

> The MCU is rated up to 480 MHz; 400 MHz is the conservative default
> shipped with this board overlay.

## Supported Peripherals (lvgl defconfig)

| Peripheral    | NuttX node          | Description                                   |
| ------------- | ------------------- | --------------------------------------------- |
| USART3 VCP    | `/dev/ttyS0` (cons) | Console via ST-LINK Virtual COM Port          |
| LEDs          | autoleds (kernel)   | LD1..LD4 used for system status indication    |
| Framebuffer   | `/dev/fb0`          | 480x272 RGB-565, framebuffer in external SDRAM|
| Touch         | `/dev/input0`       | FT5x06 single-touch                           |
| USB Host      | `/dev/ttyUSB*`/etc  | OTG HS host (mass storage, hub, etc.)         |
| RAM MTD       | `/dev/rammtd`       | RAM-backed MTD for testing                    |

## GPIO Pin Map (lvgl defconfig)

| Function          | Pin   | Notes                              |
| ----------------- | ----- | ---------------------------------- |
| USART3_TX         | PB10  | Console TX (to ST-LINK VCP)        |
| USART3_RX         | PB11  | Console RX (from ST-LINK VCP)      |
| I2C4_SCL          | PD12  | FT5x06 + WM8994 codec              |
| I2C4_SDA          | PD13  | FT5x06 + WM8994 codec              |
| LD1 (green)       | PI12  | autoled                            |
| LD2 (orange)      | PI13  | autoled                            |
| LD3 (red)         | PI14  | autoled                            |
| LD4 (blue)        | PI15  | autoled                            |
| USER button (B1)  | PC13  | user push-button                   |
| LCD R0..R7        | PI15..|                                    |
| LCD G0..G7        | PJ7..PK2|                                  |
| LCD B0..B7        | PJ12..PK7|                                 |
| LCD HSYNC/VSYNC   | PI10/PI9|                                  |
| LCD DCLK / DE     | PI14/PK7|                                  |
| LCD backlight EN  | PK0   |                                    |
| FT5x06 INT        | PK7   | touch interrupt                    |

> Full LCD pin list is encoded in `include/board.h` — the table above
> is condensed for quick reference.

## Build

```bash
cd nuttx
./tools/configure.sh stm32h750b-dk:lvgl
make -j$(nproc)
```

Build outputs:

- `nuttx`     — ELF (debug symbols)
- `nuttx.bin` — raw binary for flashing at `0x08000000`
- `nuttx.hex` — Intel HEX for flashing via STM32CubeProgrammer

### Available defconfigs

| Defconfig | Purpose                                                  |
| --------- | -------------------------------------------------------- |
| `lvgl`    | LVGL graphics demo on the on-board LCD with touch input. Default and currently only shipped configuration. |

## Flash

### Option A — STM32CubeProgrammer CLI (recommended)

```bash
STM32_Programmer_CLI -c port=SWD mode=UR reset=HWrst freq=4000 \
  -e all -w nuttx.hex -v
```

The `-e all` performs a full chip erase (safe for development); drop it
to keep existing data.

### Option B — STM32CubeProgrammer GUI

See the
[end-user tutorial](https://github.com/open-vela/docs/blob/dev/zh-cn/quickstart/development_board/STM32H750.md)
for the full step-by-step screenshot walk-through (Erase & Programming
tab, Skip flash erase, Start Programming, etc.).

### Option C — OpenOCD

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
  -c "program nuttx.hex verify reset exit"
```

## First Boot & Quick Tests

1. Open a serial terminal at 115200 8N1 on `/dev/ttyACM0` (Linux):

   ```bash
   sudo minicom -D /dev/ttyACM0 -b 115200
   # Disable hardware/software flow control in Minicom configuration.
   ```

2. Press the black RESET button. You should see the NSH banner:

   ```
   NuttShell (NSH) NuttX-x.y.z
   nsh>
   ```

3. Inspect device nodes:

   ```
   nsh> ls /dev
     console   fb0       input0    null      ttyS0     userleds  ...
   ```

4. Run the LVGL demo:

   ```
   nsh> lvgldemo
   ```

   The widgets demo should appear on the LCD; touch input is handled
   by the FT5x06 driver registered as `/dev/input0`.

5. Other quick tests:

   ```
   nsh> ostest               # OS API + FPU regression test
   nsh> ls /dev/input0       # touch input device
   nsh> cat /proc/uptime     # procfs sanity
   ```

## Customising the Configuration

```bash
make menuconfig            # interactive
make savedefconfig         # write back to configs/lvgl/defconfig
```

Common toggles for this board:

| Symbol                          | Effect                            |
| ------------------------------- | --------------------------------- |
| `CONFIG_GRAPHICS_LVGL`          | LVGL graphics library             |
| `CONFIG_LV_USE_DEMO_WIDGETS`    | Enable LVGL widgets demo          |
| `CONFIG_INPUT_FT5X06`           | FT5x06 capacitive touch driver    |
| `CONFIG_STM32H7_LTDC`           | LCD-TFT controller                |
| `CONFIG_STM32H7_LTDC_FB_BASE`   | Framebuffer base address (SDRAM)  |
| `CONFIG_STM32H7_LTDC_FB_SIZE`   | Framebuffer size (bytes)          |
| `CONFIG_STM32H7_OTGFS`          | USB OTG host                      |
| `CONFIG_FB_DOUBLE_BUFFER`       | Enable double-buffered LCD output |

The end-user tutorial linked above documents how to switch the LCD into
double-buffered mode for smoother scrolling.

## Debugging

Use OpenOCD as a GDB server with the on-board ST-LINK:

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg
```

Then in another terminal:

```bash
arm-none-eabi-gdb nuttx \
  -ex "target extended-remote :3333" \
  -ex "monitor reset halt"
```

## Known Limitations

These items are intentionally **not** wired up by the shipped
configuration; they are tracked as future enhancements:

- **Ethernet (LAN8742A)** — not enabled in the `lvgl` defconfig. The
  board has Ethernet hardware but no NSH networking config ships out of
  the box.
- **Audio codec (WM8994)** — not driven. The I2C bus and SAI pin-muxes
  are present but no in-tree codec driver wires them up.
- **eMMC + microSD (SDMMC)** — not exercised by the shipped
  configuration. The `STM32H7_SDMMC*` Kconfig options can enable it.
- **Quad-SPI external NOR (MT25QL512ABB)** — not exposed as a NuttX
  MTD device by default.
- **USB OTG HS device mode** — only host mode is brought up; device
  mode is configurable but not in this defconfig.

## License

Apache License 2.0. See the SPDX header at the top of each source
file. This board overlay is licensed identically to the rest of the
NuttX RTOS upstream.
