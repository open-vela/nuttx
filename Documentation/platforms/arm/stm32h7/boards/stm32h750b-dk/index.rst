==============
stm32h750b-dk
==============

This page discusses issues unique to NuttX configurations for the
STMicroelectronics STM32H750B-DK Discovery Kit, featuring the STM32H750XB
MCU.  The STM32H750XB is a 480MHz-capable Cortex-M7 with 128KB Flash and
1MB SRAM.  Compared to other STM32H7 family members, the STM32H750XB is
typically used together with external storage (Quad-SPI flash, SDRAM) for
graphics-intensive applications.

The board features:

  - STM32H750XBH6 microcontroller (Cortex-M7, up to 480MHz)
  - 4.3" RGB LCD (480x272) with capacitive touchscreen (FT5x06 controller)
  - 128MB SDRAM (IS42S32800J, accessed via FMC)
  - 256Mbit Quad-SPI NOR flash (MT25QL512ABB)
  - 8GB on-board eMMC
  - microSD card slot
  - Ethernet 10/100 (LAN8742A PHY) (currently unused by the default
    NuttX configuration)
  - USB OTG HS Micro-AB connector
  - Audio codec (WM8994) with stereo headphone jack and on-board MEMS
    microphone
  - On-board ST-LINK/V3E for programming, debugging and Virtual COM Port
  - Two user push-buttons (USER + WAKE-UP) and one reset button
  - 4 user LEDs (LD1 green, LD2 orange, LD3 red, LD4 blue)
  - 32.768kHz LSE crystal and 25MHz HSE crystal

Refer to the ST website https://www.st.com/en/evaluation-tools/stm32h750b-dk.html
for further information about this board.

Quick Start
===========

For the full step-by-step deployment tutorial (including STM32CubeProgrammer
flashing screenshots, Minicom serial console setup, and running the LVGL
demo) please refer to the dedicated tutorial in the openvela documentation
site:

  https://github.com/open-vela/docs/blob/dev/zh-cn/quickstart/development_board/STM32H750.md

This page focuses on the BSP-level reference information.

Clock Tree
==========

The default configuration uses the on-board 25MHz HSE crystal as the PLL
source:

  ============== ================
  HSE input      25 MHz
  PLL1 input M1  25 / 5 = 5 MHz
  PLL1 N1        x 160 = 800 MHz
  PLL1 P1        / 2   = 400 MHz  (SYSCLK / CPU)
  PLL1 Q1        / 4   = 200 MHz  (USB, SDMMC, RNG)
  ============== ================

Resulting bus frequencies:

  =============== ==========
  SYSCLK / CPUCLK 400 MHz
  AXI / HCLK      200 MHz
  APB1 / APB2     100 MHz
  APB3 / APB4     100 / 50 MHz
  =============== ==========

Serial Console
==============

The default serial console is **USART3** (Virtual COM Port through the
on-board ST-LINK), 115200 8N1.

  ============== ===
  USART3 Signal  Pin
  ============== ===
  USART3_TX      PB10
  USART3_RX      PB11
  ============== ===

Connect the host PC to the ST-LINK USB connector and open
``/dev/ttyACM0`` (Linux) at 115200 baud.

LEDs
====

Four user-controllable LEDs are wired as follows:

  ====== ==== ========
  Symbol Pin  Color
  ====== ==== ========
  LD1    PI12 Green
  LD2    PI13 Orange
  LD3    PI14 Red
  LD4    PI15 Blue
  ====== ==== ========

Buttons
=======

  ============= ====== =================================
  Button        Pin    Function
  ============= ====== =================================
  USER (B1)     PC13   Generic input, wake-up source
  RESET (BLACK) NRST   Hardware reset of the MCU
  ============= ====== =================================

I2C
===

I2C4 is wired to the on-board peripherals:

  ============== ===
  I2C4 Signal    Pin
  ============== ===
  I2C4_SCL       PD12
  I2C4_SDA       PD13
  ============== ===

The capacitive touch controller (FT5x06) and audio codec (WM8994) sit on
this bus.

LCD Display + Touch
===================

The STM32H750B-DK ships with a 4.3" RGB-565 LCD (480x272 resolution) driven
by the LTDC peripheral, and an FT5x06 capacitive touch controller on I2C4.

The framebuffer is allocated in the external SDRAM by default
(``CONFIG_STM32H7_LTDC_FB_BASE``).  Single-buffer mode is the default; for
double-buffered (smooth-scroll) operation see the tutorial referenced
above under "Quick Start".

USB Host
========

USB OTG HS is brought up through ``stm32_usbhost_initialize()`` from
``src/stm32_usb.c``.  Mass-storage devices and USB hubs are supported when
the relevant Kconfig options are enabled.

Configurations
==============

lvgl:
-----

The default and only shipped configuration runs the LVGL demo widgets on
the on-board LCD with FT5x06 touch input.  Highlights:

  - LVGL graphics library (``CONFIG_GRAPHICS_LVGL=y``)
  - LVGL demo widgets (``CONFIG_LV_USE_DEMO_WIDGETS=y``)
  - Touch input via the FT5x06 controller
  - Framebuffer driver (``CONFIG_DRIVERS_VIDEO=y``,
    ``CONFIG_VIDEO_FB=y``)
  - NuttShell on USART3 VCP
  - USB host enabled

Build
=====

::

    cd nuttx
    ./tools/configure.sh stm32h750b-dk:lvgl
    make -j$(nproc)

Output files: ``nuttx``, ``nuttx.bin``, ``nuttx.hex``.

Flashing
========

STM32CubeProgrammer (CLI) is the recommended workflow:

::

    STM32_Programmer_CLI -c port=SWD mode=UR reset=HWrst freq=4000 \
      -e all -w nuttx.hex -v

Or use the GUI walkthrough in the tutorial linked under "Quick Start"
above.

Running
=======

Open the serial console (115200 8N1) on ``/dev/ttyACM0``, reset the board,
and at the NSH prompt run::

    nsh> lvgldemo

This launches the LVGL widgets demo on the LCD.  Touch input is handled by
the FT5x06 driver registered as ``/dev/input0``.

Known Limitations
=================

  - **Ethernet** is not enabled in the shipped ``lvgl`` configuration.
  - **eMMC and microSD** are not exercised by the shipped configuration;
    the SDMMC driver can be enabled via ``make menuconfig`` if needed.
  - **Audio codec (WM8994)** is not driven by NuttX yet; the I2C4 bus and
    the SAI pin-muxes are present, but no in-tree codec driver wires them
    up.
  - **Quad-SPI external NOR flash** is not exposed as a NuttX MTD device
    in the shipped configuration.

These items are tracked as future enhancements and are intentionally not
blocking the default LVGL demo deliverable.
