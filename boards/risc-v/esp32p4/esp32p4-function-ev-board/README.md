# ESP32-P4-Function-EV-Board

openvela (NuttX) board support for the Espressif ESP32-P4-Function-EV-Board,
a dual-core RISC-V AIoT development board.

## SoC and board

- **SoC:** ESP32-P4 — dual-core RISC-V HP (up to 400 MHz) + a LP core,
  RV32IMAC, verified on silicon revision v3.2.
- **On board:** 16 MB flash, 32 MB PSRAM, 7-inch 1024x600 MIPI-DSI capacitive
  touch panel, MIPI-CSI camera header, ES8311 audio codec, an ESP32-C6 Wi-Fi
  module, RJ45 Ethernet and a MicroSD slot.

The chip layer lives under `arch/risc-v/src/esp32p4/` and keeps a private copy
of the espressif shared driver layer so the existing esp32c3/c6/h2 ports are
untouched. The Espressif HAL (esp-hal-3rdparty) is cloned at build time and the
openvela compatibility patches under
`arch/risc-v/src/esp32p4/patches/esp-hal-3rdparty/` are applied automatically.

## Building

```bash
# From the openvela workspace root:
./build.sh boards/risc-v/esp32p4/esp32p4-function-ev-board/configs/nsh -j8
```

The build self-heals after `distclean`: the HAL is re-cloned (pinned to
`b90b1837`) and re-patched by an atomic clone recipe.

## Flashing

The ESP32-P4 ROM boots from flash offset **`0x2000`** (unlike ESP32-C3/C6/H2,
which boot from `0x0`); flashing at `0x0` makes the ROM loop on
`invalid header`.

```bash
esptool -c esp32p4 -p <port> -b 921600 write_flash 0x2000 nuttx.bin
```

## Consoles

| Config family | Console |
|---|---|
| `nsh`, most configs | UART0, TX=GPIO37 / RX=GPIO38, 115200 8N1 |
| `usbconsole`        | on-chip USB-Serial-JTAG (same cable as flashing) |

## Pin map (defaults)

| Function | Pin(s) |
|---|---|
| UART0 console | TX GPIO37 / RX GPIO38 |
| I2C0 | SCL GPIO8 / SDA GPIO7 (GT911 touch @0x5d, ES8311 @0x18) |
| SPI2 | CS 28 / CLK 30 / MOSI 29 / MISO 31 |
| MIPI-DSI LCD | reset GPIO27, backlight GPIO26 (via the display subboard) |
| GT911 touch INT | GPIO23 |

## Display and touch

The board provides a self-contained MIPI-DSI display driver
(`arch/risc-v/src/esp32p4/espressif/esp_mipi_dsi.c`) that streams an RGB565
framebuffer through the DSI bridge with a hardware auto-reloading DW-GDMA
channel (no per-frame interrupt), and a NuttX framebuffer interface with double
buffering. The 7-inch EK79007 panel and the GT911 touch controller
(`drivers/input/gt9xx.c`) are wired up in the board glue
(`src/esp32p4_lcd.c`, `src/esp32p4_touch.c`).

## Verification status

Verified on real hardware (chip rev v3.2): NSH shell, a full `ostest` pass,
the 32 MB PSRAM heap, the 1024x600 MIPI-DSI panel and GT911 touch driving an
LVGL demo.
