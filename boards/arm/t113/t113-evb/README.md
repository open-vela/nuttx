# T113-EVB board

Allwinner T113-S3 (also R528) development board support for NuttX.
See ``Documentation/platforms/arm/t113/boards/t113-evb/index.rst`` for
the full description of every board config.

## Configuration notes

### `msc` - USB Mass Storage

Unified config supporting both RAM disk and SPI NAND backends. Board
bringup auto-binds `/dev/mtdblock2` (the 32 MB FTL on SPI NAND) as LUN 0
via `usbmsc_bindlun()`, so the device is exposed without NSH interaction.
To override the auto-bound LUN, use the NSH `usbmsc` command with `-d`,
e.g. `-d /dev/ram0` for the RAM disk or `-d /dev/mtdblock3` (only when
CONFIG_MTD_DHARA=y) for the 32 MB DHARA wear-levelled partition.
`CONFIG_SYSTEM_USBMSC_DEVPATH1` only seeds the manual `usbmsc` NSH
command and has no effect on the boot-time auto-binding.

This config is SMP-enabled. The MUSB OUT path preserves RXPKTRDY when no
class request is queued, so the next EP submit drains the pending packet
instead of dropping USB MSC BOT write data.

The throughput profile intentionally uses 64 write requests with 128 KiB
OUT buffers plus FTL/driver write buffering. This is a performance/debug
configuration for the 128 MiB T113-EVB memory map; size the request pool
down before reusing it on smaller-memory boards.

Exposing `/dev/mtdblock2` (FTL) or `/dev/mtdblock3` (DHARA when enabled)
gives the USB host write access to persistent SPI NAND contents. Keep
those paths for bring-up, validation, or manufacturing flows where
destructive host access is expected.

### `spinand` - SPI NAND standalone test

Unified config with both Winbond GD5F and Macronix MX35 MTD drivers
enabled, auto-detecting the chip at runtime.

### `adc` - ADC/LRADC/keypad testing

Enables GPADC (`/dev/adc0`), LRADC raw (`/dev/lradc0`), and keypad
upper-half (`/dev/kbd0`) for NSH testing. The on-board 5-button
resistor ladder (VOL+/VOL-/MENU/ENTER/HOME) is wired to the LRADC
pin; GPADC0 is routed to the J91 6-pin header for external analog
input. See
`Documentation/platforms/arm/t113/boards/t113-evb/index.rst` for
the NSH test commands.
