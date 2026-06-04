=========
t113-evb
=========

.. tags:: chip:t113, chip:allwinner, arch:cortex-a7

The T113-EVB is an evaluation board based on the Allwinner T113-S3 SoC
(dual Cortex-A7 @ 1.2GHz, 128MB DDR3, SPI NAND flash, USB OTG).

Board Features
==============

- Allwinner T113-S3 (dual Cortex-A7 + HiFi4 DSP)
- 128MB DDR3 (integrated in SoC package)
- SPI NAND flash
- USB 2.0 OTG (High-Speed 480Mbps)
- UART, I2C, SPI, PWM, GPADC, GPIO
- JTAG debug interface

Serial Console
==============

The default serial console is UART0 (PE2/PE3) at 115200 baud.

Configurations
==============

nsh
---

Basic NuttShell on UART0, single-core.  Includes ostest.

nsh_smp
-------

Same as ``nsh`` with dual-core SMP enabled.  Includes ostest.

adb
---

NuttShell on UART0 with USB ADB at 480Mbps.  Run ``adbd &`` then
connect from host::

    $ adb shell hello
    Hello, World!!

usbnsh
------

NuttShell over USB CDC-ACM serial console at 480Mbps.
After boot, connect to ``/dev/ttyACMx`` on the host.
You may need to press ENTER a few times before NSH shows up.

composite
---------

NuttShell on UART0 with USB CDC-ACM + ADB composite device at
480Mbps.  Use ``conn`` to start the composite USB device.

spinand
-------

Minimal NuttShell on UART0 with SPI NAND + LittleFS.  Both the GD5F
(Winbond) and MX35 (Macronix/FORESEE) MTD drivers are enabled so a
single config boots either chip; the driver auto-detects at runtime.
Includes ``nandtest`` and ``dd`` for read speed testing.  No USB.

adc
---

NuttShell on UART0 with GPADC + LRADC + keypad drivers enabled for
NSH testing.  Registers three device nodes:

- ``/dev/adc0``   -- GPADC (General Purpose ADC), 12-bit SAR, 1 MSPS
  max, software-triggered single-shot via ``ANIOC_TRIGGER``.  Pin
  GPADC0 is broken out to the J91 6-pin header on the EVB (pin 3)
  alongside I2C1 and AVCC-3V3 for external analog sensors.

- ``/dev/lradc0`` -- LRADC (Low-Rate ADC), 6-bit IRQ-driven key
  scanner.  Board routes the LRADC pin to a 5-button resistor
  ladder (VOL+, VOL-, MENU, ENTER, HOME).  Idle reads ~63; each
  key maps to a specific raw value (7, 14, 20, 27, 31).

- ``/dev/kbd0``   -- Keypad upper-half, decoded Linux keycodes
  (``KEY_VOLUMEUP``, ``KEY_VOLUMEDOWN``, ``KEY_MENU``, ``KEY_ENTER``,
  ``KEY_HOME``) produced from LRADC raw samples by the board-level
  keypad driver.

Test via::

    nsh> adc -p /dev/adc0   -n 5    # GPADC samples
    nsh> adc -p /dev/lradc0 -n 20   # LRADC raw key values
    nsh> cat /dev/kbd0              # decoded keypad events

The GPADC driver enables ``AUTOCALI_EN`` + ``FIRST_DLY=8`` in the
CTRL register per T113-S3 User Manual v1.1 section 9.8.6.2 so the
hardware discards the first 8 samples after channel activation and
the first user-visible sample is valid.

can
---

NuttShell on UART0 with CAN0 loopback mode enabled.  Run
``cansend`` / ``canrecv`` to test the CAN driver in software
loopback (no external transceiver required).

ce
--

NuttShell on UART0 with Crypto Engine TRNG.  ``/dev/urandom``
is backed by the hardware TRNG.  Test with::

    hexdump -C /dev/urandom -n 32

hstimer
-------

NuttShell on UART0 with High-Speed Timer exposed as
``/dev/timer0``.  Test with::

    timer -d /dev/timer0

msc
---

NuttShell on UART0 with USB Mass Storage.  Unified config supporting
both RAM disk and SPI NAND backends; the device path is selected at
runtime via ``usbmsc -d <devpath>``.  Default LUN is ``/dev/ram0``
(create it first with ``mkrd``); use ``-d /dev/mtdblock2`` to expose
the SPI NAND partition instead.

nsh_secure
----------

NuttShell on UART0, single-core, running in ARM Secure World
(TrustZone).  Includes ostest.

nsh_nonsecure
-------------

NuttShell on UART0, single-core, running in ARM Non-Secure World
(TrustZone).  Requires a secure world firmware to be running first
to configure TrustZone and hand off to non-secure.

ofloader
--------

Offline flasher configuration.  Used to write firmware images to
SPI NAND flash from DDR via ``xfel``.

boot0
-----

Optimized NuttX boot0 SPL (~29KB).  Uses ``INIT_NONE`` to bypass
VFS/ROMFS/FTL, boots directly via MTD bread().  Smallest NuttX-based
SPL variant.

boot0-noos
----------

Bare-metal boot0 SPL (~11KB).  Performs DDR init, reads AP image from
NAND at fixed 1MB offset, and jumps to DDR ``0x40000000``.  No bad
block handling.  Build with standalone Makefile::

    make -C boards/arm/t113/t113-evb/boot0-noos

boot0-nuttx
-----------

NuttX-based boot0 SPL (~132KB).  Runs a minimal NuttX kernel from
144KB SRAM with DDR heap at ``0x40200000``.  Initializes QSPI SPI NAND,
registers MTD partitions (mtd0/mtd1/mtd2), and provides the ``miniboot``
command to load the second-stage firmware.

Boot chain::

    BROM -> boot0-nuttx (SRAM) -> QSPI NAND init -> MTD partitions
      -> boot0> miniboot -> board_boot_image()
      -> skipbad bread from mtd1 -> jump 0x40000000
      -> nsh (DDR)

``board_boot_image()`` iterates mtd1 erase blocks, skips bad blocks
via ``mtd->isbad()``, and reads good blocks sequentially to DDR.
The host-side ``xfel spinand write -k`` uses the same linear-scan
skip-bad rule, so read and write sides are symmetric.

NAND partition layout (128KB erase blocks)::

    mtd0: block  0~7  (1MB) - boot0 SPL copies, read-only
    mtd1: block  8~15 (1MB) - AP image (nsh), skip-bad layout
    mtd2: block 16~end      - user data

Build and flash::

    # Build
    cmake -B build_boot0_nuttx -DBOARD_CONFIG=t113-evb/boot0-nuttx -GNinja
    ninja -C build_boot0_nuttx

    cmake -B build_nsh -DBOARD_CONFIG=t113-evb/nsh -GNinja
    ninja -C build_nsh

    # Flash (from FEL mode)
    xfel spinand erase 0 0x200000
    xfel spinand splwrite 2048 1048576 build_boot0_nuttx/nuttx.bin
    xfel spinand erase 0x100000 0x100000
    xfel spinand write -k 0x100000 build_nsh/nuttx.bin

.. note::

   ``splwrite`` appends the original binary at the 1MB offset.
   The subsequent ``erase + write -k`` overwrites this with the
   actual AP image.

After flashing, WDT reset or power cycle boots to ``boot0>`` shell.
Type ``miniboot`` to load and jump to nsh.

Crash-loop FEL rescue
~~~~~~~~~~~~~~~~~~~~~

The ``boot0-nuttx`` defconfig sets ``CONFIG_T113_FEL_RESCUE_THRESHOLD=1``
to enable the guard for development.  The Kconfig default is ``0``
(disabled) so production firmware does not pay for the feature unless
explicitly opted in.  With THRESHOLD=1, any abnormal reset after a
successful boot (silent WDT, ASSERT, PANIC, HardFault) drops the device
straight into BROM FEL mode so it can always be re-flashed with
``xfel write``.  Cold POR (true power-on when ``VDD_RTC`` was lost) is
exempt and boots normally.

Counter state lives in three RTC ``GP_DATA`` registers (``VDD_RTC``
domain, survives WDT, cleared by true POR):

- ``GP_DATA[0]`` boot marker (USER / FEL / abnormal=0), owned by
  ``t113_systemreset.c``
- ``GP_DATA[1]`` magic ``0xC0DE`` + 16-bit count, owned by
  ``t113_fel.c``
- ``GP_DATA[2]`` last-boot timestamp, owned by ``t113_rtc.c``

To tolerate transient crashes, raise the threshold (e.g. ``=5``
forces FEL only after five short-cycle abnormal resets).  Set ``=0``
to disable the guard entirely (the Kconfig default — production
firmware that does not need the rescue).

xfel reset workflow
^^^^^^^^^^^^^^^^^^^

When you flash a new image with ``xfel`` and want the next boot to
land in ``boot0>`` (rather than getting bounced back to FEL by the
guard), pre-write the USER marker before resetting::

    xfel write32 0x07090100 0x5AA50000   # GP_DATA[0] = USER marker
    xfel reset                            # WDT soft reset

``boot0`` then short-circuits the count path (treats this reset as a
clean ``reboot``) and runs ``miniboot`` normally.  Without the
USER-marker pre-write, ``xfel reset`` is indistinguishable from a
silent WDT crash on the chip side and trips the guard.

bootloader (DEPRECATED)
-----------------------

Use ``boot0-noos`` or ``boot0-nuttx`` instead.

Peripheral Support
==================

  ==================== ==========
  Peripheral           Status
  ==================== ==========
  USB Device (HS)      Working
  SMP (dual-core)      Working
  SPI NAND + DMA       Working
  CAN Controller       Working
  Crypto Engine (TRNG) Working
  High-Speed Timer     Working
  I2C                  Working
  SPI                  Working
  GPIO                 Working
  RTC                  Working
  PWM                  Working
  GPADC                Working
  LRADC                Working
  Watchdog             Working
  Tickless Timer       Working
  ==================== ==========
