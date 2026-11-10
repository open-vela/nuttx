Forlinx OK8MP Cortex-M7
======================

This board port targets the Cortex-M7 auxiliary core of the NXP i.MX8M Plus
on the Forlinx OK8MP development board.  It reuses the existing mx8mp
architecture support.  The basic configuration provides the UART4 NSH
console, SysTick scheduling and a DDR-loaded image.

The board's U-Boot must initialize DDR and allow the M7 to access UART4
before starting the auxiliary core.  The documented setup starts the M7
from U-Boot without starting A53 Linux.

UART4 console
=============

Connect the board's M7 UART4 console to the host using a suitable serial
adapter.  Use 115200 baud, 8 data bits, no parity, 1 stop bit and no hardware
flow control.  The UART4 RXD and TXD pads are configured by the existing
mx8mp serial driver using the pin definitions in include/board.h.

Memory layout
=============

  0x80000000 - 0x800fffff: vectors, code, constants and initial data image
  0x80100000 - 0x801fffff: writable data, BSS, stacks and heap

Both regions are DDR.  The linker script calls them flash and sram to match
the architecture's startup convention; no flash programming is involved.
CONFIG_RAM_START and CONFIG_RAM_SIZE must match the second region.

If an A53 operating system is also started, these regions and UART4 must be
reserved for the M7 by the system's memory and peripheral configuration.
The basic NSH configuration does not set up concurrent A53 Linux operation.

Building
========

Place the matching openvela nuttx-apps repository beside nuttx as apps.
Install the normal NuttX host build dependencies, kconfiglib and an
Arm GNU Toolchain that provides arm-none-eabi-gcc.  For example:

  git clone -b dev https://github.com/open-vela/nuttx.git nuttx
  git clone -b dev https://github.com/open-vela/nuttx-apps.git apps
  cd nuttx
  ./tools/configure.sh -l ok8mp-m7:nsh
  make -j4

Use configure.sh -m instead of -l on macOS.  The configuration enables the
standard hello application as a small check of builtin application startup.
It does not require project-specific Apps, packages, sensors or network
components.  The build produces nuttx (ELF) and nuttx.bin (raw image).

The verified host setup uses Arm GNU Toolchain 14.3.Rel1 and kconfiglib
14.1.0.  With GCC 14, the current upstream usrsock header emits an implicit
declaration diagnostic when networking is disabled.  To retain that
diagnostic as a warning for this configuration, build with:

  make -j4 EXTRAFLAGS=-Wno-error=implicit-function-declaration

macOS also needs flock on PATH for the Apps archive rules.  For example,
install flock with Homebrew, or use an existing util-linux installation.

Loading and starting
====================

Copy nuttx.bin to the FAT partition used by the board's U-Boot.  A distinct
name such as nuttx-ok8mp-basic.bin permits keeping another image on the
partition.  The tested downstream boot setup uses eMMC device 2, partition 1:

  mmc dev 2
  fatload mmc 2:1 0x80000000 nuttx-ok8mp-basic.bin
  md.l 0x80000000 2
  bootaux 0x80000000

The first word displayed by md.l is the initial stack pointer and must be
inside the writable DDR region.  The second word is the Thumb reset-handler
address and must point into the loaded image with its low bit set.

On the UART4 console, check the NSH prompt and run:

  help
  uname -a
  free
  hello
  sleep 1

Expected results are an interactive nsh> prompt, memory statistics within
the configured 1 MiB RAM region, Hello, World!! from hello, and a return to
the prompt after sleep.  Exact memory statistics depend on build options.

Scope and validation
====================

This initial BSP configuration covers DDR boot and the UART4 console.
Ethernet, CAN, I2C devices, LEDs, buttons, reset and application integration
are separate extensions and are not enabled by this configuration.

The existing downstream OK8MP port has hardware records for U-Boot bootaux,
UART4 NSH interaction, free and sleep.  Those records describe downstream
images.  Build and runtime results for this extracted upstream configuration
are recorded separately in its pull request.
