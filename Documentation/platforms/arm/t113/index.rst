==============
Allwinner T113
==============

The Allwinner T113-S3 is a dual-core ARM Cortex-A7 SoC with integrated
128MB DDR3 memory.  It is also marketed as R528 and shares the same die.

Supported SoC variants:

- **T113-S3**: Dual Cortex-A7 @ 1.2GHz + HiFi4 DSP, 128MB DDR3

Key peripherals:

- USB 2.0 OTG with High-Speed (480Mbps) MUSB controller
- SPI NAND flash controller
- UART, I2C, SPI, PWM, GPADC, LRADC, GPIO
- GIC-400 interrupt controller
- ARM Generic Timer

Technical References
====================

.. toctree::
   :maxdepth: 1

   brom-boot0-loading

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
