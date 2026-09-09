/****************************************************************************
 * arch/arm/src/bk7258/hardware/bk7258_memorymap.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __VENDOR_BEKEN_CHIPS_BK7258_HARDWARE_BK7258_MEMORYMAP_H
#define __VENDOR_BEKEN_CHIPS_BK7258_HARDWARE_BK7258_MEMORYMAP_H

/* Secure aliases used by the current BK7258 AP SPE image. */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BK7258_FLASH_BASE          0x02000000u
#define BK7258_AP_FLASH_BASE       0x02150000u
#define BK7258_AP_FLASH_SIZE       0x003d0000u

#define BK7258_AP_RAM_BASE         0x28010000u
#define BK7258_AP_RAM_SIZE         0x00054000u
#define BK7258_AP_RAM_END          (BK7258_AP_RAM_BASE + BK7258_AP_RAM_SIZE)
#define BK7258_DTCM_CPU_ID         0x20000000u

#define BK7258_SYSCTRL_BASE        0x44010000u
#define BK7258_AON_GPIO_BASE       0x44000400u
#define BK7258_UART1_BASE          0x45830000u
#define BK7258_MBOX0_BASE          0x41000000u
#define BK7258_QSPI0_BASE          0x46040000u

/* QSPI1 controller (SOC_QSPI1_REG_BASE,
 * bk_avdk_smp/ap/include/soc/bk7258_ap/reg_base.h:118).  Used for GC9D01
 * LCD panel access on this board: per the board schematic
 * (AIDK AI toy development board schematic, sheet 2/6 main chip pin
 * table + sheet 5/6
 * CN5 single-screen connector), the panel's LCD_QSPI_CLK/CS/D0/D1 net
 * labels are wired to chip pins 35/34/33/32, i.e. P2/P3/P4/P5
 * (GPIO2/GPIO3/GPIO4/GPIO5).  The chip's pinmux table
 * (bk_avdk_smp/ap/middleware/soc/bk7258_ap/soc/gpio_map.h GPIO_DEV_MAP)
 * shows those exact 4 GPIOs have QSPI1_CLK/QSPI1_CSN/QSPI1_IO0/QSPI1_IO1
 * at function-select index 6 -- QSPI1 has no other selectable pin group,
 * these 4 lines are hard-wired inside the SoC to this specific one, they
 * cannot be split across different GPIOs. Note the schematic's literal
 * net names ("D0/D1" etc.) do NOT match the SoC's real hardware role for
 * these pins (CLK/CSN, not data lines) -- PCB net labels are the
 * designer's arbitrary connector-pin numbering convention, not a
 * statement about which SoC peripheral function is multiplexed onto that
 * pad; the real role is only knowable from gpio_map.h.  This driver was
 * originally (incorrectly) built against QSPI0 (GPIO22/23/24/25,
 * function index 3) because bk_avdk_smp's generic
 * projects/spi_lcd_example reference config uses qspi_id=0 -- but that
 * reference project's own schematic (not available in this repo) wires
 * its LCD to a different GPIO group than this board's CN5 connector.
 * See bk7258_qspi.c file header for the full corrected-vs-original
 * comparison.
 */
#define BK7258_QSPI1_BASE          0x46060000u

/* Hardware I2C1 controller (SOC_I2C1_REG_BASE,
 * bk_avdk_smp/ap/include/soc/bk7258/reg_base.h:92).  Used for GC2145
 * register access on this board: per the board schematic
 * (AIDK AI toy development board schematic, sheet 4/6
 * "DVP/SENSOR/NAND/MOTOR/KEY"),
 * GC2145's SCL/SDA are wired to IIC1_SCL/IIC1_SDA (with 4.7K pull-ups to
 * VDDGPIO), which the chip's GPIO pinmux table
 * (bk_avdk_smp/ap/middleware/soc/bk7258_ap/soc/gpio_map.h
 * GPIO_I2C1_MAP_TABLE) routes to GPIO42/GPIO43 (the second of two
 * selectable I2C1 pin groups; the other group, GPIO0/GPIO1, is what this
 * driver's previous software-simulated-I2C implementation incorrectly
 * assumed and is actually wired to UART1_TXD/UART1_RXD on this board --
 * see bk7258_i2c1.c file header for the full corrected-vs-original
 * comparison).
 */
#define BK7258_I2C1_BASE           0x45860000u

/* PSRAM data region.  YUV_BUF hardware uses a fixed base address here
 * (SOC_PSRAM_DATA_BASE, ap/include/soc/bk7258/reg_base.h:43) as its line
 * buffer in YUV direct-capture mode (yuv_buf_hal_set_yuv_mode_config()
 * hard-codes em_base_addr/emr_base_addr to this address, not an
 * application-supplied one).  Size: the actual PSRAM chip on this board
 * has been observed at boot to be 16MB (serial log:
 * "psram type(16MB) not match CONFIG_PSRAM_CAPACITY 0X00800000"), so
 * 0x01000000 (16MB) is used here rather than the chip-family maximum
 * SOC_PSRAM_DATA_SIZE (0x4000000, 64MB) -- both are valid MPU region
 * sizes (power-of-two, naturally aligned to SOC_PSRAM_DATA_BASE), 16MB
 * matches the verified real hardware capacity.
 */
#define BK7258_PSRAM_BASE          0x60000000u
#define BK7258_PSRAM_SIZE          0x01000000u

/* Analog audio (AUD) controller: internal 16-bit mono DAC + analog MIC
 * ADC block (SOC_AUD_REG_BASE,
 * bk_avdk_smp/ap/include/soc/bk7258/reg_base.h:99).  This board uses the
 * chip's *internal* DAC only -- per the board schematic
 * (AIDK AI toy development board schematic: sheet 2/6 main chip pin
 * table lists
 * AUDLN/AUDLP at physical pins 24/25, and the audio sheet shows AUDLP
 * feeding U8 pin 4 (IN+) through C48), the differential AUDLP/AUDLN pair
 * drives an HT6872 class-D power amplifier directly.  There is no I2S
 * audio codec on this board, so no I2S register block is needed:
 * BK7258's two selectable I2S1 pin groups (gpio_map.h
 * GPIO_I2S_MAP_TABLE) are GPIO6-9 and GPIO40-43, and both are already
 * taken here -- GPIO9 is the vibration motor's PWM3 output
 * (bk7258_pwm.c BK7258_MOTOR_PWM_GPIO) and GPIO42/43 carry the GC2145
 * camera's control bus, driven in software by
 * bk7258_gc2145_i2c_bitbang.c.
 *
 * Note the AUD block lives inside the 0x40000000..0x5fffffff window that
 * bk7258_start.c's last MPU region already maps as DEVICE/RWRW/XN, so no
 * new MPU region is required to reach it.
 */
#define BK7258_AUD_BASE            0x47800000u

#define BK7258_NVIC_BASE           0xe000e100u
#define BK7258_SCB_BASE            0xe000ed00u
#define BK7258_SAU_BASE            0xe000edd0u

#endif
