#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_SPI_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_SPI_H

#define LS2K0300_SPI_SPCR            0x00
#define LS2K0300_SPI_SPSR            0x01
#define LS2K0300_SPI_FIFO            0x02
#define LS2K0300_SPI_SPER            0x03
#define LS2K0300_SPI_PARA            0x04
#define LS2K0300_SPI_SPCS            0x04
#define LS2K0300_SPI_SFCS            0x05
#define LS2K0300_SPI_TIMI            0x06

#define SPCR_SPE                    0x40
#define SPSR_SPIF                   0x80
#define SPSR_WCOL                   0x40
#define PARA_MEM_EN                 0x01

#endif
