#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_SPIIO_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_SPIIO_H

#define LS2K0300_SPIIO_CR1           0x00
#define LS2K0300_SPIIO_CR2           0x04
#define LS2K0300_SPIIO_CR3           0x08
#define LS2K0300_SPIIO_CR4           0x0c
#define LS2K0300_SPIIO_IER           0x10
#define LS2K0300_SPIIO_SR1           0x14
#define LS2K0300_SPIIO_SR2           0x18
#define LS2K0300_SPIIO_CFG1          0x20
#define LS2K0300_SPIIO_CFG2          0x24
#define LS2K0300_SPIIO_CFG3          0x28
#define LS2K0300_SPIIO_CRC1          0x30
#define LS2K0300_SPIIO_CRC2          0x34
#define LS2K0300_SPIIO_DR            0x40

#define CR1_SPE                     (1 << 0)
#define CR1_CSTART                  (1 << 1)
#define CR1_AUTOSUS                 (1 << 2)
#define CR1_SSREV                   (1 << 8)

#define SR1_RXA                     (1 << 0)
#define SR1_TXA                     (1 << 1)
#define SR1_EOT                     (1 << 15)

#define CFG1_CPOL                   (1 << 0)
#define CFG1_CPHA                   (1 << 1)
#define CFG1_LSBFRST                (1 << 7)
#define CFG1_DSIZE_SHIFT            8
#define CFG1_DSIZE_MASK             0x1f00

#define CFG2_BRINT_SHIFT            8
#define CFG2_BRINT_MASK             0xff00
#define CFG2_BRDEC_MASK             0xfc

#define CFG3_MSTR                   (1 << 0)
#define CFG3_DIOSWP                 (1 << 1)
#define CFG3_DIE                    (1 << 2)
#define CFG3_DOE                    (1 << 3)
#define CFG3_SSMODE_MASK            0x300

#endif
