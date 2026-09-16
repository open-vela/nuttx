#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_WDT_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_WDT_H

#define LS2K0300_WDT_RST_CTR         0x00
#define LS2K0300_WDT_CR              0x04
#define LS2K0300_WDT_TIMER           0x08

#define WDT_RST_CTR_ENABLE          (1 << 1)
#define WDT_CR_RESET                0x01

#endif
