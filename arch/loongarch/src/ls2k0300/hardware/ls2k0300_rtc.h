#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_RTC_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_RTC_H

#define LS2K0300_RTC_TOY_TRIM_REG       0x20
#define LS2K0300_RTC_TOY_WRITE0_REG     0x24
#define LS2K0300_RTC_TOY_WRITE1_REG     0x28
#define LS2K0300_RTC_TOY_READ0_REG      0x2c
#define LS2K0300_RTC_TOY_READ1_REG      0x30
#define LS2K0300_RTC_TOY_MATCH0_REG     0x34
#define LS2K0300_RTC_TOY_MATCH1_REG     0x38
#define LS2K0300_RTC_TOY_MATCH2_REG     0x3c
#define LS2K0300_RTC_CTRL_REG           0x40
#define LS2K0300_RTC_TRIM_REG           0x60
#define LS2K0300_RTC_WRITE0_REG         0x64
#define LS2K0300_RTC_READ0_REG          0x68
#define LS2K0300_RTC_MATCH0_REG         0x6c
#define LS2K0300_RTC_MATCH1_REG         0x70
#define LS2K0300_RTC_MATCH2_REG         0x74

#define RTC_TOY_MON_MASK               0x3f
#define RTC_TOY_DAY_MASK               0x1f
#define RTC_TOY_HOUR_MASK              0x1f
#define RTC_TOY_MIN_MASK               0x3f
#define RTC_TOY_SEC_MASK               0x3f

#define RTC_TOY_MON_SHIFT              26
#define RTC_TOY_DAY_SHIFT              21
#define RTC_TOY_HOUR_SHIFT             16
#define RTC_TOY_MIN_SHIFT              10
#define RTC_TOY_SEC_SHIFT              4

#define RTC_TOY_MATCH_YEAR_SHIFT       26
#define RTC_TOY_MATCH_MON_SHIFT        22
#define RTC_TOY_MATCH_DAY_SHIFT        17
#define RTC_TOY_MATCH_HOUR_SHIFT       12
#define RTC_TOY_MATCH_MIN_SHIFT        6
#define RTC_TOY_MATCH_SEC_SHIFT        0

#define RTC_TOY_MATCH_YEAR_MASK        0x3f
#define RTC_TOY_MATCH_MON_MASK         0xf
#define RTC_TOY_MATCH_DAY_MASK         0x1f
#define RTC_TOY_MATCH_HOUR_MASK        0x1f
#define RTC_TOY_MATCH_MIN_MASK         0x3f
#define RTC_TOY_MATCH_SEC_MASK         0x3f

#define RTC_CTRL_RTC_ENABLE_BIT        (1 << 13)
#define RTC_CTRL_TOY_ENABLE_BIT        (1 << 11)
#define RTC_CTRL_OSC_ENABLE_BIT        (1 << 8)

#endif
