#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_PWM_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_PWM_H

#define LS2K0300_PWM_LOW_BUFFER      0x004
#define LS2K0300_PWM_FULL_BUFFER     0x008
#define LS2K0300_PWM_CTRL            0x00c

#define PWM_CTRL_EN                 (1 << 0)
#define PWM_CTRL_OE                 (1 << 3)
#define PWM_CTRL_SINGLE             (1 << 4)
#define PWM_CTRL_INTE               (1 << 5)
#define PWM_CTRL_INT                (1 << 6)
#define PWM_CTRL_RST                (1 << 7)
#define PWM_CTRL_CAPTE              (1 << 8)
#define PWM_CTRL_INVERT             (1 << 9)
#define PWM_CTRL_DZONE              (1 << 10)

#endif
