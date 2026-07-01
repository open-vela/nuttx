/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_cec.h
 *
 * HDMI CEC (Consumer Electronics Control) driver for RK3576
 ***************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_CEC_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_CEC_H

#include <nuttx/config.h>
#include <stdint.h>

#ifndef __ASSEMBLY__



int rk3576_cec_init(void);
int rk3576_cec_enable(void);
int rk3576_cec_disable(void);
int rk3576_cec_send(const uint8_t *data, int len);
int rk3576_cec_receive(uint8_t *data, int maxlen);

#endif /* __ASSEMBLY__ */

#endif
