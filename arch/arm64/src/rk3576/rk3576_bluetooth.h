/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_bluetooth.h
 *
 * Bluetooth HCI transport driver for RK3576
 * UART-based HCI for built-in BLE controller
 ***************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_BLUETOOTH_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_BLUETOOTH_H

#include <nuttx/config.h>
#include <stdint.h>

#ifndef __ASSEMBLY__



int rk3576_bluetooth_init(int uart);
int rk3576_bluetooth_send(uint8_t type, const uint8_t *data, int len);
int rk3576_bluetooth_read(uint8_t *buf, int maxlen, int timeout_ms);
void rk3576_bluetooth_enable(bool enable);

#endif /* __ASSEMBLY__ */

#endif
