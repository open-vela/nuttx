/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_can.h
 *
 * CAN controller driver for RK3576 (Bosch M_CAN compatible)
 ***************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_CAN_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_CAN_H

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef __ASSEMBLY__



#define RK3576_CAN0              0
#define RK3576_CAN1              1

struct rk3576_can_msg_s
{
  uint32_t id;
  uint8_t dlc;
  bool extended;
  bool rtr;
  uint8_t data[8];
};

int rk3576_can_init(int can);
int rk3576_can_set_bitrate(int can, int bitrate);
int rk3576_can_send(int can, const struct rk3576_can_msg_s *msg);
int rk3576_can_receive(int can, struct rk3576_can_msg_s *msg);
int rk3576_can_enable(int can);
int rk3576_can_disable(int can);
uint32_t rk3576_can_get_status(int can);

#endif /* __ASSEMBLY__ */

#endif
