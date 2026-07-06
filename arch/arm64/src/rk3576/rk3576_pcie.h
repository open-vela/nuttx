/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_pcie.h
 *
 * PCIe controller driver for RK3576
 ***************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_PCIE_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_PCIE_H

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef __ASSEMBLY__



#define RK3576_PCIE0             0
#define RK3576_PCIE1             1

int rk3576_pcie_init(int pcie);
int rk3576_pcie_enable(int pcie);
int rk3576_pcie_disable(int pcie);
bool rk3576_pcie_is_link_up(int pcie);
uint32_t rk3576_pcie_read_config(int pcie, int bus, int dev, int func, int reg);
void rk3576_pcie_write_config(int pcie, int bus, int dev, int func, int reg, uint32_t val);

#endif /* __ASSEMBLY__ */

#endif
