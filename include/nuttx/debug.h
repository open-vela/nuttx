/* nuttx/debug.h compatibility shim
 *
 * The openvela fork keeps debug macros in <debug.h> and DEBUGASSERT in
 * <assert.h>. Upstream NuttX sources (e.g. the ESP32-P4 port) include
 * <nuttx/debug.h>; map it to the fork's headers.
 */

#ifndef __INCLUDE_NUTTX_DEBUG_H
#define __INCLUDE_NUTTX_DEBUG_H

#include <assert.h>
#include <debug.h>

#endif /* __INCLUDE_NUTTX_DEBUG_H */
