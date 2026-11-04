/****************************************************************************
 * arch/loongarch/src/common/loongarch_fork.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <arch/irq.h>
#include "sched/sched.h"

#include "loongarch_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fork_s
{
  uintreg_t sp;
  uintreg_t ra;
  uintreg_t regs[32];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fork
 *
 * Description:
 *   The fork() function has the same effect as posix fork(), except that the
 *   behavior is undefined if the process created by fork() either modifies
 *   any data other than a variable of type pid_t used to store the return
 *   value from fork(), or returns from the function in which fork() was
 *   called, or calls any other function before successfully calling _exit()
 *   or one of the exec family of functions.
 *
 *   The overall sequence is:
 *
 *   1) User code calls fork().  fork() collects context information and
 *      transfers control up loongarch_fork().
 *   2) loongarch_fork() and calls nxtask_setup_fork().
 *   3) nxtask_setup_fork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) loongarch_fork() provides any additional operating context.
 *      loongarch_fork() must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) loongarch_fork() then calls nxtask_start_fork()
 *   6) nxtask_start_fork() then executes the child thread.
 *
 * nxtask_abort_fork() may be called if an error occurs between steps 3 and
 * 6.
 *
 * Input Parameters:
 *   context - Caller context information saved by fork()
 *
 * Returned Value:
 *   Upon successful completion, fork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

pid_t loongarch_fork(const struct fork_s *context)
{
  struct tcb_s *parent = this_task();
  struct tcb_s *child;
  uintptr_t newsp;
  uintptr_t newtop;
  uintptr_t stacktop;
  uintptr_t stackutil;

  /* Allocate and initialize a TCB for the child task. */

  child = nxtask_setup_fork((start_t)context->ra);
  if (!child)
    {
      serr("ERROR: nxtask_setup_fork failed\n");
      return (pid_t)ERROR;
    }

  /* How much of the parent's stack was utilized?  The LoongArch uses
   * a push-down stack so that the current stack pointer should
   * be lower than the initial, adjusted stack pointer.  The
   * stack usage should be the difference between those two.
   */

  stacktop = (uintptr_t)parent->stack_base_ptr +
                       parent->adj_stack_size;
  DEBUGASSERT(stacktop > context->sp);
  stackutil = stacktop - context->sp;

  /* Make some feeble effort to preserve the stack contents.  This is
   * feeble because the stack surely contains invalid pointers and other
   * content that will not work in the child context.  However, if the
   * user follows all of the caveats of fork() usage, even this feeble
   * effort is overkill.
   */

  newtop = (uintptr_t)child->stack_base_ptr +
                     child->adj_stack_size;
  newsp = newtop - stackutil;
  memcpy((void *)newsp, (const void *)context->sp, stackutil);

  /* Update the stack pointer, frame pointer, and volatile registers.  When
   * the child TCB was initialized, all of the values were set to zero.
   * up_initial_state() altered a few values, but the return value in R0
   * should be cleared to zero, providing the indication to the newly started
   * child thread.
   */

  child->xcp.regs             = (void *)(newsp - XCPTCONTEXT_SIZE);

  child->xcp.regs[REG_R4]     = 0;  /* Return value = 0 for child */
  child->xcp.regs[REG_SP]     = newsp;
  child->xcp.regs[REG_RA]     = context->ra;
  child->xcp.regs[REG_EPC]    = context->ra;

  /* Copy callee-saved registers */

  child->xcp.regs[REG_R22]    = context->regs[0];  /* S0 */
  child->xcp.regs[REG_R23]    = context->regs[1];  /* S1 */
  child->xcp.regs[REG_R24]    = context->regs[2];  /* S2 */
  child->xcp.regs[REG_R25]    = context->regs[3];  /* S3 */
  child->xcp.regs[REG_R26]    = context->regs[4];  /* S4 */
  child->xcp.regs[REG_R27]    = context->regs[5];  /* S5 */
  child->xcp.regs[REG_R28]    = context->regs[6];  /* S6 */
  child->xcp.regs[REG_R29]    = context->regs[7];  /* S7 */
  child->xcp.regs[REG_R30]    = context->regs[8];  /* S8 */
  child->xcp.regs[REG_R22]    = context->regs[9];  /* FP (R22) */

  child->xcp.regs[REG_INT_CTX] = loongarch_get_newintctx();

  /* And, finally, start the child task.  On a failure, nxtask_start_fork()
   * will discard the TCB by calling nxtask_abort_fork().
   */

  return nxtask_start_fork(child);
}
