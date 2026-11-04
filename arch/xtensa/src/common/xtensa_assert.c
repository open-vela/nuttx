/****************************************************************************
 * arch/xtensa/src/common/xtensa_assert.c
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

#include <stdint.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/board.h>
#include <nuttx/syslog/syslog.h>

#include <arch/board/board.h>

#include "sched/sched.h"
#include "xtensa.h"

/* [V646] PROBE: ROM console, bypasses the /dev/kmsg ring buffer so a
 * panic dump cannot flush it away. */
extern int esp_rom_printf(const char *fmt, ...);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_panic
 *
 * Description:
 *   PANIC if an unhandled exception is received:
 *
 *   - NMI exception
 *   - Debug exception
 *   - Double exception
 *   - Kernel exception
 *   - Co-processor exception
 *   - High priority level2-6 Exception.
 *
 * Input Parameters:
 *   xcptcode - Identifies the unhandled exception (see include/esp32/irq.h)
 *   regs - The register save are at the time of the interrupt.
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

void xtensa_panic(int xptcode, uint32_t *regs)
{
  struct tcb_s *tcb = g_running_task;

  if (tcb != NULL)
    {
      tcb->xcp.regs = regs;
    }

  up_set_interrupt_context(true);

  /* We get here when a un-dispatch-able, irrecoverable exception occurs */

  board_autoled_on(LED_ASSERTION);
  /* [V886] Show the REAL faulting PC.
   *
   * up_dump_register() below prints the panic handler's OWN live
   * registers -- that is why every dump so far has shown
   * PC=up_saveusercontext+0x19 and nothing else (documented at v645 in
   * the comment further down this file).  For a kernel exception the
   * faulting instruction is in EPC1 and the bad address in EXCVADDR;
   * read both before any call can clobber them and push them straight
   * out the ROM console, which bypasses the kmsg ring buffer the panic
   * is about to flush away.
   *
   * rsr.epc1 is already used elsewhere in this tree:
   *   esp-hal-3rdparty/.../panic_arch.c:80 */
  {
    uint32_t v886_epc1;
    uint32_t v886_excvaddr;
    uint32_t v886_cause;

    __asm__ __volatile__("rsr.epc1 %0"     : "=a"(v886_epc1));
    __asm__ __volatile__("rsr.excvaddr %0" : "=a"(v886_excvaddr));
    __asm__ __volatile__("rsr.exccause %0" : "=a"(v886_cause));

    esp_rom_printf("[V886] PANIC xpt=%d cause=%08x epc1=%08x vaddr=%08x\n",
                   xptcode,
                   (unsigned)v886_cause,
                   (unsigned)v886_epc1,
                   (unsigned)v886_excvaddr);

    if (regs != NULL)
      {
        esp_rom_printf("[V886] PANIC frame r0=%08x r1=%08x "
                       "r2=%08x r3=%08x\n",
                       (unsigned)regs[0], (unsigned)regs[1],
                       (unsigned)regs[2], (unsigned)regs[3]);
        /* [V887] Is A1 still inside this task's stack?
         *
         * v886 caught a StoreProhibited (EXCCAUSE 29) at 0xfffffff1 while
         * _window_overflow12 was spilling registers.  A spill writes
         * below A1, so that address means A1 was about 0x11 -- a wrecked
         * stack pointer rather than a merely full one.  Printing the
         * stack base next to A1 turns the next crash into a yes/no:
         * if A1 is nowhere near [base, base+size] the stack context is
         * being destroyed; if it is just past the end, it is only sizing.
         *
         * Only stack_alloc_ptr is read -- that field is already used a
         * few lines further down in this file, so it is safe to touch
         * here.  No task-name lookup: get_task_name() can sleep. */
        {
          FAR struct tcb_s *v887_tcb = g_running_task;

          if (v887_tcb != NULL)
            {
              esp_rom_printf("[V887] base=%08x sp=%08x\n",
                             (unsigned)(uintptr_t)v887_tcb->stack_alloc_ptr,
                             (unsigned)regs[3]);
            }
        }

      }
  }


  /* Flush any buffered SYSLOG data (from prior to the panic) */

  syslog_flush();

  _alert("Unhandled Exception %d task: %s\n", xptcode,
         get_task_name(running_task()));

  PANIC_WITH_REGS("panic", regs);  /* Should not return */
  for (; ; );
}

/****************************************************************************
 * Name: xtensa_user
 *
 * Description:
 *   PANIC if certain User Exceptions are received.  All values for EXCCAUSE
 *   are listed below (not all generate PANICs):
 *
 *   0  IllegalInstructionCause
 *      Illegal instruction
 *   1  SyscallCause
 *      SYSCALL instruction
 *   2  InstructionFetchErrorCause
 *      Processor internal physical address or data error during instruction
 *      fetch.
 *   3  LoadStoreErrorCause
 *      Processor internal physical address or data error during load or
 *      store.
 *   4  Level1InterruptCause
 *      Level-1 interrupt as indicated by set level-1 bits in the INTERRUPT
 *      register.
 *   5  AllocaCause
 *      MOVSP instruction, if caller's registers are not in the register
 *      file.
 *   6  IntegerDivideByZeroCause
 *      QUOS, QUOU, REMS, or REMU divisor operand is zero.
 *   7  PCValueErrorCause Next PC Value Illegal
 *   8  PrivilegedCause
 *      Attempt to execute a privileged operation when CRING != 0
 *   9  LoadStoreAlignmentCause
 *      Load or store to an unaligned address.
 *   10..11 Reserved for Cadence
 *   12 InstrPIFDataErrorCause
 *      PIF data error during instruction fetch.
 *   13 LoadStorePIFDataErrorCause
 *      Synchronous PIF data error during LoadStore access.
 *   14 InstrPIFAddrErrorCause
 *      PIF address error during instruction fetch.
 *   15 LoadStorePIFAddrErrorCause
 *      Synchronous PIF address error during LoadStore access.
 *   16 InstTLBMissCause
 *      Error during Instruction TLB refill
 *   17 InstTLBMultiHitCause
 *      Multiple instruction TLB entries matched
 *   18 InstFetchPrivilegeCause
 *      An instruction fetch referenced a virtual address at a ring leve
 *      less than CRING.
 *   19 Reserved for Cadence
 *   20 InstFetchProhibitedCause
 *       An instruction fetch referenced a page mapped with an attribute
 *      that does not permit instruction fetch.
 *   21..23 Reserved for Cadence
 *   24 LoadStoreTLBMissCause
 *      Error during TLB refill for a load or store.
 *   25 LoadStoreTLBMultiHitCause
 *      Multiple TLB entries matched for a load or store.
 *   26 LoadStorePrivilegeCause
 *      A load or store referenced a virtual address at a ring level less
 *      than CRING.
 *   27 Reserved for Cadence
 *   28 LoadProhibitedCause
 *      A load referenced a page mapped with an attribute that does not
 *      permit loads.
 *   29 StoreProhibitedCause
 *     A store referenced a page mapped with an attribute that does not
 *     permit stores.
 *   30..31 Reserved for Cadence
 *   32..39 CoprocessornDisabled
 *     Coprocessor n instruction when cpn disabled. n varies 0..7 as the
 *     cause varies 32..39.
 *   40..63 Reserved
 *
 * Input Parameters:
 *   exccause - Identifies the EXCCAUSE of the user exception
 *   regs - The register save are at the time of the interrupt.
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

void xtensa_user_panic(int exccause, uint32_t *regs)
{
  struct tcb_s *tcb = g_running_task;

  if (tcb != NULL)
    {
      tcb->xcp.regs = regs;
    }

  up_set_interrupt_context(true);

  /* We get here when a un-dispatch-able, irrecoverable exception occurs */

  board_autoled_on(LED_ASSERTION);

  /* [V646] PROBE: dump the *exception* register frame to the ROM console.
   * up_dump_register() ends up printing the panic handler's own live
   * registers (v645: PC=up_saveusercontext+0x19, A0=_assert+0x38), so the
   * faulting PC/A0 have never been visible.  regs[] is the frame the user
   * exception vector saved, so r[0]=PC and r[2]=A0 are the real ones.
   * Index map: r[0]=PC r[1]=PS r[2]=A0 .. r[17]=A15 r[18]=SAR
   *            r[19]=EXCCAUSE r[20]=EXCVADDR   (r[N+2] == aN)
   */

  {
    const uint32_t *r = (const uint32_t *)(uintptr_t)regs;

    esp_rom_printf("[V646] UP cause=%04x regs=%08x rtcb=%08x sp=%08x\n",
                   exccause, (unsigned)(uintptr_t)regs,
                   (unsigned)(uintptr_t)g_running_task,
                   (unsigned)(uintptr_t)__builtin_frame_address(0));
    esp_rom_printf("[V646] UP pc=%08x ps=%08x a0=%08x a1=%08x\n",
                   (unsigned)r[0], (unsigned)r[1],
                   (unsigned)r[2], (unsigned)r[3]);
    esp_rom_printf("[V646] UP a2=%08x a3=%08x a4=%08x a5=%08x\n",
                   (unsigned)r[4], (unsigned)r[5],
                   (unsigned)r[6], (unsigned)r[7]);
    esp_rom_printf("[V646] UP a6=%08x a7=%08x a8=%08x a9=%08x\n",
                   (unsigned)r[8], (unsigned)r[9],
                   (unsigned)r[10], (unsigned)r[11]);
    esp_rom_printf("[V646] UP a10=%08x a11=%08x a12=%08x a13=%08x\n",
                   (unsigned)r[12], (unsigned)r[13],
                   (unsigned)r[14], (unsigned)r[15]);
    esp_rom_printf("[V646] UP a14=%08x a15=%08x sar=%08x vaddr=%08x\n",
                   (unsigned)r[16], (unsigned)r[17],
                   (unsigned)r[18], (unsigned)r[20]);
  }

  /* [V648-D] Cross-check the two ways of finding "the current task":
   * this_task() reads the per-CPU TCB out of MISC0
   * (CONFIG_XTENSA_PERCPU_TCB_IN_MISC0, see arch/xtensa/include/irq.h:167),
   * while the dump below uses g_running_task.  MISC0 is only refreshed from
   * sched_addreadytorun/sched_removereadytorun/sched_mergepending/nx_start/
   * esp32s3_cpustart, so if they ever disagree it means MISC0 went stale.
   */
  {
    struct tcb_s *mt = this_task();
    struct tcb_s *st = g_running_task;

    

    if (mt != NULL && mt->stack_alloc_ptr != NULL)
      {
        const unsigned *w = (const unsigned *)mt->stack_alloc_ptr;

        
      }

    if (st != NULL && st->stack_alloc_ptr != NULL)
      {
        const unsigned *w = (const unsigned *)st->stack_alloc_ptr;

        
      }
  }

  /* Flush any buffered SYSLOG data (from prior to the error) */

  syslog_flush();

  _alert("User Exception: EXCCAUSE=%04x task: %s\n",
         exccause, get_task_name(running_task()));

  PANIC_WITH_REGS("user panic", regs); /* Should not return */
  for (; ; );
}
