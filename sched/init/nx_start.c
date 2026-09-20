/****************************************************************************
 * sched/init/nx_start.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/compiler.h>
#include <nuttx/sched.h>
#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>
#include <nuttx/mm/iob.h>
#include <nuttx/mm/kmap.h>
#include <nuttx/mm/mm.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched_note.h>
#include <nuttx/trace.h>
#include <nuttx/binfmt/binfmt.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/init.h>

#ifdef CONFIG_SCHED_PERF_EVENTS
#  include <nuttx/perf.h>
#endif

#include "task/task.h"
#include "sched/sched.h"
#include "signal/signal.h"
#include "semaphore/semaphore.h"
#include "mqueue/mqueue.h"
#include "mqueue/msg.h"
#include "clock/clock.h"
#include "timer/timer.h"
#include "irq/irq.h"
#include "group/group.h"
#include "init/init.h"
#include "instrument/instrument.h"
#include "tls/tls.h"
#include "wqueue/wqueue.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is an array of task control block (TCB) for the IDLE thread of each
 * CPU.  For the non-SMP case, this is a a single TCB; For the SMP case,
 * there is one TCB per CPU.  NOTE: The system boots on CPU0 into the IDLE
 * task.  The IDLE task later starts the other CPUs and spawns the user
 * initialization task.  That user initialization task is responsible for
 * bringing up the rest of the system.
 */

DEFINE_PER_CPU_BSS(struct tcb_s, g_idletcb);

/* Task Lists ***************************************************************/

/* The state of a task is indicated both by the task_state field of the TCB
 * and by a series of task lists.  All of these tasks lists are declared
 * below. Although it is not always necessary, most of these lists are
 * prioritized so that common list handling logic can be used (only the
 * g_readytorun, the g_pendingtasks, and the g_waitingforsemaphore lists
 * need to be prioritized).
 */

/* This is the list of all tasks that are ready to run.  This is a
 * prioritized list with head of the list holding the highest priority
 * (unassigned) task.  In the non-SMP case, the head of this list is the
 * currently active task and the tail of this list, the lowest priority
 * task, is always the IDLE task.
 */

#undef g_readytorun
DEFINE_PER_CPU_BMP(dq_queue_t, g_readytorun) =
{
#if !defined(CONFIG_SMP)
  (FAR dq_entry_t *)&per_cpu_var_smp(g_idletcb, 0),
  (FAR dq_entry_t *)&per_cpu_var_smp(g_idletcb, 0),
#endif
};
#define g_readytorun this_cpu_var_bmp(g_readytorun)

/* In order to support SMP, the function of the g_readytorun list changes,
 * The g_readytorun is still used but in the SMP case it will contain only:
 *
 *  - Only tasks/threads that are eligible to run, but not currently running,
 *    and
 *  - Tasks/threads that have not been assigned to a CPU.
 *
 * Otherwise, the running TCB will be retained in g_assignedtasks vector.
 * As its name suggests, on 'g_assignedtasks vector for CPU
 * 'n' would contain the task/thread which is assigned to CPU 'n'.  Tasks/
 * threads would be assigned a particular CPU by one of two mechanisms:
 *
 *  - (Semi-)permanently through an RTOS interfaces such as
 *    pthread_attr_setaffinity(), or
 *  - Temporarily through scheduling logic when a previously unassigned task
 *    is made to run.
 */

#ifdef CONFIG_SMP
DEFINE_PER_CPU_BSS_SMP(struct tcb_s *, g_assignedtasks);

DEFINE_PER_CPU_BSS_SMP(enum task_deliver_e, g_delivertasks);
#endif

/* g_running_tasks[] holds a references to the running task for each CPU.
 * It is valid only when up_interrupt_context() returns true.
 */

DEFINE_PER_CPU_BSS(FAR struct tcb_s *, g_running_tasks);

/* This is the list of all tasks that are ready-to-run, but cannot be placed
 * in the g_readytorun list because:  (1) They are higher priority than the
 * currently active task at the head of the g_readytorun list, and (2) the
 * currently active task has disabled pre-emption.
 */

#ifndef CONFIG_SMP
#  undef g_pendingtasks
DEFINE_PER_CPU_BSS_BMP(dq_queue_t, g_pendingtasks);
#  define g_pendingtasks this_cpu_var_bmp(g_pendingtasks)
#endif

/* This is the list of all tasks that are blocked waiting for a signal */

#undef g_waitingforsignal
DEFINE_PER_CPU_BSS_BMP(dq_queue_t, g_waitingforsignal);
#define g_waitingforsignal this_cpu_var_bmp(g_waitingforsignal)

#ifdef CONFIG_LEGACY_PAGING
/* This is the list of all tasks that are blocking waiting for a page fill */

#undef g_waitingforfill
DEFINE_PER_CPU_BSS_BMP(dq_queue_t, g_waitingforfill);
#define g_waitingforfill this_cpu_var_bmp(g_waitingforfill)
#endif

#ifdef CONFIG_SIG_SIGSTOP_ACTION
/* This is the list of all tasks that have been stopped
 * via SIGSTOP or SIGTSTP
 */
#undef g_stoppedtasks
DEFINE_PER_CPU_BSS_BMP(dq_queue_t, g_stoppedtasks);
#define g_stoppedtasks this_cpu_var_bmp(g_stoppedtasks)
#endif

/* This list of all tasks that have been initialized, but not yet
 * activated. NOTE:  This is the only list that is not prioritized.
 */

#undef g_inactivetasks
DEFINE_PER_CPU_BSS_BMP(dq_queue_t, g_inactivetasks);
#define g_inactivetasks this_cpu_var_bmp(g_inactivetasks)

/* This is the value of the last process ID assigned to a task */

#undef g_lastpid
DEFINE_PER_CPU_BMP(volatile pid_t, g_lastpid) = CONFIG_SMP_NCPUS - 1;
#define g_lastpid this_cpu_var_bmp(g_lastpid)

/* The following hash table is used for two things:
 *
 * 1. This hash table greatly speeds the determination of a new unique
 *    process ID for a task, and
 * 2. Is used to quickly map a process ID into a TCB.
 */

#undef g_pidhash
DEFINE_PER_CPU_BSS_BMP(FAR struct tcb_s **, g_pidhash);
#define g_pidhash this_cpu_var_bmp(g_pidhash)

#undef g_npidhash
DEFINE_PER_CPU_BSS_BMP(volatile int, g_npidhash);
#define g_npidhash this_cpu_var_bmp(g_npidhash)

#undef g_pidhashlock
DEFINE_PER_CPU_BMP(spinlock_t, g_pidhashlock) = SP_UNLOCKED;
#define g_pidhashlock this_cpu_var_bmp(g_pidhashlock)

/* This is a table of task lists.  This table is indexed by the task state
 * enumeration type (tstate_t) and provides a pointer to the associated
 * static task list (if there is one) as well as a set of attribute flags
 * indicating properties of the list, for example, if the list is an
 * ordered list or not.
 */

#undef g_tasklisttable
DEFINE_PER_CPU_BSS_BMP(tasklist_table_t, g_tasklisttable);
#define g_tasklisttable this_cpu_var_bmp(g_tasklisttable)

/* This is the current initialization state.  The level of initialization
 * is only important early in the start-up sequence when certain OS or
 * hardware resources may not yet be available to the kernel logic.
 */

#undef g_nx_initstate
DEFINE_PER_CPU_BSS_BMP(volatile enum nx_initstate_e, g_nx_initstate);
#define g_nx_initstate this_cpu_var_bmp(g_nx_initstate)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tasklist_table_initialize
 *
 * Description:
 *   Initialization of table of task lists.This table is indexed by the
 *   task state enumeration type (tstate_t) and provides a pointer to
 *   the associated static task list (if there is one) as well as a set
 *   of attribute flags indicating properties of the list, for example,
 *   if the list is an ordered list or not.
 *
 ****************************************************************************/

static void tasklist_table_initialize(void)
{
  FAR struct tasklist_s *tlist = (FAR void *)&g_tasklisttable;

  /* TSTATE_TASK_INVALID */

  tlist[TSTATE_TASK_INVALID].list = NULL;
  tlist[TSTATE_TASK_INVALID].attr = 0;

#ifdef CONFIG_SMP

  /* TSTATE_TASK_READYTORUN */

  tlist[TSTATE_TASK_READYTORUN].list = list_readytorun();
  tlist[TSTATE_TASK_READYTORUN].attr = TLIST_ATTR_PRIORITIZED;

#else

  /* TSTATE_TASK_PENDING */

  tlist[TSTATE_TASK_PENDING].list = list_pendingtasks();
  tlist[TSTATE_TASK_PENDING].attr = TLIST_ATTR_PRIORITIZED;

  /* TSTATE_TASK_READYTORUN */

  tlist[TSTATE_TASK_READYTORUN].list = list_readytorun();
  tlist[TSTATE_TASK_READYTORUN].attr = TLIST_ATTR_PRIORITIZED |
                                       TLIST_ATTR_RUNNABLE;

  /* TSTATE_TASK_RUNNING */

  tlist[TSTATE_TASK_RUNNING].list = list_readytorun();
  tlist[TSTATE_TASK_RUNNING].attr = TLIST_ATTR_PRIORITIZED |
                                    TLIST_ATTR_RUNNABLE;
#endif

  /* TSTATE_TASK_INACTIVE */

  tlist[TSTATE_TASK_INACTIVE].list = list_inactivetasks();
  tlist[TSTATE_TASK_INACTIVE].attr = 0;

  /* TSTATE_WAIT_SEM */

  tlist[TSTATE_WAIT_SEM].list = (FAR dq_queue_t *)offsetof(sem_t, waitlist);
  tlist[TSTATE_WAIT_SEM].attr = TLIST_ATTR_PRIORITIZED |
                                TLIST_ATTR_OFFSET;

  /* TSTATE_WAIT_SIG */

  tlist[TSTATE_WAIT_SIG].list = list_waitingforsignal();
  tlist[TSTATE_WAIT_SIG].attr = 0;

#ifndef CONFIG_DISABLE_MQUEUE

  /* TSTATE_WAIT_MQNOTEMPTY */

  tlist[TSTATE_WAIT_MQNOTEMPTY].list =
    (FAR dq_queue_t *)offsetof(struct mqueue_inode_s, cmn.waitfornotempty);
  tlist[TSTATE_WAIT_MQNOTEMPTY].attr = TLIST_ATTR_PRIORITIZED |
                                       TLIST_ATTR_OFFSET;

  /* TSTATE_WAIT_MQNOTFULL */

  tlist[TSTATE_WAIT_MQNOTFULL].list =
    (FAR dq_queue_t *)offsetof(struct mqueue_inode_s, cmn.waitfornotfull);
  tlist[TSTATE_WAIT_MQNOTFULL].attr = TLIST_ATTR_PRIORITIZED |
                                      TLIST_ATTR_OFFSET;
#endif

#ifdef CONFIG_LEGACY_PAGING

  /* TSTATE_WAIT_PAGEFILL */

  tlist[TSTATE_WAIT_PAGEFILL].list = list_waitingforfill();
  tlist[TSTATE_WAIT_PAGEFILL].attr = TLIST_ATTR_PRIORITIZED;
#endif

#ifdef CONFIG_SIG_SIGSTOP_ACTION

  /* TSTATE_TASK_STOPPED */

  tlist[TSTATE_TASK_STOPPED].list = list_stoppedtasks();
  tlist[TSTATE_TASK_STOPPED].attr = 0;

#endif
}

/****************************************************************************
 * Name: idle_task_initialize
 *
 * Description:
 *   IDLE Task Initialization
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
static void idle_task_initialize(void)
{
  FAR struct tcb_s *tcb;
  int i;

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      tcb             = &per_cpu_var_smp(g_idletcb, i);
      tcb->pid        = i;
      tcb->cpu        = i;
      tcb->flags      = TCB_FLAG_TTYPE_KERNEL | TCB_FLAG_CPU_LOCKED;
      tcb->affinity   = CONFIG_SMP_DEFAULT_CPUSET & SCHED_ALL_CPUS;
      if (i != 0)
        {
          tcb->start      = nx_idle_trampoline;
          tcb->entry.main = (main_t)nx_idle_trampoline;
        }
      else
        {
          tcb->start      = nx_start;
          tcb->entry.main = (main_t)nx_start;
        }

      tcb->task_state = TSTATE_TASK_RUNNING;
      tcb->lockcount  = 1;
      tcb->refs       = 1;

#if CONFIG_TASK_NAME_SIZE > 0
      snprintf(tcb->name, CONFIG_TASK_NAME_SIZE, "CPU%d IDLE", i);
#endif
      sem_init(&tcb->exit_sem, 0, 0);

      /* Then add the idle task's TCB to the head of the current ready to
       * run list.
       */

      per_cpu_var_smp(g_running_tasks, i) = tcb;
      per_cpu_var_smp(g_assignedtasks, i) = tcb;
    }

  up_update_task(&this_cpu_var(g_idletcb));
}
#else
static void idle_task_initialize(void)
{
  FAR struct tcb_s *tcb = &this_cpu_var(g_idletcb);
  tcb->pid              = 0;
  atomic_set(&tcb->flags, TCB_FLAG_TTYPE_KERNEL);
  tcb->start            = nx_start;
  tcb->entry.main       = (main_t)nx_start;

  tcb->task_state       = TSTATE_TASK_RUNNING;
  tcb->lockcount        = 1;
  atomic_set(&tcb->refs, 1);

#if CONFIG_TASK_NAME_SIZE >= 12
  snprintf(tcb->name, CONFIG_TASK_NAME_SIZE, "CPU%d IDLE", up_cpu_index());
#elif CONFIG_TASK_NAME_SIZE > 0
  strlcpy(tcb->name, "Idle_Task", CONFIG_TASK_NAME_SIZE);
#endif
  sem_init(&tcb->exit_sem, 0, 0);

  /* Then add the idle task's TCB to the head of the current ready to
   * run list.
   */

#ifdef CONFIG_BMP
  dq_init(&g_readytorun);
  dq_addfirst((FAR dq_entry_t *)tcb, &g_readytorun);
#endif

  g_running_task        = tcb;

  up_update_task(tcb);
}
#endif

/****************************************************************************
 * Name: idle_group_initialize
 *
 * Description:
 *   IDLE Group Initialization
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
static void idle_group_initialize(void)
{
  FAR struct tcb_s *tcb;
  int hashndx;
  int i;

  /* Assign the process ID(s) of ZERO to the idle task(s) */

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      tcb = &per_cpu_var_smp(g_idletcb, i);

      hashndx = PIDHASH(i);
      g_pidhash[hashndx] = tcb;

      /* Allocate the IDLE group */

      DEBUGVERIFY(group_initialize(tcb, atomic_read(&tcb->flags), 0));

      /* Initialize the task join */

      nxtask_joininit(tcb);

      /* Create a stack for all CPU IDLE threads (except CPU0 which already
       * has a stack).
       */

      if (i > 0)
        {
          DEBUGVERIFY(up_cpu_idlestack(i, tcb, CONFIG_IDLETHREAD_STACKSIZE));
        }

      /* Initialize the processor-specific portion of the TCB */

      up_initial_state(tcb);

      /* Initialize the thread local storage */

      tls_init_info(tcb);

#ifdef CONFIG_SMP
      /* up_initial_state() builds the initial register frame at the top of
       * the idle stack.  tls_init_info() may reserve/adjust stack space, so
       * secondary CPU idle contexts must be rebuilt after TLS setup.
       */

      if (i > 0)
        {
          up_initial_state(tcb);
        }
#endif

      /* Complete initialization of the IDLE group.  Suppress retention
       * of child status in the IDLE group.
       */

      group_postinitialize(tcb);
      atomic_set(&tcb->group->tg_flags,
                 GROUP_FLAG_NOCLDWAIT | GROUP_FLAG_PRIVILEGED);
    }
}
#else
static void idle_group_initialize(void)
{
  FAR struct tcb_s *tcb = &this_cpu_var(g_idletcb);

  g_pidhash[0] = tcb;

  /* Allocate the IDLE group */

  DEBUGVERIFY(group_initialize(tcb, atomic_read(&tcb->flags), 0));

  /* Initialize the task join */

  nxtask_joininit(tcb);

  /* Initialize the processor-specific portion of the TCB */

  up_initial_state(tcb);

  /* Initialize the thread local storage */

  tls_init_info(tcb);

  /* Complete initialization of the IDLE group.  Suppress retention
   * of child status in the IDLE group.
   */

  group_postinitialize(tcb);
  atomic_set(&tcb->group->tg_flags,
              GROUP_FLAG_NOCLDWAIT | GROUP_FLAG_PRIVILEGED);
}
#endif

/****************************************************************************
 * Name: tasklist_initialize
 *
 * Description:
 *   Tramsition to the OSINIT_TASKLISTS state.
 *
 ****************************************************************************/

static void tasklist_initialize(void)
{
  /* Initialize the IDLE task TCB *******************************************/

  idle_task_initialize();

  /* Initialize wdog list ***************************************************/

  wdlist_initialize();

  /* Initialize wqueue list *************************************************/

  worklist_initialize();

  /* Initialize task list table *********************************************/

  tasklist_table_initialize();

  /* Initialize early drivers ***********************************************/

  drivers_early_initialize();

  /* Initialize RTOS facilities *********************************************/

  /* Initialize the semaphore facility.  This has to be done very early
   * because many subsystems depend upon fully functional semaphores.
   */

  nxsem_initialize();

  /* Task lists are initialized */

  g_nx_initstate = OSINIT_TASKLISTS;
  sched_trace_mark("TASKLISTS");
}

/****************************************************************************
 * Name: memory_initialize
 *
 * Description:
 *   Tramsition to the OSINIT_MEMORY state.
 *
 ****************************************************************************/

static void memory_initialize(void)
{
  int i;

#if defined(MM_KERNEL_USRHEAP_INIT) || defined(CONFIG_MM_KERNEL_HEAP) || \
    defined(CONFIG_MM_PGALLOC)
  /* Initialize the memory manager */

    {
      FAR void *heap_start;
      size_t heap_size;

#ifdef CONFIG_MM_KERNEL_HEAP
      /* Get the kernel-mode heap from the platform specific code and
       * configure the kernel-mode memory allocator.
       */

      up_allocate_kheap(&heap_start, &heap_size);
      kmm_initialize(heap_start, heap_size);
#endif

#ifdef MM_KERNEL_USRHEAP_INIT
      /* Get the user-mode heap from the platform specific code and configure
       * the user-mode memory allocator.
       */

      up_allocate_heap(&heap_start, &heap_size);
      kumm_initialize(heap_start, heap_size);
#endif

#ifdef CONFIG_MM_PGALLOC
      /* If there is a page allocator in the configuration, then get the page
       * heap information from the platform-specific code and configure the
       * page allocator.
       */

      up_allocate_pgheap(&heap_start, &heap_size);
      mm_pginitialize(heap_start, heap_size);
#endif
    }
#endif

#ifdef CONFIG_MM_KMAP
  /* Initialize the kernel dynamic mapping module */

  kmm_map_initialize();
#endif

#ifdef CONFIG_ARCH_HAVE_EXTRA_HEAPS
  /* Initialize any extra heap. */

  up_extraheaps_init();
#endif

#ifdef CONFIG_MM_IOB
  /* Initialize IO buffering */

  iob_initialize();
#endif

#ifdef CONFIG_SCHED_PERF_EVENTS
  perf_event_init();
#endif

  /* Initialize the logic that determine unique process IDs. */

  i = CONFIG_PID_INITIAL_COUNT;
#if CONFIG_SMP_NCPUS > CONFIG_PID_INITIAL_COUNT
  while (i <= CONFIG_SMP_NCPUS)
    {
      i <<= 1;
    }
#endif

  g_pidhash = kmm_zalloc(sizeof(*g_pidhash) * (size_t)i);
  DEBUGASSERT(g_pidhash);

  g_npidhash = i;

  /* IDLE Group Initialization **********************************************/

  idle_group_initialize();

  /* The memory manager is available */

  g_nx_initstate = OSINIT_MEMORY;
  sched_trace_mark("MEMORY");
}

/****************************************************************************
 * Name: hardware_initialize
 *
 * Description:
 *   Tramsition to the OSINIT_HARDWARE state.
 *
 ****************************************************************************/

static void hardware_initialize(void)
{
  /* Initialize tasking data structures */

  task_initialize();

  /* Initialize the instrument function */

  instrument_initialize();

  /* Initialize the file system (needed to support device drivers) */

  fs_initialize();

  /* Initialize the interrupt handling subsystem (if included) */

  irq_initialize();

  /* Initialize the POSIX timer facility (if included in the link) */

  clock_initialize();

#ifndef CONFIG_DISABLE_POSIX_TIMERS
  timer_initialize();
#endif

  /* Initialize the signal facility (if in link) */

  nxsig_initialize();

#if !defined(CONFIG_DISABLE_MQUEUE) || !defined(CONFIG_DISABLE_MQUEUE_SYSV)
  /* Initialize the named message queue facility (if in link) */

  nxmq_initialize();
#endif

#ifdef CONFIG_NET
  /* Initialize the networking system */

  net_initialize();
#endif

#ifndef CONFIG_BINFMT_DISABLE
  /* Initialize the binfmt system */

  binfmt_initialize();
#endif

  /* Initialize Hardware Facilities *****************************************/

  /* The processor specific details of running the operating system
   * will be handled here.  Such things as setting up interrupt
   * service routines and starting the clock are some of the things
   * that are different for each  processor and hardware platform.
   */

  up_initialize();

  /* Initialize common drivers */

  drivers_initialize();

#ifdef CONFIG_BOARD_EARLY_INITIALIZE
  /* Call the board-specific up_initialize() extension to support
   * early initialization of board-specific drivers and resources
   * that cannot wait until board_late_initialize.
   */

  boards_trace_begin();
  board_early_initialize();
  boards_trace_end();
#endif

  /* Hardware resources are now available */

  g_nx_initstate = OSINIT_HARDWARE;
  sched_trace_mark("HARDWARE");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_start
 *
 * Description:
 *   This function is called to initialize the operating system and to spawn
 *   the user initialization thread of execution.  This is the initial entry
 *   point into NuttX.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return.
 *
 ****************************************************************************/

void nx_start(void)
{
#ifdef CONFIG_SMP
  int i;
#endif

  /* Boot up is complete */

  g_nx_initstate = OSINIT_BOOT;
  sched_trace_begin();
  sinfo("Entry\n");

  /* Head of ready-to-run/assigned task lists valid */

  tasklist_initialize();

  /* The memory manager has been initialized */

  memory_initialize();

  /* MCU-specific hardware is initialized */

  hardware_initialize();

  /* Setup for Multi-Tasking ************************************************/

  /* Announce that the CPU0 IDLE task has started */

  sched_note_start(&this_cpu_var(g_idletcb));

  /* Initialize stdio for the IDLE task of each CPU */

  /* Create stdout, stderr, stdin on the CPU0 IDLE task.  These
   * will be inherited by all of the threads created by the CPU0
   * IDLE task.
   */

  DEBUGVERIFY(group_setupidlefiles());

#ifdef CONFIG_SMP
  for (i = 1; i < CONFIG_SMP_NCPUS; i++)
    {
      /* Clone stdout, stderr, stdin from the CPU0 IDLE task. */

      DEBUGVERIFY(group_setuptaskfiles(&per_cpu_var_smp(g_idletcb, i),
                                        NULL, true));
    }

  /* Start all CPUs *********************************************************/

  /* A few basic sanity checks */

  DEBUGASSERT(this_cpu() == 0);

  /* Then start the other CPUs */

  DEBUGVERIFY(nx_smp_start());

#endif /* CONFIG_SMP */

  /* Bring Up the System ****************************************************/

  /* The OS is fully initialized and we are beginning multi-tasking */

  g_nx_initstate = OSINIT_OSREADY;
  sched_trace_mark("OSREADY");

  /* Create initial tasks and bring-up the system */

  nx_bringup();

  /* Enter to idleloop */

  g_nx_initstate = OSINIT_IDLELOOP;
  sched_trace_mark("IDLELOOP");

#ifdef CONFIG_SMP
  for (i = 1; i < CONFIG_SMP_NCPUS; i++)
    {
      up_send_smp_sched(i);
    }
#endif

  /* Let other threads have access to the memory manager */

  sched_trace_end();
  sched_unlock();

  /* The IDLE Loop **********************************************************/

  /* When control is return to this point, the system is idle. */

  sinfo("CPU0: Beginning Idle Loop\n");
#ifndef CONFIG_DISABLE_IDLE_LOOP
  for (; ; )
    {
      /* Perform any processor-specific idle state operations */

      up_idle();
    }
#endif
}
