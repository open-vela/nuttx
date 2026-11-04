/****************************************************************************
 * sched/task/task_create.c
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

#include <sys/types.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>
#include <nuttx/kthread.h>
#include <nuttx/fs/fs.h>

#include "sched/sched.h"
#include "group/group.h"
#include "task/task.h"

/* [V813] Static TCB pool -- see the allocation site for why it exists.
 *
 * v804_tcb_is_static() is what lets kmm_free() route a block back here instead
 * of into the heap allocator, which is the only thing that makes such a pool
 * safe: handing one of these blocks to mm_free() would corrupt the allocator.
 */

#define V813_TCB_POOL_SLOTS 4
#define V813_TCB_BYTES       1024

static struct
{
  uint32_t in_use;
  uint8_t  mem[V813_TCB_BYTES];
} g_v813_tcb_pool[V813_TCB_POOL_SLOTS];

static FAR struct tcb_s *v813_tcb_static_alloc(size_t need)
{
  int i;

  /* Refuse rather than truncate: a short TCB corrupts the task. */
  if (need > V813_TCB_BYTES)
    {
      return NULL;
    }

  for (i = 0; i < V813_TCB_POOL_SLOTS; i++)
    {
      if (g_v813_tcb_pool[i].in_use)
        {
          continue;
        }

      memset(g_v813_tcb_pool[i].mem, 0, sizeof(g_v813_tcb_pool[i].mem));
      g_v813_tcb_pool[i].in_use = 1;

      syslog(LOG_ERR,
             "[V813] TCB from static pool slot %d (%u bytes needed of %u; "
             "the heap had %u free when esp_wifi_malloc was failing)\n",
             i, (unsigned)need, (unsigned)V813_TCB_BYTES,
             (unsigned)kmm_mallinfo().fordblks);
      return (FAR struct tcb_s *)g_v813_tcb_pool[i].mem;
    }

  syslog(LOG_ERR, "[V813] static TCB pool exhausted (%d slots)\n",
         V813_TCB_POOL_SLOTS);
  return NULL;
}

/* True when this block came from the pool rather than the heap. */
bool v813_tcb_is_static(FAR const void *p);

bool v813_tcb_is_static(FAR const void *p)
{
  const uint8_t *b = (const uint8_t *)p;
  int            i;

  if (p == NULL)
    {
      return false;
    }

  for (i = 0; i < V813_TCB_POOL_SLOTS; i++)
    {
      if (b >= g_v813_tcb_pool[i].mem &&
          b < g_v813_tcb_pool[i].mem + sizeof(g_v813_tcb_pool[i].mem))
        {
          return true;
        }
    }

  return false;
}

void v813_tcb_static_free(FAR void *p);

void v813_tcb_static_free(FAR void *p)
{
  const uint8_t *b = (const uint8_t *)p;
  int            i;

  if (p == NULL)
    {
      return;
    }

  for (i = 0; i < V813_TCB_POOL_SLOTS; i++)
    {
      if (b >= g_v813_tcb_pool[i].mem &&
          b < g_v813_tcb_pool[i].mem + sizeof(g_v813_tcb_pool[i].mem))
        {
          g_v813_tcb_pool[i].in_use = 0;
          return;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxthread_create
 *
 * Description:
 *   This function creates and activates a new thread of the specified type
 *   with a specified priority and returns its system-assigned ID.  It is the
 *   internal, common implementation of task_create() and kthread_create().
 *   See comments with task_create() for further information.
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   ttype      - Type of the new task
 *   priority   - Priority of the new task
 *   stack_addr - Address of the stack needed
 *   stack_size - Size (in bytes) of the stack needed
 *   entry      - Entry point of a new task
 *   arg        - A pointer to an array of input parameters.  The array
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *   envp       - A pointer to an array of environment strings. Terminated
 *                with a NULL entry.
 *
 * Returned Value:
 *   Returns the positive, non-zero process ID of the new task or a negated
 *   errno value to indicate the nature of any failure.  If memory is
 *   insufficient or the task cannot be created -ENOMEM will be returned.
 *
 ****************************************************************************/

int nxthread_create(FAR const char *name, int ttype, int priority,
                    FAR void *stack_addr, int stack_size, main_t entry,
                    FAR char * const argv[], FAR char * const envp[])
{
  posix_spawnattr_t attr;
  FAR struct tcb_s *tcb;
  pid_t pid;
  int ret = -ENOMEM;

  posix_spawnattr_init(&attr);
  posix_spawnattr_setstacksize(&attr, (size_t)stack_size);
  posix_spawnattr_setstackaddr(&attr, stack_addr);
  posix_spawnattr_setpriority(&attr, priority);

  /* Allocate a TCB for the new task. */

  tcb = kmm_zalloc(ttype == TCB_FLAG_TTYPE_KERNEL ?
                   sizeof(struct tcb_s) :
                   sizeof(struct tcb_s) + sizeof(struct task_group_s));

  /* [V813] Static TCB fallback for user threads.
   *
   * A cold boot that ends in a black screen has this signature:
   *
   *   [V794] kmm_malloc FAILED requested=1700 ... ra=esp_wifi_malloc
   *   [agent] network gate passed
   *   [V736] xiaozhi_ui create failed (try 0): -12      (x24)
   *   [XZUI] xiaozhi_ui                                 (never starts)
   *
   * The WiFi firmware asks the kernel heap for 1700-byte buffers at exactly the
   * moment the UI thread is being created, and the two collide.  The heap is not
   * small -- the same line reports fordblks=122240 mxordblk=122224, i.e. 119 KiB
   * free in one piece -- but WiFi takes it first and the task creation loses.
   * The framebuffer was moved to PSRAM by v757 for the same class of reason, and
   * the note there names WiFi's DHCP/TX buffers as the other tenant of that heap.
   *
   * v804 added this pool once already, and it was removed in v809 as a phantom:
   * at that time the "fragmentation" being measured was the probe's own doing
   * (vox3_intram_maxblock allocated its way up the heap from the V633 heartbeat,
   * fixed in v807).  With that gone the failures are real again -- this call site
   * is not in any probe -- so the fallback is back, for the actual reason.
   *
   * The measurement that sizes it: the driver prints the requirement on first
   * use, and it is 224 bytes.  The pool slot is 1024.
   *
   * Only user threads draw from it, since kernel threads never carry a
   * task_group, so this cannot perturb the kernel's own allocations. */
  if (!tcb && ttype != TCB_FLAG_TTYPE_KERNEL)
    {
      tcb = v813_tcb_static_alloc(sizeof(struct tcb_s) +
                                 sizeof(struct task_group_s));
    }

  if (!tcb)
    {
      serr("ERROR: Failed to allocate TCB\n");
    }
  else
    {
      /* Setup the task type */

      atomic_set(&tcb->flags, ttype | TCB_FLAG_FREE_TCB);

      /* Initialize the task */

      ret = nxtask_init(tcb, name, entry, NULL, &attr, argv, envp);
      posix_spawnattr_destroy(&attr);
      if (ret < OK)
        {
          kmm_free(tcb);
        }
      else
        {
          /* Get the assigned pid before we start the task */

          pid = tcb->pid;

          /* Activate the task */

          nxtask_activate(tcb);

          ret = pid;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nxtask_create
 *
 * Description:
 *   This function creates and activates a new task with a specified
 *   priority and returns its system-assigned ID.
 *
 *   The entry address entry is the address of the "main" function of the
 *   task.  This function will be called once the C environment has been
 *   set up.  The specified function will be called with four arguments.
 *   Should the specified routine return, a call to exit() will
 *   automatically be made.
 *
 *   Note that four (and only four) arguments must be passed for the spawned
 *   functions.
 *
 *   nxtask_create() is identical to the function task_create(), differing
 *   only in its return value:  This function does not modify the errno
 *   variable.  This is a non-standard, internal OS function and is not
 *   intended for use by application logic.  Applications should use
 *   task_create().
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   priority   - Priority of the new task
 *   stack_addr - Address of the stack needed
 *   stack_size - Size (in bytes) of the stack needed
 *   entry      - Entry point of a new task
 *   arg        - A pointer to an array of input parameters.  The array
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *   envp       - A pointer to an array of environment strings. Terminated
 *                with a NULL entry.
 *
 * Returned Value:
 *   Returns the positive, non-zero process ID of the new task or a negated
 *   errno value to indicate the nature of any failure.  If memory is
 *   insufficient or the task cannot be created -ENOMEM will be returned.
 *
 ****************************************************************************/

int nxtask_create(FAR const char *name, int priority,
                  FAR void *stack_addr, int stack_size, main_t entry,
                  FAR char * const argv[], FAR char * const envp[])
{
  return nxthread_create(name, TCB_FLAG_TTYPE_TASK, priority, stack_addr,
                         stack_size, entry, argv, envp ? envp : environ);
}

/****************************************************************************
 * Name: task_create
 *
 * Description:
 *   This function creates and activates a new task with a specified
 *   priority and returns its system-assigned ID.
 *
 *   The entry address entry is the address of the "main" function of the
 *   task.  This function will be called once the C environment has been
 *   set up.  The specified function will be called with four arguments.
 *   Should the specified routine return, a call to exit() will
 *   automatically be made.
 *
 *   Note that four (and only four) arguments must be passed for the spawned
 *   functions.
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   priority   - Priority of the new task
 *   stack_size - size (in bytes) of the stack needed
 *   entry      - Entry point of a new task
 *   arg        - A pointer to an array of input parameters.  The array
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *
 * Returned Value:
 *   Returns the non-zero process ID of the new task or ERROR if memory is
 *   insufficient or the task cannot be created.  The errno will be set in
 *   the failure case to indicate the nature of the error.
 *
 ****************************************************************************/

#ifndef CONFIG_BUILD_KERNEL
int task_create_with_stack(FAR const char *name, int priority,
                           FAR void *stack_addr, int stack_size,
                           main_t entry, FAR char * const argv[])
{
  int ret = nxtask_create(name, priority, stack_addr,
                          stack_size, entry, argv, NULL);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

int task_create(FAR const char *name, int priority,
                int stack_size, main_t entry, FAR char * const argv[])
{
  return task_create_with_stack(name, priority, NULL,
                                stack_size, entry, argv);
}
#endif

/****************************************************************************
 * Name: kthread_create_with_stack
 *
 * Description:
 *   This function creates and activates a kernel thread task with
 *   kernel-mode privileges. It is identical to kthread_create() except
 *   that it get the stack memory from caller.
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   priority   - Priority of the new task
 *   stack_addr - Stack buffer of the new task
 *   stack_size - Stack size of the new task
 *   entry      - Entry point of a new task
 *   arg        - A pointer to an array of input parameters.  The array
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *
 * Returned Value:
 *   Returns the positive, non-zero process ID of the new task or a negated
 *   errno value to indicate the nature of any failure.  If memory is
 *   insufficient or the task cannot be created -ENOMEM will be returned.
 *
 ****************************************************************************/

int kthread_create_with_stack(FAR const char *name, int priority,
                              FAR void *stack_addr, int stack_size,
                              main_t entry, FAR char * const argv[])
{
  return nxthread_create(name, TCB_FLAG_TTYPE_KERNEL, priority,
                         stack_addr, stack_size, entry, argv, NULL);
}

/****************************************************************************
 * Name: kthread_create
 *
 * Description:
 *   This function creates and activates a kernel thread task with kernel-
 *   mode privileges.  It is identical to task_create() except that it
 *   configures the newly started thread to run in kernel model.
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   priority   - Priority of the new task
 *   stack_size - size (in bytes) of the stack needed
 *   entry      - Entry point of a new task
 *   arg        - A pointer to an array of input parameters.  The array
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *
 * Returned Value:
 *   Returns the positive, non-zero process ID of the new task or a negated
 *   errno value to indicate the nature of any failure.  If memory is
 *   insufficient or the task cannot be created -ENOMEM will be returned.
 *
 ****************************************************************************/

int kthread_create(FAR const char *name, int priority,
                   int stack_size, main_t entry, FAR char * const argv[])
{
  return kthread_create_with_stack(name, priority,
                                   NULL, stack_size, entry, argv);
}
