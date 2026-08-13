/****************************************************************************
 * include/nuttx/spinlock.h
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

#ifndef __INCLUDE_NUTTX_SPINLOCK_H
#define __INCLUDE_NUTTX_SPINLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <stdint.h>
#include <sched.h>

#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <nuttx/atomic.h>

#include <nuttx/spinlock_type.h>
#include <nuttx/sched_note.h>

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0
void nxsched_critmon_busywait(bool state, FAR void *caller);
#else
#  define nxsched_critmon_busywait(state, caller)
#endif

#ifdef CONFIG_SPINLOCK_DEBUG
void spinlock_mark_locking(FAR spinlock_debug_t *info);
void spinlock_mark_locked(FAR spinlock_debug_t *info);
void spinlock_mark_unlocked(FAR spinlock_debug_t *info);
void spinlock_switch_context(FAR struct tcb_s *from);
pid_t spinlock_get_holder(FAR spinlock_debug_t *info);
#else
#  define spinlock_mark_locking(info)
#  define spinlock_mark_locked(info)
#  define spinlock_mark_unlocked(info)
#  define spinlock_switch_context(from)
#  define spinlock_get_holder(info) INVALID_PROCESS_ID
#endif

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

/* This is the spinlock that enforces critical sections when interrupts are
 * disabled.
 */

DECLARE_PER_CPU_BMP(rspinlock_t, g_schedlock);
#define g_schedlock this_cpu_var_bmp(g_schedlock)

/****************************************************************************
 * Name: spin_lock_init
 *
 * Description:
 *   Initialize a non-reentrant spinlock object to its initial,
 *   unlocked state.
 *
 * Input Parameters:
 *   lock  - A reference to the spinlock object to be initialized.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#define spin_lock_init(l) \
  do \
    { \
      if (sizeof(*(l)) > 0u) \
        { \
          memset((FAR void *)(l), 0, sizeof(*(l))); \
        } \
    } \
  while (0)

/****************************************************************************
 * Name: rspin_lock_init
 *
 * Description:
 *   Initialize a recursive spinlock object to its initial,
 *   unlocked state.
 *
 * Input Parameters:
 *   lock  - A reference to the struct rspinlock_s object to be initialized.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#define rspin_lock_init(l) \
   do { memset((FAR void *)l, 0, sizeof(*(l))); } while (0)

/****************************************************************************
 * Name: spin_lock_notrace
 *
 * Description:
 *   If this CPU does not already hold the spinlock, then loop until the
 *   spinlock is successfully locked.
 *
 *   This implementation is the same as the above spin_lock() except that
 *   it does not perform instrumentation logic.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   None.  When the function returns, the spinlock was successfully locked
 *   by this CPU.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function noinstrument_function
void spin_lock_notrace(FAR volatile spinlock_t *lock)
{
#ifdef CONFIG_TICKET_SPINLOCK
  int ticket = atomic_add_relaxed(&lock->next, 1);
  while (atomic_read_acquire(&lock->owner) != ticket);
#else /* CONFIG_TICKET_SPINLOCK */
  while (atomic_xchg_acquire(&lock->lock, 1) == 1);
#endif
}
#else
#  define spin_lock_notrace(lock) ((void)(lock))
#endif /* CONFIG_SPINLOCK */

/****************************************************************************
 * Name: spin_lock
 *
 * Description:
 *   If this CPU does not already hold the spinlock, then loop until the
 *   spinlock is successfully locked.
 *
 *   This implementation is non-reentrant and is prone to deadlocks in
 *   the case that any logic on the same CPU attempts to take the lock
 *   more than once.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   None.  When the function returns, the spinlock was successfully locked
 *   by this CPU.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function void spin_lock(FAR volatile spinlock_t *lock)
{
  /* Mark that we want to hold lock */

  spinlock_mark_locking(&lock->info);

  /* Notify that we are waiting for a spinlock */

  sched_note_spinlock(lock, NOTE_SPINLOCK_LOCK);

  /* If CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0, count busy-waiting. */

  nxsched_critmon_busywait(true, return_address(0));

  /* Lock without trace note */

  spin_lock_notrace(lock);

  /* Mark the spinlock has been locked */

  spinlock_mark_locked(&lock->info);

  /* Get the lock, end counting busy-waiting */

  nxsched_critmon_busywait(false, return_address(0));

  /* Notify that we have the spinlock */

  sched_note_spinlock(lock, NOTE_SPINLOCK_LOCKED);
}
#else
#  define spin_lock(lock) ((void)(lock))
#endif /* CONFIG_SPINLOCK */

/****************************************************************************
 * Name: spin_trylock_notrace
 *
 * Description:
 *   Try once to lock the spinlock.  Do not wait if the spinlock is already
 *   locked.
 *
 *   This implementation is the same as the above spin_trylock() except that
 *   it does not perform instrumentation logic.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   false - Failure, the spinlock was already locked
 *   true  - Success, the spinlock was successfully locked
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function noinstrument_function bool
spin_trylock_notrace(FAR volatile spinlock_t *lock)
{
#ifdef CONFIG_TICKET_SPINLOCK
  /* The expected value must live in a local.  A failed compare-exchange
   * writes the current value of the target object back through the
   * expected pointer, so passing &lock->owner here would clobber the
   * owner counter and make a lock held by another CPU appear unlocked.
   *
   * The exchange succeeds only when next == owner, which is the unlocked
   * state of a ticket lock.
   */

  atomic_t expected = atomic_read(&lock->owner);

  return atomic_cmpxchg_acquire(&lock->next, &expected, expected + 1);
#else /* CONFIG_TICKET_SPINLOCK */
  return atomic_xchg_acquire(&lock->lock, 1) != 1;
#endif /* CONFIG_TICKET_SPINLOCK */
}
#else
#  define spin_trylock_notrace(lock) ((void)(lock), true)
#endif /* CONFIG_SPINLOCK */

/****************************************************************************
 * Name: spin_trylock
 *
 * Description:
 *   Try once to lock the spinlock.  Do not wait if the spinlock is already
 *   locked.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   false - Failure, the spinlock was already locked
 *   true  - Success, the spinlock was successfully locked
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function bool spin_trylock(FAR volatile spinlock_t *lock)
{
  bool locked;

  /* Notify that we are waiting for a spinlock */

  sched_note_spinlock(lock, NOTE_SPINLOCK_LOCK);

  /* Try lock without trace note */

  locked = spin_trylock_notrace(lock);
  if (locked)
    {
      /* Mark the spinlock has been locked */

      spinlock_mark_locked(&lock->info);

      /* Notify that we have the spinlock */

      sched_note_spinlock(lock, NOTE_SPINLOCK_LOCKED);
    }
  else
    {
      /* Notify that we abort for a spinlock */

      sched_note_spinlock(lock, NOTE_SPINLOCK_ABORT);
    }

  return locked;
}
#else
#  define spin_trylock(lock) ((void)(lock), true)
#endif /* CONFIG_SPINLOCK */

/****************************************************************************
 * Name: spin_unlock_notrace
 *
 * Description:
 *   Release one count on a non-reentrant spinlock.
 *
 *   This implementation is the same as the above spin_unlock() except that
 *   it does not perform instrumentation logic.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to unlock.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function noinstrument_function void
spin_unlock_notrace(FAR volatile spinlock_t *lock)
{
#ifdef CONFIG_TICKET_SPINLOCK
  atomic_set_release(&lock->owner, lock->owner + 1);
#else
  atomic_set_release(&lock->lock, 0);
#endif
}
#else
#  define spin_unlock_notrace(lock) ((void)(lock))
#endif /* CONFIG_SPINLOCK */

/****************************************************************************
 * Name: spin_unlock
 *
 * Description:
 *   Release one count on a non-reentrant spinlock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to unlock.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function void spin_unlock(FAR volatile spinlock_t *lock)
{
  /* Mark the spinlock has been unlocked */

  spinlock_mark_unlocked(&lock->info);

  /* Unlock without trace note */

  spin_unlock_notrace(lock);

  /* Notify that we are unlocking the spinlock */

  sched_note_spinlock(lock, NOTE_SPINLOCK_UNLOCK);
}
#else
#  define spin_unlock(lock) ((void)(lock))
#endif /* CONFIG_SPINLOCK */

/****************************************************************************
 * Name: spin_is_locked
 *
 * Description:
 *   Release one count on a non-reentrant spinlock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to test.
 *
 * Returned Value:
 *   A boolean value: true the spinlock is locked; false if it is unlocked.
 *
 ****************************************************************************/

/* bool spin_islocked(FAR spinlock_t lock); */
#ifdef CONFIG_TICKET_SPINLOCK
#  define spin_is_locked(l) \
    (atomic_read(&(*l).owner) != atomic_read(&(*l).next))
#else
#  define spin_is_locked(l) (atomic_read(&(l)->lock) == 1)
#endif

/****************************************************************************
 * Name: spin_lock_irqsave_notrace
 *
 * Description:
 *   This function is no trace version of spin_lock_irqsave()
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function noinstrument_function
irqstate_t spin_lock_irqsave_notrace(FAR volatile spinlock_t *lock)
{
  irqstate_t flags;
  flags = up_irq_save();

  spin_lock_notrace(lock);

  return flags;
}
#else
#  define spin_lock_irqsave_notrace(l) ((void)(l), up_irq_save())
#endif

/****************************************************************************
 * Name: spin_lock_irqsave
 *
 * Description:
 *   If SMP is enabled:
 *     Disable local interrupts and take the lock spinlock and return
 *     the interrupt state.
 *
 *     NOTE: This API is very simple to protect data (e.g. H/W register
 *     or internal data structure) in SMP mode. But do not use this API
 *     with kernel APIs which suspend a caller thread. (e.g. nxsem_wait)
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_save().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock. not NULL.
 *
 * Returned Value:
 *   An opaque, architecture-specific value that represents the state of
 *   the interrupts prior to the call to spin_lock_irqsave(lock);
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function
irqstate_t spin_lock_irqsave(FAR volatile spinlock_t *lock)
{
  irqstate_t flags;

  /* Mark that we want to hold lock */

  spinlock_mark_locking(&lock->info);

  /* Notify that we are waiting for a spinlock */

  sched_note_spinlock(lock, NOTE_SPINLOCK_LOCK);

  /* If CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0, count busy-waiting. */

  nxsched_critmon_busywait(true, return_address(0));

  /* Lock without trace note */

  flags = spin_lock_irqsave_notrace(lock);

  /* Mark the spinlock has been locked */

  spinlock_mark_locked(&lock->info);

  /* Get the lock, end counting busy-waiting */

  nxsched_critmon_busywait(false, return_address(0));

  /* Notify that we have the spinlock */

  sched_note_spinlock(lock, NOTE_SPINLOCK_LOCKED);

  return flags;
}
#else
#  define spin_lock_irqsave(l) ((void)(l), up_irq_save())
#endif

/****************************************************************************
 * Name: spin_lock_irqsave_nopreempt
 *
 * Description:
 *   If SMP is enabled:
 *     Disable local interrupts, sched_lock and take the lock spinlock and
 *     return the interrupt state.
 *
 *     NOTE: This API is very simple to protect data (e.g. H/W register
 *     or internal data structure) in SMP mode. But do not use this API
 *     with kernel APIs which suspend a caller thread. (e.g. nxsem_wait)
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_save() + sched_lock().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock. not NULL.
 *
 * Returned Value:
 *   An opaque, architecture-specific value that represents the state of
 *   the interrupts prior to the call to spin_lock_irqsave(lock);
 *
 ****************************************************************************/

static inline_function
irqstate_t spin_lock_irqsave_nopreempt(FAR volatile spinlock_t *lock)
{
  irqstate_t flags;
  flags = spin_lock_irqsave(lock);
  sched_lock();
  return flags;
}

/****************************************************************************
 * Name: rspin_lock_is_recursive
 *
 * Description:
 *   This function check whether the recursive spinlock is currently held
 *   recursively. That is, whether it's locked more than once by the
 *   current holder.
 *   Note that this is inherently racy unless the calling thread is
 *   holding the rspinlock.
 *
 * Parameters:
 *   lock - Recursive spinlock descriptor.
 *
 * Return Value:
 *  If rspinlock has returned to True recursively, otherwise returns false.
 *
 ****************************************************************************/

static inline_function
bool rspin_lock_is_recursive(FAR volatile rspinlock_t *lock)
{
  return lock->count > 1;
}

/****************************************************************************
 * Name: rspin_lock_is_hold
 *
 * Description:
 *   This function checks whether the recursive spinlock is currently held
 *   by the current holder. That is, whether it's locked at least once.
 *   Note that this is inherently racy unless the current cpu is holding
 *   the rspinlock.
 *
 * Parameters:
 *   lock - Recursive spinlock descriptor.
 *
 * Return Value:
 *   True if the rspinlock is held (count > 0) and owned by the current
 *   cpu, otherwise returns false.
 *
 ****************************************************************************/

static inline_function
bool rspin_lock_is_hold(FAR volatile rspinlock_t *lock)
{
  bool ret;

  /* If this interface is not called while holding the rspinlock, the current
   * task may be switched to another CPU during a context switch,
   * leading to incorrect judgment.
   */

  irqstate_t flags = up_irq_save();
  ret = (lock->owner == this_cpu() + 1);
  up_irq_restore(flags);
  return ret;
}

/****************************************************************************
 * Name: rspin_lock_is_locked
 *
 * Description:
 *   This function checks whether the recursive spinlock is currently held
 *   by any holder. That is, whether it's locked at least once.
 *   Note that this is inherently racy unless the calling thread is
 *   holding the rspinlock.
 *
 * Parameters:
 *   lock - Recursive spinlock descriptor.
 *
 * Return Value:
 *   True if the rspinlock is held (count > 0), otherwise returns false.
 *
 ****************************************************************************/

static inline_function
bool rspin_lock_is_locked(FAR volatile rspinlock_t *lock)
{
  return lock->count > 0;
}

/****************************************************************************
 * Name: rspin_lock_count
 *
 * Description:
 *   This function return rspinlock count.
 *
 * Parameters:
 *   lock - Recursive spinlock descriptor.
 *
 * Return Value:
 *  Rspinlock count.
 *
 ****************************************************************************/

static inline_function
uint16_t rspin_lock_count(FAR volatile rspinlock_t *lock)
{
  return lock->count;
}

/****************************************************************************
 * Name: rspin_lock/rspin_lock_irqsave/rspin_lock_irqsave_nopreempt
 *
 * Description:
 *   Nest supported spinlock, can support UINT16_MAX max depth.
 *   As we should not disable irq for long time, sched also locked.
 *   Similar feature with enter_critical_section, but isolate by instance.
 *
 *   If SPINLOCK is enabled:
 *     Will take spinlock each cpu first call.
 *
 *   If SPINLOCK is not enabled:
 *     Equivalent to up_irq_save() + sched_lock().
 *     Will only sched_lock once when first called.
 *
 *   NOTE: This function is only allowed to execute with irq disabled.
 *
 * Input Parameters:
 *   lock - Caller specific rspinlock_s. not NULL.
 *
 * Returned Value:
 *   An opaque, architecture-specific value that represents the state of
 *   the interrupts prior to the call to spin_lock_irqsave(lock);
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function
void rspin_lock(FAR rspinlock_t *lock)
{
  rspinlock_t new_val;
  rspinlock_t old_val;
  int         cpu = this_cpu() + 1;

  /* Already owned this lock. */

  old_val.val = atomic_read(&lock->val);

  if (old_val.owner == cpu)
    {
      lock->count += 1;
    }
  else
    {
      /* Mark that we want to hold lock */

      spinlock_mark_locking(&lock->info);

      /* Try seize the ownership of the lock using CAS. */

      new_val.count = 1;
      new_val.owner = cpu;

      do
        {
          old_val.val = 0;
        }
      while (!atomic_cmpxchg_acquire(&lock->val,
             &old_val.val, new_val.val));

      /* Mark locked only when the lock is acquired for the first time. */

      spinlock_mark_locked(&lock->info);
    }
}

static inline_function
irqstate_t rspin_lock_irqsave(FAR rspinlock_t *lock)
{
  irqstate_t flags = up_irq_save();
  rspin_lock(lock);

  return flags;
}
#else
static inline_function
irqstate_t rspin_lock_irqsave(FAR rspinlock_t *lock)
{
  irqstate_t flags = up_irq_save();
  lock->count++;
  lock->owner = this_cpu() + 1;
  return flags;
}
#endif

static inline_function
irqstate_t rspin_lock_irqsave_nopreempt(FAR rspinlock_t *lock)
{
  irqstate_t flags;
  flags = rspin_lock_irqsave(lock);
  sched_lock();
  return flags;
}

/****************************************************************************
 * Name: rspin_trylock/rspin_trylock_irqsave/rspin_trylock_irqsave_nopreempt
 *
 * Description:
 *   Nest supported spinlock, try once to lock the rspinlock, can support
 *   UINT16_MAX max depth.
 *   As we should not disable irq for long time, sched also locked.
 *   Similar feature with enter_critical_section, but isolate by instance.
 *
 *   If SPINLOCK is enabled:
 *     Will take spinlock each cpu first call.
 *
 *   If SPINLOCK is not enabled:
 *     Equivalent to up_irq_save() + sched_lock().
 *     Will only sched_lock once when first called.
 *
 * Input Parameters:
 *   lock - Caller specific rspinlock_s. not NULL.
 *
 * Returned Value:
 *   true  - Success, the spinlock was successfully locked
 *   false - Failure, the spinlock was already locked
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function bool rspin_trylock(FAR rspinlock_t *lock)
{
  rspinlock_t new_val;
  rspinlock_t old_val;
  int         cpu = this_cpu() + 1;
  bool        ret = false;

  /* Already owned this lock. */

  old_val.val = atomic_read(&lock->val);

  if (old_val.owner == cpu)
    {
      lock->count += 1;
      ret = true;
    }
  else if (old_val.val == 0)
    {
      /* Try seize the ownership of the lock using CAS. */

      new_val.count = 1;
      new_val.owner = cpu;

      ret = atomic_cmpxchg_acquire(&lock->val,
                                   &old_val.val,
                                   new_val.val);

      /* Mark locked when the lock is successfully acquired. */

      if (ret)
        {
          spinlock_mark_locked(&lock->info);
        }
    }

  return ret;
}

#  define rspin_trylock_irqsave(l, f) \
    ({ \
      (f) = up_irq_save(); \
      rspin_trylock(l) ? \
      true : ({ up_irq_restore(f); false; }); \
    })
#else
#  define rspin_trylock_irqsave(l, f) \
    ({ \
      (f) = up_irq_save(); \
      (l)->count++; \
      (l)->owner = this_cpu() + 1; \
      true; \
    })
#endif

#define rspin_trylock_irqsave_nopreempt(l, f) \
  ({ \
    rspin_trylock_irqsave(l, f) ? \
    ({ sched_lock(); true; }) : false; \
  })

/****************************************************************************
 * Name: spin_trylock_irqsave_notrace
 *
 * Description:
 *   Try once to lock the spinlock.  Do not wait if the spinlock is already
 *   locked.
 *
 *   This implementation is the same as the above spin_trylock() except that
 *   it does not perform instrumentation logic.
 *
 * Input Parameters:
 *   lock  - A reference to the spinlock object to lock.
 *   flags - flag of interrupts status
 *
 * Returned Value:
 *   true  - Success, the spinlock was successfully locked
 *   false - Failure, the spinlock was already locked
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
#  define spin_trylock_irqsave_notrace(l, f) \
({ \
  f = up_irq_save(); \
  spin_trylock_notrace(l) ? \
  true : ({ up_irq_restore(f); false; }); \
})
#else
#  define spin_trylock_irqsave_notrace(l, f) \
({ \
  (void)(l); \
  f = up_irq_save(); \
  true; \
})
#endif /* CONFIG_SPINLOCK */

/****************************************************************************
 * Name: spin_trylock_irqsave
 *
 * Description:
 *   Try once to lock the spinlock.  Do not wait if the spinlock is already
 *   locked.
 *
 * Input Parameters:
 *   lock  - A reference to the spinlock object to lock.
 *   flags - flag of interrupts status
 *
 * Returned Value:
 *   true  - Success, the spinlock was successfully locked
 *   false - Failure, the spinlock was already locked
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
#  define spin_trylock_irqsave(l, f) \
({ \
  f = up_irq_save(); \
  spin_trylock(l) ? \
  true : ({ up_irq_restore(f); false; }); \
})
#else
#  define spin_trylock_irqsave(l, f) \
({ \
  (void)(l); \
  f = up_irq_save(); \
  true; \
})
#endif /* CONFIG_SPINLOCK */

/****************************************************************************
 * Name: spin_trylock_irqsave_nopreempt
 *
 * Description:
 *   Try once to lock the spinlock and disable preemption if successful. Do
 *   not wait if the spinlock is already locked.
 *
 * Input Parameters:
 *   lock   - A reference to the spinlock object to lock.
 *   flags  - flag of interrupts status
 *
 * Returned Value:
 *   true  - Failure, the spinlock was already locked
 *   false - Success, the spinlock was successfully locked and
 *                 preemption is disabled
 *
 ****************************************************************************/

#define spin_trylock_irqsave_nopreempt(lock, flags) \
  ({ \
    spin_trylock_irqsave(lock, flags) ? \
    ({ sched_lock(); true; }) : false; \
  })

/****************************************************************************
 * Name: spin_unlock_irqrestore_notrace
 *
 * Description:
 *   This function is no trace version of spin_unlock_irqrestore()
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function noinstrument_function
void spin_unlock_irqrestore_notrace(FAR volatile spinlock_t *lock,
                                    irqstate_t flags)
{
  spin_unlock_notrace(lock);

  up_irq_restore(flags);
}
#else
#  define spin_unlock_irqrestore_notrace(l, f) ((void)(l), up_irq_restore(f))
#endif

/****************************************************************************
 * Name: spin_unlock_irqrestore
 *
 * Description:
 *   If SMP is enabled:
 *     Release the lock and restore the interrupt state as it was prior
 *     to the previous call to spin_lock_irqsave(lock).
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_restore().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock. not NULL
 *
 *   flags - The architecture-specific value that represents the state of
 *           the interrupts prior to the call to spin_lock_irqsave(lock);
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function
void spin_unlock_irqrestore(FAR volatile spinlock_t *lock, irqstate_t flags)
{
  /* Mark the spinlock has been unlocked */

  spinlock_mark_unlocked(&lock->info);

  /* Unlock without trace note */

  spin_unlock_irqrestore_notrace(lock, flags);

  /* Notify that we are unlocking the spinlock */

  sched_note_spinlock(lock, NOTE_SPINLOCK_UNLOCK);
}
#else
#  define spin_unlock_irqrestore(l, f) ((void)(l), up_irq_restore(f))
#endif

/****************************************************************************
 * Name: spin_unlock_irqrestore_nopreempt
 *
 * Description:
 *   If SMP is enabled:
 *     Release the lock and restore the interrupt state, sched_unlock
 *     as it was prior to the previous call to
 *     spin_unlock_irqrestore_nopreempt(lock).
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_restore() + sched_unlock().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock. not NULL
 *
 *   flags - The architecture-specific value that represents the state of
 *           the interrupts prior to the call to
 *           spin_unlock_irqrestore_nopreempt(lock);
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define spin_unlock_irqrestore_nopreempt(lock, flags) \
  do \
    { \
      spin_unlock_irqrestore(lock, flags); \
      sched_unlock(); \
    } \
  while (0)

/****************************************************************************
 * Name: rspin_unlock_irqrestore/rspin_unlock_irqrestore_nopreempt
 *
 * Description:
 *   Nest supported spinunlock, can support UINT16_MAX max depth.
 *   Should work with rspin_lock_irqsave_nopreempt().
 *   Similar feature with leave_critical_section, but isolate by instance.
 *
 *   If SPINLOCK is enabled:
 *     Will release spinlock each cpu last call.
 *
 *   If SPINLOCK is not enabled:
 *     Equivalent to sched_unlock() + up_irq_restore().
 *     Will only sched_unlock once when last called.
 *
 * Input Parameters:
 *   lock - Caller specific rspinlock_s.
 *
 *   flags - The architecture-specific value that represents the state of
 *           the interrupts prior to the call to
 *           spin_unlock_irqrestore_nopreempt(lock);
 *
 * Returned Value:
 *   true  - Indicates exiting the spinlock.
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function
bool rspin_unlock(FAR rspinlock_t *lock)
{
  bool ret = false;

  DEBUGASSERT(lock->owner == this_cpu() + 1);
  DEBUGASSERT(lock->count >= 1);

  if (--lock->count == 0)
    {
      spinlock_mark_unlocked(&lock->info);
      atomic_set_release(&lock->val, 0);
      ret = true;
    }

  return ret;
}

static inline_function
void rspin_unlock_irqrestore(FAR rspinlock_t *lock, irqstate_t flags)
{
  if (rspin_unlock(lock))
    {
      up_irq_restore(flags);
    }

  /* If not last rspinlock restore,  up_irq_restore should not required */
}
#else
static inline_function
void rspin_unlock_irqrestore(FAR rspinlock_t *lock, irqstate_t flags)
{
  if (--lock->count == 0)
    {
      lock->owner = 0;
      up_irq_restore(flags);
    }

  /* If not last rspinlock restore,  up_irq_restore should not required */
}
#endif

#define rspin_unlock_irqrestore_nopreempt(lock, flags) \
  do \
    { \
      rspin_unlock_irqrestore(lock, flags); \
      sched_unlock(); \
    } \
  while (0)

#ifdef CONFIG_SPINLOCK
static inline_function
uint16_t rspin_breaklock(FAR rspinlock_t *lock)
{
  int oldcount = 0;

  if (lock->owner == this_cpu() + 1)
    {
      oldcount = lock->count;
      lock->count = 1;
      rspin_unlock(lock);
    }

  return oldcount;
}

static inline_function
void rspin_restorelock(FAR rspinlock_t *lock, uint16_t count)
{
  if (count != 0)
    {
      rspin_lock(lock);
      lock->count = count;
    }
}
#else
static inline_function
uint16_t rspin_breaklock(FAR rspinlock_t *lock)
{
  int oldcount = 0;

  if (lock->owner == this_cpu() + 1)
    {
      oldcount = lock->count;
      lock->val = 0;
    }

  return oldcount;
}

static inline_function
void rspin_restorelock(FAR rspinlock_t *lock, uint16_t count)
{
  if (count != 0)
    {
      lock->owner = this_cpu() + 1;
      lock->count = count;
    }
}
#endif

#if defined(CONFIG_RW_SPINLOCK)

/****************************************************************************
 * Name: rwlock_init
 *
 * Description:
 *   Initialize a non-reentrant spinlock object to its initial,
 *   unlocked state.
 *
 * Input Parameters:
 *   lock  - A reference to the spinlock object to be initialized.
 *
 * Returned Value:
 *   None.
 *
 *
 ****************************************************************************/

#define rwlock_init(l) \
   do { memset((FAR void *)l, 0, sizeof(*(l))); } while (0)

/****************************************************************************
 * Name: read_lock
 *
 * Description:
 *   If this task does not already hold the spinlock, then loop until the
 *   spinlock is successfully locked.
 *
 *   This implementation is non-reentrant and set a bit of lock.
 *
 *  The reader's priority is higher than the writer's priority.  If a reader
 *  holds the lock, a new reader can get its lock but a writer can't get this
 *  lock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   None.  When the function returns, the spinlock was successfully locked
 *   by this CPU.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

static inline_function void read_lock(FAR volatile rwlock_t *lock)
{
  nxsched_critmon_busywait(true, return_address(0));
  while (true)
    {
      int old = atomic_read(&lock->lock);

      if (old > RW_SP_WRITE_LOCKED &&
          atomic_cmpxchg_acquire(&lock->lock, &old, old + 1))
        {
          break;
        }
    }

  nxsched_critmon_busywait(false, return_address(0));
}

/****************************************************************************
 * Name: read_trylock
 *
 * Description:
 *   If this task does not already hold the spinlock, then try to get the
 * lock.
 *
 *   This implementation is non-reentrant and set a bit of lock.
 *
 *  The reader's priority is higher than the writer's priority.  If a reader
 *  holds the lock, a new reader can get its lock but a writer can't get this
 *  lock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   false   - Failure, the spinlock was already locked
 *   true    - Success, the spinlock was successfully locked
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

static inline_function bool read_trylock(FAR volatile rwlock_t *lock)
{
  bool ret = false;

  while (true)
    {
      int old = atomic_read(&lock->lock);

      if (old <= RW_SP_WRITE_LOCKED)
        {
          DEBUGASSERT(old == RW_SP_WRITE_LOCKED);
          break;
        }
      else if (atomic_cmpxchg_acquire(&lock->lock, &old, old + 1))
        {
          ret = true;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: read_unlock
 *
 * Description:
 *   Release a bit on a non-reentrant spinlock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to unlock.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

static inline_function void read_unlock(FAR volatile rwlock_t *lock)
{
  DEBUGASSERT(atomic_read(&lock->lock) >= RW_SP_READ_LOCKED);

  atomic_sub_release(&lock->lock, 1);
}

/****************************************************************************
 * Name: write_lock
 *
 * Description:
 *   If this CPU does not already hold the spinlock, then loop until the
 *   spinlock is successfully locked.
 *
 *   This implementation is non-reentrant and set all bit on lock to avoid
 *   readers and writers.
 *
 *  The reader's priority is higher than the writer's priority.  If a reader
 *  holds the lock, a new reader can get its lock but a writer can't get this
 *  lock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   None.  When the function returns, the spinlock was successfully locked
 *   by this CPU.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

static inline_function void write_lock(FAR volatile rwlock_t *lock)
{
  /* Mark that we want to hold lock */

  spinlock_mark_locking(&lock->info);
  nxsched_critmon_busywait(true, return_address(0));

  while (true)
    {
      atomic_t zero = RW_SP_UNLOCKED;
      if (atomic_cmpxchg_acquire(&lock->lock, &zero, RW_SP_WRITE_LOCKED))
        {
          break;
        }
    }

  spinlock_mark_locked(&lock->info);
  nxsched_critmon_busywait(false, return_address(0));
}

/****************************************************************************
 * Name: write_trylock
 *
 * Description:
 *   If this task does not already hold the spinlock, then loop until the
 *   spinlock is successfully locked.
 *
 *   This implementation is non-reentrant and set all bit on lock to avoid
 *   readers and writers.
 *
 *  The reader's priority is higher than the writer's priority.  If a reader
 *  holds the lock, a new reader can get its lock but a writer can't get this
 *  lock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   false   - Failure, the spinlock was already locked
 *   true    - Success, the spinlock was successfully locked
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

static inline_function bool write_trylock(FAR volatile rwlock_t *lock)
{
  atomic_t zero = RW_SP_UNLOCKED;
  bool ret = atomic_cmpxchg_acquire(&lock->lock, &zero, RW_SP_WRITE_LOCKED);

  if (ret)
    {
      spinlock_mark_locked(&lock->info);
    }

  return ret;
}

/****************************************************************************
 * Name: write_unlock
 *
 * Description:
 *   Release all bit on a non-reentrant spinlock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to unlock.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

static inline_function void write_unlock(FAR volatile rwlock_t *lock)
{
  /* Ensure this cpu already get write lock */

  DEBUGASSERT(atomic_read(&lock->lock) == RW_SP_WRITE_LOCKED);

  spinlock_mark_unlocked(&lock->info);
  atomic_set_release(&lock->lock, RW_SP_UNLOCKED);
}

/****************************************************************************
 * Name: read_lock_irqsave
 *
 * Description:
 *   If SMP is enabled:
 *     The argument lock should be specified,
 *     disable local interrupts and take the lock spinlock and return
 *     the interrupt state.
 *
 *     NOTE: This API is very simple to protect data (e.g. H/W register
 *     or internal data structure) in SMP mode. Do not use this API
 *     with kernel APIs which suspend a caller thread. (e.g. nxsem_wait)
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_save().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock, not NULL.
 *
 * Returned Value:
 *   An opaque, architecture-specific value that represents the state of
 *   the interrupts prior to the call to write_lock_irqsave(lock);
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
irqstate_t read_lock_irqsave(FAR rwlock_t *lock);
#else
#  define read_lock_irqsave(l) ((void)(l), up_irq_save())
#endif

/****************************************************************************
 * Name: read_unlock_irqrestore
 *
 * Description:
 *   If SMP is enabled:
 *     The argument lock should be specified, release the lock and
 *     restore the interrupt state as it was prior to the previous call to
 *     read_lock_irqsave(lock).
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_restore().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock, not NULL.
 *
 *   flags - The architecture-specific value that represents the state of
 *           the interrupts prior to the call to read_lock_irqsave(lock);
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
void read_unlock_irqrestore(FAR rwlock_t *lock, irqstate_t flags);
#else
#  define read_unlock_irqrestore(l, f) ((void)(l), up_irq_restore(f))
#endif

/****************************************************************************
 * Name: write_lock_irqsave
 *
 * Description:
 *   If SMP is enabled:
 *     The argument lock should be specified,
 *     disable local interrupts and take the lock spinlock and return
 *     the interrupt state.
 *
 *     NOTE: This API is very simple to protect data (e.g. H/W register
 *     or internal data structure) in SMP mode. But do not use this API
 *     with kernel APIs which suspend a caller thread. (e.g. nxsem_wait)
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_save().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock, not NULL.
 *
 * Returned Value:
 *   An opaque, architecture-specific value that represents the state of
 *   the interrupts prior to the call to write_lock_irqsave(lock);
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
irqstate_t write_lock_irqsave(FAR rwlock_t *lock);
#else
#  define write_lock_irqsave(l) ((void)(l), up_irq_save())
#endif

/****************************************************************************
 * Name: write_unlock_irqrestore
 *
 * Description:
 *   If SMP is enabled:
 *     The argument lock should be specified, release the lock and
 *     restore the interrupt state as it was prior to the previous call to
 *     write_lock_irqsave(lock).
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_restore().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock, not NULL.
 *
 *   flags - The architecture-specific value that represents the state of
 *           the interrupts prior to the call to write_lock_irqsave(lock);
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
void write_unlock_irqrestore(FAR rwlock_t *lock, irqstate_t flags);
#else
#  define write_unlock_irqrestore(l, f) ((void)(l), up_irq_restore(f))
#endif

#endif /* CONFIG_RW_SPINLOCK */

/****************************************************************************
 * Name: enter_critical_section
 *
 * Description:
 *   If thread-specific IRQ counter is enabled (for SMP or other
 *   instrumentation):
 *
 *     Take the CPU IRQ lock and disable interrupts on all CPUs.  A thread-
 *     specific counter is incremented to indicate that the thread has IRQs
 *     disabled and to support nested calls to enter_critical_section().
 *
 *     NOTE: Most architectures do not support disabling all CPUs from one
 *     CPU.  ARM is an example.  In such cases, logic in
 *     enter_critical_section() will still manage entrance into the
 *     protected logic using spinlocks.
 *
 *   If thread-specific IRQ counter is not enabled:
 *
 *     This function is equivalent to up_irq_save().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An opaque, architecture-specific value that represents the state of
 *   the interrupts prior to the call to enter_critical_section();
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
#  define enter_critical_section_notrace() rspin_lock_irqsave(&g_schedlock)
#else
#  define enter_critical_section_notrace() up_irq_save()
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0 || \
    defined(CONFIG_SCHED_INSTRUMENTATION_CSECTION)
irqstate_t enter_critical_section(void) noinstrument_function;
#else
#  define enter_critical_section() enter_critical_section_notrace()
#endif

/****************************************************************************
 * Name: in_critical_section
 *
 * Description:
 *   check whether we are in critical section
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True if we are in critical section, otherwise returns false.
 *
 ****************************************************************************/

#define in_critical_section() rspin_lock_is_hold(&g_schedlock)

/****************************************************************************
 * Name: leave_critical_section
 *
 * Description:
 *   If thread-specific IRQ counter is enabled (for SMP or other
 *   instrumentation):
 *
 *     Decrement the IRQ lock count and if it decrements to zero then release
 *     the spinlock and restore the interrupt state as it was prior to the
 *     previous call to enter_critical_section().
 *
 *   If thread-specific IRQ counter is not enabled:
 *
 *     This function is equivalent to up_irq_restore().
 *
 * Input Parameters:
 *   flags - The architecture-specific value that represents the state of
 *           the interrupts prior to the call to enter_critical_section();
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
#  define leave_critical_section_notrace(f) rspin_unlock_irqrestore(&g_schedlock, f)
#else
#  define leave_critical_section_notrace(f) up_irq_restore(f)
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0 || \
    defined(CONFIG_SCHED_INSTRUMENTATION_CSECTION)
void leave_critical_section(irqstate_t flags) noinstrument_function;
#else
#  define leave_critical_section(f) leave_critical_section_notrace(f)
#endif

/****************************************************************************
 * Name: break_critical_section
 *
 * Description:
 *   Break the critical_section
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0 || \
    defined(CONFIG_SCHED_INSTRUMENTATION_CSECTION)
uint16_t break_critical_section(void);
#else
#  ifdef CONFIG_SMP
#    define break_critical_section() rspin_breaklock(&g_schedlock)
#  else
#    define break_critical_section()
#  endif
#endif

/****************************************************************************
 * Name: irq_save_nopreempt
 *
 * Description:
 *     Disable local interrupts and disable preemption.
 *
 * Input Parameters:
 *   lock - The spinlock to acquire
 *
 * Returned Value:
 *   The previous IRQ state
 *
 ****************************************************************************/

static inline_function
irqstate_t irq_save_nopreempt(void)
{
  irqstate_t flags = up_irq_save();
  sched_lock();

  return flags;
}

/****************************************************************************
 * Name: irq_restore_nopreempt
 *
 * Description:
 *     Enable preemption and restore the interrupt state.
 *
 * Input Parameters:
 *   flags  - flag of interrupts status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline_function
void irq_restore_nopreempt(irqstate_t flags)
{
  sched_unlock();
  up_irq_restore(flags);
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_SPINLOCK_H */
