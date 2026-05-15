/****************************************************************************
 * include/nuttx/mutex.h
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

#ifndef __INCLUDE_NUTTX_MUTEX_H
#define __INCLUDE_NUTTX_MUTEX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <stdbool.h>
#include <execinfo.h>

#include <nuttx/clock.h>
#include <nuttx/mutex_type.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: nxmutex_add_backtrace
 *
 * Description:
 *   This function add the backtrace of the holder of the mutex.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_MUTEX_BACKTRACE
static inline_function void nxmutex_add_backtrace(FAR mutex_t *mutex)
{
  mutex->stack = backtrace_record(0);
}

static inline_function void nxmutex_remove_backtrace(FAR mutex_t *mutex)
{
  backtrace_remove(mutex->stack);
  mutex->stack = NULL;
}
#else
#  define nxmutex_add_backtrace(mutex)
#  define nxmutex_remove_backtrace(mutex)
#endif

/****************************************************************************
 * Name: nxmutex_init
 *
 * Description:
 *   This function initializes the UNNAMED mutex. Following a
 *   successful call to nxmutex_init(), the mutex may be used in subsequent
 *   calls to nxmutex_lock(), nxmutex_unlock(), and nxmutex_trylock().  The
 *   mutex remains usable until it is destroyed.
 *
 * Parameters:
 *   mutex - Semaphore to be initialized
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxmutex_init(FAR mutex_t *mutex);
#else
static inline_function int nxmutex_init(FAR mutex_t *mutex)
{
  int ret = nxsem_init(&mutex->sem, 0, NXSEM_NO_MHOLDER);

  if (ret >= 0)
    {
#ifdef CONFIG_PRIORITY_INHERITANCE
      nxsem_set_protocol(&mutex->sem, SEM_TYPE_MUTEX | SEM_PRIO_INHERIT);
#else
      nxsem_set_protocol(&mutex->sem, SEM_TYPE_MUTEX);
#endif
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nxmutex_get_holder
 *
 * Description:
 *   This function get the holder of the mutex referenced by 'mutex'.
 *   Note that this is inherently racy unless the calling thread is
 *   holding the mutex.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
pid_t nxmutex_get_holder(FAR mutex_t *mutex);
#else
static inline_function pid_t nxmutex_get_holder(FAR mutex_t *mutex)
{
  uint32_t mholder = mutex->sem.val.mholder & ~NXSEM_MBLOCKING_BIT;
  return NXSEM_MACQUIRED(mholder) ? (pid_t)mholder : -1;
}
#endif

/****************************************************************************
 * Name: nxmutex_is_hold
 *
 * Description:
 *   This function check whether the calling thread hold the mutex
 *   referenced by 'mutex'.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
bool nxmutex_is_hold(FAR mutex_t *mutex);
#else
static inline_function
bool nxmutex_is_hold(FAR mutex_t *mutex)
{
  return nxmutex_get_holder(mutex) == _SCHED_GETTID();
}
#endif

/****************************************************************************
 * Name: nxrmutex_is_hold
 *
 * Description:
 *   This function check whether the calling thread hold the recursive mutex
 *   referenced by 'rmutex'.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
bool nxrmutex_is_hold(FAR rmutex_t *rmutex);
#else
static inline_function bool nxrmutex_is_hold(FAR rmutex_t *rmutex)
{
  return nxmutex_is_hold(&rmutex->mutex);
}
#endif

/****************************************************************************
 * Name: nxmutex_ticklock
 *
 * Description:
 *   This function attempts to lock the mutex referenced by 'mutex'.  If the
 *   mutex value is (<=) zero, then the calling task will not return until it
 *   successfully acquires the lock or timed out
 *
 * Input Parameters:
 *   mutex   - Mutex object
 *   delay   - Ticks to wait from the start time until the semaphore is
 *             posted.  If ticks is zero, then this function is equivalent
 *             to nxmutex_trylock().
 *
 * Returned Value:
 *   OK        The mutex successfully acquires
 *   EINVAL    The mutex argument does not refer to a valid mutex.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The mutex could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxmutex_ticklock(FAR mutex_t *mutex, clock_t delay);
#else
static inline_function
int nxmutex_ticklock(FAR mutex_t *mutex, clock_t delay)
{
  clock_t end;
  int ret;

  ret = nxmutex_trywait(&mutex->sem);
  if (ret >= 0)
    {
      nxmutex_add_backtrace(mutex);
    }
  else
    {
      /* Wait until we get the lock or until the timeout expires */

      end = clock_delay2abstick(delay);

      for (; ; )
        {
          ret = nxsem_tickwait_slow(&mutex->sem, delay);
          if (ret >= 0)
            {
              nxmutex_add_backtrace(mutex);
              break;
            }
          else if (ret != -EINTR && ret != -ECANCELED)
            {
              break;
            }

          delay = end - clock();
          if ((int32_t)delay <= 0)
            {
              ret = -ETIMEDOUT;
              break;
            }
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nxmutex_clocklock
 *
 * Description:
 *   This function attempts to lock the mutex referenced by 'mutex'.  If the
 *   mutex value is (<=) zero, then the calling task will not return until it
 *   successfully acquires the lock or timed out
 *
 * Input Parameters:
 *   mutex   - Mutex object
 *   clockid - The clock to be used as the time base
 *   abstime - The absolute time when the mutex lock timed out
 *
 * Returned Value:
 *   OK        The mutex successfully acquires
 *   EINVAL    The mutex argument does not refer to a valid mutex.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The mutex could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxmutex_clocklock(FAR mutex_t *mutex, clockid_t clockid,
                      FAR const struct timespec *abstime);
#else
static inline_function
int nxmutex_clocklock(FAR mutex_t *mutex, clockid_t clockid,
                      FAR const struct timespec *abstime)
{
  int ret;

  /* Try to take the semaphore without waiting. */

  ret = nxmutex_trywait(&mutex->sem);
  if (ret >= 0)
    {
      nxmutex_add_backtrace(mutex);
    }
  else
    {
      /* Wait until we get the lock or until the timeout expires */

      for (; ; )
        {
          if (abstime)
            {
              ret = nxsem_clockwait_slow(&mutex->sem, clockid, abstime);
            }
          else
            {
              ret = nxmutex_wait(&mutex->sem);
            }

          if (ret >= 0)
            {
              nxmutex_add_backtrace(mutex);
              break;
            }
          else if (ret != -EINTR && ret != -ECANCELED)
            {
              break;
            }
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nxmutex_timedlock
 *
 * Description:
 *   This function attempts to lock the mutex .  If the mutex value
 *   is (<=) zero,then the calling task will not return until it
 *   successfully acquires the lock or timed out
 *
 * Input Parameters:
 *   mutex   - Mutex object
 *   timeout - The time when mutex lock timed out
 *
 * Returned Value:
 *   OK        The mutex successfully acquires
 *   EINVAL    The mutex argument does not refer to a valid mutex.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The mutex could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxmutex_timedlock(FAR mutex_t *mutex, unsigned int timeout);
#else
static inline_function
int nxmutex_timedlock(FAR mutex_t *mutex, unsigned int timeout)
{
  struct timespec now;
  struct timespec delay;
  struct timespec rqtp;

  clock_gettime(CLOCK_MONOTONIC, &now);
  clock_ticks2time(&delay, MSEC2TICK(timeout));
  clock_timespec_add(&now, &delay, &rqtp);

  /* Wait until we get the lock or until the timeout expires */

  return nxmutex_clocklock(mutex, CLOCK_MONOTONIC, &rqtp);
}
#endif

/****************************************************************************
 * Name: nxmutex_lock
 *
 * Description:
 *   This function attempts to lock the mutex referenced by 'mutex'.  The
 *   mutex is implemented with a semaphore, so if the semaphore value is
 *   (<=) zero, then the calling task will not return until it successfully
 *   acquires the lock.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxmutex_lock(FAR mutex_t *mutex);
#else
static inline_function int nxmutex_lock(FAR mutex_t *mutex)
{
  int ret;

  for (; ; )
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxmutex_wait(&mutex->sem);
      if (ret >= 0)
        {
          nxmutex_add_backtrace(mutex);
          break;
        }
      else if (ret != -EINTR && ret != -ECANCELED)
        {
          break;
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nrxmutex_lock
 *
 * Description:
 *   This function attempts to lock the recursive mutex referenced by
 *   'rmutex'.The recursive mutex can be locked multiple times in the same
 *   thread.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxrmutex_lock(FAR rmutex_t *rmutex);
#else
static inline_function int nxrmutex_lock(FAR rmutex_t *rmutex)
{
  int ret = OK;

  if (!nxrmutex_is_hold(rmutex))
    {
      for (; ; )
        {
          /* Take the semaphore (perhaps waiting) */

          ret = nxmutex_wait(&rmutex->mutex.sem);
          if (ret >= 0 || (ret != -EINTR && ret != -ECANCELED))
            {
              break;
            }
        }

      if (ret >= 0)
        {
          /* We have just acquired the underlying mutex from a non-held
           * state.  Reset count to zero in case the previous holder was
           * killed/cancelled while still holding the mutex (recovered via
           * the dead-holder path in nxsem_wait_slow): in that case the
           * count was left non-zero and would otherwise leak forever.
           */

          rmutex->count = 0;
        }
    }

  if (ret >= 0)
    {
      DEBUGASSERT(rmutex->count < UINT_MAX);
      ++rmutex->count;

      nxmutex_add_backtrace(&rmutex->mutex);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nxmutex_trylock
 *
 * Description:
 *   This function locks the mutex only if the mutex is currently not locked.
 *   If the mutex has been locked already, the call returns without blocking.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 *     -EINVAL - Invalid attempt to lock the mutex
 *     -EAGAIN - The mutex is not available.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxmutex_trylock(FAR mutex_t *mutex);
#else
static inline_function int nxmutex_trylock(FAR mutex_t *mutex)
{
  int ret;

  ret = nxmutex_trywait(&mutex->sem);
  if (ret >= 0)
    {
      nxmutex_add_backtrace(mutex);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nxrmutex_trylock
 *
 * Description:
 *   This function locks the recursive mutex if the recursive mutex is
 *   currently not locked or the same thread call.
 *   If the recursive mutex is locked and other thread call it,
 *   the call returns without blocking.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 *     -EINVAL - Invalid attempt to lock the recursive mutex
 *     -EAGAIN - The recursive mutex is not available.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxrmutex_trylock(FAR rmutex_t *rmutex);
#else
static inline_function
int nxrmutex_trylock(FAR rmutex_t *rmutex)
{
  int ret = OK;

  if (!nxrmutex_is_hold(rmutex))
    {
      ret = nxmutex_trylock(&rmutex->mutex);
      if (ret >= 0)
        {
          rmutex->count = 0;
        }
    }

  if (ret >= 0)
    {
      DEBUGASSERT(rmutex->count < UINT_MAX);
      ++rmutex->count;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nxrmutex_ticklock
 *
 * Description:
 *   This function attempts to lock the mutex referenced by 'mutex'.  If the
 *   mutex value is (<=) zero, then the calling task will not return until it
 *   successfully acquires the lock or timed out
 *
 * Input Parameters:
 *   rmutex  - Rmutex object
 *   delay   - Ticks to wait from the start time until the semaphore is
 *             posted.  If ticks is zero, then this function is equivalent
 *             to nxrmutex_trylock().
 *
 * Returned Value:
 *   OK        The mutex successfully acquires
 *   EINVAL    The mutex argument does not refer to a valid mutex.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The mutex could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxrmutex_ticklock(FAR rmutex_t *rmutex, uint32_t delay);
#else
static inline_function
int nxrmutex_ticklock(FAR rmutex_t *rmutex, uint32_t delay)
{
  int ret = OK;

  if (!nxrmutex_is_hold(rmutex))
    {
      ret = nxmutex_ticklock(&rmutex->mutex, delay);
      if (ret >= 0)
        {
          rmutex->count = 0;
        }
    }

  if (ret >= 0)
    {
      DEBUGASSERT(rmutex->count < UINT_MAX);
      ++rmutex->count;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nxrmutex_clocklock
 *
 * Description:
 *   This function attempts to lock the mutex referenced by 'mutex'.  If the
 *   mutex value is (<=) zero, then the calling task will not return until it
 *   successfully acquires the lock or timed out
 *
 * Input Parameters:
 *   rmutex  - Rmutex object
 *   clockid - The clock to be used as the time base
 *   abstime - The absolute time when the mutex lock timed out
 *
 * Returned Value:
 *   OK        The mutex successfully acquires
 *   EINVAL    The mutex argument does not refer to a valid mutex.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The mutex could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxrmutex_clocklock(FAR rmutex_t *rmutex, clockid_t clockid,
                       FAR const struct timespec *abstime);
#else
static inline_function
int nxrmutex_clocklock(FAR rmutex_t *rmutex, clockid_t clockid,
                       FAR const struct timespec *abstime)
{
  int ret = OK;

  if (!nxrmutex_is_hold(rmutex))
    {
      ret = nxmutex_clocklock(&rmutex->mutex, clockid, abstime);
      if (ret >= 0)
        {
          rmutex->count = 0;
        }
    }

  if (ret >= 0)
    {
      DEBUGASSERT(rmutex->count < UINT_MAX);
      ++rmutex->count;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nxrmutex_timedlock
 *
 * Description:
 *   This function attempts to lock the mutex .  If the mutex value
 *   is (<=) zero,then the calling task will not return until it
 *   successfully acquires the lock or timed out
 *
 * Input Parameters:
 *   rmutex  - Rmutex object
 *   timeout - The time when mutex lock timed out
 *
 * Returned Value:
 *   OK        The mutex successfully acquires
 *   EINVAL    The mutex argument does not refer to a valid mutex.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The mutex could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *   ECANCELED May be returned if the thread is canceled while waiting.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxrmutex_timedlock(FAR rmutex_t *rmutex, unsigned int timeout);
#else
static inline_function
int nxrmutex_timedlock(FAR rmutex_t *rmutex, unsigned int timeout)
{
  int ret = OK;

  if (!nxrmutex_is_hold(rmutex))
    {
      ret = nxmutex_timedlock(&rmutex->mutex, timeout);
      if (ret >= 0)
        {
          rmutex->count = 0;
        }
    }

  if (ret >= 0)
    {
      DEBUGASSERT(rmutex->count < UINT_MAX);
      ++rmutex->count;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nxmutex_unlock
 *
 * Description:
 *   This function attempts to unlock the mutex referenced by 'mutex'.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxmutex_unlock(FAR mutex_t *mutex);
#else
static inline_function
int nxmutex_unlock(FAR mutex_t *mutex)
{
  nxmutex_remove_backtrace(mutex);
  return nxmutex_post(&mutex->sem);
}
#endif

/****************************************************************************
 * Name: nxrmutex_unlock
 *
 * Description:
 *   This function attempts to unlock the recursive mutex
 *   referenced by 'rmutex'.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxrmutex_unlock(FAR rmutex_t *rmutex);
#else
static inline_function
int nxrmutex_unlock(FAR rmutex_t *rmutex)
{
  int ret = OK;

  DEBUGASSERT(rmutex->count > 0u);

  if (--rmutex->count == 0u)
    {
      ret = nxmutex_unlock(&rmutex->mutex);
      if (ret < 0)
        {
          ++rmutex->count;
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nrxmutex_breaklock
 *
 * Description:
 *   This function attempts to break the recursive mutex
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxrmutex_breaklock(FAR rmutex_t *rmutex, FAR unsigned int *count);
#else
static inline_function
int nxrmutex_breaklock(FAR rmutex_t *rmutex, FAR unsigned int *count)
{
  int ret = OK;

  *count = 0u;
  if (nxrmutex_is_hold(rmutex))
    {
      *count = rmutex->count;
      rmutex->count = 0u;
      ret = nxmutex_unlock(&rmutex->mutex);
      if (ret < 0)
        {
          rmutex->count = *count;
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nxrmutex_restorelock
 *
 * Description:
 *   This function attempts to restore the recursive mutex.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxrmutex_restorelock(FAR rmutex_t *rmutex, unsigned int count);
#else
static inline_function
int nxrmutex_restorelock(FAR rmutex_t *rmutex, unsigned int count)
{
  int ret = OK;

  if (count != 0u)
    {
      ret = nxmutex_lock(&rmutex->mutex);
      if (ret >= 0)
        {
          rmutex->count = count;
        }
    }

  return ret;
}
#endif

#define nxrmutex_set_protocol(rmutex, protocol) \
        nxmutex_set_protocol(&(rmutex)->mutex, protocol)
#define nxrmutex_getprioceiling(rmutex, prioceiling) \
        nxmutex_getprioceiling(&(rmutex)->mutex, prioceiling)
#define nxrmutex_setprioceiling(rmutex, prioceiling, old_ceiling) \
        nxmutex_setprioceiling(&(rmutex)->mutex, prioceiling, old_ceiling)

/****************************************************************************
 * Name: nxmutex_is_locked
 *
 * Description:
 *   This function get the lock state the mutex referenced by 'mutex'.
 *   Note that this is inherently racy unless the calling thread is
 *   holding the mutex.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
bool nxmutex_is_locked(FAR mutex_t *mutex);
#else
static inline_function bool nxmutex_is_locked(FAR mutex_t *mutex)
{
  return NXSEM_MACQUIRED(mutex->sem.val.mholder);
}
#endif

/****************************************************************************
 * Name: nxmutex_destroy
 *
 * Description:
 *   This function destroys the specified mutex. After a successful call,
 *   the mutex is no longer valid and must not be used in subsequent calls
 *   to nxmutex_lock(), nxmutex_unlock(), or nxmutex_trylock() unless it is
 *   reinitialized with nxmutex_init().
 *
 * Parameters:
 *   mutex - Semaphore to be destroyed
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxmutex_destroy(FAR mutex_t *mutex);
#else
static inline_function int nxmutex_destroy(FAR mutex_t *mutex)
{
  return nxsem_destroy(&mutex->sem);
}
#endif

/****************************************************************************
 * Name: nxmutex_reset
 *
 * Description:
 *   This function reset lock state.
 *
 * Parameters:
 *   mutex - mutex descriptor.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
void nxmutex_reset(FAR mutex_t *mutex);
#else
static inline_function void nxmutex_reset(FAR mutex_t *mutex)
{
  nxmutex_remove_backtrace(mutex);
  nxsem_reset(&mutex->sem, 1);
}
#endif

/****************************************************************************
 * Name: nxmutex_breaklock
 *
 * Description:
 *   This function attempts to break the mutex
 *
 * Parameters:
 *   mutex   - Mutex descriptor.
 *   locked  - Is the mutex break success
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxmutex_breaklock(FAR mutex_t *mutex,
                                             FAR unsigned int *locked);
#else
static inline_function int nxmutex_breaklock(FAR mutex_t *mutex,
                                             FAR unsigned int *locked)
{
  int ret = OK;

  *locked = false;
  ret = nxmutex_unlock(mutex);
  if (ret >= 0)
    {
      *locked = true;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nxmutex_restorelock
 *
 * Description:
 *   This function attempts to restore the mutex.
 *
 * Parameters:
 *   mutex   - mutex descriptor.
 *   locked  - true: it's mean that the mutex is broke success
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxmutex_restorelock(FAR mutex_t *mutex, unsigned int locked);
#else
static inline_function int nxmutex_restorelock(FAR mutex_t *mutex,
                                               unsigned int locked)
{
  return locked ? nxmutex_lock(mutex) : OK;
}
#endif

/****************************************************************************
 * Name: nxmutex_set_protocol
 *
 * Description:
 *   This function attempts to set the priority protocol of a mutex.
 *
 * Parameters:
 *   mutex        - mutex descriptor.
 *   protocol     - mutex protocol value to set.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxmutex_set_protocol(FAR mutex_t *mutex, int protocol);
#else
static inline_function int nxmutex_set_protocol(FAR mutex_t *mutex,
                                                int protocol)
{
  return nxsem_set_protocol(&mutex->sem, protocol);
}
#endif

/****************************************************************************
 * Name: nxmutex_getprioceiling
 *
 * Description:
 *   This function attempts to get the priority ceiling of a mutex.
 *
 * Parameters:
 *   mutex        - mutex descriptor.
 *   prioceiling  - location to return the mutex priority ceiling.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure
 *
 ****************************************************************************/

#ifdef CONFIG_PRIORITY_PROTECT
#  ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int
nxmutex_getprioceiling(FAR const mutex_t *mutex, FAR int *prioceiling);
#  else
static inline_function int nxmutex_getprioceiling(FAR const mutex_t *mutex,
                                                  FAR int *prioceiling)
{
  return nxsem_getprioceiling(&mutex->sem, prioceiling);
}
#  endif
#endif

/****************************************************************************
 * Name: nxmutex_setprioceiling
 *
 * Description:
 *   This function attempts to set the priority ceiling of a mutex.
 *
 * Parameters:
 *   mutex        - mutex descriptor.
 *   prioceiling  - mutex priority ceiling value to set.
 *   old_ceiling  - location to return the mutex ceiling priority set before.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure
 *
 ****************************************************************************/

#ifdef CONFIG_PRIORITY_PROTECT
#  ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxmutex_setprioceiling(FAR mutex_t *mutex, int prioceiling,
                           FAR int *old_ceiling);
#  else
static inline_function int nxmutex_setprioceiling(FAR mutex_t *mutex,
                                                  int prioceiling,
                                                  FAR int *old_ceiling)
{
  return nxsem_setprioceiling(&mutex->sem, prioceiling, old_ceiling);
}
#  endif
#endif

/****************************************************************************
 * Name: nxrmutex_init
 *
 * Description:
 *   This function initializes the UNNAMED recursive mutex. Following a
 *   successful call to nxrmutex_init(), the recursive mutex may be used in
 *   subsequent calls to nxrmutex_lock(), nxrmutex_unlock(),
 *   and nxrmutex_trylock(). The recursive mutex remains usable
 *   until it is destroyed.
 *
 * Parameters:
 *   rmutex - Recursive mutex to be initialized
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxrmutex_init(FAR rmutex_t *rmutex);
#else
static inline_function int nxrmutex_init(FAR rmutex_t *rmutex)
{
  rmutex->count = 0;
  return nxmutex_init(&rmutex->mutex);
}
#endif

/****************************************************************************
 * Name: nxrmutex_destroy
 *
 * Description:
 *   This function destroy the UNNAMED recursive mutex.
 *
 * Parameters:
 *   rmutex - Recursive mutex to be destroyed
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxrmutex_destroy(FAR rmutex_t *rmutex);
#else
static inline_function int nxrmutex_destroy(FAR rmutex_t *rmutex)
{
  int ret = nxmutex_destroy(&rmutex->mutex);

  if (ret >= 0)
    {
      rmutex->count = 0;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nxrmutex_is_recursive
 *
 * Description:
 *   This function check whether the recursive mutex is currently held
 *   recursively. That is, whether it's locked more than once by the
 *   current holder.
 *   Note that this is inherently racy unless the calling thread is
 *   holding the mutex.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *  If rmutex has returned to True recursively, otherwise returns false.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
bool nxrmutex_is_recursive(FAR rmutex_t *rmutex);
#else
static inline_function bool nxrmutex_is_recursive(FAR rmutex_t *rmutex)
{
  return rmutex->count > 1;
}
#endif

/****************************************************************************
 * Name: nxrmutex_get_holder
 *
 * Description:
 *   This function get the holder of the mutex referenced by 'mutex'.
 *   Note that this is inherently racy unless the calling thread is
 *   holding the mutex.
 *
 * Parameters:
 *   rmutex - Rmutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
int nxrmutex_get_holder(FAR rmutex_t *rmutex);
#else
static inline_function int nxrmutex_get_holder(FAR rmutex_t *rmutex)
{
  return nxmutex_get_holder(&rmutex->mutex);
}
#endif

/****************************************************************************
 * Name: nxrmutex_is_locked
 *
 * Description:
 *   This function get the lock state the recursive mutex
 *   referenced by 'rmutex'.
 *   Note that this is inherently racy unless the calling thread is
 *   holding the mutex.
 *
 * Parameters:
 *   rmutex - Recursive mutex descriptor.
 *
 * Return Value:
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
bool nxrmutex_is_locked(FAR rmutex_t *rmutex);
#else
static inline_function bool nxrmutex_is_locked(FAR rmutex_t *rmutex)
{
  return nxmutex_is_locked(&rmutex->mutex);
}
#endif

/****************************************************************************
 * Name: nxrmutex_reset
 *
 * Description:
 *   This function reset lock state.
 *
 * Parameters:
 *   rmutex - rmutex descriptor.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SEM_MUTEX_NOINLINE
void nxrmutex_reset(FAR rmutex_t *rmutex);
#else
static inline_function void nxrmutex_reset(FAR rmutex_t *rmutex)
{
  rmutex->count = 0;
  nxmutex_reset(&rmutex->mutex);
}
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_MUTEX_H */
