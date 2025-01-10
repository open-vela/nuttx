/****************************************************************************
 * include/nuttx/android/binder.h
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

#ifndef __INCLUDE_NUTTX_ANDROID_BINDER_H__
#define __INCLUDE_NUTTX_ANDROID_BINDER_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <nuttx/fs/ioctl.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BINDER_IOC(t, n, s)                 (((t) << 24) | ((s) << 16) | _IOC(_BINDERBASE, n))
#define BINDER_IOC_SIZE(c)                  (((c) >> 16) & 0xff)
#define BINDER_IO(t, n)                     BINDER_IOC(t, n, 0)
#define BINDER_IOR(t, n, a)                 BINDER_IOC(t, n, (sizeof(a)))
#define BINDER_IOW(t, n, a)                 BINDER_IOC(t, n, (sizeof(a)))
#define BINDER_IOWR(t, n, a)                BINDER_IOC(t, n, (sizeof(a)))

#define BINDER_WRITE_READ                   BINDER_IOWR('b', 1, struct binder_write_read)
#define BINDER_SET_IDLE_TIMEOUT             BINDER_IOW('b', 3, int64_t)
#define BINDER_SET_MAX_THREADS              BINDER_IOW('b', 5, uint32_t)
#define BINDER_SET_IDLE_PRIORITY            BINDER_IOW('b', 6, int32_t)
#define BINDER_SET_CONTEXT_MGR              BINDER_IOW('b', 7, int32_t)
#define BINDER_THREAD_EXIT                  BINDER_IOW('b', 8, int32_t)
#define BINDER_VERSION                      BINDER_IOWR('b', 9, struct binder_version)
#define BINDER_GET_NODE_DEBUG_INFO          BINDER_IOWR('b', 11, struct binder_node_debug_info)
#define BINDER_GET_NODE_INFO_FOR_REF        BINDER_IOWR('b', 12, struct binder_node_info_for_ref)
#define BINDER_SET_CONTEXT_MGR_EXT          BINDER_IOW('b', 13, struct flat_binder_object)
#define BINDER_FREEZE                       BINDER_IOW('b', 14, struct binder_freeze_info)
#define BINDER_GET_FROZEN_INFO              BINDER_IOWR('b', 15, struct binder_frozen_status_info)
#define BINDER_ENABLE_ONEWAY_SPAM_DETECTION BINDER_IOW('b', 16, uint32_t)
#define BINDER_GET_EXTENDED_ERROR           BINDER_IOWR('b', 17, struct binder_extended_error)
#define BINDER_FLUSH                        BINDER_IO('b', 18)

#define B_PACK_CHARS(c1, c2, c3, c4) \
  (((c1) << 24) | ((c2) << 16) | ((c3) << 8) | (c4))
#define B_TYPE_LARGE 0x85

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum
{
  BINDER_TYPE_BINDER      = B_PACK_CHARS('s', 'b', '*', B_TYPE_LARGE),
  BINDER_TYPE_WEAK_BINDER = B_PACK_CHARS('w', 'b', '*', B_TYPE_LARGE),
  BINDER_TYPE_HANDLE      = B_PACK_CHARS('s', 'h', '*', B_TYPE_LARGE),
  BINDER_TYPE_WEAK_HANDLE = B_PACK_CHARS('w', 'h', '*', B_TYPE_LARGE),
  BINDER_TYPE_FD          = B_PACK_CHARS('f', 'd', '*', B_TYPE_LARGE),
  BINDER_TYPE_FDA         = B_PACK_CHARS('f', 'd', 'a', B_TYPE_LARGE),
  BINDER_TYPE_PTR         = B_PACK_CHARS('p', 't', '*', B_TYPE_LARGE),
};

/**
 * enum flat_binder_object_shifts: shift values for flat_binder_object_flags
 * FLAT_BINDER_FLAG_SCHED_POLICY_SHIFT: shift for getting scheduler policy.
 *
 */

enum flat_binder_object_shifts
{
  FLAT_BINDER_FLAG_SCHED_POLICY_SHIFT = 9,
};

/**
 * enum flat_binder_object_flags - flags for use in flat_binder_object.flags
 */

enum flat_binder_object_flags
{
  /**
   * FLAT_BINDER_FLAG_PRIORITY_MASK: bit-mask for min scheduler priority
   *
   * These bits can be used to set the minimum scheduler priority
   * at which transactions into this node should run. Valid values
   * in these bits depend on the scheduler policy encoded in
   * FLAT_BINDER_FLAG_SCHED_POLICY_MASK.
   *
   * For SCHED_NORMAL/SCHED_BATCH, the valid range is between [-20..19]
   * For SCHED_FIFO/SCHED_RR, the value can run between [1..99]
   */

  FLAT_BINDER_FLAG_PRIORITY_MASK = 0xff,

  /**
   * FLAT_BINDER_FLAG_ACCEPTS_FDS: whether the node accepts fds.
   */

  FLAT_BINDER_FLAG_ACCEPTS_FDS = 0x100,

  /**
   * FLAT_BINDER_FLAG_SCHED_POLICY_MASK: bit-mask for scheduling policy
   *
   * These two bits can be used to set the min scheduling policy at which
   * transactions on this node should run. These match the UAPI
   * scheduler policy values, eg:
   * 00b: SCHED_NORMAL
   * 01b: SCHED_FIFO
   * 10b: SCHED_RR
   * 11b: SCHED_BATCH
   */

  FLAT_BINDER_FLAG_SCHED_POLICY_MASK =
       3u << FLAT_BINDER_FLAG_SCHED_POLICY_SHIFT,

  /**
   * FLAT_BINDER_FLAG_INHERIT_RT: whether the node inherits RT policy
   *
   * Only when set, calls into this node will inherit a real-time
   * scheduling policy from the caller (for synchronous transactions).
   */

  FLAT_BINDER_FLAG_INHERIT_RT = 0x800,

  /**
   * FLAT_BINDER_FLAG_TXN_SECURITY_CTX: request security contexts
   *
   * Only when set, causes senders to include their security
   * context
   */

  FLAT_BINDER_FLAG_TXN_SECURITY_CTX = 0x1000,
};

#ifdef BINDER_IPC_32BIT
typedef uint32_t binder_size_t;
typedef uint32_t binder_uintptr_t;
#else
typedef uint64_t binder_size_t;
typedef uint64_t binder_uintptr_t;
typedef uint64_t binder_unaligned_uintptr_t __attribute__((aligned(1)));
#endif

/**
 * struct binder_object_header
 *  - header shared by all binder metadata objects.
 * type: type of the object
 */

struct binder_object_header
{
  uint32_t type;
};

/* This is the flattened representation of a Binder object for transfer
 * between processes.  The 'offsets' supplied as part of a binder transaction
 * contains offsets into the data where these structures occur.  The Binder
 * driver takes care of re-writing the structure type and data as it moves
 * between processes.
 */

begin_packed_struct struct flat_binder_object
{
  struct binder_object_header hdr;
  uint32_t flags;

  /* 8 bytes of data. */

  union
  {
    binder_uintptr_t binder; /* Local object */
    uint32_t handle;         /* Remote object */
  };

  /* Extra data associated with local object */

  binder_uintptr_t cookie;
} end_packed_struct;

/**
 * struct binder_fd_object - describes a filedescriptor to be fixed up.
 * hdr: common header structure
 * pad_flags: padding to remain compatible with old userspace code
 * pad_binder: padding to remain compatible with old userspace code
 * fd: file descriptor
 * cookie: opaque data, used by user-space
 */

struct binder_fd_object
{
  struct binder_object_header hdr;
  uint32_t pad_flags;
  union
  {
    binder_uintptr_t pad_binder;
    uint32_t fd;
  };

  binder_uintptr_t cookie;
};

/* struct binder_buffer_object - object describing a userspace buffer
 * hdr: common header structure
 * flags: one or more BINDER_BUFFER_* flags
 * buffer: address of the buffer
 * length: length of the buffer
 * parent: index in offset array pointing to parent buffer
 * parent_offset: offset in parent pointing to this buffer
 *
 * A binder_buffer object represents an object that the
 * binder kernel driver can copy verbatim to the target
 * address space. A buffer itself may be pointed to from
 * within another buffer, meaning that the pointer inside
 * that other buffer needs to be fixed up as well. This
 * can be done by setting the BINDER_BUFFER_FLAG_HAS_PARENT
 * flag in flags, by setting parent buffer to the index
 * in the offset array pointing to the parent binder_buffer_object,
 * and by setting parent_offset to the offset in the parent buffer
 * at which the pointer to this buffer is located.
 */

struct binder_buffer_object
{
  struct binder_object_header hdr;
  uint32_t flags;
  binder_uintptr_t buffer;
  binder_size_t length;
  binder_size_t parent;
  binder_size_t parent_offset;
};

enum
{
  BINDER_BUFFER_FLAG_HAS_PARENT = 0x01,
};

/* struct binder_fd_array_object -
 * object describing an array of fds in a buffer
 * hdr: common header structure
 * pad: padding to ensure correct alignment
 * num_fds: number of file descriptors in the buffer
 * parent: index in offset array to buffer holding the fd array
 * parent_offset: start offset of fd array in the buffer
 *
 * A binder_fd_array object represents an array of file
 * descriptors embedded in a binder_buffer_object. It is
 * different from a regular binder_buffer_object because it
 * describes a list of file descriptors to fix up, not an opaque
 * blob of memory, and hence the kernel needs to treat it differently.
 *
 * An example of how this would be used is with Android's
 * native_handle_t object, which is a struct with a list of integers
 * and a list of file descriptors. The native_handle_t struct itself
 * will be represented by a struct binder_buffer_objct, whereas the
 * embedded list of file descriptors is represented by a
 * struct binder_fd_array_object with that binder_buffer_object as
 * a parent.
 */

struct binder_fd_array_object
{
  struct binder_object_header hdr;
  uint32_t pad;
  binder_size_t num_fds;
  binder_size_t parent;
  binder_size_t parent_offset;
};

/* On 64-bit platforms where user code may run in 32-bits the driver must
 * translate the buffer (and local binder) addresses appropriately.
 */

struct binder_write_read
{
  binder_size_t write_size;         /* Bytes to write */
  binder_size_t write_consumed;     /* Bytes consumed by driver */
  binder_uintptr_t write_buffer;
  binder_size_t read_size;          /* Bytes to read */
  binder_size_t read_consumed;      /* Bytes consumed by driver */
  binder_uintptr_t read_buffer;
};

/* Use with BINDER_VERSION, driver fills in fields. */

struct binder_version
{
  /* Driver protocol version -- increment with incompatible change */

  int32_t protocol_version;
};

/* This is the current protocol version. */

#ifdef BINDER_IPC_32BIT
#define BINDER_CURRENT_PROTOCOL_VERSION 7
#else
#define BINDER_CURRENT_PROTOCOL_VERSION 8
#endif

/* Use with BINDER_GET_NODE_DEBUG_INFO, driver reads ptr, writes to all
 * fields.
 * Set ptr to NULL for the first call to get the info for the first node, and
 * then repeat the call passing the previously returned value to get the next
 * nodes.  ptr will be 0 when there are no more nodes.
 */

struct binder_node_debug_info
{
  binder_uintptr_t ptr;
  binder_uintptr_t cookie;
  uint32_t has_strong_ref;
  uint32_t has_weak_ref;
};

struct binder_node_info_for_ref
{
  uint32_t handle;
  uint32_t strong_count;
  uint32_t weak_count;
  uint32_t reserved1;
  uint32_t reserved2;
  uint32_t reserved3;
};

struct binder_freeze_info
{
  uint32_t pid;
  uint32_t enable;
  uint32_t timeout_ms;
};

struct binder_frozen_status_info
{
  uint32_t pid;

  /* Process received sync transactions since last frozen
   * bit 0: received sync transaction after being frozen
   * bit 1: new pending sync transaction during freezing
   */

  uint32_t sync_recv;

  /* Process received async transactions since last frozen */

  uint32_t async_recv;
};

/* struct binder_extened_error - extended error information
 * id:      identifier for the failed operation
 * command: command as defined by binder_driver_return_protocol
 * param:   parameter holding a negative errno value
 *
 * Used with BINDER_GET_EXTENDED_ERROR. This extends the error information
 * returned by the driver upon a failed operation. Userspace can pull this
 * data to properly handle specific error scenarios.
 */

struct binder_extended_error
{
  uint32_t id;
  uint32_t command;
  int32_t param;
};

/* NOTE: Two special error codes you should check for when calling
 * in to the driver are:
 *
 * EINTR -- The operation has been interupted.  This should be
 * handled by retrying the ioctl() until a different error code
 * is returned.
 *
 * ECONNREFUSED -- The driver is no longer accepting operations
 * from your process.  That is, the process is being destroyed.
 * You should handle this by exiting from your process.  Note
 * that once this error code is returned, all further calls to
 * the driver from any thread will return this same code.
 */

enum transaction_flags
{
  TF_ONE_WAY     = 0x01, /* This is a one-way call: async, no return */
  TF_ROOT_OBJECT = 0x04, /* Contents are the component's root object */
  TF_STATUS_CODE = 0x08, /* Contents are a 32-bit status code */
  TF_ACCEPT_FDS  = 0x10, /* Allow replies with file descriptors */
  TF_CLEAR_BUF   = 0x20, /* Clear buffer on txn complete */
  TF_UPDATE_TXN  = 0x40, /* Update the outdated pending async txn */
};

begin_packed_struct struct binder_transaction_data
{
  /* The first two are only used for bcTRANSACTION and brTRANSACTION,
   * identifying the target and contents of the transaction.
   */

  union
  {
    /* Target descriptor of command transaction */

    uint32_t handle;

    /* Target descriptor of return transaction */

    binder_uintptr_t ptr;
  } target;
  binder_uintptr_t cookie;  /* Target object cookie */
  uint32_t code;            /* Transaction command */

  /* General information about the transaction. */

  uint32_t flags;
  int32_t sender_pid;
  uint32_t sender_euid;
  binder_size_t data_size;      /* Number of bytes of data */
  binder_size_t offsets_size;   /* Number of bytes of offsets */

  /* If this transaction is inline, the data immediately
   * follows here; otherwise, it ends with a pointer to
   * the data buffer.
   */

  union
  {
    struct
    {
      /* Transaction data */

      binder_uintptr_t buffer;

      /* Offsets from buffer to flat_binder_object structs */

      binder_uintptr_t offsets;
    } ptr;
    uint8_t buf[8];
  } data;
} end_packed_struct;

struct binder_transaction_data_secctx
{
  struct binder_transaction_data transaction_data;
  binder_uintptr_t secctx;
};

struct binder_transaction_data_sg
{
  struct binder_transaction_data transaction_data;
  binder_size_t buffers_size;
};

struct binder_ptr_cookie
{
  binder_uintptr_t ptr;
  binder_uintptr_t cookie;
};

begin_packed_struct struct binder_handle_cookie
{
  uint32_t handle;
  binder_uintptr_t cookie;
} end_packed_struct;

struct binder_pri_desc
{
  int32_t priority;
  uint32_t desc;
};

struct binder_pri_ptr_cookie
{
  int32_t priority;
  binder_uintptr_t ptr;
  binder_uintptr_t cookie;
};

enum binder_driver_return_protocol
{
  BR_ERROR = BINDER_IOR('r', 0, int32_t),

  /* int: error code */

  BR_OK = BINDER_IO('r', 1),

  /* No parameters! */

  BR_TRANSACTION_SEC_CTX = BINDER_IOR('r', 2,
                                      struct binder_transaction_data_secctx),

  /* binder_transaction_data_secctx: the received command. */

  BR_TRANSACTION = BINDER_IOR('r', 2, struct binder_transaction_data),
  BR_REPLY = BINDER_IOR('r', 3, struct binder_transaction_data),

  /* binder_transaction_data: the received command. */

  BR_ACQUIRE_RESULT = BINDER_IOR('r', 4, int32_t),

  /* Not currently supported
   * int: 0 if the last bcATTEMPT_ACQUIRE was not successful.
   * Else the remote object has acquired a primary reference.
   */

  BR_DEAD_REPLY = BINDER_IO('r', 5),

  /* The target of the last transaction (either a bcTRANSACTION or
   * a bcATTEMPT_ACQUIRE) is no longer with us.  No parameters.
   */

  BR_TRANSACTION_COMPLETE = BINDER_IO('r', 6),

  /* No parameters... always refers to the last transaction requested
   * (including replies).  Note that this will be sent even for
   * asynchronous transactions.
   */

  BR_INCREFS = BINDER_IOR('r', 7, struct binder_ptr_cookie),
  BR_ACQUIRE = BINDER_IOR('r', 8, struct binder_ptr_cookie),
  BR_RELEASE = BINDER_IOR('r', 9, struct binder_ptr_cookie),
  BR_DECREFS = BINDER_IOR('r', 10, struct binder_ptr_cookie),

  /* void *: ptr to binder
   * void *: cookie for binder
   */

  BR_ATTEMPT_ACQUIRE = BINDER_IOR('r', 11, struct binder_pri_ptr_cookie),

  /* Not currently supported
   * int:  priority
   * void *: ptr to binder
   * void *: cookie for binder
   */

  BR_NOOP = BINDER_IO('r', 12),

  /* No parameters.  Do nothing and examine the next command.  It exists
   * primarily so that we can replace it with a BR_SPAWN_LOOPER command.
   */

  BR_SPAWN_LOOPER = BINDER_IO('r', 13),

  /* No parameters.  The driver has determined that a process has no
   * threads waiting to service incoming transactions.  When a process
   * receives this command, it must spawn a new service thread and
   * register it via bcENTER_LOOPER.
   */

  BR_FINISHED = BINDER_IO('r', 14),

  /* Not currently supported
   * Stop threadpool thread
   */

  BR_DEAD_BINDER = BINDER_IOR('r', 15, binder_uintptr_t),

  /* void *: cookie */

  BR_CLEAR_DEATH_NOTIFICATION_DONE = BINDER_IOR('r', 16, binder_uintptr_t),

  /* void *: cookie */

  BR_FAILED_REPLY = BINDER_IO('r', 17),

  /* The last transaction (either a bcTRANSACTION or
   * a bcATTEMPT_ACQUIRE) failed (e.g. out of memory).  No parameters.
   */

  BR_FROZEN_REPLY = BINDER_IO('r', 18),

  /* The target of the last transaction (either a bcTRANSACTION or
   * a bcATTEMPT_ACQUIRE) is frozen.  No parameters.
   */

  BR_ONEWAY_SPAM_SUSPECT = BINDER_IO('r', 19),

  /* Current process sent too many oneway calls to target, and the last
   * asynchronous transaction makes the allocated async buffer size exceed
   * detection threshold.  No parameters.
   */

  BR_TRANSACTION_PENDING_FROZEN = BINDER_IO('r', 20),

  /* The target of the last async transaction is frozen.  No parameters. */
};

enum binder_driver_command_protocol
{
  BC_TRANSACTION = BINDER_IOW('c', 0, struct binder_transaction_data),
  BC_REPLY       = BINDER_IOW('c', 1, struct binder_transaction_data),

  /* binder_transaction_data: the sent command. */

  BC_ACQUIRE_RESULT = BINDER_IOW('c', 2, int32_t),

  /* Not currently supported
   * int:  0 if the last BR_ATTEMPT_ACQUIRE was not successful.
   * Else you have acquired a primary reference on the object.
   */

  BC_FREE_BUFFER = BINDER_IOW('c', 3, binder_uintptr_t),

  /* void *: ptr to transaction data received on a read */

  BC_INCREFS = BINDER_IOW('c', 4, uint32_t),
  BC_ACQUIRE = BINDER_IOW('c', 5, uint32_t),
  BC_RELEASE = BINDER_IOW('c', 6, uint32_t),
  BC_DECREFS = BINDER_IOW('c', 7, uint32_t),

  /* int: descriptor */

  BC_INCREFS_DONE = BINDER_IOW('c', 8, struct binder_ptr_cookie),
  BC_ACQUIRE_DONE = BINDER_IOW('c', 9, struct binder_ptr_cookie),

  /* void *: ptr to binder
   * void *: cookie for binder
   */

  BC_ATTEMPT_ACQUIRE = BINDER_IOW('c', 10, struct binder_pri_desc),

  /* Not currently supported
   * int: priority
   * int: descriptor
   */

  BC_REGISTER_LOOPER = BINDER_IO('c', 11),

  /* No parameters.
   * Register a spawned looper thread with the device.
   */

  BC_ENTER_LOOPER = BINDER_IO('c', 12),
  BC_EXIT_LOOPER  = BINDER_IO('c', 13),

  /* No parameters.
   * These two commands are sent as an application-level thread
   * enters and exits the binder loop, respectively.  They are
   * used so the binder can have an accurate count of the number
   * of looping threads it has available.
   */

  BC_REQUEST_DEATH_NOTIFICATION =  BINDER_IOW('c', 14,
                                              struct binder_handle_cookie),

  /* int: handle
   * void *: cookie
   */

  BC_CLEAR_DEATH_NOTIFICATION =  BINDER_IOW('c', 15,
                                            struct binder_handle_cookie),

  /* int: handle
   * void *: cookie
   */

  BC_DEAD_BINDER_DONE = BINDER_IOW('c', 16, binder_uintptr_t),

  /* void *: cookie */

  BC_TRANSACTION_SG = BINDER_IOW('c', 17, struct binder_transaction_data_sg),
  BC_REPLY_SG       = BINDER_IOW('c', 18, struct binder_transaction_data_sg),

  /* binder_transaction_data_sg: the sent command. */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int binder_initialize(void);

#endif /* __INCLUDE_NUTTX_ANDROID_BINDER_H__ */
