############################################################################
# tools/pynuttx/nxtrace/perfetto_trace.py
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

import logging
from enum import IntEnum

try:
    from google.protobuf.message_factory import GetMessageClass
except ImportError:
    print("Please execute the following command to install dependencies:")
    print("pip install protobuf==4.25.3")
    exit(1)

from . import perfetto_trace_pb2 as pb2

logger = logging.getLogger(__name__)
logger.setLevel(logging.WARN)


class LogPriority:
    UNSPECIFIED = pb2.AndroidLogPriority.PRIO_UNSPECIFIED
    UNUSED = pb2.AndroidLogPriority.PRIO_UNUSED
    VERBOSE = pb2.AndroidLogPriority.PRIO_VERBOSE
    DEBUG = pb2.AndroidLogPriority.PRIO_DEBUG
    INFO = pb2.AndroidLogPriority.PRIO_INFO
    WARN = pb2.AndroidLogPriority.PRIO_WARN
    ERROR = pb2.AndroidLogPriority.PRIO_ERROR
    FATAL = pb2.AndroidLogPriority.PRIO_FATAL


class TaskState(IntEnum):
    RUNNING = 0
    INTERRUPTIBLE = 1
    UNINTERRUPTIBLE = 2
    STOPPED = 4
    TRACED = 8
    DEAD = 16
    ZOMBIE = 32
    PARKED = 64
    INVALID = 128


class TaskInfo:
    def __init__(self, comm, pid, prio, tid=None):
        self.pid = pid
        self.comm = comm
        self.prio = prio
        self.tid = tid if tid is not None else pid


class TraceHead:
    def __init__(self, ts, pid, cpu, tid=None):
        self.ts = ts
        self.pid = pid
        self.cpu = cpu
        self.tid = tid if tid is not None else pid


class PerfettoTraceBadArgsError(ValueError):
    pass


class TraceClassFactory:
    def __new__(cls, message_name):
        message_descriptor = cls._get_message_descriptor(message_name)
        return GetMessageClass(message_descriptor)

    @classmethod
    def _get_message_descriptor(cls, message_name):
        """
        Recursively find the descriptor for nested messages.
        """
        parts = message_name.split(".")
        descriptor = pb2.DESCRIPTOR

        for part in parts:
            # Handle FileDescriptor
            if hasattr(descriptor, "message_types_by_name"):
                if part not in descriptor.message_types_by_name:
                    raise ValueError(f"Message {message_name} not found in descriptor.")
                descriptor = descriptor.message_types_by_name[part]
            # Handle MessageDescriptor
            elif hasattr(descriptor, "nested_types_by_name"):
                if part not in descriptor.nested_types_by_name:
                    raise ValueError(f"Message {message_name} not found in descriptor.")
                descriptor = descriptor.nested_types_by_name[part]
            else:
                raise ValueError(f"Invalid descriptor type for {message_name}")
        return descriptor


class TraceInstanceFactory:
    def __new__(cls, message_name, **kwargs):
        # Get the message class
        message_class = TraceClassFactory(message_name)
        instance = message_class()
        for field, value in kwargs.items():
            setattr(instance, field, value)
        return instance


class PerfettoTrace:
    DEFAULT_TRUSTED_PACKET_SEQUENCE_ID = 0

    SLICE_BEGIN = pb2.TrackEvent.Type.TYPE_SLICE_BEGIN
    SLICE_END = pb2.TrackEvent.Type.TYPE_SLICE_END
    INSTANT = pb2.TrackEvent.Type.TYPE_INSTANT
    COUNTER = pb2.TrackEvent.Type.TYPE_COUNTER

    def __init__(self, filename: str):
        """Create a trace"""
        self.flush_threshold = 10000

        self.trace = pb2.Trace()
        self.file = open(filename, "wb")

        self.current_uuid = 0

    def fill_debug_annotation_value(self, debug_annotation, value):
        if isinstance(value, dict):
            for key in value:
                dict_entry = pb2.DebugAnnotation()
                dict_entry.name = key
                self.fill_debug_annotation_value(dict_entry, value[key])
                debug_annotation.dict_entries.append(dict_entry)
        elif isinstance(value, list):
            for item in value:
                array_value = pb2.DebugAnnotation()
                self.fill_debug_annotation_value(array_value, item)
                debug_annotation.array_values.append(array_value)
        elif isinstance(value, bool):
            debug_annotation.bool_value = value
        elif isinstance(value, int):
            debug_annotation.int_value = value
        elif isinstance(value, float):
            debug_annotation.double_value = value
        elif isinstance(value, str):
            debug_annotation.string_value = value
        elif value is None:
            debug_annotation.string_value = "None"
        else:
            raise PerfettoTraceBadArgsError(
                f"cannot convert value {value} of type {type(value)} to DebugAnnotation value"
            )

    def next_uuid(self):
        self.current_uuid += 1
        return self.current_uuid

    def init(self):
        # Initialize clock snapshots
        clocks_class = TraceClassFactory("ClockSnapshot.Clock")
        clocks = [clocks_class(clock_id=id, timestamp=0) for id in range(1, 7)]

        clock_snapshot = TraceInstanceFactory("ClockSnapshot")
        clock_snapshot.primary_trace_clock = pb2.BUILTIN_CLOCK_BOOTTIME
        clock_snapshot.clocks.extend(clocks)

        pkt = TraceInstanceFactory("TracePacket", trusted_packet_sequence_id=1)
        pkt.clock_snapshot.CopyFrom(clock_snapshot)
        self.trace.packet.append(pkt)

        # Configure trace packets
        pkt = self.trace.packet.add(trusted_packet_sequence_id=1)
        pkt.trace_config.buffers.add().size_kb = 1024
        pkt.trace_config.data_sources.add().config.name = "track_event"

        pkt = self.trace.packet.add(trusted_packet_sequence_id=2, sequence_flags=1)
        pkt.trace_packet_defaults.track_event_defaults.track_uuid = 1
        pkt.trace_packet_defaults.timestamp_clock_id = 1

        pkt = self.trace.packet.add()
        pkt.trusted_packet_sequence_id = 1
        pkt.trace_config.buffers.add().size_kb = 1024
        pkt.trace_config.data_sources.add().config.name = "track_event"

        pkt = self.trace.packet.add()
        pkt.trusted_packet_sequence_id = 2
        pkt.trace_packet_defaults.track_event_defaults.track_uuid = 1
        pkt.trace_packet_defaults.timestamp_clock_id = 1
        pkt.sequence_flags = 1

    def flush(self):
        """Flush trace. This creates a perfetto trace packet and writes to disk."""
        self.file.write(self.trace.SerializeToString())
        self.file.flush()
        self.trace = pb2.Trace()

    def __del__(self):
        self.flush()
        self.file.close()

    # Ftrace common events API

    def log(self, ts, tag, pid, tid=None, prio=LogPriority.INFO, msg="", args=None):
        tid = tid if tid is not None else pid
        pkt = self.trace.packet.add()

        log_event = TraceInstanceFactory(
            "AndroidLogPacket.LogEvent",
            timestamp=ts,
            log_id=pb2.AndroidLogId.LID_DEFAULT,
            pid=pid,
            tid=tid,
            uid=0,
            tag=tag,
            prio=prio,
            message=msg,
        )
        pkt.android_log.events.append(log_event)
        logger.debug(pkt)

    def ftrace_event(self, head: TraceHead, event):
        pkt = self.trace.packet.add()
        ftrace_events = TraceInstanceFactory("FtraceEventBundle", cpu=head.cpu)
        events = TraceInstanceFactory("FtraceEvent", pid=head.pid, timestamp=head.ts)

        event_descriptor = event.DESCRIPTOR

        for oneof in events.DESCRIPTOR.oneofs:
            for field in oneof.fields:
                if field.message_type == event_descriptor:
                    getattr(events, field.name).CopyFrom(event)
                    break

        ftrace_events.event.append(events)
        pkt.ftrace_events.CopyFrom(ftrace_events)
        logger.debug(pkt)
        return pkt

    # Ftrace events API

    def sched_wakeup_new(self, head: TraceHead, task: TaskInfo):
        wakeup = TraceInstanceFactory(
            "SchedWakeupNewFtraceEvent",
            pid=task.pid,
            comm=task.comm,
            prio=task.prio,
            target_cpu=head.cpu,
        )
        return self.ftrace_event(head, wakeup)

    def sched_waking(self, head: TraceHead, task: TaskInfo):
        waking = TraceInstanceFactory(
            "SchedWakingFtraceEvent",
            pid=task.pid,
            comm=task.comm,
            prio=task.prio,
            target_cpu=head.cpu,
        )
        return self.ftrace_event(head, waking)

    def sched_wakeup(self, head: TraceHead, task: TaskInfo):
        wakeup = TraceInstanceFactory(
            "SchedWakeupFtraceEvent",
            pid=task.pid,
            comm=task.comm,
            prio=task.prio,
            target_cpu=head.cpu,
        )
        return self.ftrace_event(head, wakeup)

    def sched_switch(
        self,
        head: TraceHead,
        prev_state=0,
        task_prev: TaskInfo = None,
        task_next: TaskInfo = None,
    ):
        switch = pb2.SchedSwitchFtraceEvent(
            prev_comm=task_prev.comm,
            prev_pid=task_prev.pid,
            prev_prio=task_prev.prio,
            prev_state=prev_state,
            next_comm=task_next.comm,
            next_pid=task_next.pid,
            next_prio=task_next.prio,
        )

        return self.ftrace_event(head, switch)

    def irq_entry(self, head: TraceHead, irq: int, name: str, handler: int):
        irq_entry = TraceInstanceFactory(
            "IrqHandlerEntryFtraceEvent", irq=irq, name=name, handler=handler
        )
        return self.ftrace_event(head, irq_entry)

    def irq_exit(self, head: TraceHead, irq, ret=0):
        irq_exit = TraceInstanceFactory("IrqHandlerExitFtraceEvent", irq=irq, ret=ret)
        return self.ftrace_event(head, irq_exit)

    def print(self, head: TraceHead, buf=None):
        print_event = TraceInstanceFactory("PrintFtraceEvent", buf=buf)
        return self.ftrace_event(head, print_event)

    # Atrace events API

    def atrace_begin(self, head: TraceHead, msg):
        return self.print(head, f"B|{head.pid}|{msg}")

    def atrace_end(self, head: TraceHead):
        return self.print(head, "E")

    def atrace_async_begin(self, head: TraceHead, msg, cookie):
        return self.print(head, f"S|{head.pid}|{msg}|{cookie}")

    def atrace_async_end(self, head: TraceHead, msg, cookie):
        return self.print(head, f"F|{head.pid}|{msg}|{cookie}")

    def atrace_async_for_track_begin(self, head: TraceHead, track_name, msg, cookie):
        return self.print(head, f"G|{head.pid}|{track_name}|{msg}|{cookie}")

    def atrace_async_for_track_end(self, head: TraceHead, track_name, msg, cookie):
        return self.print(head, f"H|{head.pid}|{track_name}|{msg}|{cookie}")

    def atrace_instant(self, head: TraceHead, msg):
        return self.print(head, f"I|{head.pid}|{msg}")

    def atrace_instant_for_track(self, head: TraceHead, track_name, msg):
        return self.print(head, f"N|{head.pid}|{track_name}|{msg}")

    def atrace_int(self, head: TraceHead, msg, value):
        return self.print(head, f"C|{head.pid}|{msg}|{value}")

    def add_process(self, uuid, pid, name):
        self.trace.packet.append(
            pb2.TracePacket(
                track_descriptor=pb2.TrackDescriptor(
                    uuid=uuid, process=pb2.ProcessDescriptor(pid=pid, process_name=name)
                ),
                trusted_packet_sequence_id=self.DEFAULT_TRUSTED_PACKET_SEQUENCE_ID,
            )
        )

    def add_thread(self, uuid, parent_uuid, tid, pid, name):
        self.trace.packet.append(
            pb2.TracePacket(
                track_descriptor=pb2.TrackDescriptor(
                    uuid=uuid,
                    parent_uuid=parent_uuid,
                    thread=pb2.ThreadDescriptor(pid=pid, tid=tid, thread_name=name),
                ),
                trusted_packet_sequence_id=self.DEFAULT_TRUSTED_PACKET_SEQUENCE_ID,
            )
        )

    def trace_event(self, uuid, ts, type, name=None, flow_ids=[], args={}):
        pkt = self.trace.packet.add()
        pkt.timestamp = ts
        pkt.track_event.type = type
        pkt.track_event.track_uuid = uuid
        pkt.trusted_packet_sequence_id = self.DEFAULT_TRUSTED_PACKET_SEQUENCE_ID

        if args:
            if not isinstance(args, dict):
                raise PerfettoTraceBadArgsError("slice args is not dict")
            annotations = []

            for key in args:
                debug_annotation = pb2.DebugAnnotation()
                debug_annotation.name = key
                self.fill_debug_annotation_value(debug_annotation, args[key])

                annotations.append(debug_annotation)

            pkt.track_event.debug_annotations.extend(annotations)

        if name:
            pkt.track_event.name = name
        if flow_ids:
            pkt.track_event.flow_ids = flow_ids

    def trace_slice(self, uuid, start, end, name, flow_ids=[], args={}):
        self.trace_event(uuid, start, self.SLICE_BEGIN, name, flow_ids, args)
        self.trace_event(uuid, end, self.SLICE_END)
