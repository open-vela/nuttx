############################################################################
# tools/pynuttx/nxgdb/noteram.py
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

import gdb
from nxelf.elf import ELFParser
from nxtrace.trace import NoteFactory

from . import utils


class NoteRam:
    def __init__(self, driver_name: str):
        """Initialize NoteRam object with driver structure"""
        self.driver = None
        self.buffer = None
        self.bufsize = 0
        self.head = 0
        self.read = 0

        self.driver = utils.gdb_eval_or_none(driver_name)
        if not self.driver:
            return
        self.head = int(self.driver["header"]["head"])
        self.read = int(self.driver["header"]["read"])
        self.bufsize = int(self.driver["bufsize"])
        self.buffer = (
            gdb.selected_inferior()
            .read_memory(self.driver["buffer"].cast("uintptr_t"), self.bufsize)
            .tobytes()
        )

    def events(self):
        """Generate events from circular buffer"""
        if not self.buffer or not self.bufsize:
            return

        uintptr_size = utils.sizeof("uintptr_t")
        buffer, bufsize, head, read = (
            self.buffer,
            self.bufsize,
            self.head,
            self.read,
        )
        while (unread := (head - read) % bufsize) > 0:
            event_len = int(buffer[read])
            if event_len <= 0 or event_len > unread:
                raise BufferError(
                    f"Invalid event length: {event_len}, available space {unread}"
                )
            end = read + event_len
            event = bytes(
                buffer[read:end]
                if end <= bufsize
                else buffer[read:bufsize] + buffer[: end % bufsize]
            )
            yield event
            read = (
                read + ((event_len + uintptr_size - 1) & ~(uintptr_size - 1))
            ) % bufsize


class NoteRamCommand(gdb.Command):
    """GDB command to parse and dump noteram data into a Perfetto trace file"""

    def __init__(self):
        if not utils.get_field_nitems("struct noteram_driver_s", "buffer"):
            return
        super().__init__("noteram", gdb.COMMAND_USER)
        self.initialized = False

    def init_note_factory(self):
        """Initialize NoteFactory once and only once"""
        if self.initialized:
            return True
        if path := gdb.objfiles()[0].filename:
            NoteFactory.init_instance(ELFParser(path), "trace.perfetto")
            self.initialized = True
            return True
        return False

    def collect_notes(self):
        """Collect and parse notes from noteram events"""
        notes = []
        if not self.init_note_factory():
            print("NoteFactory initialization failed")
            return notes

        noteram = NoteRam("g_noteram_driver")
        if not noteram.buffer:
            print("No valid noteram buffer")
            return notes

        for idx, event in enumerate(noteram.events()):
            try:
                if note := NoteFactory.parse(event):
                    notes.append(note)
            except Exception as e:
                print(f"Error parsing event {idx} ({event}): {e}")
        return notes

    @utils.dont_repeat_decorator
    def invoke(self, args, from_tty):
        NoteFactory.dump(self.collect_notes())

    def diagnose(self, *args, **kwargs):
        return {
            "title": "Noteram Report",
            "summary": "noteram dump",
            "command": "noteram",
            "result": "info",
            "message": self.collect_notes(),
        }
