############################################################################
# tools/pynuttx/nxgdb/notesnap.py
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

from . import utils

CONFIG_SMP = utils.has_field("struct notesnap_chunk_s", "cpu")
CONFIG_DRIVERS_NOTESNAP_NBUFFERS = utils.get_field_nitems("struct notesnap_s", "buffer")


class NoteSnapChunk:
    def __init__(self, notetype, pid, args, cpu=0):
        self.notetype = notetype
        self.pid = pid
        self.args = args
        self.cpu = cpu

    @property
    def taskname(self):
        return utils.get_task_name(utils.get_tcb(self.pid))

    def __str__(self):
        args = [
            self.taskname,
        ]
        if CONFIG_SMP:
            args.append(self.cpu)
        args.extend([self.pid, self.notetype, self.args])
        return (
            "[%s] " + ("[CPU%d] " if CONFIG_SMP else "") + "[%d] %-16s %#x"
        ) % tuple(args)


class NoteSnap(gdb.Command):
    """Dump notesnap"""

    def __init__(self):
        if not CONFIG_DRIVERS_NOTESNAP_NBUFFERS:
            return
        super().__init__("notesnap", gdb.COMMAND_USER)

    def parse_notesnap(self):
        """Parse note snapshot data"""

        notesnap = utils.parse_and_eval("g_notesnap")
        notesnap_type = utils.parse_and_eval("g_notesnap_type")
        output = []

        for note in utils.ArrayIterator(notesnap.buffer):
            if not note:
                continue

            chunk = {
                "notetype": notesnap_type[note["type"]].string(),
                "pid": note.pid,
                "args": note.args,
                "cpu": note.cpu if CONFIG_SMP else 0,
            }

            output.append(NoteSnapChunk(**chunk))
        return output

    def diagnose(self, *args, **kwargs):
        return {
            "title": "Notesnap Report",
            "summary": "notesnap dump",
            "command": "notesnap",
            "result": "pass",
            "message": self.parse_notesnap(),
        }

    @utils.dont_repeat_decorator
    def invoke(self, args, from_tty):
        try:
            print(*(self.parse_notesnap()), sep="\n")
        except Exception as e:
            print(f"Error happened during notesnap: {e}")
