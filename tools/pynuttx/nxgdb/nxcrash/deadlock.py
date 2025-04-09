############################################################################
# tools/pynuttx/nxgdb/nxcrash/deadlock.py
#
# SPDX-License-Identifier: Apache-2.0
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

from __future__ import annotations

import gdb

from .. import utils


class DeadLock(gdb.Command):
    """Detect and report if threads have deadlock."""

    def __init__(self):
        super().__init__("deadlock", gdb.COMMAND_USER)

    def has_deadlock(self, pid):
        """Check if the thread has a deadlock"""
        tcb = utils.get_tcb(pid)
        if not tcb or not tcb["waitobj"]:
            return False

        sem = tcb["waitobj"].cast(utils.lookup_type("sem_t").pointer())
        if not utils.sem_is_mutex(sem["flags"]):
            return False

        # It's waiting on a mutex
        mutex = tcb["waitobj"].cast(utils.lookup_type("mutex_t").pointer())
        holder = mutex["holder"]
        if holder in self.holders:
            return True

        self.holders.append(holder)
        return self.has_deadlock(holder)

    def collect(self, tcbs):
        """Collect the deadlock information"""

        detected = []
        collected = []
        for tcb in tcbs:
            self.holders = []  # Holders for this tcb
            pid = tcb["pid"]
            if pid in detected or not self.has_deadlock(tcb["pid"]):
                continue

            # Deadlock detected
            detected.append(pid)
            detected.extend(self.holders)
            collected.append((pid, self.holders))

        return collected

    def diagnose(self, *args, **kwargs):
        collected = self.collect(utils.get_tcbs())

        return {
            "title": "Deadlock Report",
            "summary": f"{'No' if not collected else len(collected)} deadlocks",
            "result": "fail" if collected else "pass",
            "command": "crash deadlock",
            "deadlocks": {int(pid): [i for i in h] for pid, h in collected},
        }

    @utils.dont_repeat_decorator
    def invoke(self, args, from_tty):
        collected = self.collect(utils.get_tcbs())
        if not collected:
            gdb.write("No deadlock detected.\n")
            return

        for pid, holders in collected:
            gdb.write(f'Thread {pid} "{utils.get_task_name(pid)}" has deadlocked!\n')
            gdb.write(f"  holders: {pid}->")
            gdb.write("->".join(str(pid) for pid in holders))
            gdb.write("\n")
