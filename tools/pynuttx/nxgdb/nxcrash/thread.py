############################################################################
# tools/pynuttx/nxgdb/nxcrash/thread.py
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

from dataclasses import dataclass
from typing import List

import gdb

from .. import utils


@dataclass
class ThreadInfo:
    """Thread information"""

    pid: int
    name: str
    backtrace: utils.Backtrace
    crashed: bool = False


class CrashThread(gdb.Command):
    """Analyse and collect the crashed threads"""

    def __init__(self):
        super().__init__("crash thread", gdb.COMMAND_USER)

    def collect(self, tcbs) -> List[ThreadInfo]:
        """Collect threads that crashed information"""

        def is_thread_crashed(pid):
            """Check if the thread is crashed"""
            # Check if the thread is in the crashed state
            for frame in utils.get_thread_frames(pid):
                if "_assert" in utils.get_frame_func_name(frame):
                    return True

        def get_thread_info(tcb):
            pid = int(tcb.pid)
            return ThreadInfo(
                pid=pid,
                name=utils.get_task_name(tcb),
                backtrace=utils.Backtrace(utils.get_backtrace(pid)),
                crashed=True,
            )

        collected = []
        pid = 0
        for tcb in tcbs:
            pid = int(tcb["pid"])
            if is_thread_crashed(pid):
                collected.append(get_thread_info(tcb))

        return collected or [get_thread_info(tcb) for tcb in utils.get_running_tcbs()]

    @utils.dont_repeat_decorator
    def invoke(self, arg: str, from_tty: bool) -> None:
        collected = self.collect(utils.get_tcbs())

        if not collected:
            gdb.write("No crashed threads found.\n")
            return

        print(f"Found {len(collected)} crashed threads\n{'PID':<4} {'Name':<10}")
        for thread in collected:
            print("{:<4} {:<10}".format(thread.pid, thread.name))
            for _, func, _, _ in thread.backtrace:
                print("{:<4} {:<10}".format("", func))

    def diagnose(self, *args, **kwargs):
        threads = self.collect(utils.get_tcbs())
        return {
            "title": "Threads that seem crashed",
            "summary": f"{'No' if not threads else len(threads)} threads seem crashed",
            "result": "fail" if threads else "pass",
            "command": "crash thread",
            "thread": [
                {
                    "pid": thread.pid,
                    "name": thread.name,
                    "entry": utils.get_task_entry(utils.get_tcb(thread.pid)),
                    "backtrace": thread.backtrace,
                }
                for thread in threads
            ],
        }
