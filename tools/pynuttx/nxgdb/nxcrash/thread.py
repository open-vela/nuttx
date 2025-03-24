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

import gdb

from .. import utils


class CrashThread(gdb.Command):
    """Analyse and collect the crashed threads"""

    def __init__(self):
        super().__init__("crash thread", gdb.COMMAND_USER)

    def collect(self, tcbs):
        """Collect threads that crashed information"""

        collected = []
        for tcb in tcbs:
            pid = int(tcb["pid"])
            for frame in utils.get_thread_frames(pid):
                if "_assert" in utils.get_frame_func_name(frame):
                    collected.append(tcb)
                    break

        return collected or utils.get_running_tcbs()

    def invoke(self, arg: str, from_tty: bool) -> None:
        collected = self.collect(utils.get_tcbs())

        if not collected:
            gdb.write("No crashed threads found.\n")
            return

        print(f"Found crashed threads\n{'PID':<4} {'Name':<10}")
        for tcb in collected:
            print("{:<4} {:<10}".format(tcb["pid"], utils.get_task_name(tcb)))

    def diagnose(self, *args, **kwargs):
        tcbs = self.collect(utils.get_tcbs())
        return {
            "title": "Threads that seem crashed",
            "summary": f"{'No' if not tcbs else len(tcbs)} threads seem crashed",
            "result": "fail" if tcbs else "pass",
            "command": "crash thread",
            "thread": [
                {
                    "pid": tcb["pid"],
                    "name": utils.get_task_name(tcb),
                    "backtrace": utils.Backtrace(utils.get_backtrace(int(tcb["pid"]))),
                }
                for tcb in tcbs
            ],
        }
