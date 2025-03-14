############################################################################
# tools/pynuttx/nxgdb/nxcrash/busyloop.py
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

BUSYLOOP_THRESHOLD = 0.9 / utils.get_ncpus()


class CrashBusyloop(gdb.Command):
    """Analyse and collect the busyloop threads"""

    def __init__(self):
        if not utils.get_symbol_value("CONFIG_SCHED_CPULOAD_NONE"):
            super().__init__("crash busyloop", gdb.COMMAND_USER)

    def collect(self, tcbs):
        """Get busyloop thread"""
        collected = []
        cpuload_total = int(utils.parse_and_eval("g_cpuload_total"))

        for tcb in tcbs:
            cpuload = int(tcb["ticks"]) / cpuload_total
            if cpuload > BUSYLOOP_THRESHOLD and not utils.task_is_idle(tcb):
                collected.append((tcb, cpuload))

        return collected

    def invoke(self, arg: str, from_tty: bool) -> None:
        collected = self.collect(utils.get_tcbs())
        if not collected:
            gdb.write("No busyloop threads found.\n")
            return

        print(f"Found busyloop thread\n{'PID':<4} {'Name':<10} Load")
        for tcb, cpuload in collected:
            print(
                "{:<4} {:<10} {}".format(tcb["pid"], utils.get_task_name(tcb), cpuload)
            )

    def diagnose(self, *args, **kwargs):
        collected = self.collect(utils.get_tcbs())

        return {
            "title": "Busyloop Report",
            "summary": f"{'No' if not collected else len(collected)} threads occur busyloop",
            "result": "fail" if collected else "pass",
            "command": "crash busyloop",
            "thread": [
                {
                    "pid": tcb["pid"],
                    "name": utils.get_task_name(tcb),
                    "cpuload": cpuload,
                }
                for tcb, cpuload in collected
            ],
        }
