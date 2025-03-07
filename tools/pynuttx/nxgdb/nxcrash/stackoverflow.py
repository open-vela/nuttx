############################################################################
# tools/pynuttx/nxgdb/nxcrash/stackoverflow.py
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
from ..stack import Stack

STACK_FILL_THRESHOLD = 0.9


class CrashStackOverflow(gdb.Command):
    """Analyse and collect the stack overflow threads"""

    def __init__(self):
        super().__init__("crash stackoverflow", gdb.COMMAND_USER)

    def collect(self, tcbs):
        """Collect the stack overflow information"""

        collected = []
        for tcb in tcbs:
            st = Stack(
                utils.get_task_name(tcb),
                hex(tcb["entry"]["pthread"]),  # should use main?
                int(tcb["stack_base_ptr"]),
                int(tcb["stack_alloc_ptr"]),
                int(tcb["adj_stack_size"]),
                utils.get_sp(tcb),
                4,
            )

            filled = st.max_usage() / st._stack_size
            if filled > STACK_FILL_THRESHOLD:
                collected.append((tcb, filled))

        return collected

    def invoke(self, arg: str, from_tty: bool) -> None:
        collected = self.collect(utils.get_tcbs())
        if not collected:
            gdb.write("No stack overflow found\n")
            return

        print(f"Found stack overflow threads\n{'PID':<4} {'name':<10} {'FILLED'}")
        for tcb, filled in collected:
            print(f"{tcb['pid']:<4} {utils.get_task_name(tcb):<10} {filled:.2%}")

    def diagnose(self, *args, **kwargs):
        collected = self.collect(utils.get_tcbs())
        return {
            "title": "Stack OverFlow Report",
            "summary": f"{'No' if not collected else len(collected)} threads{'s' if len(collected) != 1 else ''} found",
            "result": "fail" if collected else "pass",
            "command": "crash stackoverflow",
            "thread": [
                {"pid": tcb["pid"], "name": utils.get_task_name(tcb), "filled": filled}
                for tcb, filled in collected
            ],
        }
