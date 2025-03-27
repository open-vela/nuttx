############################################################################
# tools/pynuttx/nxgdb/critmon.py
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
from typing import Optional

import gdb

from . import utils


@dataclass
class CriticalData:
    """Data class to hold critical data information"""

    pid: Optional[int] = None
    name: Optional[str] = None
    premp_max: Optional[int] = None
    premp_max_caller: Optional[int] = None
    crit_max: Optional[int] = None
    crit_max_caller: Optional[int] = None
    busywait_start: Optional[int] = None
    busywait_caller: Optional[int] = None
    busywait_max: Optional[int] = None
    busywait_max_caller: Optional[int] = None
    busywait_total: Optional[int] = None
    run_max: Optional[int] = None
    run_time: Optional[int] = None

    def __str__(self):
        """Format the information for output"""

        def fmt(field, hex: bool = False):
            val = getattr(self, field)
            if val is None:
                return "-"
            if hex and isinstance(val, int):
                return hex(val)
            return str(val)

        formatter = (
            "{:<3}    {:<18}     "
            "{:<10} {:<9}        "
            "{:<11} {:<9}        "
            "{:<13} {:<6}        "
            "{:<11} {:<9}        "
            "{:<11}        "
            "{:<12}        {}"
        )

        return formatter.format(
            fmt("pid"),
            fmt("name"),
            fmt("premp_max"),
            fmt("premp_max_caller", hex=True),
            fmt("crit_max"),
            fmt("crit_max_caller", hex=True),
            fmt("busywait_start"),
            fmt("busywait_caller", hex=True),
            fmt("busywait_max"),
            fmt("busywait_max_caller", hex=True),
            fmt("busywait_total"),
            fmt("run_max"),
            fmt("run_time"),
        )


class Critmon(gdb.Command):
    """Dump critical resource busy-wait time"""

    def __init__(self):
        if not utils.get_symbol_value("CONFIG_SYSTEM_CRITMONITOR"):
            print("Critmon is not enabled in the current configuration")
            return
        super().__init__("critmon", gdb.COMMAND_USER)

    def process_task_critical_data(self, tcb):
        """Generate information for a single task's critical section"""
        try:
            data = CriticalData()
            for name in utils.get_fieldnames("struct tcb_s"):
                if hasattr(CriticalData, name):
                    value = getattr(tcb, name)
                    if name == "name":
                        value = value.cast(gdb.lookup_type("char").pointer()).string()
                    setattr(data, name, value)
            return data
        except Exception as e:
            print(f"Error processing task critical section information: {str(e)}")
            return None

    def process_global_critical_data(self):
        """Generate information for global critical sections"""
        try:
            output = []
            for cpu, data in enumerate(
                [CriticalData() for _ in range(utils.get_ncpus())]
            ):
                setattr(data, "name", f"CPU{cpu}")
                for key in ("premp_max", "crit_max", "busywait_max", "busywait_total"):
                    value = utils.gdb_eval_or_none(f"g_{key}")
                    if not value:
                        continue
                    setattr(data, key, int(value[cpu]))
                output.append(data)
            return output
        except Exception as e:
            print(f"Error processing global critical section information: {str(e)}")
            return []

    @utils.dont_repeat_decorator
    def invoke(self, args, from_tty):
        # header for critical section output
        print(
            "PID    NAME                   "
            "Preemption MaxCaller        "
            "CritSection MaxCaller        "
            "BusywaitStart Caller        "
            "BusywaitMax MaxCaller        "
            "BusywaitAll        "
            "ThreadRunMax        ThreadRunAll"
        )

        try:
            # process global critical data
            for data in self.process_global_critical_data():
                print(data)

            # process task critical data
            for tcb in utils.get_tcbs():
                if not (data := self.process_task_critical_data(tcb)):
                    continue
                print(data)
        except Exception as e:
            print(f"Error invoking critmon command: {str(e)}")

    def diagnose(self, *args, **kwargs):
        return {
            "title": "critmon report",
            "summary": "critical resource busy-wait time dump",
            "result": "info",
            "command": "critmon",
            "data": gdb.execute("critmon", to_string=True),
        }
