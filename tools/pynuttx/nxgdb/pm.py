############################################################################
# tools/pynuttx/nxgdb/pm.py
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

from . import utils

PM_COUNT = utils.get_symbol_value("PM_COUNT")


class PMStateInfo:
    def __init__(self, wake_time, sleep_time):
        self.wake_time = wake_time
        self.sleep_time = sleep_time
        self.total_time = wake_time + sleep_time


def has_inferior():
    inferior = gdb.selected_inferior()
    return inferior is not None and inferior.is_valid()


def get_pm_state_info_list(domain):
    state_info_list = []
    sum_time = 0

    for state in range(PM_COUNT):
        wake_time = int(domain["wake"][state]["tv_sec"])
        sleep_time = int(domain["sleep"][state]["tv_sec"])

        sum_time += wake_time + sleep_time
        state_info_list.append(PMStateInfo(wake_time, sleep_time))

    sum_time = sum_time if sum_time > 0 else 1
    return state_info_list, sum_time


class Pmconfig(gdb.Command):
    """Display power management configuration information

    Usage: pmconfig
    """

    state_formatter = "{:<12} {:<12} {:<12} {:<12}"
    state_header = ("WAKE", "SLEEP", "TOTAL")

    def __init__(self):
        super().__init__("pmconfig", gdb.COMMAND_USER)

    def get_time_str(self, time_val, sum_time):
        percentage = (time_val / sum_time) * 100 if sum_time > 0 else 0
        return f"{int(time_val)}s {int(percentage):02}%"

    def print_state_info(self, domain_name, domain):
        state_path = f"/proc/pm/state{domain_name[-1]}"
        gdb.write(f"{state_path}:\n")
        gdb.write(self.state_formatter.format(domain_name, *self.state_header) + "\n")

        state_info_list, sum_time = get_pm_state_info_list(domain)
        pm_state = gdb.parse_and_eval("g_pm_state")

        for state in range(PM_COUNT):
            state_info = state_info_list[state]
            wake_str = self.get_time_str(state_info.wake_time, sum_time)
            sleep_str = self.get_time_str(state_info.sleep_time, sum_time)
            total_str = self.get_time_str(state_info.total_time, sum_time)

            state_name = pm_state[state].string()
            gdb.write(
                self.state_formatter.format(state_name, wake_str, sleep_str, total_str)
                + "\n"
            )

    def invoke(self, arg: str, from_tty: bool) -> None:
        if not has_inferior():
            return

        pmdomains = gdb.parse_and_eval("g_pmdomains")
        ndomains = utils.nitems(pmdomains)

        domains = [f"DOMAIN{i}" for i in range(ndomains)]
        for i, domain_name in enumerate(domains):
            domain = pmdomains[i]
            self.print_state_info(domain_name, domain)
