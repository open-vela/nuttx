############################################################################
# tools/pynuttx/nxgdb/tlsdump.py
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

import argparse

import gdb

from . import utils
from .lists import NxDQueue

CONFIG_TLS_TASK_NELEM = utils.get_field_nitems("struct task_info_s", "ta_telem")
CONFIG_TLS_NELEM = utils.get_field_nitems("struct tls_info_s", "tl_elem")

tls_info_s = utils.lookup_type("struct tls_info_s").pointer()


class TlsDump(gdb.Command):
    """Dump and check the integrity of tls_info and task_info"""

    def __init__(self):
        if not CONFIG_TLS_NELEM:
            print("TLS is not enabled in the current configuration")
            return
        super().__init__("tlsdump", gdb.COMMAND_USER)

    def parse_arguments(self, argv):
        parser = argparse.ArgumentParser(description=self.__doc__)
        parser.add_argument(
            "-p",
            "--pid",
            type=int,
            help="Dump the tls info of the thread/task represented by this pid",
            default=None,
        )
        parser.add_argument(
            "-c",
            "--check",
            action="store_true",
            help="Integrity check.All threads in a task must share the same task_info",
        )

        try:
            args = parser.parse_args(argv)
        except SystemExit:
            return None

        return args

    def dump_tls(self, tcb):
        """dump tls info"""

        try:
            if not tcb or not utils.get_tcb_type(tcb):
                return

            pid = int(tcb.pid)
            type = utils.get_tcb_type(tcb)
            task_info = tcb.stack_alloc_ptr.cast(tls_info_s).tl_task
            print(f"PID:{pid}, {type}, task_info addr:{hex(task_info)}")

            if type == "TASK":
                print("task tls elements:")
                for i in range(CONFIG_TLS_TASK_NELEM):
                    tls = utils.get_task_tls(pid, i)
                    print(f"{hex(tls)}")

            print("thread tls elements:")
            for i in range(CONFIG_TLS_NELEM):
                tls = utils.get_thread_tls(pid, i)
                print(f"{hex(tls)}")
        except gdb.error:
            return

    def check_corruption(self, tcb):
        """integrity check"""

        try:
            if not tcb or not utils.get_tcb_type(tcb):
                print("No tcb found, or the tcb type is invalid")
                return True

            # Get the task_info of the task, and compare it with the task_info of each thread
            # If the tcb is a task, get the task_info of the task
            # If the tcb is a thread, get the task_info of the task to which the thread belongs
            task = (
                tcb
                if utils.get_tcb_type(tcb) == "TASK"
                else utils.get_tcb(tcb.group.tg_pid)
            )
            if not task:
                print("Can not find the task within the group")
                return True

            task_info = task.stack_alloc_ptr.cast(tls_info_s).tl_task
            corrupted = False
            # Traverse all threads under this task through the group linked list
            for tcb in NxDQueue(task.group.tg_members, "struct tcb_s", "member"):
                # Get the task_info of this thread
                info = tcb.stack_alloc_ptr.cast(tls_info_s).tl_task
                # Only report when corrupted
                if info != task_info:
                    pid = int(tcb.pid)
                    print(
                        f"PID:{pid} is corrupted, task_info addr:{hex(task_info)}, got {hex(info)}"
                    )
                    corrupted = True
            return corrupted
        except gdb.error:
            print("Error occurred during integrity check")
            return True

    def invoke(self, args, from_tty):
        args = self.parse_arguments(gdb.string_to_argv(args))
        if not args:
            return

        # tlsdump -c
        # Do integrity check
        if args.check:
            corrupted = any(self.check_corruption(tcb) for tcb in utils.get_tcbs())
            print(f"Check: {'FAILED' if corrupted else 'PASS'}")
            return

        # tlsdump / tlsdump -p pid
        pid = args.pid
        tcbs = [utils.get_tcb(pid)] if pid is not None else utils.get_tcbs()
        for tcb in tcbs:
            if not tcb:
                print(f"Pid={pid}, no task or thread found")
                continue
            self.dump_tls(tcb)

    def diagnose(self, *args, **kwargs):
        corrupted = any(self.check_corruption(tcb) for tcb in utils.get_tcbs())

        return {
            "title": "tlsdump report",
            "summary": "integrity check",
            "result": "failed" if corrupted else "pass",
            "command": "tlsdump",
            "data": gdb.execute("tlsdump", to_string=True),
        }
