############################################################################
# tools/pynuttx/tests/test_runtime_thread.py
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

import unittest

import gdb
from nxgdb import utils

# The following test cases require running the program as
# we need to access the memory of the program


class TestThread(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    def test_info_threads(self):
        gdb.execute("info threads")

    def test_thread(self):
        gdb.execute("thread")

    def test_thread_bad_args(self):
        out = gdb.execute("thread apply", to_string=True)
        self.assertEqual(out, "Please specify a thread ID list\n")

        out = gdb.execute("thread apply bad", to_string=True)
        self.assertEqual(out, "Please specify a command following the thread ID list\n")

        out = gdb.execute("thread apply badid0 cmd", to_string=True)
        self.assertEqual(out, "Please specify a thread ID list and command\n")

        out = gdb.execute("thread badid", to_string=True)
        self.assertEqual(out, "Invalid thread id badid\n")

        out = gdb.execute("thread 256", to_string=True)
        self.assertEqual(out, "Invalid thread id 256\n")

    def test_thread_apply_all(self):
        out = gdb.execute("thread apply all bt", to_string=True)
        self.assertTrue("#0" in out and "Thread" in out, msg=f"Got: {out}")

    def test_thread_apply_with_ids(self):
        out = gdb.execute("thread apply 0 bt", to_string=True)
        self.assertTrue("#0" in out and "Thread 0" in out, msg=f"Got: {out}")

        out = gdb.execute("thread apply 0 1 info reg", to_string=True)
        # info reg has format like
        # reg      xxx      xxx
        self.assertGreaterEqual(len(out.split("\n")[0].split()), 2, msg=f"Got: {out}")

    def test_thread_with_id(self):
        # This command suppose to switch the stack frames
        # make sure we have at least two threads
        if gdb.parse_and_eval("g_npidhash") < 2:
            out = gdb.execute("thread 0", to_string=True)
            self.assertEqual(out, "")
            return

        # Get the current running thread first, after testing we should switch back to it
        # and then continue, otherwise, we might fail on an assertion. For example, we will
        # fail to continue running a task wich is waiting for a mutex if the current running
        # task was actually the idle task.

        cur_thread_id = gdb.parse_and_eval("g_running_tasks")["pid"]

        gdb.execute("thread 0")
        gdb.execute("frame 0")
        cur_sp = utils.get_sp()

        sps = []
        for i in range(gdb.parse_and_eval("g_npidhash")):
            gdb.execute(f"thread {i}")
            # switch frame here
            gdb.execute("frame 0")
            new_sp = utils.get_sp()
            sps.append(new_sp)

        # We should have some different stack pointers
        self.assertFalse(all([int(sp) == int(cur_sp) for sp in sps]))

        gdb.execute(f"thread {cur_thread_id}")
