############################################################################
# tools/pynuttx/tests/test_runtime_fs.py
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

# The following test cases require running the program as
# we need to access the memory of the program


class TestFs(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    def check_output(self, out, expect=""):
        if expect not in out:
            self.fail(f"Got: {out}")

    def test_fdinfo(self):
        out = gdb.execute("fdinfo", to_string=True)
        self.check_output(out, expect="PID: 1")

    def test_fdinfo_pid(self):
        out = gdb.execute("fdinfo -p 1", to_string=True)
        self.check_output(out, expect="PID: 1")

    def test_mount(self):
        out = gdb.execute("mount", to_string=True)
        self.check_output(out, expect="/proc type procfs")

    def test_foreach_inode(self):
        out = gdb.execute("foreach inode", to_string=True)
        self.check_output(out, expect=" console ")

    def test_foreach_inode_addr(self):
        pass

    def test_info_shm(self):
        pass
