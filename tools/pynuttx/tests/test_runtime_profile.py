############################################################################
# tools/pynuttx/tests/test_runtime_profile.py
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


class TestProfile(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    def check_output(self, out, expect=""):
        if expect not in out:
            self.fail(f"Got: {out}")

    def test_profile(self):

        out = gdb.execute(" profile p 0x123", to_string=True)
        self.check_output(out, expect="function calls in")
        self.check_output(out, expect="printing.py")
        # Example of output:

    # (gdb) profile p 0x123
    # $3 = 291
    #          12 function calls in 0.001 seconds

    #    Ordered by: cumulative time

    #    ncalls  tottime  percall  cumtime  percall filename:lineno(function)
    #         1    0.000    0.000    0.001    0.001 {built-in method builtins.exec}
    #         1    0.000    0.000    0.000    0.000 <string>:1(<module>)
    #         1    0.000    0.000    0.000    0.000 {built-in method _gdb.execute}
    #         2    0.000    0.000    0.000    0.000 printing.py:205(__call__)
    #         2    0.000    0.000    0.000    0.000 types.py:22(get_basic_type)
    #         2    0.000    0.000    0.000    0.000 {method 'search' of 're.Pattern' objects}
    #         2    0.000    0.000    0.000    0.000 {method 'unqualified' of 'gdb.Type' objects}
    #         1    0.000    0.000    0.000    0.000 {method 'disable' of '_lsprof.Profiler' objects}

    def test_time(self):
        # (gdb) time p 0x123
        # $4 = 291
        # Time elapsed: 0.000295s

        out = gdb.execute("time", to_string=True)
        self.check_output(out, expect="Time elapsed")
