############################################################################
# tools/pynuttx/tests/test_runtime_net.py
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


class TestNet(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    def check_output(self, out, expect=""):
        if expect not in out:
            self.fail(f"Got: {out}")

    def test_netstat(self):
        out = gdb.execute("netstat", to_string=True)
        self.check_output(
            out, expect="IOB:       size    ntotal     nfree     nwait nthrottle"
        )
        self.check_output(
            out,
            expect="TCP Conn:  st flg ref tmr uack nrt       txbuf       "
            "rxbuf+ofo           local_address        remote_address",
        )

    def test_netcheck(self):
        out = gdb.execute("netcheck", to_string=True)
        self.check_output(out, expect="IOB check: PASS")
