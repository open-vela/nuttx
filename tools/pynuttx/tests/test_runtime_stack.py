############################################################################
# tools/pynuttx/tests/test_runtime_stack.py
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

import unittest

import gdb
from nxgdb.stack import fetch_stacks

# The following test cases require running the program as
# we need to access the memory of the program


class TestStack(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    def test_fetch_stacks(self):
        stacks = fetch_stacks()
        self.assertNotEqual(stacks, dict())

    def test_list_stacks(self):
        gdb.execute("stack-usage")
