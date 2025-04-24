#! /usr/bin/python
############################################################################
# tools/pynuttx/tests/runner.py
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

import os
import unittest

if __name__ == "__main__":
    test_dir = os.path.dirname(os.path.abspath(__file__))
    testsuit_runtime = unittest.defaultTestLoader.discover(
        test_dir, pattern="test_runtime*.py"
    )

    test_runner = unittest.TextTestRunner()

    # NOTE: Run the runtime testsuit first, as we are going to mock
    # lots of methods as well as classes later!
    result_runtime = test_runner.run(testsuit_runtime)

    """ testsuit_mock = unittest.defaultTestLoader.discover(
        test_dir, pattern="test_mock*.py"
    )
    result_mock = test_runner.run(testsuit_mock) """

    if not result_runtime.wasSuccessful():
        print("GDB Plugin Test Failed")
    else:
        print("GDB Plugin Test Successfully")
