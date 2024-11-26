############################################################################
# tools/pynuttx/tests/test_mock_utils.py
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
from unittest.mock import MagicMock, patch

import gdb
from nxgdb import utils

# Loading gdb/__init__.py will append the parent directory to sys.path
# Hence we don't need to do importing manually here


class TestGDBEvalOrNone(unittest.TestCase):
    """
    TODO
    """

    pass


class TestGetSymbolValue(unittest.TestCase):
    """
    TODO
    """

    pass


class TestIsDecimal(unittest.TestCase):
    """
    TODO
    """

    pass


class TestIsHexDecimal(unittest.TestCase):
    """
    TODO
    """

    pass


class TestHexdump(unittest.TestCase):
    """
    TODO
    """

    pass


class TestGetTargetEndianness(unittest.TestCase):
    """
    TODO
    """

    pass


class TestReadBinaryData(unittest.TestCase):
    """
    TODO
    """

    pass


class TestIsTargetArch(unittest.TestCase):
    def setUp(self):
        # Mock the gdb.newest_frame().architecture() method
        gdb.newest_frame = MagicMock()
        self._mock_frame = gdb.Frame = MagicMock()
        self._mock_gdb_exec = gdb.execute = MagicMock()
        self._mock_arch_name = gdb.newest_frame().architecture().name

    def tearDown(self):
        self._mock_frame.reset_mock()
        self._mock_gdb_exec.reset_mock()
        self._mock_arch_name.reset_mock()
        gdb.newest_frame.reset_mock()

    def test_is_target_arch_exact_match(self):
        self._mock_arch_name.return_value = "i386:x86_64"
        # Test if the test() function returns True
        self.assertTrue(utils.is_target_arch("i386:x86_64", exact=True))
        self.assertFalse(utils.is_target_arch("x86", exact=True))

    def test_is_target_arch_non_exact_match(self):
        self._mock_arch_name.return_value = "i386:x86_64"
        # Test if the test() function returns True
        self.assertTrue(utils.is_target_arch("x86"))
        self.assertFalse(utils.is_target_arch("arm"))

    @patch("nxgdb.utils.target_arch", None)
    def test_is_target_arch_no_attr_architecture_auto(self):
        del self._mock_frame.architecture

        self._mock_gdb_exec.return_value = (
            'The target architecture is set to "auto" (currently "armv7e-m").'
        )
        self.assertTrue(utils.is_target_arch("arm"))

    @patch("nxgdb.utils.target_arch", None)
    def test_is_target_arch_no_attr_architecture_specific(self):
        del self._mock_frame.architecture

        self._mock_gdb_exec.return_value = 'The target architecture is set to "riscv"'
        self.assertTrue(utils.is_target_arch("riscv", exact=True))


class TestIsTargetSMP(unittest.TestCase):
    """
    TODO
    """

    pass


class TestInInterruptContext(unittest.TestCase):
    """
    TODO
    """

    pass


class TestGetArchRegister(unittest.TestCase):
    """
    TODO
    """

    pass


class TestGetTcbs(unittest.TestCase):
    """
    TODO
    """

    pass
