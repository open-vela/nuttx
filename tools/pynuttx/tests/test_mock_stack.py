############################################################################
# tools/pynuttx/tests/test_mock_stack.py
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
from nxgdb.stack import Stack, fetch_stacks


class TestStack(unittest.TestCase):
    """
    This test basically creates some mock objects to verify the
    correctness of the Stack Class
    """

    def test_stack_init_badsize(self):
        with self.assertWarns(RuntimeWarning):
            name = "test_thread"
            entry = hex(0xABCD)
            base = 0x2000
            alloc = 0x1000
            size = 0x0
            cursp = 0x1500
            align = 4
            Stack(name, entry, base, alloc, size, cursp, align)

    def test_stack_init_bad_allocaddr(self):
        with self.assertWarns(RuntimeWarning):
            name = "test_thread"
            entry = hex(0xABCD)
            alloc = 0x1000
            base = alloc - 1
            size = 0x1000
            cursp = 0x1500
            align = 4
            Stack(name, entry, base, alloc, size, cursp, align)

    def test_stack_init_bad_sp(self):
        with self.assertWarns(RuntimeWarning):
            name = "test_thread"
            entry = hex(0xABCD)
            base = 0x2000
            alloc = 0x1000
            size = 0x1000
            cursp = base - 1
            align = 4
            Stack(name, entry, base, alloc, size, cursp, align)

        with self.assertWarns(RuntimeWarning):
            name = "test_thread"
            entry = hex(0xABCD)
            base = 0x2000
            alloc = 0x1000
            size = 0x1000
            cursp = base + size + 1
            align = 4
            Stack(name, entry, base, alloc, size, cursp, align)

    @patch("gdb.write")
    @patch("gdb.lookup_type")
    @patch("gdb.Value")
    def test_stack_cur_usage_no_overflow(
        self, mock_gdb_value, mock_gdb_lookup_type, mock_gdb_write
    ):
        name = "test_thread"
        entry = hex(0xABCD)
        base = 0x1000
        alloc = 0x900
        size = 0x1000
        cursp = 0x1500
        align = 4
        stack = Stack(name, entry, base, alloc, size, cursp, align)

        mock_gdb_value.return_value.cast.return_value = [0xC0FEBABE] * (size // align)
        mock_gdb_lookup_type.return_value.pointer.return_value = MagicMock()

        self.assertEqual(stack.cur_usage(), base + size - cursp)
        mock_gdb_write.assert_not_called()

    @patch("gdb.write")
    @patch("gdb.lookup_type")
    @patch("gdb.Value")
    def test_stack_cur_usage_with_overflow(
        self, mock_gdb_value, mock_gdb_lookup_type, mock_gdb_write
    ):
        name = "test_thread"
        entry = hex(0xABCD)
        base = 0x1000
        alloc = 0x900
        size = 0x1000
        cursp = 0x1500
        align = 4
        stack = Stack(name, entry, base, alloc, size, cursp, align)

        mock_gdb_value.return_value.cast.return_value = [0xC0FEBABE] * (size // align)
        mock_gdb_lookup_type.return_value.pointer.return_value = MagicMock()

        # After constructing the stack, we modify the stack sp to mock an
        # overflow behaviour
        stack._cur_sp = base - align

        with self.assertRaises(gdb.GdbError) as context:
            stack.cur_usage()

        self.assertTrue("pls check your stack size!" in str(context.exception))

    @patch("nxgdb.utils.get_symbol_value")
    def test_stack_check_max_usage_no_color(self, mock_utils_get_symbol_value):
        name = "test_thread"
        entry = hex(0xABCD)
        base = 0x1000
        alloc = 0x900
        size = 0x1000
        cursp = 0x1500
        align = 4
        stack = Stack(name, entry, base, alloc, size, cursp, align)

        mock_utils_get_symbol_value.return_value = None

        self.assertEqual(stack.max_usage(), 0)


class TestFetchStacks(unittest.TestCase):
    @patch("nxgdb.utils.is_target_arch")
    @patch("nxgdb.utils.in_interrupt_context")
    @patch("nxgdb.utils.get_register_byname")
    @patch("nxgdb.utils.get_tcbs")
    @patch("nxgdb.utils.get_task_name")
    def test_fetch_stacks(self, *args):
        (
            mock_get_task_name,
            mock_get_tcbs,
            mock_get_register_byname,
            mock_in_interrupt_context,
            mock_is_target_arch,
        ) = args

        mock_get_tcbs.return_value = [
            {
                "task_state": 3,
                "pid": 123,
                "name": "test",
                "entry": {"pthread": 0x1000},
                "stack_base_ptr": 0x2000,
                "stack_alloc_ptr": 0x1000,
                "adj_stack_size": 0x4000,
            }
        ]
        mock_get_task_name.return_value = "test"
        mock_is_target_arch.return_value = True
        mock_in_interrupt_context.return_value = False
        mock_get_register_byname.return_value = 0x5000
        gdb.parse_and_eval = MagicMock(
            side_effect=(
                lambda x: (
                    mock_get_tcbs.return_value
                    if x == "g_pidhash"
                    else (1 if x == "g_npidhash" else 3)
                )
            )
        )

        stacks = fetch_stacks()

        self.assertNotEqual(stacks, dict())
        self.assertEqual(len(stacks), 1)
        self.assertIn(123, stacks)
        self.assertIsNotNone(stacks[123])
        stack = stacks[123]
        self.assertEqual(str(stack._thread_name), "test")
        self.assertEqual(stack._thread_entry, hex(0x1000))
        self.assertEqual(stack._stack_base, 0x2000)
        self.assertEqual(stack._stack_alloc, 0x1000)
        self.assertEqual(stack._stack_size, 0x4000)
        self.assertEqual(stack._cur_sp, 0x5000)
        self.assertEqual(stack._align, 4)

        # If we get an error while reading the register, we expect an
        # empry stack object
        mock_get_register_byname.side_effect = gdb.GdbError
        with self.assertRaises(gdb.GdbError):
            stacks = fetch_stacks()
            self.assertEqual(stacks, dict())
