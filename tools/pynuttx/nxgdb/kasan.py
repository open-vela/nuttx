############################################################################
# tools/pynuttx/nxgdb/kasan.py
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

import argparse
import itertools

import gdb

from . import utils


class KASanGeneric:
    def __init__(self, scope, begin, end, shadow, bitwidth, scale, shift=0):
        self.scope = scope
        self.begin = int(begin)
        self.end = int(end)
        self.shadow = shadow
        self.bitwidth = int(bitwidth)
        self.scale = int(scale)
        self.shift = int(shift)

    def check_addr(self, addr):
        return self.get_allocation_tag(addr)

    def contains(self, addr):
        return self.begin <= self.untag_addr(addr) <= self.end

    def get_allocation_tag(self, addr):
        distance = (addr - self.begin) // self.scale
        index = distance // self.bitwidth
        bit = distance % self.bitwidth
        return True if self.shadow[index] >> bit & 0x01 else False

    def get_logical_tag(self, addr):
        return int(addr >> self.shift)

    def untag_addr(self, addr):
        return addr


class KASanSwtags(KASanGeneric):
    def check_addr(self, addr):
        return self.get_allocation_tag(addr) != self.get_logical_tag(addr)

    def get_allocation_tag(self, addr):
        untag_addr = self.untag_addr(addr)
        distance = untag_addr - self.begin
        index = distance // self.scale
        return self.shadow[index]

    def untag_addr(self, addr):
        return addr & ~(0xFF << self.shift)


command_actions = {
    "check": lambda addr, region: print(
        f"[{region.scope}] Addr 0x{addr:X} is {'invalid' if region.check_addr(addr) else 'valid'}"
    ),
    "print-allocation-tag": lambda addr, region: print(
        f"[{region.scope}] Addr 0x{addr:X} allocation tag is: 0x{region.get_allocation_tag(addr):X}"
    ),
    "print-logical-tag": lambda addr, region: print(
        f"[{region.scope}] Addr 0x{addr:X} logical tag is: 0x{region.get_logical_tag(addr):X}"
    ),
}


class KASan(gdb.Command):

    def __init__(self):
        if utils.get_symbol_value("CONFIG_MM_KASAN") is None:
            return

        super().__init__("kasan", gdb.COMMAND_USER)

        """ Common bit width and kasan alignment length in multiple modes """
        bitwidth = utils.sizeof("long") * 8
        scale = utils.get_symbol_value("KASAN_SHADOW_SCALE")

        """ Get the array of KASan regions """
        region_count = utils.parse_and_eval("g_region_count")
        region = utils.parse_and_eval("g_region")

        self.regions: list[KASanGeneric] = []
        if utils.get_symbol_value("CONFIG_MM_KASAN_GENERIC") is not None:
            print("KASan Mode: Generic")
            for region in utils.ArrayIterator(region, region_count):
                self.regions.append(
                    KASanGeneric(
                        "Heap",
                        region["begin"],
                        region["end"],
                        region["shadow"],
                        bitwidth,
                        scale,
                    )
                )

        if utils.get_symbol_value("CONFIG_MM_KASAN_SW_TAGS") is not None:
            print("KASan Mode: Softtags")
            shift = utils.get_symbol_value("KASAN_TAG_SHIFT")
            for region in utils.ArrayIterator(region, region_count):
                self.regions.append(
                    KASanSwtags(
                        "Heap",
                        region["begin"],
                        region["end"],
                        region["shadow"],
                        bitwidth,
                        scale,
                        shift,
                    )
                )

        if utils.get_symbol_value("CONFIG_MM_KASAN_GLOBAL") is not None:
            print("KASan Support Checking Global Variables")
            scale = utils.get_symbol_value("CONFIG_MM_KASAN_GLOBAL_ALIGN")
            global_region = utils.parse_and_eval("g_global_region")
            for index in itertools.count(0):
                if global_region[index] == 0:
                    break
                region = global_region[index].dereference()
                self.regions.append(
                    KASanGeneric(
                        "Globals",
                        region["begin"],
                        region["end"],
                        region["shadow"],
                        bitwidth,
                        scale,
                    )
                )

    @utils.dont_repeat_decorator
    def invoke(self, args, from_tty):
        parser = argparse.ArgumentParser(
            description="Memory Tagging Commands", add_help=False
        )
        subparsers = parser.add_subparsers(dest="command")
        subparsers.add_parser(
            "check",
            help="Validate a pointer's logical tag against the allocation tag.",
        )
        subparsers.add_parser(
            "print-allocation-tag", help="Print the allocation tag for ADDRESS."
        )
        subparsers.add_parser(
            "print-logical-tag", help="Print the logical tag from POINTER."
        )

        try:
            parsed_args, addresses = parser.parse_known_args(args.split())
        except SystemExit:
            print(parser.format_help())
            return

        if not parsed_args.command:
            print(parser.format_help())
            return

        regions = {}
        for addr in addresses:
            addr = utils.parse_arg(addr)
            for region in self.regions:
                if region.contains(addr):
                    regions[addr] = region
                    break
            else:
                print(f"Addr 0x{addr:X} Not in any region")

        if parsed_args.command in command_actions:
            for addr, region in regions.items():
                command_actions[parsed_args.command](addr, region)
        else:
            print(f"Unknown command: {parsed_args.command}")
