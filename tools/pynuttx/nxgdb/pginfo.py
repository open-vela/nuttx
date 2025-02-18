############################################################################
# tools/pynuttx/nxgdb/pginfo.py
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

import gdb

from .utils import read_u64


class PageTable:
    """Base class for page table handling."""

    def __init__(self):
        pass

    def walk_table(self, base, level, prefix=""):
        """Placeholder for architecture-specific table walking logic."""
        raise NotImplementedError("Subclasses must implement this method.")

    def dump(self):
        """Placeholder for architecture-specific dumping logic."""
        raise NotImplementedError("Subclasses must implement this method.")


class PageTableX86(PageTable):
    """x86_64-specific page table walker."""

    PAGE_SIZE = 4096
    NUM_ENTRIES = 512
    ENTRY_SIZE = 8
    PAGE_PRESENT = 1 << 0
    PAGE_WRITABLE = 1 << 1
    PAGE_PSE = 1 << 7

    PAGE_LEVELS = {0: "PML4", 1: "PDPT", 2: "PD", 3: "PT"}
    PAGE_SIZES = {1: "1G", 2: "2M", 3: "4K"}

    def __init__(self):
        super().__init__()

    def walk_table(self, base, level, prefix=""):
        """x86_64-specific implementation to traverse the page table and yield entries."""
        if level > 3:
            return

        table_name = self.PAGE_LEVELS.get(level, f"L{level}")
        for i in range(self.NUM_ENTRIES):
            entry_addr = base + i * self.ENTRY_SIZE + 0x100000000
            data = gdb.selected_inferior().read_memory(entry_addr, self.ENTRY_SIZE)
            entry = read_u64(data, 0)

            if not (entry & self.PAGE_PRESENT):
                continue

            addr = entry & ~0xFFF
            flags = "RW" if entry & self.PAGE_WRITABLE else "R"
            if level == 3 or (level in (2, 1) and entry & self.PAGE_PSE):
                size = self.PAGE_SIZES.get(level, "")
            else:
                size = ""

            if size:
                line = f"{prefix}[{table_name} {i}] -> {addr:#x} ({flags} {size})"
            else:
                line = f"{prefix}[{table_name} {i}] -> {addr:#x} ({flags})"
            yield line

            if level < 3 and not (level in (2, 1) and entry & self.PAGE_PSE):
                yield from self.walk_table(addr, level + 1, prefix + "    ")

            if level == 0:
                break

    def dump(self):
        pml4_addr = int(gdb.parse_and_eval("&pml4")) & ~0xFFF
        print(f"pml4_addr: {pml4_addr:#x}")

        for line in self.walk_table(pml4_addr, 0):
            print(line)


class DumpPageTableCommand(gdb.Command):
    """GDB command to dump page tables based on architecture."""

    def __init__(self):
        super().__init__("dump_pagetable", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):

        arch = gdb.selected_inferior().architecture().name()
        arch_classes = {
            "i386:x86-64": PageTableX86,  # Add other architectures as needed
        }

        clz = arch_classes.get(arch)
        if not clz:
            print(f"Unsupported architecture: {arch}")
            return

        pagetable = clz()
        pagetable.dump()
