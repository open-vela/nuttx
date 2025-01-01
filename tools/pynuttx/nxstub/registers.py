############################################################################
# tools/pynuttx/nxstub/registers.py
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

import logging
from typing import Union

from . import utils

UINT16_MAX = 0xFFFF

# GDB default register information for different architectures
#
# Format: ("reg-name", regnum, g/G offset, fixed-value(optional))
#
# To get the default register offset in g/G packet, using GDB command:
# (gdb) set architecture <arch> (e.g. set architecture arm), leave empty to see all supported architectures
# (gdb) maint print remote-registers

g_reg_table = {
    "arm": {
        "architecture": "arm",
        "feature": "org.gnu.gdb.arm.m-profile",
        "registers": [
            ("r0", 0, 0),
            ("r1", 1, 0),
            ("r2", 2, 0),
            ("r3", 3, 0),
            ("r4", 4, 0),
            ("r5", 5, 0),
            ("r6", 6, 0),
            ("fp", 7, 0),
            ("r8", 8, 0),
            ("sb", 9, 0),
            ("sl", 10, 0),
            ("r11", 11, 0),
            ("ip", 12, 0),
            ("sp", 13, 0),
            ("lr", 14, 0),
            ("pc", 15, 0),
            ("xpsr", 16, 0),
        ],
    },
    "arm-a": {
        "architecture": "arm",
        "feature": "org.gnu.gdb.arm",
        "registers": [
            ("r0", 0, 0),
            ("r1", 1, 0),
            ("r2", 2, 0),
            ("r3", 3, 0),
            ("r4", 4, 0),
            ("r5", 5, 0),
            ("r6", 6, 0),
            ("fp", 7, 0),
            ("r8", 8, 0),
            ("sb", 9, 0),
            ("sl", 10, 0),
            ("r11", 11, 0),
            ("ip", 12, 0),
            ("sp", 13, 0),
            ("lr", 14, 0),
            ("pc", 15, 0),
            ("cpsr", 25, 164),
        ],
    },
    "arm64": {
        "architecture": "aarch64",
        "feature": "org.gnu.gdb.aarch64",
        "registers": [
            ("x0", 0, 0),
            ("x1", 1, 0),
            ("x2", 2, 0),
            ("x3", 3, 0),
            ("x4", 4, 0),
            ("x5", 5, 0),
            ("x6", 6, 0),
            ("x7", 7, 0),
            ("x8", 8, 0),
            ("x9", 9, 0),
            ("x10", 10, 0),
            ("x11", 11, 0),
            ("x12", 12, 0),
            ("x13", 13, 0),
            ("x14", 14, 0),
            ("x15", 15, 0),
            ("x16", 16, 0),
            ("x17", 17, 0),
            ("x18", 18, 0),
            ("x19", 19, 0),
            ("x20", 20, 0),
            ("x21", 21, 0),
            ("x22", 22, 0),
            ("x23", 23, 0),
            ("x24", 24, 0),
            ("x25", 25, 0),
            ("x26", 26, 0),
            ("x27", 27, 0),
            ("x28", 28, 0),
            ("x29", 29, 0),
            ("x30", 30, 0),
            ("sp_elx", 31, 0),  # SP
            ("elr", 32, 0),  # PC
        ],
    },
    "riscv": {
        "architecture": "riscv:rv32",
        "feature": "org.gnu.gdb.riscv:rv32",
        "registers": [
            ("zero", 0, 0),
            ("ra", 1, 0),
            ("sp", 2, 0),
            ("gp", 3, 0),
            ("tp", 4, 0),
            ("t0", 5, 0),
            ("t1", 6, 0),
            ("t2", 7, 0),
            ("fp", 8, 0),
            ("s1", 9, 0),
            ("a0", 10, 0),
            ("a1", 11, 0),
            ("a2", 12, 0),
            ("a3", 13, 0),
            ("a4", 14, 0),
            ("a5", 15, 0),
            ("a6", 16, 0),
            ("a7", 17, 0),
            ("s2", 18, 0),
            ("s3", 19, 0),
            ("s4", 20, 0),
            ("s5", 21, 0),
            ("s6", 22, 0),
            ("s7", 23, 0),
            ("s8", 24, 0),
            ("s9", 25, 0),
            ("s10", 26, 0),
            ("s11", 27, 0),
            ("t3", 28, 0),
            ("t4", 29, 0),
            ("t5", 30, 0),
            ("t6", 31, 0),
            ("epc", 33, 0),
        ],
    },
    "x86-64": {
        "architecture": "i386:x86-64",
        "feature": "org.gnu.gdb.i386:x86-64",
        "registers": [
            ("rax", 0, 0),
            ("rbx", 1, 0),
            ("rcx", 2, 0),
            ("rdx", 3, 0),
            ("rsi", 4, 0),
            ("rdi", 5, 0),
            ("rbp", 6, 0),
            ("fsp", 7, 0),
            ("r8", 8, 0),
            ("r9", 9, 0),
            ("r10", 10, 0),
            ("r11", 11, 0),
            ("r12", 12, 0),
            ("r13", 13, 0),
            ("r14", 14, 0),
            ("r15", 15, 0),
            ("rip", 16, 0),
            ("rflags", 17, 0),
            ("cs", 18, 0),
            ("ss", 19, 0),
            ("ds", 20, 0),
            ("es", 21, 0),
            ("fs", 22, 0),
        ],
    },
    "xtensa": {
        "architecture": "xtensa",  # Use xtensa-esp32s3-elf-gdb
        "feature": "",
        "registers": [
            ("pc", 0, 0),
            ("ps", 73, 292, 0x40000),  # g_reg_offs placed it in the second position
            ("a0", 1, 0),
            ("a1", 2, 0),
            ("a2", 3, 0),
            ("a3", 4, 0),
            ("a4", 5, 0),
            ("a5", 6, 0),
            ("a6", 7, 0),
            ("a7", 8, 0),
            ("a8", 9, 0),
            ("a9", 10, 0),
            ("a10", 11, 0),
            ("a11", 12, 0),
            ("a12", 13, 0),
            ("a13", 14, 0),
            ("a14", 15, 0),
            ("a15", 16, 0),
            ("windowbase", 69, 276, 0),
            ("windowstart", 70, 280, 1),
        ],
    },
}


class Register:
    def __init__(
        self, name, regnum, size: int, offset=0, tcb_reg_off=0, value=0, fixedvalue=None
    ):
        self.name = name
        self.regnum = regnum
        self.size = size  # size in bytes
        self.offset = offset
        self.tcb_reg_off = tcb_reg_off
        self._value = value
        self.fixedvalue = fixedvalue
        self.logger = logging.getLogger(__name__)

    def __str__(self):
        return f"{self.name}({self.regnum}, size:{self.size}, offset:{self.offset},{self.tcb_reg_off} value:{self.value:#x})"

    def __repr__(self):
        return self.__str__()

    def __bytes__(self):
        return self.value.to_bytes(self.size, "little", signed=self._value < 0)

    @property
    def has_value(self):
        return self._value is not None or self.fixedvalue is not None

    @property
    def value(self):
        return self._value if self.fixedvalue is None else self.fixedvalue

    @value.setter
    def value(self, value):
        """Set register value from bytes or value"""
        if self.name == "":
            return

        if isinstance(value, bytes) or isinstance(value, bytearray):
            if len(value) != self.size:
                raise ValueError(
                    f"Invalid value, expected {self.size} bytes, got {len(value)} bytes"
                )
            self._value = int.from_bytes(bytes(value), "little")
        elif isinstance(value, int):
            self._value = value
        else:
            raise ValueError(f"Invalid value type: {type(value)}")
        self.logger.debug(f"Set {self.name} = {self._value:#x}")


class Registers:
    def __init__(self, elf, arch=None):
        """
        Registers class to store register information

        :param arch: architecture name, or use current gdb architecture by default
        """
        # if we don't have register names in elf, fallback to hardcoded register layouts
        if not arch:
            raise ValueError("Architecture is required to get register names")

        self.logger = logging.getLogger(__name__)
        self.arch = arch
        self._registers = []
        regsize = utils.get_pointer_size(elf)
        reginfo = utils.get_reginfo(elf)
        layouts = g_reg_table.get(self.arch, {}).get("registers", [])
        regoffsets = [r.tcb_offset for r in reginfo if r.tcb_offset != UINT16_MAX]
        for i, (name, regnum, offset, *fixed) in enumerate(layouts):
            register = Register(
                name=name,
                regnum=regnum,
                size=regsize,
                offset=offset,
                tcb_reg_off=regoffsets[i] if i < len(regoffsets) else 0,
                fixedvalue=fixed[0] if fixed else None,
            )

            self._registers.append(register)
            self.logger.debug(
                f"Register {name}({regnum}) offset: {offset}, tcb_off: {register.tcb_reg_off}"
            )

        self._registers.sort(key=lambda x: x.regnum)

    def __str__(self):
        return f"({self.arch} x{len(self._registers)}, {self.sizeof()} bytes)"

    def __repr__(self):
        return f"({self.arch} x{len(self._registers)}, {self.sizeof()} bytes)"

    def sizeof(self):
        """Return total register size in byte"""
        return sum(r.size for r in self._registers)

    def get(self, regnum: int = None, name: str = None) -> Register:
        """Get register by register number"""
        for reg in self._registers:
            if reg.regnum == regnum:
                return reg

            if reg.name == name:
                return reg

        return None

    def set(self, value: Union[int, bytes], regnum: int = None, name: str = None):
        """Set register value by register number"""
        reg = self.get(regnum=regnum, name=name)
        if not reg:
            raise KeyError(f"Register {name or regnum} not found")
        reg.value = value

    def load(self, xcpregs: bytes = None):
        """
        Load register values from various sources.

        :param tcb: load register values from tcb.xcp.regs
        :param address: load register values from specified address which points to regs in context
        """

        if xcpregs:
            for reg in self._registers:
                reg.value = xcpregs[reg.tcb_reg_off : reg.tcb_reg_off + reg.size]
        else:
            raise ValueError("No valid source to load register values")

        return self  # allow to build and use Register().load() directly

    def __iter__(self):
        return iter(self._registers)

    def __len__(self):
        return len(self._registers)

    def __getitem__(self, key):
        return self._registers[key]
