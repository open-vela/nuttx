############################################################################
# tools/pynuttx/nxreg/register.py
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
from binascii import hexlify
from typing import List, Union

from construct import Array, Int8ul, Int16ul, PaddedString, Struct
from nxelf.elf import LiefELF

UINT16_MAX = 0xFFFF
REGINFO_OFFSET_AUTO = 0

g_reg_table = {
    "arm": {
        "architecture": [
            "arm",
            "armv6-m",
            "armv6s-m",
            "armv7e-m",
            "armv8-m",
            "armv8-m.base",
            "armv8-m.main",
            "armv8.1-m.main",
        ],
        "feature": "org.gnu.gdb.arm.m-profile",
        "registers": [
            ("r0", 0, REGINFO_OFFSET_AUTO),
            ("r1", 1, REGINFO_OFFSET_AUTO),
            ("r2", 2, REGINFO_OFFSET_AUTO),
            ("r3", 3, REGINFO_OFFSET_AUTO),
            ("r4", 4, REGINFO_OFFSET_AUTO),
            ("r5", 5, REGINFO_OFFSET_AUTO),
            ("r6", 6, REGINFO_OFFSET_AUTO),
            ("r7", 7, REGINFO_OFFSET_AUTO),
            ("r8", 8, REGINFO_OFFSET_AUTO),
            ("r9", 9, REGINFO_OFFSET_AUTO),
            ("r10", 10, REGINFO_OFFSET_AUTO),
            ("r11", 11, REGINFO_OFFSET_AUTO),
            ("r12", 12, REGINFO_OFFSET_AUTO),
            ("sp", 13, REGINFO_OFFSET_AUTO),
            ("lr", 14, REGINFO_OFFSET_AUTO),
            ("pc", 15, REGINFO_OFFSET_AUTO),
            ("xpsr", 25, REGINFO_OFFSET_AUTO),
        ],
    },
    "arm-a": {
        "architecture": ["armv6", "armv7", "armv8-a", "armv8-r", "armv9-a"],
        "feature": "org.gnu.gdb.arm",
        "registers": [
            ("r0", 0, REGINFO_OFFSET_AUTO),
            ("r1", 1, REGINFO_OFFSET_AUTO),
            ("r2", 2, REGINFO_OFFSET_AUTO),
            ("r3", 3, REGINFO_OFFSET_AUTO),
            ("r4", 4, REGINFO_OFFSET_AUTO),
            ("r5", 5, REGINFO_OFFSET_AUTO),
            ("r6", 6, REGINFO_OFFSET_AUTO),
            ("r7", 7, REGINFO_OFFSET_AUTO),
            ("r8", 8, REGINFO_OFFSET_AUTO),
            ("r9", 9, REGINFO_OFFSET_AUTO),
            ("r10", 10, REGINFO_OFFSET_AUTO),
            ("r11", 11, REGINFO_OFFSET_AUTO),
            ("r12", 12, REGINFO_OFFSET_AUTO),
            ("sp", 13, REGINFO_OFFSET_AUTO),
            ("lr", 14, REGINFO_OFFSET_AUTO),
            ("pc", 15, REGINFO_OFFSET_AUTO),
            ("cpsr", 25, 164),
        ],
    },
    "arm64": {
        "architecture": ["aarch64", "aarch64:ilp32", "aarch64:armv8-r"],
        "feature": "org.gnu.gdb.aarch64",
        "registers": [
            ("x0", 0, REGINFO_OFFSET_AUTO),
            ("x1", 1, REGINFO_OFFSET_AUTO),
            ("x2", 2, REGINFO_OFFSET_AUTO),
            ("x3", 3, REGINFO_OFFSET_AUTO),
            ("x4", 4, REGINFO_OFFSET_AUTO),
            ("x5", 5, REGINFO_OFFSET_AUTO),
            ("x6", 6, REGINFO_OFFSET_AUTO),
            ("x7", 7, REGINFO_OFFSET_AUTO),
            ("x8", 8, REGINFO_OFFSET_AUTO),
            ("x9", 9, REGINFO_OFFSET_AUTO),
            ("x10", 10, REGINFO_OFFSET_AUTO),
            ("x11", 11, REGINFO_OFFSET_AUTO),
            ("x12", 12, REGINFO_OFFSET_AUTO),
            ("x13", 13, REGINFO_OFFSET_AUTO),
            ("x14", 14, REGINFO_OFFSET_AUTO),
            ("x15", 15, REGINFO_OFFSET_AUTO),
            ("x16", 16, REGINFO_OFFSET_AUTO),
            ("x17", 17, REGINFO_OFFSET_AUTO),
            ("x18", 18, REGINFO_OFFSET_AUTO),
            ("x19", 19, REGINFO_OFFSET_AUTO),
            ("x20", 20, REGINFO_OFFSET_AUTO),
            ("x21", 21, REGINFO_OFFSET_AUTO),
            ("x22", 22, REGINFO_OFFSET_AUTO),
            ("x23", 23, REGINFO_OFFSET_AUTO),
            ("x24", 24, REGINFO_OFFSET_AUTO),
            ("x25", 25, REGINFO_OFFSET_AUTO),
            ("x26", 26, REGINFO_OFFSET_AUTO),
            ("x27", 27, REGINFO_OFFSET_AUTO),
            ("x28", 28, REGINFO_OFFSET_AUTO),
            ("x29", 29, REGINFO_OFFSET_AUTO),
            ("x30", 30, REGINFO_OFFSET_AUTO),
            ("sp", 31, REGINFO_OFFSET_AUTO),  # SP
            ("pc", 32, REGINFO_OFFSET_AUTO),  # PC
        ],
    },
    "riscv": {
        "architecture": ["riscv", "riscv:rv32", "riscv:rv64"],
        "feature": "org.gnu.gdb.riscv:rv32",
        "registers": [
            ("zero", 0, REGINFO_OFFSET_AUTO),
            ("ra", 1, REGINFO_OFFSET_AUTO),
            ("sp", 2, REGINFO_OFFSET_AUTO),
            ("gp", 3, REGINFO_OFFSET_AUTO),
            ("tp", 4, REGINFO_OFFSET_AUTO),
            ("t0", 5, REGINFO_OFFSET_AUTO),
            ("t1", 6, REGINFO_OFFSET_AUTO),
            ("t2", 7, REGINFO_OFFSET_AUTO),
            ("fp", 8, REGINFO_OFFSET_AUTO),
            ("s1", 9, REGINFO_OFFSET_AUTO),
            ("a0", 10, REGINFO_OFFSET_AUTO),
            ("a1", 11, REGINFO_OFFSET_AUTO),
            ("a2", 12, REGINFO_OFFSET_AUTO),
            ("a3", 13, REGINFO_OFFSET_AUTO),
            ("a4", 14, REGINFO_OFFSET_AUTO),
            ("a5", 15, REGINFO_OFFSET_AUTO),
            ("a6", 16, REGINFO_OFFSET_AUTO),
            ("a7", 17, REGINFO_OFFSET_AUTO),
            ("s2", 18, REGINFO_OFFSET_AUTO),
            ("s3", 19, REGINFO_OFFSET_AUTO),
            ("s4", 20, REGINFO_OFFSET_AUTO),
            ("s5", 21, REGINFO_OFFSET_AUTO),
            ("s6", 22, REGINFO_OFFSET_AUTO),
            ("s7", 23, REGINFO_OFFSET_AUTO),
            ("s8", 24, REGINFO_OFFSET_AUTO),
            ("s9", 25, REGINFO_OFFSET_AUTO),
            ("s10", 26, REGINFO_OFFSET_AUTO),
            ("s11", 27, REGINFO_OFFSET_AUTO),
            ("t3", 28, REGINFO_OFFSET_AUTO),
            ("t4", 29, REGINFO_OFFSET_AUTO),
            ("t5", 30, REGINFO_OFFSET_AUTO),
            ("t6", 31, REGINFO_OFFSET_AUTO),
            ("pc", 33, REGINFO_OFFSET_AUTO),  # PC
        ],
    },
    "x86-64": {
        "architecture": ["i386", "i386:x86-64", "i386:x86-64:intel", "i386:intel"],
        "feature": "org.gnu.gdb.i386:x86-64",
        "registers": [
            ("rax", 0, REGINFO_OFFSET_AUTO),
            ("rbx", 1, REGINFO_OFFSET_AUTO),
            ("rcx", 2, REGINFO_OFFSET_AUTO),
            ("rdx", 3, REGINFO_OFFSET_AUTO),
            ("rsi", 4, REGINFO_OFFSET_AUTO),
            ("rdi", 5, REGINFO_OFFSET_AUTO),
            ("rbp", 6, REGINFO_OFFSET_AUTO),
            ("fsp", 7, REGINFO_OFFSET_AUTO),
            ("r8", 8, REGINFO_OFFSET_AUTO),
            ("r9", 9, REGINFO_OFFSET_AUTO),
            ("r10", 10, REGINFO_OFFSET_AUTO),
            ("r11", 11, REGINFO_OFFSET_AUTO),
            ("r12", 12, REGINFO_OFFSET_AUTO),
            ("r13", 13, REGINFO_OFFSET_AUTO),
            ("r14", 14, REGINFO_OFFSET_AUTO),
            ("r15", 15, REGINFO_OFFSET_AUTO),
            ("rip", 16, REGINFO_OFFSET_AUTO),
            ("rflags", 17, REGINFO_OFFSET_AUTO),
            ("cs", 18, REGINFO_OFFSET_AUTO),
            ("ss", 19, REGINFO_OFFSET_AUTO),
            ("ds", 20, REGINFO_OFFSET_AUTO),
            ("es", 21, REGINFO_OFFSET_AUTO),
            ("fs", 22, REGINFO_OFFSET_AUTO),
        ],
    },
    "esp32s3": {
        "architecture": ["esp32s3"],  # Use xtensa-esp32s3-elf-gdb
        "feature": "",
        "registers": [
            ("pc", 0, 0),
            ("ps", 73, 292, 0x40000),  # g_reg_offs placed it in the second position
            ("a0", 1, REGINFO_OFFSET_AUTO),
            ("a1", 2, REGINFO_OFFSET_AUTO),
            ("a2", 3, REGINFO_OFFSET_AUTO),
            ("a3", 4, REGINFO_OFFSET_AUTO),
            ("a4", 5, REGINFO_OFFSET_AUTO),
            ("a5", 6, REGINFO_OFFSET_AUTO),
            ("a6", 7, REGINFO_OFFSET_AUTO),
            ("a7", 8, REGINFO_OFFSET_AUTO),
            ("a8", 9, REGINFO_OFFSET_AUTO),
            ("a9", 10, REGINFO_OFFSET_AUTO),
            ("a10", 11, REGINFO_OFFSET_AUTO),
            ("a11", 12, REGINFO_OFFSET_AUTO),
            ("a12", 13, REGINFO_OFFSET_AUTO),
            ("a13", 14, REGINFO_OFFSET_AUTO),
            ("a14", 15, REGINFO_OFFSET_AUTO),
            ("a15", 16, REGINFO_OFFSET_AUTO),
            ("windowbase", 69, 276, REGINFO_OFFSET_AUTO),
            ("windowstart", 70, 280, 1),
        ],
    },
    "xtensa": {
        "architecture": ["xtensa"],  # Use xt-gdb
        "feature": "",
        "registers": [
            ("pc", 32, REGINFO_OFFSET_AUTO),
            ("ps", 742, 472, 0x40000),
            ("a0", 256, 4),
            ("a1", 257, REGINFO_OFFSET_AUTO),
            ("a2", 258, REGINFO_OFFSET_AUTO),
            ("a3", 259, REGINFO_OFFSET_AUTO),
            ("a4", 260, REGINFO_OFFSET_AUTO),
            ("a5", 261, REGINFO_OFFSET_AUTO),
            ("a6", 262, REGINFO_OFFSET_AUTO),
            ("a7", 263, REGINFO_OFFSET_AUTO),
            ("a8", 264, REGINFO_OFFSET_AUTO),
            ("a9", 265, REGINFO_OFFSET_AUTO),
            ("a10", 266, REGINFO_OFFSET_AUTO),
            ("a11", 267, REGINFO_OFFSET_AUTO),
            ("a12", 268, REGINFO_OFFSET_AUTO),
            ("a13", 269, REGINFO_OFFSET_AUTO),
            ("a14", 270, REGINFO_OFFSET_AUTO),
            ("a15", 271, REGINFO_OFFSET_AUTO),
            ("windowbase", 584, 308, REGINFO_OFFSET_AUTO),
            ("windowstart", 585, 312, 1),
        ],
    },
    "tricore": {
        "architecture": "tricore",  # Use tricore-gdb
        "feature": "",
        "registers": [
            ("pcx", 34, 136),
            ("pc", 36, 144),
            ("d0", 0, 0),
            ("d1", 1, 4),
            ("d2", 2, 8),
            ("d3", 3, 12),
            ("d4", 4, 16),
            ("d5", 5, 20),
            ("d6", 6, 24),
            ("d7", 7, 28),
            ("a2", 18, 72),
            ("a3", 19, 76),
            ("a4", 20, 80),
            ("a5", 21, 84),
            ("a6", 22, 88),
            ("a7", 23, 92),
            ("psw", 35, 140),
            ("sp", 26, 104),
            ("a11", 27, 108),
            ("d8", 8, 32),
            ("d9", 9, 36),
            ("d10", 10, 40),
            ("d11", 11, 44),
            ("a12", 28, 112),
            ("a13", 29, 116),
            ("a14", 30, 120),
            ("a15", 31, 124),
            ("d12", 12, 48),
            ("d13", 13, 52),
            ("d14", 14, 56),
            ("d15", 15, 60),
        ],
    },
}


class RegInfo:
    def __init__(self, name, size, tcb_offset=0):
        self.name = name
        self.size = size
        self.tcb_offset = tcb_offset

    def __str__(self):
        return f"{self.name}({self.size})"

    def __repr__(self):
        return f"REG({self.name}, {self.size}, {self.tcb_offset})"


def get_reginfo(elf: LiefELF) -> List[RegInfo]:
    # Now get register offset in TCB
    _, data = elf.read_symbol("g_reginfo")

    reginfo_s = Struct(
        "name" / PaddedString(8, "utf-8"),
        "size" / Int8ul,
        "regnum" / Int8ul,
        "toffset" / Int16ul,
        "goffset" / Int16ul,
    )

    regsnum = len(data) // reginfo_s.sizeof()
    reginfo_t = Array(regsnum, reginfo_s)
    reginfo = reginfo_t.parse(data)

    return [RegInfo(reg.name, elf.bits // 8, reg.toffset) for reg in reginfo]


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


class GeneralRegisters:
    def __init__(self, elf: LiefELF, arch=None, readmem=None):
        """
        Registers class to store register information

        :param elf: Parsed ELF file
        :param arch: architecture name, or use current gdb architecture by default
        :param readmem: function to read memory
        """
        # if we don't have register names in elf, fallback to hardcoded register layouts
        if not arch:
            raise ValueError("Architecture is required to get register names")

        self.logger = logging.getLogger(__name__)
        self.arch = arch
        self.readmem = readmem
        self._registers: List[Register] = []
        regsize = elf.get_pointer_size()

        reginfo = get_reginfo(elf)
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
        ncpu = elf.get_symbol("g_running_tasks").size // elf.get_pointer_size()
        self.xcp_reg_size = elf.get_symbol("g_last_regs").size // ncpu

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

    def load(self, addr: int):
        """
        Load register values from various sources.

        :param tcb: load register values from tcb.xcp.regs
        :param address: load register values from specified address which points to regs in context
        """

        if self.readmem is None:
            raise ValueError("readmem is not set")

        xcpregs = self.readmem(addr, self.xcp_reg_size)
        if xcpregs:
            for reg in self._registers:
                reg.value = xcpregs[reg.tcb_reg_off : reg.tcb_reg_off + reg.size]
        else:
            raise ValueError("No valid source to load register values.\n")
        return self  # allow to build and use Register().load() directly

    def to_g(self):
        """Return GDB RSP g packet"""
        reply = b""
        offset = 0
        for reg in self._registers:
            if reg.offset and reg.offset != offset:
                reply += b"xx" * (reg.offset - offset)

            if not reg.has_value:
                reply += b"xx" * reg.size
            else:
                reply += hexlify(bytes(reg))

            offset += reg.offset + reg.size
        return reply

    def from_g(self, data: bytes):
        """Parse GDB RSP G packet"""
        offset = 0
        for reg in self._registers:
            offset = reg.offset if reg.offset else offset
            self.logger.debug(
                f"Parse {reg.name}({reg.regnum}) from {offset}, data: {data[offset:offset+reg.size]}"
            )
            reg.value = data[offset : offset + reg.size]
            offset += reg.size

    def __iter__(self):
        return iter(self._registers)

    def __len__(self):
        return len(self._registers)

    def __getitem__(self, key):
        return self._registers[key]


class TricoreRegisters(GeneralRegisters):
    def __init__(self, elf, arch=None, readmem=None):
        super().__init__(elf, arch, readmem)

    # tricore read csa
    def load(self, addr: int):
        lower_count = 16
        upper_count = 16
        lpcx = None
        xcpregs = self.readmem(addr, lower_count * 4)  # read lower csa
        xcp_table = [reg[0] for reg in g_reg_table["tricore"]["registers"]]

        if not xcpregs:
            raise ValueError("No valid source to load register values.\n")

        for name in xcp_table[:lower_count]:
            reg = self.get(name=name)
            reg.value = xcpregs[reg.tcb_reg_off : reg.tcb_reg_off + reg.size]
            if name == "pcx":
                lpcx = reg.value

        if not lpcx:
            raise ValueError("Invalid lpcx register loaded.\n")

        PCXI_UL = 1 << 20  # determine wheteher it is the upper csa flag
        is_upper = (lpcx & PCXI_UL) != 0

        def csa2addr(csa):
            # #define tricore_csa2addr(csa) ((uintptr_t *)((((csa) & 0x000F0000) << 12) \
            #                                  | (((csa) & 0x0000FFFF) << 6)))
            return (csa & 0x000F0000) << 12 | (csa & 0x0000FFFF) << 6

        if is_upper:
            xcpregs = self.readmem(csa2addr(lpcx), upper_count * 4)
            for name in xcp_table[lower_count:]:
                reg = self.get(name=name)
                if reg.tcb_reg_off + reg.size > len(xcpregs):
                    raise ValueError("No valid source to load register values.\n")
                reg.value = xcpregs[reg.tcb_reg_off : reg.tcb_reg_off + reg.size]

        return self


def Registers(elf, arch=None, readmem=None) -> GeneralRegisters:
    if arch == "tricore":
        return TricoreRegisters(elf, arch, readmem)
    else:
        return GeneralRegisters(elf, arch, readmem)
