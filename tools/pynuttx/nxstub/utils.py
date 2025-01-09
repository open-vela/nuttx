############################################################################
# tools/pynuttx/nxstub/utils.py
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

try:
    import lief
    from construct import (
        Array,
        Construct,
        Int8ul,
        Int16ul,
        Int32sb,
        Int32sl,
        Int32ub,
        Int32ul,
        Int64sb,
        Int64sl,
        Int64ub,
        Int64ul,
        Struct,
    )
except ImportError:
    print('Package missing, please do "pip install lief construct"')

import logging
from typing import List, Tuple


class RegInfo:
    def __init__(self, name, size, tcb_offset=0):
        self.name = name
        self.size = size
        self.tcb_offset = tcb_offset

    def __str__(self):
        return f"{self.name}({self.size})"

    def __repr__(self):
        return f"REG({self.name}, {self.size}, {self.tcb_offset})"


def parse_elf(elf_file: str) -> lief.Binary:
    elf = lief.parse(elf_file)

    if not elf:
        logging.error(f"Failed to parse ELF file: {elf_file}")
        return None

    return elf


def get_architecture(elf: lief.Binary):
    return elf.abstract.header.architecture


def get_endian(elf: lief.Binary) -> str:
    return (
        "l" if elf.abstract.header.endianness == lief.Header.ENDIANNESS.LITTLE else "b"
    )


def read_from(elf: lief.Binary, addr, len=1) -> memoryview:
    for section in elf.sections:
        if section.type == lief.ELF.Section.TYPE.PROGBITS:
            off = addr - section.virtual_address
            if (
                section.virtual_address <= addr < section.virtual_address + section.size
                and section.size - off >= len
            ):
                return section.content[off : off + len]

    for segment in elf.segments:
        if segment.type == lief.ELF.Segment.TYPE.LOAD:
            off = addr - segment.virtual_address
            if (
                segment.virtual_address
                <= addr
                < segment.virtual_address + segment.virtual_size
                and segment.virtual_size - off >= len
            ):
                return segment.content[off : off + len]

    return None


def get_symbol(elf: lief.Binary, symbol) -> lief.Symbol:
    return elf.get_symbol(symbol)


def read_symbol(
    elf: lief.Binary, symbol, struct: Construct = None
) -> Tuple[lief.Symbol, memoryview]:
    sym = elf.get_symbol(symbol)
    if sym is None:
        return None

    data = read_from(elf, sym.value, sym.size)
    if struct:
        data = struct.parse(data)
    return sym, data


def read_string(elf: lief.Binary, addr):
    """Read const string from ELF file"""
    output = b""
    while True:
        c = read_from(elf, addr, 1)
        if c == b"\0":
            break

        output += c.tobytes()
        addr += 1

    return output.decode("utf-8")


def get_inttype(elf: lief.Binary) -> Construct:
    endian = get_endian(elf)
    bits = 64 if elf.abstract.header.is_64 else 32
    return {
        "32l": Int32sl,
        "32b": Int32sb,
        "64l": Int64sl,
        "64b": Int64sb,
    }.get(f"{bits}{endian}", Int32sl)


def get_pointer_type(elf: lief.Binary) -> Construct:
    endian = get_endian(elf)
    bits = 64 if elf.abstract.header.is_64 else 32
    return {
        "32l": Int32ul,
        "32b": Int32ub,
        "64l": Int64ul,
        "64b": Int64ub,
    }.get(f"{bits}{endian}", Int32ul)


def get_pointer_size(elf: lief.Binary):
    return 8 if elf.abstract.header.is_64 else 4


def get_ncpus(elf: lief.Binary) -> int:
    # FAR struct tcb_s *g_running_tasks[CONFIG_SMP_NCPUS];
    #
    # g_running_tasks is an pointer array in length of ncpu
    g_running_tasks = elf.get_symbol("g_running_tasks")

    return g_running_tasks.size // get_pointer_size(elf)


def get_regsize(elf: lief.Binary) -> int:
    """Register size in context"""
    sym = elf.get_symbol("g_last_regs")
    return sym.size // get_ncpus(elf)


def get_tcbinfo(elf: lief.Binary):
    tcbinfo_s = Struct(
        "pid_off" / Int16ul,  # FIXME: only little endian supported
        "state_off" / Int16ul,
        "pri_off" / Int16ul,
        "name_off" / Int16ul,
        "stack_off" / Int16ul,
        "stack_size_off" / Int16ul,
        "regs_off" / Int16ul,
        "regs_num" / Int16ul,
    )

    _, data = read_symbol(elf, "g_tcbinfo")
    return tcbinfo_s.parse(data)


def get_tcb_size(elf: lief.Binary) -> int:
    # static struct tcb_s g_idletcb[CONFIG_SMP_NCPUS];
    # Idle TCB happen to be an array of tcb_s

    g_idletcb = elf.get_symbol("g_idletcb")
    ncpus = get_ncpus(elf)
    return g_idletcb.size // ncpus


def get_reginfo(elf: lief.Binary) -> List[RegInfo]:
    bits = 64 if elf.abstract.header.is_64 else 32

    # Now get register offset in TCB
    _, data = read_symbol(elf, "g_reg_offs")
    reg_offs = Array(len(data) // 2, Int16ul).parse(data)
    return [RegInfo("", bits // 8, off) for off in reg_offs]


def parse_array(data, type_, narray):
    return Array(narray, type_).parse(data)


def get_statenames(elf: lief.Binary) -> List[str]:
    pointer = get_pointer_type(elf)
    sym, addr = read_symbol(elf, "g_statenames")
    names = parse_array(addr, pointer, sym.size // pointer.sizeof())
    names = [read_string(elf, name) for name in names]
    return names


def uint16_t(data: bytes) -> int:
    return Int16ul.parse(data)


def uint8_t(data: bytes) -> int:
    return Int8ul.parse(data)


def get_packet(sock) -> bytes:
    buffer = bytearray()
    started = False
    escaping = False
    checksum = 0
    while True:
        c = sock.recv(1)
        if not started:
            if c in (b"\x03", b"+", b"-"):  # Special packets
                return c
            if c == b"$":
                started = True
            continue

        if escaping:
            c = chr(ord(c) ^ 0x20)
            escaping = False
        elif c == b"}":
            escaping = True
            checksum += ord(c)
            continue

        if c == b"#":
            expected = sock.recv(2)
            expected = int(expected.decode("ascii"), 16)
            if expected != checksum & 0xFF:
                checksum = 0
                started = False
                buffer = bytearray()
                continue
            else:
                break
        else:
            checksum += ord(c)
            buffer.append(ord(c))
    return buffer


def encode_packet(packet: bytes) -> bytes:
    output = list()
    for c in packet:
        if c in b"$#*}":
            output.append(ord("}"))
            c ^= 0x20
        output.append(c)
    return bytes(output)
