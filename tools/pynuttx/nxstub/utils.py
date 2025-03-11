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
    from construct import Array, Int8ul, Int16ul, Struct
except ImportError:
    print('Package missing, please do "pip install construct"')

from typing import List

from nxelf.elf import LiefELF


def get_ncpus(elf: LiefELF) -> int:
    # FAR struct tcb_s *g_running_tasks[CONFIG_SMP_NCPUS];
    #
    # g_running_tasks is an pointer array in length of ncpu
    running_tasks = elf.get_symbol("g_running_tasks")

    return running_tasks.size // elf.get_pointer_size()


def get_regsize(elf: LiefELF) -> int:
    """Register size in context"""
    sym = elf.get_symbol("g_last_regs")
    return sym.size // get_ncpus(elf)


def get_tcbinfo(elf: LiefELF):
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

    _, data = elf.read_symbol("g_tcbinfo")
    return tcbinfo_s.parse(data)


def get_tcb_size(elf: LiefELF) -> int:
    # static struct tcb_s g_idletcb[CONFIG_SMP_NCPUS];
    # Idle TCB happen to be an array of tcb_s

    idletcb = elf.get_symbol("g_idletcb")
    ncpus = get_ncpus(elf)
    return idletcb.size // ncpus


def parse_array(data, type_, narray):
    return Array(narray, type_).parse(data)


def get_statenames(elf: LiefELF) -> List[str]:
    pointer = elf.get_pointer_type()
    sym, addr = elf.read_symbol("g_statenames")
    names = parse_array(addr, pointer, sym.size // pointer.sizeof())
    names = [elf.read_string(name) for name in names]
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
