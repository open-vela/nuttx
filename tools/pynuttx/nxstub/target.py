############################################################################
# tools/pynuttx/nxstub/target.py
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
import traceback
from typing import List

from . import utils
from .registers import Registers


class RawMemory:
    def __init__(self, address: int, data: bytearray):
        self.address = address
        self.data = bytearray(data)

    def __len__(self):
        return len(self.data)

    def __str__(self):
        return f"Memory({self.address:#x}~{self.address + len(self.data):#x})"

    def __repr__(self):
        return self.__str__()


class ThreadInfo:
    def __init__(self, name, pid, state, registers: Registers):
        self.name = name
        self.pid = pid
        self.state = state
        self.registers = registers

    def __str__(self):
        return f"{self.name}({self.pid}) {self.state}"

    def __repr__(self):
        return f"Thread({self.name}, {self.pid}, {self.state})"


class Target:
    PID0_ID = 0xFFFF

    def __init__(
        self,
        elf,
        arch=None,
        registers: Registers = None,
        memories: List[RawMemory] = None,
        core=None,
    ):
        """
        The target that GDB stub operations on.
        :param elf: The ELF file path.
        :param registers: The optional initial register value, normally used for crash log analysis.
        :param memories: The optional initial memory regions, normally used for raw memory dump.
        :param arch: The architecture of the target, e.g. "arm", "riscv", "mips", etc.
        """
        self.logger = logging.getLogger(__name__)
        self.elf = elf
        self.core = core
        self.registers = registers or Registers(elf, arch=arch)
        self.memories = []
        self.arch = arch
        self.pid = 0  # Current thread PID

        for mem in memories or []:
            # Go through the write process to merge overlapping memory regions
            self.memory_write(mem.address, mem.data)

        self.logger.debug(f"Memory regions: {self.memories}")

    def _read_symbol(self, symbol: str, length: int = 0) -> bytes:
        sym = utils.get_symbol(self.elf, symbol)
        data = self.memory_read(sym.value, length or sym.size)
        return data, sym

    def _read_int(self, symbol: str) -> int:
        inttype = utils.get_inttype(self.elf)
        data, sym = self._read_symbol(symbol, inttype.sizeof())
        if not data:
            return None, sym
        return inttype.parse(data), sym

    def _read_str(self, address: int) -> str:
        output = b""
        while (b := self.memory_read(address, 1)) != b"\0":
            output += b
            address += 1

        return output.decode("utf-8")

    def update_threads(self) -> List[ThreadInfo]:
        """Update the latest threads information"""

        self.threads = (ThreadInfo("main", self.PID0_ID, "Running", self.registers),)

        try:
            pointer = utils.get_pointer_type(self.elf)
            tcbsize = utils.get_tcb_size(self.elf)
            tcbinfo = utils.get_tcbinfo(self.elf)
            try:
                states = utils.get_statenames(self.elf)
            except Exception:
                # Fallback to pure number. Don't bother to parse value of NUM_TASK_STATES, 256 is enough.
                states = [str(i) for i in range(256)]

            g_npidhash, sym = self._read_int("g_npidhash")
            self.logger.debug(f"g_npidhash: {g_npidhash}@{sym.value:#x}")
            if not g_npidhash:
                self.logger.error(f"No threads info found: {g_npidhash}")
                return self.threads

            def parse_tcb(address: int) -> ThreadInfo:
                data = self.memory_read(address, tcbsize)
                if tcbinfo.name_off == 0:
                    name = "<noname>"
                else:
                    name = self._read_str(address + tcbinfo.name_off)
                self.logger.debug(f"loading thread: {name}")
                pid = utils.uint16_t(data[tcbinfo.pid_off : tcbinfo.pid_off + 2])
                pid = pid if pid != 0 else self.PID0_ID
                state = utils.uint8_t(data[tcbinfo.state_off : tcbinfo.state_off + 1])
                state = states[state] if state < len(states) else "Unknown"
                registers = Registers(self.elf, arch=self.arch)
                xcpregs = data[tcbinfo.regs_off : tcbinfo.regs_off + pointer.sizeof()]
                xcpregs = pointer.parse(xcpregs)
                xcpregs = self.memory_read(xcpregs, utils.get_regsize(self.elf))
                registers.load(xcpregs=xcpregs)
                self.logger.debug(f"Parse TCB: {name}({pid},{state})")
                return ThreadInfo(name, pid, state, registers)

            data, sym = self._read_symbol("g_pidhash")
            g_pidhash = pointer.parse(data)
            data = self.memory_read(g_pidhash, pointer.sizeof() * g_npidhash)
            g_pidhash = utils.parse_array(data, pointer, g_npidhash)
            self.logger.debug(f"g_pidhash: {g_pidhash}@{sym.value:#x}")

            self.threads = [parse_tcb(tcb) for tcb in g_pidhash if tcb]
            self.logger.debug(f"Found {self.threads}")
        except Exception as e:
            self.logger.error(f"No threads info: {e}\n{traceback.format_exc()}")

        return self.threads

    def switch_thread(self, pid: int = None) -> Registers:
        """
        Switch to the thread with PID, or the next running thread.
        Return the registers for the thread
        """
        self.logger.debug(f"Switch to thread {pid}")
        pid = pid if pid is not None and pid > 0 else self.PID0_ID

        for t in self.threads:
            if pid == t.pid:
                self.pid = pid
                self.registers = t.registers
                return self.registers

    def memory_read(self, address: int, length: int) -> bytes:
        self.logger.debug(f"Read: {address:#x} {length}Bytes")
        # Try cached memory
        for mem in self.memories:
            if mem.address <= address < mem.address + len(mem):
                offset = address - mem.address
                # Limit the length to available data
                length = min(length, len(mem) - offset)
                return mem.data[offset : offset + length]

        # Try core
        if self.core and (value := utils.read_from(self.core, address, length)):
            return bytes(value)

        # Try elf
        return bytes(utils.read_from(self.elf, address, length) or [])

    def memory_write(self, address, data):
        memories = self.memories
        mem = RawMemory(address, bytearray(data))
        self.logger.debug(f"Write: {mem}")

        if not memories or address > memories[-1].address + len(memories[-1]):
            memories.append(mem)  # New memory region in the end
            return
        elif address + len(data) < memories[0].address:
            memories.insert(0, mem)  # New memory region in the beginning
            return

        for i, m in enumerate(memories):
            if address > m.address + len(m):
                continue

            if address + len(data) < m.address:
                memories.insert(i, mem)  # New memory region in the middle
                return

            if (offset := address - m.address) >= 0:
                # Overwrite and append data to existing memory
                m.data[offset : offset + len(data)] = data
            else:
                # Prepend data to existing memory
                offset = address + len(m) - m.address
                m.data = data + m.data[offset:]
                m.address = address

            # Remove overlapping memory regions
            end = m.address + len(m)
            for m2 in memories[i + 1 :]:
                if end < m2.address:
                    break

                memories.remove(m2)
                m.data += m2.data[end - m2.address :]
