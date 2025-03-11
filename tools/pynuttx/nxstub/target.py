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

from nxreg.register import Registers

from . import utils


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
        remap=None,
        core=None,
    ):
        """
        The target that GDB stub operations on.
        :param elf: The ELF file path.
        :param arch: The architecture of the target, e.g. "arm", "riscv", "mips", etc.
        :param registers: The optional initial register value, normally used for crash log analysis.
        :param memories: The optional initial memory regions, normally used for raw memory dump.
        :param remap: The optional memory remap table, used to remap memory regions. A list of tuple (from, to, length).
        :param core: The optional core dump file path.
        """
        self.logger = logging.getLogger(__name__)
        self.elf: utils.LiefELF = elf
        self.core: utils.LiefELF = core
        self.registers = registers or Registers(elf, arch=arch)
        self.memories: List[RawMemory] = []
        self.arch = arch
        self.pid = self.PID0_ID  # Current thread PID
        self.remap = remap or []
        for mem in memories or []:
            # Go through the write process to merge overlapping memory regions
            self.memory_write(mem.address, mem.data)

        self.logger.debug(f"Memory regions: {self.memories}")

    def _read_symbol(self, symbol: str, length: int = 0) -> bytes:
        sym = self.elf.get_symbol(symbol)
        data = self.memory_read(sym.value, length or sym.size)
        return data, sym

    def _read_int(self, symbol: str) -> int:
        inttype = self.elf.get_inttype()
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
            pointer = self.elf.get_pointer_type()
            tcbsize = utils.get_tcb_size(self.elf)
            tcbinfo = utils.get_tcbinfo(self.elf)
            try:
                states = utils.get_statenames(self.elf)
            except Exception:
                # Fallback to pure number. Don't bother to parse value of NUM_TASK_STATES, 256 is enough.
                states = [str(i) for i in range(256)]

            npidhash, sym = self._read_int("g_npidhash")
            self.logger.debug(f"g_npidhash: {npidhash}@{sym.value:#x}")
            if not npidhash:
                self.logger.error(f"No threads info found: {npidhash}")
                return self.threads

            ncpus = utils.get_ncpus(self.elf)
            regsize = utils.get_regsize(self.elf)

            data, sym = self._read_symbol("g_running_tasks")  # an array of pointers
            running_tasks = utils.parse_array(data, pointer, ncpus)
            self.logger.debug(f"g_running_tasks: {running_tasks}@{sym.value:#x}")

            def parse_tcb(address: int) -> ThreadInfo:
                registers = Registers(
                    self.elf, arch=self.arch, readmem=self.memory_read
                )
                data = self.memory_read(address, tcbsize)
                if not data or len(data) != tcbsize:
                    self.logger.error(f"Invalid TCB size: {len(data)} != {tcbsize}")
                    return ThreadInfo("<invalid>", 0, "Invalid", registers)

                if tcbinfo.name_off == 0:
                    name = "<noname>"
                else:
                    name = self._read_str(address + tcbinfo.name_off)
                self.logger.debug(f"loading thread: {name}, tcb@{address:#x}")
                pid = utils.uint16_t(data[tcbinfo.pid_off : tcbinfo.pid_off + 2])
                pid = pid if pid != 0 else self.PID0_ID
                state = utils.uint8_t(data[tcbinfo.state_off : tcbinfo.state_off + 1])
                state = states[state] if state < len(states) else "Unknown"

                if address in running_tasks:
                    # Running task registers is not in memory, best chance is the registers
                    # stored in g_last_regs when assert happened.
                    last_regs = self.elf.get_symbol("g_last_regs").value
                    cpu = running_tasks.index(address)
                    xcpregs = cpu * regsize + last_regs
                else:
                    off = tcbinfo.regs_off
                    xcpregs = data[off : off + pointer.sizeof()]
                    xcpregs = pointer.parse(xcpregs)

                try:
                    registers.load(addr=xcpregs)
                except ValueError as e:
                    self.logger.error(f"Failed to load registers: {e}")

                self.logger.debug(f"Parse TCB: {name}({pid},{state})")
                return ThreadInfo(name, pid, state, registers)

            data, sym = self._read_symbol("g_pidhash")
            pidhash = pointer.parse(data)
            data = self.memory_read(pidhash, pointer.sizeof() * npidhash)
            pidhash = utils.parse_array(data, pointer, npidhash)
            self.logger.debug(f"g_pidhash: {pidhash}@{sym.value:#x}")

            self.threads = [parse_tcb(tcb) for tcb in pidhash if tcb]
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
        if pid == -1 or pid == 0:
            # -1: all threads, 0: current thread. No need to switch for both cases
            # See https://sourceware.org/gdb/current/onlinedocs/gdb.html/Packets.html#Packets
            return self.registers

        if pid is None:
            # None is only used during our initial setup, not of GDB RSP protocol
            pid = self.PID0_ID

        for t in self.threads:
            if pid == t.pid:
                self.pid = pid
                self.registers = t.registers
                return self.registers

    def current_thread(self) -> int:
        return self.pid

    def memory_read(self, address: int, length: int) -> bytes:
        self.logger.debug(f"Read: {address:#x} {length}Bytes")

        # Check the real address from remap table
        for fromaddr, toaddr, total in self.remap:
            if toaddr <= address < toaddr + total:
                address = fromaddr + (address - toaddr)
                if address + length > fromaddr + total:
                    # Do not support cross region read
                    length = fromaddr + total - address
                self.logger.debug(f"Remap to: {address:#x} {length}Bytes")
                break

        # Try cached memory
        for mem in self.memories:
            if mem.address <= address < mem.address + len(mem):
                offset = address - mem.address
                # Limit the length to available data
                length = min(length, len(mem) - offset)
                return mem.data[offset : offset + length]

        # Try core
        if self.core and (value := self.core.read_from(address, length)):
            return bytes(value)

        # Try elf
        return bytes(self.elf.read_from(address, length) or [])

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

    def monitor_command(self, command: bytes) -> str:
        """Handle monitor command"""
        self.logger.debug(f"Monitor command: {command}")
        if command.startswith(b"setregs"):
            command = command.decode("ascii")
            _, address = command.split(" ")
            try:
                address = (
                    int(address, 16)
                    if "0x" in address or "0X" in address
                    else int(address)
                )
            except ValueError:
                # try if it's a symbol, note that expression is not supported.
                address = self.elf.get_symbol(address).value

            self.registers.load(address)
            return f"Loaded registers from {address:#x}\n"
        elif command.startswith(b"help"):
            return (
                "setregs <address|symbol>: Load registers from address or symbol.\n"
                "\tNeed to execute `maint flush register-cache` to see latest registers\n"
                "help: Show this help message\n"
            )
