############################################################################
# tools/pynuttx/nxtrace/general_rtt.py
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
from abc import ABC, abstractmethod
from ctypes import Structure, c_char, c_int32, c_uint32, c_uint64, c_void_p, sizeof
from multiprocessing import shared_memory
from typing import Union

try:
    import lauterbach.trace32.rcl as trace32
    from pylink.jlink import JLink
except ImportError:
    print("Please execute the following command to install dependencies:")
    print("pip install lauterbach-trace32-rcl pylink-square")
    exit(1)

log = logging.getLogger(__name__)


class SEGGER_RTT_CB(Structure):
    _fields_ = [
        ("acID", c_char * 16),
        ("MaxNumUpBuffers", c_int32),
        ("MaxNumDownBuffers", c_int32),
    ]


class SEGGER_RTT_BUFFER_UP(Structure):
    _fields_ = [
        ("sName", c_void_p),
        ("pBuffer", c_void_p),
        ("SizeOfBuffer", c_uint32),
        ("WrOff", c_uint32),
        ("RdOff", c_uint32),
        ("Flags", c_uint32),
    ]


class SEGGER_RTT_BUFFER_DOWN(Structure):
    _fields_ = [
        ("sName", c_void_p),
        ("pBuffer", c_void_p),
        ("SizeOfBuffer", c_uint32),
        ("WrOff", c_uint32),
        ("RdOff", c_uint32),
        ("Flags", c_uint32),
    ]


def struct_factory(cls, bitwidth=32):
    class MeteStructure(type):
        def __new__(cls, name, bases, attrs, bitwidth=32):
            if "_fields_" in attrs:
                realfields = list()
                for field in attrs["_fields_"]:
                    if field[1] == c_void_p:
                        field = (field[0], c_uint32 if bitwidth == 32 else c_uint64)
                    realfields.append(field)
            else:
                raise ValueError("No _fields_ in class")

            class DynamicStructure(Structure):
                _fields_ = realfields
                fields = {field[0]: field[1] for field in realfields}

            return DynamicStructure

    class DynamicStructure(metaclass=MeteStructure, bitwidth=bitwidth):
        _fields_ = cls._fields_

    return DynamicStructure


class MemoryController(ABC):
    @abstractmethod
    def read(self, addr: int, size: int) -> bytes:
        """Read data from memory"""
        pass

    @abstractmethod
    def write(self, addr: int, data: Union[bytes, int]) -> int:
        """Write data to memory"""
        pass


# This class is used to read and write to a structure in memory
class Value:
    def __init__(
        self,
        controller: MemoryController,
        struct: type(Structure),
        addr: int = None,
    ):
        if addr is None:
            raise ValueError("Must specify addr")

        self.struct = struct
        self.controller = controller
        self.address = addr

    def read(self):
        data = self.controller.read(self.address, sizeof(self.struct))
        if len(data) == 0:
            return b""
        parsed = self.struct.from_buffer_copy(data)
        for field, _ in self.struct._fields_:
            self._cache[field] = getattr(parsed, field)

    def __getattr__(self, key):
        if key in self.struct.fields:
            offset = getattr(self.struct, key).offset
            size = getattr(self.struct, key).size
            data = self.controller.read(self.address + offset, size)
            parsed = self.struct.fields[key].from_buffer_copy(data).value
            log.debug("parsed %s: %s", key, parsed)
            return parsed
        else:
            return super().__getattribute__(key)

    def __setattr__(self, key, value):
        if key == "struct":
            super().__setattr__(key, value)
        elif key in self.struct.fields:
            offset = getattr(self.struct, key).offset
            self.controller.write(self.address + offset, value)
        else:
            super().__setattr__(key, value)


class RTTUpChannel:
    def __init__(self, up: Value):
        self.up = up

    def read(self):
        address = self.up.pBuffer
        wroff = self.up.WrOff
        rdoff = self.up.RdOff

        if wroff == rdoff:
            return b""
        elif wroff > rdoff:
            data = self.up.controller.read(address + rdoff, wroff - rdoff)
            self.up.RdOff = wroff
            return data
        else:
            size = self.up.SizeOfBuffer - rdoff
            data = self.up.controller.read(address + rdoff, size)
            data += self.up.controller.read(address, wroff)
            self.up.RdOff = wroff
            return data


class RTTDownChannel:
    def __init__(self, down: Value):
        self.down = down

    def write(self, data: bytes, blocking: bool = False):
        if blocking:
            while data:
                sent = self.write(data, blocking=False)
                data = data[sent:]
            return

        address = self.down.pBuffer
        wroff = self.down.WrOff
        rdoff = self.down.RdOff
        size = self.down.SizeOfBuffer

        if wroff >= size or rdoff >= size:
            raise ValueError("Invalid down buffer")

        written = 0
        if wroff >= rdoff:
            remaining = size - wroff
            if rdoff == 0:
                remaining -= 1

            data_to_write = data[:remaining]
            self.down.controller.write(address + wroff, data_to_write)
            written = len(data_to_write)
            data = data[written:]
            wroff = (wroff + written) % size

        remaining = rdoff - wroff - 1
        if remaining < 0:
            remaining = 0

        num = min(remaining, len(data))
        self.down.controller.write(address + wroff, data[:num])
        written += num
        wroff += num

        self.down.WrOff = wroff
        return written


class SEGGER_RTT:
    def __init__(self, memory: MemoryController, addr: int = None, bitwidth: int = 32):
        self.SEGGER_RTT_CB = struct_factory(SEGGER_RTT_CB, bitwidth)
        self.SEGGER_RTT_BUFFER_UP = struct_factory(SEGGER_RTT_BUFFER_UP, bitwidth)
        self.SEGGER_RTT_BUFFER_DOWN = struct_factory(SEGGER_RTT_BUFFER_DOWN, bitwidth)

        self.rtt_cb = Value(memory, self.SEGGER_RTT_CB, addr=addr)
        self.upbuffer = [bytes() for _ in range(self.rtt_cb.MaxNumUpBuffers)]
        self.downbuffer = [bytes() for _ in range(self.rtt_cb.MaxNumDownBuffers)]

        self.aUp = [
            RTTUpChannel(
                Value(
                    memory,
                    self.SEGGER_RTT_BUFFER_UP,
                    self.rtt_cb.address
                    + sizeof(self.SEGGER_RTT_BUFFER_UP) * i
                    + sizeof(self.SEGGER_RTT_CB),
                )
            )
            for i in range(self.rtt_cb.MaxNumUpBuffers)
        ]

        self.aDown = [
            RTTDownChannel(
                Value(
                    memory,
                    self.SEGGER_RTT_BUFFER_DOWN,
                    self.rtt_cb.address
                    + sizeof(self.SEGGER_RTT_BUFFER_DOWN) * i
                    + sizeof(self.SEGGER_RTT_BUFFER_UP) * self.rtt_cb.MaxNumUpBuffers
                    + sizeof(self.SEGGER_RTT_CB),
                )
            )
            for i in range(self.rtt_cb.MaxNumDownBuffers)
        ]

    def read(self, channel: int, size: int = -1) -> bytes:
        if channel in range(self.rtt_cb.MaxNumUpBuffers):
            self.upbuffer[channel] += self.aUp[channel].read()
            if size == -1:
                data = self.upbuffer[channel]
                self.upbuffer[channel] = bytes()
            else:
                data = self.upbuffer[channel][:size]
                self.upbuffer[channel] = self.upbuffer[channel][size:]
            return data
        else:
            raise ValueError("Invalid channel")

    def write(self, channel: int, data: bytes, blocking: bool = False):
        if channel in range(self.rtt_cb.MaxNumDownBuffers):
            self.aDown[channel].write(data, blocking)
        else:
            raise ValueError("Invalid channel")


class Trace32MemoryController(MemoryController):
    def __init__(self, debugger: trace32.Debugger):
        self.debugger = debugger
        self._addr = dict()

    def read(self, addr: int, size: int) -> bytes:
        if addr not in self._addr:
            self._addr[addr] = self.debugger.address(access="E", value=addr)

        ret = self.debugger.memory.read_uint8_array(
            self._addr[addr], length=size
        ).tobytes()
        log.debug("trace32 read %#x len:%d %s", addr, size, ret)
        return ret

    def write(self, addr: int, data: Union[bytes, int]) -> int:
        if addr not in self._addr:
            self._addr[addr] = self.debugger.address(access="D", value=addr)

        if isinstance(data, int):
            value = data.to_bytes(length=4, byteorder="little")
        else:
            value = data

        ret = self.debugger.memory.write_uint8_array(self._addr[addr], value)
        log.debug("trace32 write %#x len:%d %s", addr, len(value), ret)
        return ret


class JlinkMemoryController(MemoryController):
    def __init__(self, debugger: JLink):
        self.debugger = debugger

    def read(self, addr: int, size: int) -> bytes:
        ret = bytes(self.debugger.memory_read8(addr=addr, num_bytes=size))
        log.debug("jlink read %#x len:%d %s", addr, size, ret)
        return ret

    def write(self, addr: int, data: Union[bytes, int]) -> int:
        if isinstance(data, int):
            data = data.to_bytes(length=4, byteorder="little")

        ret = self.debugger.memory_write8(addr=addr, data=data)
        log.debug("jlink write %#x len:%d %s", addr, len(data), ret)
        return ret


class ShmMemoryController(MemoryController):
    def __init__(self, name: str, rttaddr: int):
        self.shm = shared_memory.SharedMemory(name=name)
        self.rttaddr = rttaddr
        self.rttoffset = self.shm.buf.tobytes().find(b"SEGGER RTT")

    def read(self, addr: int, size: int) -> bytes:
        offset = addr - self.rttaddr + self.rttoffset
        if offset < 0 or offset + size > self.shm.size:
            raise ValueError("Invalid address")

        ret = self.shm.buf[offset : offset + size].tobytes()
        log.debug(f"shm read 0x{addr:x}(offset:{offset}) size:{size} {ret}")
        return ret

    def write(self, addr: int, data: Union[bytes, int]) -> int:
        offset = addr - self.rttaddr + self.rttoffset
        if isinstance(data, int):
            data = data.to_bytes(length=4, byteorder="little")

        if offset < 0 or offset + len(data) > self.shm.size:
            raise ValueError("Invalid address")

        self.shm.buf[offset : offset + len(data)] = data
        log.debug(f"shm write 0x{addr:x}(offset:{offset}) size:{len(data)} {data}")
        return len(data)
