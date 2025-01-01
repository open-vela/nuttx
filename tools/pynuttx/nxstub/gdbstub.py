############################################################################
# tools/pynuttx/nxstub/gdbstub.py
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
import socket
import traceback
from binascii import hexlify, unhexlify
from typing import Union

from .target import Target


class GDBStub:
    def __init__(
        self,
        target: Target,
        port=1234,
    ):
        self.threads = target.update_threads()
        self.registers = target.switch_thread()
        self.target = target
        self.exiting = False
        self.socket = None
        self.port = port
        self.logger = logging.getLogger(__name__)

    def run(self):
        self.socket = self.wait_connect()
        if self.socket is None:
            return

        while not self.exiting:
            try:
                packet = self.get_packet()
                if packet is None:
                    self.logger.info("Connection closed")
                    return

                self.process_packet(packet)

            except Exception as e:
                self.logger.error(f"Error in stub thread: {e} {traceback.format_exc()}")
        self.logger.info("Stub thread exited")

    def wait_connect(self) -> socket.socket:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        port = self.port
        try:
            listener.bind(("localhost", port))
            self.logger.info(f"Listening on localhost:{port}")
            listener.listen(1)
        except socket.error as e:
            self.logger.error(
                f"Cannot listen on localhost:{port}: error {e[0]} - {e[1]}"
            )
            return None

        client, addr = listener.accept()
        self.logger.info(f"Client connected from {addr[0]}:{addr[1]}")
        listener.close()
        return client

    def get_packet(self) -> Union[bytes, None]:
        buffer = bytearray()
        started = False
        escaping = False
        checksum = 0
        while True:
            c = self.socket.recv(1)
            if not started:
                if c == b"\x03":
                    return b"\x03"
                if c == b"$":
                    started = True
                continue

            if escaping:
                c ^= 0x20
                escaping = False
            elif c == b"}":
                escaping = True
                checksum += ord(c)
                continue

            if c == b"#":
                expected = self.socket.recv(2)
                expected = int(expected.decode("ascii"), 16)
                if expected != checksum & 0xFF:
                    self.logger.error(
                        f"checksum error: {expected:#x} != {checksum & 0xFF:#x}"
                    )
                    self.socket.send(b"-")  # No ack
                    checksum = 0
                    started = False
                    buffer = bytearray()
                    continue
                else:
                    self.socket.send(b"+")
                    break
            else:
                checksum += ord(c)
                buffer.append(ord(c))

        self.logger.debug(f"Received packet: {buffer}")
        return bytes(buffer)

    def send_raw_packet(self, data: bytes, nowait=False) -> bool:
        checksum = sum(data) & 0xFF
        self.socket.send(b"$")
        self.socket.send(data)
        self.socket.send(b"#")
        self.socket.send(b"%02x" % checksum)
        self.logger.debug(f"Sent packet: {data}")
        if nowait:
            return True
        ack = self.socket.recv(1)
        return ack == b"+"

    def send_packet(self, packet: Union[bytes, str], nowait=False) -> None:
        if isinstance(packet, str):
            packet = packet.encode("ascii")

        output = list()
        for c in packet:
            if c in b"$#*}":
                output.append(ord("}"))
                c ^= 0x20
            output.append(c)
        return self.send_raw_packet(bytes(output), nowait)

    def send_unsupported(self):
        self.send_packet("")

    def process_packet(self, packet: bytes):
        attribute = "handle_" + chr(packet[0])
        handler = {
            "handle_?": self.handle_questionmark,
            "handle_\x03": self.handle_etx,
        }.get(attribute)

        if not handler and not hasattr(self, attribute):
            self.logger.error(f"Unsupported packet: {packet}")
            self.send_unsupported()
            return

        handler = handler or getattr(self, attribute)

        try:
            handler(packet)
        except Exception as e:
            self.logger.error(f"Error packet{packet}: {e}\n {traceback.format_exc()}")
            self.send_packet("EF1")

    def handle_q(self, packet: bytes):
        packet = packet.decode("ascii")
        if packet.startswith("qSupported"):
            self.send_packet("PacketSize=FFFF")
        elif packet.startswith("qC"):
            pid = next((t.pid for t in self.threads if t.state == "Running"), 0)
            self.logger.debug(f"Current thread: {pid}")
            self.send_packet(f"QC{pid:x}")
        elif packet.startswith("qfThreadInfo"):
            reply = "".join(f"{thread.pid:x}," for thread in self.threads)
            reply = "m" + reply[:-1] if reply else "l"
            self.send_packet(reply)
        elif packet.startswith("qsThreadInfo"):
            self.send_packet("l")
        elif packet.startswith("qThreadExtraInfo"):
            pid = int(packet.split(",")[1], 16)
            info = next((t for t in self.threads if t.pid == pid), None)
            info = f"{info.name},{info.state}" if info else f"Invalid PID {pid}"
            self.send_packet(hexlify(info.encode("ascii")))
        else:
            self.send_unsupported()

    def handle_v(self, packet: bytes):
        if packet.startswith(b"vMustReplyEmpty"):
            self.send_packet("")
        else:
            self.send_unsupported()

    def handle_questionmark(self, packet: bytes):
        self.send_packet("S05")

    def handle_g(self, packet: bytes):
        reply = b""
        offset = 0
        for reg in self.registers:
            if reg.offset and reg.offset != offset:
                reply += b"xx" * (reg.offset - offset)

            if not reg.has_value:
                reply += b"xx" * reg.size
            else:
                reply += hexlify(bytes(reg))

            offset += reg.offset + reg.size
        self.send_packet(reply)

    def handle_p(self, packet: bytes):
        packet = packet.decode("ascii")
        regnum = int(packet[1:], 16)
        reg = self.registers.get(regnum=regnum)
        self.send_packet(hexlify(bytes(reg)) if reg else b"xx" * 4)

    def handle_P(self, packet: bytes):
        packet = packet.decode("ascii")
        regnum, value = packet[1:].split("=")
        regnum = int(regnum, 16)
        value = unhexlify(value)
        self.registers.set(regnum=regnum, value=value)
        self.send_packet("OK")

    def handle_m(self, packet: bytes):
        packet = packet.decode("ascii")
        addr, length = packet[1:].split(",")
        reply = self.target.memory_read(int(addr, 16), int(length, 16))
        reply = hexlify(reply)
        self.send_packet(reply)

    def handle_M(self, packet: bytes):
        packet = packet.decode("ascii")
        addr, length_and_data = packet[1:].split(",")
        length, data = length_and_data.split(":")
        ok = self.target.memory_write(int(addr, 16), unhexlify(data), int(length, 16))
        self.send_packet("OK" if ok else "")

    def handle_etx(self, packet: bytes):
        # FIXME no reply needed
        self.send_packet("S00")
        self.logger.info("Ctrl-C received")

    def handle_k(self, packet: bytes):
        self.exiting = True
        self.logger.info("Kill request received")

    def handle_T(self, packet):
        self.send_packet("OK")

    def handle_H(self, packet):
        pid = int(packet[2:], 16)
        registers = self.target.switch_thread(pid)
        if not registers:
            self.send_packet("E01")
            return
        self.registers = registers
        self.send_packet("OK")
