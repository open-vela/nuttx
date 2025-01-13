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

from . import utils
from .target import Target


class GDBStub:
    def __init__(
        self,
        target: Target,
        port=1234,
        proxymode=False,
    ):
        self.proxymode = proxymode
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
                packet = utils.get_packet(self.socket)
                if packet in b"+-":
                    continue

                self.socket.send(b"+")
                if packet is None:
                    self.logger.info("Connection closed")
                    return

                if self.proxymode:
                    self.forward_packet(packet)
                else:
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

        output = utils.encode_packet(packet)
        return self.send_raw_packet(output, nowait)

    def send_unsupported(self):
        self.send_packet("")

    def process_packet(self, packet: bytes):
        self.logger.debug(f"Process packet: {packet}")
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

    def forward_packet(self, packet: bytes):
        packet_type = packet[0]
        if packet_type in b"iIzZsSmMxX":
            try:
                self.logger.debug(f"Forwarding packet: {packet[:20]}...")
                response = self.target.send_packet(packet)
                self.logger.debug(f"Received response: {response and response[:20]}...")
            except Exception as e:
                self.logger.error(f"Error forwarding packet: {e}")
                return

            if response:
                # The response is the raw packet received via RSP, excluding start, end and checksum
                self.send_raw_packet(response, nowait=True)
            else:
                self.send_packet("")
        elif packet_type in b"?vqgpPTHc\x03D":
            self.process_packet(packet)
        else:
            self.logger.debug(f"Ignore packet: {packet}")
            self.send_unsupported()

    def handle_q(self, packet: bytes):
        packet = packet.decode("ascii")
        if packet.startswith("qSupported"):
            self.send_packet("PacketSize=FFFF")
        elif packet.startswith("qC"):
            pid = self.target.current_thread()
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
        elif packet.startswith("qRcmd"):
            try:
                _, command = packet.split(",")
                command = unhexlify(command)
                response = self.target.monitor_command(command)
                if response is None:
                    self.send_unsupported()
                    return
            except Exception as e:
                self.logger.error(f"Error executing monitor command: {e}")
                # Note that older GDB may treat it as normal output instead of error, but nothing hurts.
                response = f"E.{e}\n"

            if isinstance(response, str):
                response = response.encode("ascii")
            self.send_packet(hexlify(response))
        else:
            self.send_unsupported()

    def handle_v(self, packet: bytes):
        if packet.startswith(b"vMustReplyEmpty"):
            self.send_packet("")
        else:
            self.send_unsupported()

    def handle_questionmark(self, packet: bytes):
        running = next((t for t in self.threads if t.state == "Running"), None)
        # FIXME t.state may be a number when g_statesnames gets optimized out.
        # That's fine for now since we can identify the running thread manually.
        if running:
            self.registers = self.target.switch_thread(running.pid)
            self.send_packet(f"T02thread:{running.pid:x};")
        else:
            self.send_packet("S05")

    def handle_g(self, packet: bytes):
        self.send_packet(self.registers.to_g())

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
        self.logger.info("Ctrl-C received")
        if self.proxymode:
            self.target.stop()
        else:
            self.send_packet("S00")

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

    def handle_c(self, packet):
        if hasattr(self.target, "cont"):

            def stopped(stopreason):
                # FIXME callback from another thread
                self.logger.info(f"Target stopped: {stopreason}")
                self.handle_questionmark(None)

            # Notify the stub when it's stopped
            self.target.cont(stopped)
        else:
            self.send_unsupported()
