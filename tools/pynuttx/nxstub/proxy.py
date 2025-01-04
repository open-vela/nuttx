############################################################################
# tools/pynuttx/nxstub/proxy.py
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

import atexit
import logging
import queue
import socket
import threading
from binascii import unhexlify

from . import utils
from .target import Target


class TargetProxy(Target):
    def __init__(self, elf, arch=None, port=None):
        super().__init__(elf, arch=arch)
        self.exit = False
        self.logger = logging.getLogger(__name__)

        def send_thread(queue: queue.Queue, sock: socket.socket):
            while not self.exit:
                packet = queue.get()
                self.logger.debug(f"Send: {packet}")
                sock.send(packet)
                self.logger.debug("Sent.")

        def recv_thread(queue: queue.Queue, sock: socket.socket):
            while not self.exit:
                packet = utils.get_packet(sock)
                queue.put(packet)
                self.logger.debug(f"Received: {packet}")

        sock = self.connect_target(port)
        self.txqueue = queue.Queue()
        self.rxqueue = queue.Queue()
        self.sender = threading.Thread(target=send_thread, args=(self.txqueue, sock))
        self.receiver = threading.Thread(target=recv_thread, args=(self.rxqueue, sock))

        self.sender.start()
        self.receiver.start()

        self.clear_rxqueue()
        atexit.register(self.__del__)

    def __del__(self):
        self.logger.warning("Destructor called")
        self.exit = True
        self.txqueue.put(None)
        self.rxqueue.put(None)
        self.sender.join()
        self.receiver.join()

    def connect_target(self, port):
        target = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        target.connect(("localhost", port))
        self.logger.info(f"Connected to target: localhost:{port}")
        return target

    def clear_rxqueue(self):
        while (packet := self.receive_packet(block=False)) is not None:
            self.logger.debug(f"Clearing queue: {packet}")

    def send_packet(self, packet, timeout=None, noresponse=False):
        encoded = utils.encode_packet(packet)
        checksum = sum(encoded) & 0xFF
        encoded = b"$" + encoded + b"#" + b"%02x" % checksum

        self.clear_rxqueue()
        self.txqueue.put(encoded)
        ack = self.receive_packet(timeout)
        if ack != b"+":
            self.logger.warning(f"Not ack received: {ack}, try again...")
            ack = self.receive_packet(timeout=1)
            if ack != b"+":
                self.logger.error(f"Failed to send packet: {packet}, get ack: '{ack}'")
                return

        self.logger.debug(f"Sent packet: {packet}")
        if noresponse:
            return

        response = self.receive_packet(timeout)
        self.logger.debug(f"Target response: {response}")
        return response

    def receive_packet(self, timeout=1, block=True):
        try:
            return self.rxqueue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None

    def memory_read(self, address: int, length: int) -> bytes:
        self.logger.debug(f"Read target: {address:#x} {length}Bytes")
        response = self.send_packet(b"m%x,%x" % (address, length))
        data = unhexlify(response)
        return data

    def memory_write(self, address, data):
        # We do not support write for now for debug purpose
        self.logger.warning(f"Ignore write target: {address:#x} {len(data)}Bytes")
        pass

    def read_registers(self):
        response = self.send_packet(b"g")
        response = unhexlify(response)
        self.logger.debug(f"Raw registers: {response}")
        return response

    def update_threads(self):
        threads = super().update_threads()
        # Since we are doing live debugging, we need to correct the registers of running threads
        for t in threads:
            if t.state == "Running":
                t.registers.from_g(self.read_registers())
                break
        return threads

    def cont(self, callback):
        self.send_packet(b"c", noresponse=True)  # continue packet has no response

        self.running = True

        def checker(callback):
            self.logger.debug("Cont checker started, waiting for response...")
            while self.running:
                self.logger.debug("Checking...")
                if response := self.receive_packet(timeout=0.1):
                    self.logger.debug(f"Cont got response: {response}")
                    self.running = False
                    self.send_packet(b"z0")
                    self.update_threads()
                    callback(response)
                    return

            self.logger.error(f"Cont checker stopped, target: {self.state}")

        threading.Thread(target=checker, args=(callback,)).start()

    def stop(self):
        # Send a break signal, instead of a full packet.
        self.txqueue.put(b"\x03")

    def forward_packet(self, packet):
        self.txqueue.put(packet)
        return self.receive_packet()
