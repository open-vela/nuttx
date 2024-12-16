############################################################################
# tools/pynuttx/nxtrace/rtt.py
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
import os
import subprocess
import threading
import time

logger = logging.getLogger(__name__)
logger.setLevel(logging.WARN)


class SeggerRTT:
    def __init__(
        self,
        device,
        interface,
        address=None,
        speed=None,
        channel=None,
    ):
        self.receive_total = 0
        self.receive_rate = 0

        # Check if the JLinkRTTLogger command exists
        if not any(
            os.access(os.path.join(path, "JLinkRTTLogger"), os.X_OK)
            for path in os.environ["PATH"].split(os.pathsep)
        ):
            raise RuntimeError(
                "JLinkRTTLogger command not found, please ensure SEGGER JLink software is installed"
            )

        if channel is None:
            channel = 0
        if speed is None:
            speed = 16000

        # Create a named pipe
        self.pipe = f"/tmp/jlink_rtt_pipe_{channel}"
        if not os.path.exists(self.pipe):
            os.mkfifo(self.pipe)

        # Start the JLinkRTTLogger process
        cmd = [
            "JLinkRTTLogger",
            "-If",
            interface,
            "-device",
            device,
            "-RTTChannel",
            f"{channel}",
            "-Speed",
            f"{speed}",
        ]

        if address is not None:
            cmd.append("-RTTAddress")
            cmd.append(f"0x{address:x}")

        cmd.append(self.pipe)
        self.running = False
        self.buffer = bytearray()
        self.condition = threading.Condition()
        print(f"Starting JLinkRTTLogger with command: {cmd}")
        self.jlink_process = subprocess.Popen(cmd)
        self.thread = threading.Thread(target=self.process_data, daemon=True)

    def __del__(self):
        if self.running:
            self.stop()

    def process_data(self):
        last_receive_total = 0
        last_time = time.time()

        # Independent thread receives data to avoid data loss
        with open(self.pipe, "rb") as f:
            while self.running:
                data = f.read1()

                if data is None:
                    continue

                if time.time() - last_time > 0.5:
                    now_time = time.time()
                    self.receive_rate = (self.receive_total - last_receive_total) / (
                        now_time - last_time
                    )
                    last_time = now_time
                    last_receive_total = self.receive_total
                    logger.debug(
                        f"Receive rate: {self.receive_rate/1024:.2f} KB/s, total: {self.receive_total/1024/1024:.2f} MB"
                    )
                    self.receive_total += len(data)

                # Notify the waiting thread that data is available
                with self.condition:
                    self.buffer.extend(data)
                    self.condition.notify()

    def read(self, size=1024):
        try:
            with self.condition:
                while not self.buffer:
                    # Wait for data to be available
                    self.condition.wait()

            data = bytes(self.buffer[:size])
            self.buffer = self.buffer[size:]
            return data
        except Exception as e:
            logger.error(f"Error reading data: {e}")
            exit(1)

    def start(self):
        if self.jlink_process.poll() is not None:
            logger.error("Failed to start JLinkRTTLogger process")
            raise RuntimeError("Failed to start JLinkRTTLogger process")
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()
        os.unlink(self.pipe)
