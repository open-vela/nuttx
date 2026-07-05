############################################################################
# tools/pynuttx/nxgdb/nxrtt.py
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

import argparse
import atexit
import fcntl
import os
import select
import signal
import socket
import termios
import threading

import gdb
import nxgdb.autocompeletion as autocompeletion
import nxgdb.utils as utils


class RttConsoleSession:
    def __init__(self):
        self.sock = None
        self.thread = None
        self.running = False
        self.enabled = False
        self.tty_fd = None
        self.old_termios = None
        self.old_flags = None
        self.lock = threading.Lock()
        self.connected_events = ()

    def active(self):
        with self.lock:
            return self.running

    def resolve_rtt_addr(self):
        try:
            sym = utils.get_global_var("_SEGGER_RTT")
            if sym is not None:
                return int(sym.value().address)

            return int(gdb.parse_and_eval("&_SEGGER_RTT"))
        except gdb.error as exc:
            raise gdb.GdbError("Cannot resolve _SEGGER_RTT from current ELF") from exc

    def set_jlink_rtt_addr(self, addr):
        command = f"monitor exec SetRTTAddr 0x{addr:x}"
        gdb.write(f"RTT: _SEGGER_RTT = 0x{addr:x}; {command}\n")
        try:
            gdb.execute(command)
        except gdb.error as exc:
            raise gdb.GdbError(
                "RTT: failed to set RTT address; connect JLinkGDBServer first"
            ) from exc

    def connect(self):
        try:
            self.sock = socket.create_connection(("127.0.0.1", 19021), timeout=3.0)
            self.sock.setblocking(False)
        except OSError as exc:
            self.sock = None
            raise gdb.GdbError(
                "RTT: failed to connect 127.0.0.1:19021; "
                "check JLinkGDBServer -RTTTelnetPort and other RTT clients"
            ) from exc

        gdb.write("RTT: connected to 127.0.0.1:19021\n")

    def save_terminal(self):
        try:
            self.tty_fd = os.open("/dev/tty", os.O_RDWR | os.O_NONBLOCK)
        except OSError as exc:
            raise gdb.GdbError("RTT: nxrtt run needs an interactive terminal") from exc

        self.old_termios = termios.tcgetattr(self.tty_fd)
        self.old_flags = fcntl.fcntl(self.tty_fd, fcntl.F_GETFL)

    def enter_console_mode(self):
        attrs = termios.tcgetattr(self.tty_fd)
        attrs[3] &= ~(termios.ECHO | termios.ICANON | termios.IEXTEN)
        attrs[6][termios.VMIN] = 1
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.tty_fd, termios.TCSADRAIN, attrs)
        fcntl.fcntl(self.tty_fd, fcntl.F_SETFL, self.old_flags | os.O_NONBLOCK)

    def restore_terminal(self):
        if self.tty_fd is None:
            return

        if self.old_termios is not None:
            try:
                termios.tcsetattr(self.tty_fd, termios.TCSADRAIN, self.old_termios)
            except termios.error:
                pass
            self.old_termios = None

        if self.old_flags is not None:
            try:
                fcntl.fcntl(self.tty_fd, fcntl.F_SETFL, self.old_flags)
            except OSError:
                pass
            self.old_flags = None

        try:
            os.close(self.tty_fd)
        except OSError:
            pass
        self.tty_fd = None

    def attach(self):
        with self.lock:
            if self.running:
                return False

        addr = self.resolve_rtt_addr()
        self.set_jlink_rtt_addr(addr)
        self.connect()
        self.save_terminal()
        self.enter_console_mode()

        with self.lock:
            self.running = True

        thread_class = getattr(gdb, "Thread", threading.Thread)
        self.thread = thread_class(target=self.relay_loop, name="nxrtt", daemon=True)
        self.thread.start()

        self.connect_events()
        gdb.write("RTT: console active; Ctrl-C interrupts target\n")
        return True

    def start(self):
        if not self.attach():
            raise gdb.GdbError("RTT: console is already active")

    def connect_events(self):
        self.connected_events = (
            (gdb.events.stop, self.stop),
            (gdb.events.exited, self.stop),
        )
        for registry, handler in self.connected_events:
            registry.connect(handler)

    def disconnect_events(self):
        for registry, handler in self.connected_events:
            try:
                registry.disconnect(handler)
            except RuntimeError:
                pass
        self.connected_events = ()

    def close_socket(self):
        if self.sock is not None:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def stop(self, event=None):
        was_running = False
        with self.lock:
            was_running = self.running
            self.running = False

        self.disconnect_events()
        self.close_socket()

        if (
            self.thread is not None
            and self.thread.is_alive()
            and self.thread is not threading.current_thread()
        ):
            self.thread.join(timeout=1.0)
        if self.thread is not threading.current_thread():
            self.thread = None

        self.restore_terminal()
        if was_running:
            gdb.write("\nRTT: console stopped, GDB terminal restored\n")

    def request_interrupt(self):
        interrupt = getattr(gdb, "interrupt", None)
        if interrupt:
            interrupt()
        else:
            os.kill(os.getpid(), signal.SIGINT)

    def request_stop_from_relay(self):
        def stop():
            self.stop()
            try:
                gdb.execute("interrupt", to_string=True)
            except gdb.error:
                pass

        gdb.post_event(stop)

    def relay_loop(self):
        while self.active():
            try:
                readable, _, _ = select.select([self.tty_fd, self.sock], [], [], 0.1)
            except (OSError, ValueError):
                break

            if self.tty_fd in readable:
                try:
                    data = os.read(self.tty_fd, 1024)
                except BlockingIOError:
                    data = b""
                except OSError:
                    break

                if b"\x03" in data:
                    data = data.replace(b"\x03", b"")
                    self.request_interrupt()

                if data:
                    try:
                        self.sock.sendall(data)
                    except OSError:
                        break

            if self.sock in readable:
                try:
                    data = self.sock.recv(4096)
                except BlockingIOError:
                    data = b""
                except OSError:
                    break

                if not data:
                    break

                try:
                    os.write(self.tty_fd, data)
                except OSError:
                    break

        with self.lock:
            need_cleanup = self.running
            self.running = False

        self.close_socket()
        if need_cleanup:
            self.request_stop_from_relay()


_SESSION = RttConsoleSession()
atexit.register(_SESSION.stop)


class NxrttPrefix(gdb.Command):
    """RTT console commands prefix."""

    def __init__(self):
        super().__init__("nxrtt", gdb.COMMAND_USER, prefix=True)


def _on_continue(event):
    if not _SESSION.enabled:
        return

    try:
        _SESSION.attach()
    except gdb.GdbError as exc:
        gdb.write(f"RTT: auto attach failed: {exc}\n")


gdb.events.cont.connect(_on_continue)


@autocompeletion.complete
class NxrttAuto(gdb.Command):
    """Enable or disable automatic RTT console on continue.

    Usage: nxrtt auto [on|off]
    """

    def get_argparser(self):
        parser = argparse.ArgumentParser(description=self.__doc__)
        parser.add_argument("state", nargs="?", choices=("on", "off"))
        return parser

    def __init__(self):
        super().__init__("nxrtt auto", gdb.COMMAND_USER)
        self.parser = self.get_argparser()

    @utils.dont_repeat_decorator
    def invoke(self, arg, from_tty):
        try:
            args = self.parser.parse_args(gdb.string_to_argv(arg))
        except SystemExit:
            return

        if args.state:
            _SESSION.enabled = args.state == "on"
        else:
            _SESSION.enabled = not _SESSION.enabled

        state = "on" if _SESSION.enabled else "off"
        gdb.write(f"RTT: auto console {state}\n")


@autocompeletion.complete
class NxrttRun(gdb.Command):
    """Run target with JLink RTT console attached.

    Usage: nxrtt run
    """

    def get_argparser(self):
        return argparse.ArgumentParser(description=self.__doc__)

    def __init__(self):
        super().__init__("nxrtt run", gdb.COMMAND_USER)
        self.parser = self.get_argparser()

    @utils.dont_repeat_decorator
    def invoke(self, arg, from_tty):
        try:
            self.parser.parse_args(gdb.string_to_argv(arg))
        except SystemExit:
            return

        try:
            _SESSION.start()
            gdb.execute("continue")
        finally:
            _SESSION.stop()
