############################################################################
# tools/pynuttx/nxgdb/libuv.py
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

import gdb

from . import lists, utils


class UVQueue(lists.NxList):
    def __init__(
        self, list: gdb.Value, container_type=None, member=None, reverse=False
    ):
        super().__init__(list, container_type, member, reverse)

    def _get_first(self):
        return self.list

    def _get_next(self, node):
        return node["next"] if node["next"] != self.list else None

    def _get_prev(self, node):
        return node["prev"] if node["prev"] != self.list else None


class UVDump(gdb.Command):
    """Dump the information of uv_loop & handle_queue & uv_worker in libuv"""

    def __init__(self):
        super(UVDump, self).__init__("uv dump", gdb.COMMAND_USER)

    def dump_loop(self, loop):
        if not loop:
            return
        gdb.write("### dump uv_loop ###\n")
        gdb.write(f"uv_loop address: 0x{loop}, main content of loop: {{\n")
        gdb.write(f"  data: {loop['data']}\n")
        gdb.write(f"  active_handles: {loop['active_handles']}\n")
        gdb.write(f"  active_reqs: {loop['active_reqs']}\n")
        gdb.write(f"  stop_flag: {loop['stop_flag']}\n")
        gdb.write(f"  close_flag: {loop['close_flag']}\n")
        gdb.write(f"  flags: {loop['flags']}\n")
        gdb.write(f"  backend_fd: {loop['backend_fd']}\n")
        gdb.write(f"  nwatchers: {loop['nwatchers']}\n")
        gdb.write(f"  nfds: {loop['nfds']}\n")
        gdb.write(f"  emfile_fd: {loop['emfile_fd']}\n")
        gdb.write(f"  poll_fds: {loop['poll_fds']}\n")
        gdb.write(f"  poll_fds_used: {loop['poll_fds_used']}\n")
        gdb.write(f"  poll_fds_size: {loop['poll_fds_size']}\n")
        gdb.write(f"  poll_fds_iterating: {loop['poll_fds_iterating']}\n")
        gdb.write("}\n")

        return

    def dump_handle_queue(self, loop):
        gdb.write("### dump uv_handle queue & backtrace ###\n")
        if not loop:
            return
        head = loop["handle_queue"].address
        if not head:
            gdb.write("handle queue is None\n")
            return

        UV_HANDLE = utils.enum("enum uv_handle_flag")
        UV_HANDLE_BACKTRACE = utils.get_symbol_value("UV_HANDLE_BACKTRACE")
        flags = [
            (UV_HANDLE.HANDLE_REF, "R"),
            (UV_HANDLE.HANDLE_ACTIVE, "A"),
            (UV_HANDLE.HANDLE_INTERNAL, "I"),
        ]
        handle_queue = UVQueue(head, gdb.lookup_type("uv_handle_t"), "handle_queue")
        for i, uv_handle in enumerate(handle_queue):
            output = ""
            for handle, flag in flags:
                output += flag if handle.value & uv_handle["flags"] else "-"
            gdb.write(f"[{i}] [{output}] {uv_handle['type']} {uv_handle}\n")
            if UV_HANDLE_BACKTRACE > 0:
                gdb.write(f"backtrace: {uv_handle['backtrace']}\n")
            if self.detail:
                uv_handle_content = uv_handle.dereference().format_string(
                    styling=True, pretty_arrays=True, pretty_structs=True
                )
                gdb.write(f"uv_handle detail content: {uv_handle_content}\n")

    def get_loop_in_thread(self, thread):
        # TODO: find loop variable in current thread
        # depends on the bugfix in block.superblock
        if not thread:
            return None
        gdb.write(f"### [TODO] get uv_loop in thread {thread.ptid} ###\n")

    def get_loop(self, args):
        if args.loop:
            loop = utils.gdb_eval_or_none(args.loop)
            if loop is None:
                gdb.write("invalid loop ptr\n")
                return None
            return loop.cast(gdb.lookup_type("uv_loop_t").pointer())

        if args.pid:
            for thread in gdb.selected_inferior().threads():
                if args.pid == thread.ptid[1]:
                    return self.get_loop_in_thread(thread)
            gdb.write(f"invalid pid {args.pid}\n")
        else:
            return self.get_loop_in_thread(gdb.selected_thread())

        return None

    @utils.dont_repeat_decorator
    def invoke(self, argument: str, from_tty: bool):
        parser = argparse.ArgumentParser(description="libuv dump command")
        parser.add_argument(
            "mode",
            nargs="?",
            choices=["loop", "handle_queue", "uv_worker"],
            default="loop",
        )
        parser.add_argument("-p", "--pid", type=int, help="Thread PID of Quickapp")
        parser.add_argument("-l", "--loop", type=str, help="address of uv_loop_t")
        parser.add_argument(
            "-d",
            "--detail",
            action="store_const",
            const=True,
            default=False,
            help="print more detail information",
        )

        try:
            args = parser.parse_args(gdb.string_to_argv(argument))
        except SystemExit:
            gdb.write("invalid arguments.\n")
            return

        gdb.write(f"### dump libuv in {args.mode} mode ###\n")

        loop = self.get_loop(args)
        self.detail = args.detail

        if args.mode == "loop":
            self.dump_loop(loop)
        elif args.mode == "handle_queue":
            self.dump_handle_queue(loop)
        return
