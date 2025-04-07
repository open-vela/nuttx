############################################################################
# tools/pynuttx/nxgdb/uname.py
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

from . import utils


class UnameCommand(gdb.Command):
    """Output specific system information"""

    def __init__(self):
        super().__init__("uname", gdb.COMMAND_USER)

    def parse_arguments(self, argv):
        parser = argparse.ArgumentParser(description=self.__doc__)
        parser.add_argument(
            "-a",
            "--all",
            action="store_true",
            help="Output all information in the following order",
        )
        parser.add_argument(
            "-s", "--kernel-name", action="store_true", help="Output the kernel name"
        )
        parser.add_argument(
            "-n",
            "--nodename",
            action="store_true",
            help="Output the hostname of the network node",
        )
        parser.add_argument(
            "-r",
            "--kernel-release",
            action="store_true",
            help="Output the kernel release number",
        )
        parser.add_argument(
            "-v",
            "--kernel-version",
            action="store_true",
            help="Output the kernel version number",
        )
        parser.add_argument(
            "-m",
            "--machine",
            action="store_true",
            help="Output the hardware architecture name of the host",
        )
        parser.add_argument(
            "--version",
            action="store_true",
            help="Display version information and exit",
        )

        try:
            args = parser.parse_args(argv)
        except SystemExit:
            return None

        return args

    def invoke(self, args, from_tty):

        args = self.parse_arguments(gdb.string_to_argv(args))
        if not args:
            return

        kernel_name = "NuttX"
        node_name = str(utils.get_symbol_value("CONFIG_LIBC_HOSTNAME") or "").replace(
            '"', ""
        )
        kernel_release = str(
            utils.get_symbol_value("CONFIG_VERSION_STRING") or ""
        ).replace('"', "")
        kernel_version = str(utils.parse_and_eval("g_version") or "").replace('"', "")
        machine = str(utils.get_symbol_value("CONFIG_ARCH") or "").replace('"', "")
        tool_version = "1.0.0"

        if args.all:
            print(
                kernel_name,
                node_name,
                kernel_release,
                kernel_version,
                machine,
                tool_version,
            )
            return
        if args.kernel_name:
            print(kernel_name)
        if args.nodename:
            print(node_name)
        if args.kernel_release:
            print(kernel_release)
        if args.kernel_version:
            print(kernel_version)
        if args.machine:
            print(machine)
        if args.version:
            print(tool_version)

    def diagnose(self, *args, **kwargs):
        return {
            "title": "Uname Report",
            "summary": "dump the kernel version number",
            "command": "uname",
            "result": "info",
            "message": gdb.execute("uname -a", to_string=True),
        }
