############################################################################
# tools/pynuttx/nxgdb/diagnose.py
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
from gc import get_referrers

import gdb

from . import utils


class DiagnosePrefix(gdb.Command):
    """Diagnostic related commands."""

    def __init__(self):
        super().__init__("diagnose", gdb.COMMAND_USER, prefix=True)


class DiagnoseReport(gdb.Command):
    """Run diagnostics to generate reports."""

    def __init__(self):
        super().__init__("diagnose report", gdb.COMMAND_USER)

    @utils.dont_repeat_decorator
    def invoke(self, args, from_tty):
        parser = argparse.ArgumentParser(description=self.__doc__)
        parser.add_argument(
            "-o",
            "--output",
            type=str,
            help="report output file name",
        )
        parser.add_argument(
            "-c",
            "--command",
            nargs="+",
            help="only run specified commands (case-insensitive)",
        )
        parser.add_argument(
            "-l", "--list", action="store_true", help="list available commands"
        )

        try:
            args = parser.parse_args(gdb.string_to_argv(args))
        except SystemExit:
            return

        reportfile = (
            args.output
            if args.output
            else gdb.objfiles()[0].filename + ".diagnostics.json"
        )

        modules = utils.gather_modules()
        modules.remove("prefix")

        commands = utils.gather_gdbcommands(modules=modules)

        registered_command_types = {
            obj.__class__.__name__.lower()
            for cls in gdb.Command.__subclasses__()
            for obj in get_referrers(cls)
            if isinstance(obj, cls) and not isinstance(obj, type)
        }

        only_set = set(name.lower() for name in args.command) if args.command else None

        results = []
        for clz in commands:
            if hasattr(clz, "diagnose"):
                command = clz()
                name = clz.__name__.lower()
                if name not in registered_command_types:
                    continue

                if args.list:
                    gdb.write(f"{name}\n")
                    continue

                if only_set:
                    if name not in only_set:
                        continue

                gdb.write(f"Run command: {name}\n")
                try:
                    result = command.diagnose()
                except Exception as e:
                    result = {
                        "title": f"Command {name} failed",
                        "summary": "Command execution failed",
                        "result": "info",
                        "command": name,
                        "message": str(e),
                    }

                    gdb.write(f"Failed: {e}\n")

                result.setdefault("command", name)
                results.append(result)

        if args.list:
            return

        gdb.write(f"Write report to {reportfile}\n")
        with open(reportfile, "w") as f:
            f.write(utils.jsonify(results, indent=4))
