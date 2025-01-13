############################################################################
# tools/pynuttx/nxgdb/target.py
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

import multiprocessing

import gdb
import nxstub


class Target(gdb.Command):
    """Use nxstub to parse crash log dump, core dump or memory dump, as target."""

    def __init__(self):
        super().__init__("target stub", gdb.COMMAND_USER)
        self.process = None

    def invoke(self, args, from_tty):
        self.dont_repeat()

        if "-e" not in args and "--elf" not in args:
            args += f" -e {gdb.objfiles()[0].filename}"

        if "-a" not in args and "--arch" not in args:
            args += f" -a {gdb.selected_inferior().architecture().name().lower()}"

        args = gdb.string_to_argv(args)
        try:
            parsed = nxstub.parse_args(args)
        except SystemExit:
            return

        # If currently has connection to target, disconnect it
        inferior = gdb.selected_inferior()
        if inferior and inferior.connection and inferior.connection.is_valid():
            gdb.execute("disconnect")

        def kill(event=None):
            if self.process:
                self.process.kill()
                self.process.join()
                self.process = None
                print("nxstub process killed")

        if self.process:
            y = input("nxstub process already running, kill it? [y/n] ")
            if y.lower() != "y":
                return

            kill()

        process = multiprocessing.Process(target=nxstub.main, args=(args,))
        process.start()
        self.process = process

        gdb.events.gdb_exiting.connect(kill)
        print("")

        # Wait server to start
        gdb.execute(f"target remote :{parsed.port}", from_tty=True)
