############################################################################
# tools/pynuttx/nxgdb/elf.py
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
import os

import gdb

try:
    import lief
except ImportError:
    print('Package missing, please do "pip install lief"')

from .utils import dont_repeat_decorator, get_tcbs, has_field

CONFIG_ARCH_USE_SEPARATED_SECTION = has_field("struct module_s", "sectalloc")


class ElfImport(gdb.Command):
    def __init__(self):
        if has_field("struct task_group_s", "tg_bininfo"):
            super().__init__("elfimport", gdb.COMMAND_USER)

    @dont_repeat_decorator
    def invoke(self, args, from_tty):
        parser = argparse.ArgumentParser(description="import elf symbols to gdb")
        parser.add_argument("elfpath", type=str, help="elf file path, etc: apps/bin")
        try:
            args = parser.parse_args(args.split())
        except SystemExit:
            return None

        tcbs = get_tcbs()
        if tcbs is None:
            print("No TCBs found")
            return
        modules = []
        for tcb in tcbs:
            if tcb.group != 0 and tcb.group.tg_bininfo != 0:
                modules.append(tcb.group.tg_bininfo.mod)

        if not modules:
            print("No modules in current environment")
            return

        print(f"Found {len(modules)} modules in current environment")

        for mod in modules:
            sections = {}
            cmd = ""
            if CONFIG_ARCH_USE_SEPARATED_SECTION:
                for i in range(mod.nsect):
                    section = mod.sectalloc[i]
                    if section:
                        sections[i] = hex(section)
                filename = os.path.join(args.elfpath, mod.modname.string())
                cmd = f"add-symbol-file {filename} "
                elf = lief.parse(filename)
                for i, section in sections.items():
                    cmd += f"-s {elf.sections[i].name} {sections[i]} "
            else:
                filename = os.path.join(args.elfpath, mod.modname.string())
                cmd = f"add-symbol-file {filename} -s .text {hex(mod.textalloc)} -s .data {hex(mod.dataalloc)}"

            print(cmd)
            gdb.execute(cmd)
