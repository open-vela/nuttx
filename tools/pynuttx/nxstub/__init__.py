############################################################################
# tools/pynuttx/nxstub/__init__.py
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

__version__ = "0.0.1"

import argparse
import logging
import multiprocessing
import re
import signal
import subprocess
import traceback
from typing import List

from . import utils
from .gdbstub import GDBStub, Target
from .proxy import TargetProxy
from .registers import Registers, g_reg_table
from .target import RawMemory

# Default GDB init command, note it should start with a space
DEFAULT_GDB_INIT_CMD = " -ex 'bt full' -ex 'info reg' -ex 'display /40i $pc-40'"


def parse_log(elf, arch, logfile):
    memories: List[List[RawMemory]] = []
    registers: List[Registers] = []
    if not logfile:
        return Registers(elf, arch), memories

    def is_register_dump(line):
        return len(line.split("up_dump_register: ")) == 2

    def parse_register(regs, line):
        line = line.split("up_dump_register: ")
        if not (find_res := re.findall(r"([\w_]+): *([0-9xa-fXA-F]+)", line[1])):
            return False

        for name, value in find_res:
            name = name.lower()
            value = int(value, 16)
            try:
                regs.set(value, name=name)
            except KeyError:
                logging.warning(f"Ignore register {name}:{value}")

        return True

    def parse_stack(line):
        result = re.match(
            r".*stack_dump: (?P<ADDR>[0-9a-fxA-FX]+): (?P<VALS>( ?\w+)+)", line
        )
        if result is None:
            return None

        results = result.groupdict()

        addr = int(results["ADDR"], 16)
        data = b""
        for val in results["VALS"].split():
            # For little endian, the hex bytes should be reversed
            data += bytes.fromhex(val)[::-1]
        return RawMemory(addr, data)

    registerdump = []
    parsing_regs = False
    lines = []
    with open(logfile, "r") as f:
        for line in f:
            line = line.strip()
            if is_register_dump(line):
                if not parsing_regs:
                    parsing_regs = True
                    if lines:  # Started
                        registers.append(tmpregisters)  # noqa
                        memories.append(tempmemories)  # noqa
                        registerdump.append(lines)
                    lines = []
                    tempmemories = []
                    tmpregisters = Registers(elf, arch)

                lines.append(line)
                parse_register(tmpregisters, line)
            elif memory := parse_stack(line):
                parsing_regs = False
                tempmemories.append(memory)

        if lines:
            registers.append(tmpregisters)
            memories.append(tempmemories)
            registerdump.append(lines)

    if len(memories) > 1:
        for i, line in enumerate(registerdump):
            print(f"[{i}]: {'    '.join(line[:2])}")

        choice = input("Multiple dump found, please choose one:\n").strip() or 0
    else:
        choice = 0

    return registers[int(choice)], memories[int(choice)]


def gdbstub_start(args):
    memories = []
    registers = None

    for rawfile in args.rawfile or []:
        filename, address = rawfile.split(":")
        address = int(address, 16)
        with open(filename, "rb") as f:
            memories.append(RawMemory(address, f.read()))
            print(f"Add memory dump: {memories[-1]}")

    elf = utils.parse_elf(args.elffile)
    registers, mem = parse_log(elf, args.arch, args.log)
    memories.extend(mem)

    core = utils.parse_elf(args.core) if args.core else None

    memremap = []
    if args.remap:
        for remap in args.remap:
            fromaddr, toaddr, length = map(lambda x: int(x, 16), remap.split(","))
            memremap.append((fromaddr, toaddr, length))

    if args.proxy is not None:
        print(f"Try proxying localhost:{args.proxy}...")
        target = TargetProxy(elf, args.arch, args.proxy)
    else:
        target = Target(elf, args.arch, registers, memories, memremap, core)

    stub = GDBStub(target=target, port=args.port, proxymode=args.proxy is not None)

    print(f"Start GDB server on port {args.port}...")
    stub.run()
    print("GDB server exited.")


def gdb_start(args):
    def gdb_run(cmd):
        try:
            subprocess.run(cmd, shell=True)
        except KeyboardInterrupt:
            pass

    gdb_init_cmd = args.init_cmd or DEFAULT_GDB_INIT_CMD
    gdb_exec = args.gdb or "gdb-multiarch"
    gdb_cmd = (
        f"{gdb_exec} {args.elffile} -ex 'target remote localhost:{args.port}' "
        f"{gdb_init_cmd}"
    )
    print(f"Start GDB session: {gdb_cmd}")
    return multiprocessing.Process(target=gdb_run, args=(gdb_cmd,)).start()


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        prog="nxstub",
        description=f"nxstub v{__version__} - NuttX GDB server based on crash log, core dump or memory dump.",
    )
    parser.add_argument(
        "-a",
        "--arch",
        type=str,
        required=True,
        choices=g_reg_table.keys(),
        help="The architecture of the target.",
    )
    parser.add_argument(
        "-e",
        "--elffile",
        type=str,
        required=True,
        help="The elf file.",
    )
    parser.add_argument(
        "-p",
        "--port",
        type=int,
        default=1234,
        help="The GDB server port.",
    )
    parser.add_argument(
        "--proxy",
        type=int,
        default=None,
        help="The original GDB server port for proxy.",
    )
    parser.add_argument(
        "-r",
        "--rawfile",
        type=str,
        nargs="*",
        help="The memory dump file, in format of 'memdump1.bin:address1 memdump2.bin:address2'.",
    )
    parser.add_argument(
        "-c",
        "--core",
        type=str,
        help="The core dump file.",
    )
    parser.add_argument(
        "--remap",
        type=str,
        nargs="*",
        help="Remap the memory to another address, argument in format of 'from,to,length'.",
    )
    parser.add_argument(
        "-l",
        "--log",
        type=str,
        help="The crash dump log file.",
    )
    parser.add_argument(
        "-d",
        "--debug",
        action="store_true",
        help="Show debug messages.",
    )
    parser.add_argument(
        "-g",
        "--gdb",
        help="Optional path to GDB executable, once specified, will automatically start GDB session.",
        type=str,
    )
    parser.add_argument(
        "-i",
        "--init-cmd",
        type=str,
        help=f'Optional custom GDB init command when GDB launches, default: "{DEFAULT_GDB_INIT_CMD}".',
    )

    return parser.parse_args(args)


def main(args=None):
    args = parse_args(args)

    gdb = None
    if args.debug:
        logging.basicConfig(
            format="%(asctime)s:%(levelname)s:%(name)s:%(message)s",
        )
        logger = logging.getLogger(__name__)
        logger.setLevel(logging.DEBUG if args.debug else logging.INFO)

    if args.gdb:
        gdb = gdb_start(args)
        # Ignore the Ctrl+C signal
        signal.signal(signal.SIGINT, signal.SIG_IGN)

    try:
        gdbstub_start(args)
    except Exception as e:
        print(f"GDBStub error: {e}:\n {traceback.format_exc()}")

    if gdb:
        gdb.kill()
        gdb.join()
