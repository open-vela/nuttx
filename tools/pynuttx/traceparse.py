#!/usr/bin/env python3
############################################################################
# tools/pynuttx/traceparse.py
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
import signal
import sys

from nxelf.elf import ELFParser
from nxtrace.rtt import SeggerRTT
from nxtrace.trace import NoteParser


def note_binary_parser(elf_parser, note_parser, binary_file):
    note_parser.parse_file(binary_file)
    note_parser.dump()


def rtt_parser(elf_parser, note_parser, device, interface, speed, channel):
    # Get the address of the _SEGGER_RTT symbol
    address = elf_parser.symbol_addr("_SEGGER_RTT")
    running = True
    if address is None:
        raise RuntimeError(
            "Symbol _SEGGER_RTT not found, please check if the ELF is correct or if SEGGER RTT is enabled"
        )

    rtt = SeggerRTT(
        device,
        interface,
        speed=speed,
        channel=channel,
        address=address,
    )

    def signal_handler(signum, frame):
        nonlocal running
        if running:
            print("\nReceived signal\n")
            running = False
            rtt.stop()
            note_parser.flush()
            sys.exit(0)
        else:
            print("Already received signal, exiting...")
            sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rtt.start()
    print("Starting RTT...")
    while running:
        data = rtt.read()
        if not data:
            continue

        notes = note_parser.parse(data)
        note_parser.dump(notes)


def arg_parse():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--elf", help="ELF file", required=True)
    parser.add_argument("-o", "--output", help="Output file", default="trace.perfetto")
    parser.add_argument("-b", "--binary", help="Binary note file")
    parser.add_argument("-d", "--device", help="RTT device")
    parser.add_argument("-i", "--interface", help="RTT interface")
    parser.add_argument("-s", "--speed", type=int, help="RTT speed")
    parser.add_argument("-c", "--channel", type=int, help="RTT channel")
    args = parser.parse_args()

    # Validate the arguments
    if args.binary:
        return vars(args)
    elif args.device and args.interface:
        return vars(args)
    elif args.device or args.interface:
        print("Error: Both --device and --interface must be specified together.")
        sys.exit(1)
    else:
        print(
            "Error: You must specify either a binary file or both RTT parameters (--device and --interface)."
        )
        sys.exit(1)


if __name__ == "__main__":
    args = arg_parse()

    elf_parser = ELFParser(args["elf"])
    note_parser = NoteParser(elf_parser, output=args["output"])

    if args.get("binary"):
        note_binary_parser(elf_parser, note_parser, args["binary"])
    elif args.get("device"):
        rtt_parser(
            elf_parser,
            note_parser,
            args["device"],
            args["interface"],
            args.get("speed"),
            args.get("channel"),
        )
