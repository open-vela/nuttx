#!/usr/bin/env python3
############################################################################
# tools/pynuttx/etm2trace.py
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
import logging

from nxelf.elf import AngrElf
from nxtrace.etmv4_decoder import (
    ATOM_TYPE_E,
    ATOM_TYPE_N,
    ETBStream,
    ETMv4Decoder,
    ETMv4Packet,
    ETMv4PacketType,
    decode_etb,
)
from nxtrace.perfetto_trace import PerfettoTrace

log = logging.getLogger(__name__)


class ETMTimestamp:
    def __init__(self, freq: int = 1000000000):
        self.freq = freq
        self.ts = 0

    def cycle2ts(self, cycle: int):
        return int(cycle * 1000000000 / self.freq)

    def update(self, cycle: int = None, ts: int = None):
        if cycle:
            self.ts += self.cycle2ts(cycle)
        elif ts:
            self.ts = ts
        else:
            log.debug("Cycle or ts not provided! timestamp will not be updated")
            return

        log.debug(f"timestamp - {self.ts} ns")

    def compensate(self, bb):
        cycle = len(bb.block.capstone.insns) if bb.block else 0
        self.ts += self.cycle2ts(cycle)
        log.debug(f"Compensate - {self.ts} ns")

    def __str__(self):
        return str(self.ts)


class BasicBlock:
    def __init__(self, elf, addr: int, timestamp: ETMTimestamp):
        try:
            self.elf = elf
            self.addr = addr
            self.timestamp = timestamp
            self.symbol = elf.addr2sym(self.addr)
            self.func = elf.addr2func(self.addr)
            self.block = self.func.get_block(self.addr)
        except Exception as e:
            log.error(f"BasicBlock error: {self.addr:#08x} {e}")
            self.block = None

    def is_endpoint(self):
        for retblock in self.func.endpoints:
            if self.addr >= retblock.addr and self.addr < retblock.addr + retblock.size:
                return True
        return False

    def is_entrypoint(self):
        return self.func.startpoint.addr == self.addr

    def todict(self):
        return {
            "addr": hex(self.addr),
            "timestamp": self.timestamp.ts if self.timestamp else 0,
            "symbol": self.symbol,
            "block_addr": hex(self.block.addr) if self.block else 0,
            "block_size": self.block.size if self.block else 0,
        }


class TimeLine:
    last_symbol = None
    symbol_uuid = dict()

    def __init__(self, ptrace):
        self.ptrace = ptrace

    def uuid(self, symbol):
        uuid = self.symbol_uuid.get(symbol, 0)
        if not uuid:
            uuid = self.ptrace.next_uuid()
            self.ptrace.add_thread(
                uuid=uuid, parent_uuid=0, tid=uuid, pid=1, name=symbol
            )
            self.symbol_uuid[symbol] = uuid

        return uuid

    def trace_event(self, event, bb: BasicBlock):
        self.ptrace.trace_event(
            uuid=self.uuid(bb.symbol),
            ts=bb.timestamp.ts,
            type=event,
            name=bb.symbol,
            args=bb.todict(),
        )

    def parse(self, now_bb: BasicBlock, last_bb: BasicBlock = None):
        if last_bb and last_bb.symbol != now_bb.symbol:
            self.trace_event(self.ptrace.SLICE_END, last_bb)
            self.trace_event(self.ptrace.SLICE_BEGIN, now_bb)


class CallStack:
    executing_func = dict()

    def __init__(self, ptrace):
        self.ptrace = ptrace
        self.tid = ptrace.next_uuid()
        self.ptrace.add_thread(
            uuid=self.tid, parent_uuid=0, tid=self.tid, pid=0, name="MainThread"
        )

    def trace_event(self, event, bb: BasicBlock):
        self.ptrace.trace_event(
            uuid=self.tid,
            ts=bb.timestamp.ts,
            type=event,
            name=bb.symbol,
            args=bb.todict(),
        )

    def parse(self, now_bb: BasicBlock, last_bb: BasicBlock = None):
        # If the last executed basic block is a function return point, an end event is generated
        if last_bb and last_bb.is_endpoint():
            self.trace_event(self.ptrace.SLICE_END, last_bb)
            self.executing_func.pop(last_bb.symbol, None)

        if now_bb.is_entrypoint() or not self.executing_func.get(now_bb.symbol):
            # If the current basic block is a function entry point, a begin event is generated
            # Or if the basic block is not a function entry, but it is not in
            # the list of executed functions, it means that we have lost the function entry event.
            # We immediately add one here.
            self.trace_event(self.ptrace.SLICE_BEGIN, now_bb)
            self.executing_func[now_bb.symbol] = [now_bb]


class ETM2Trace:
    index = 0
    thumb_mode = False
    bb = None

    def __init__(self, elf, parser: list = [], freq: int = 1000000000):
        self.elf = elf
        self.timestamp = ETMTimestamp(freq)
        self.parser = parser

    def trace_cycle(self, pkt: ETMv4Packet):
        self.timestamp.update(cycle=getattr(pkt, "cycle", None))

    def trace_timestamp(self, pkt: ETMv4Packet):
        self.timestamp.update(
            ts=getattr(pkt, "timestamp", None), cycle=getattr(pkt, "cycle", None)
        )

    def register_parser(self, parser):
        self.parser.append(parser)

    def execute_address(self, addr: int):
        bb = BasicBlock(self.elf, addr, timestamp=self.timestamp)
        log.debug(f"--> ts: {self.timestamp.ts} address - {bb.symbol}({addr:#08x})")

        # If the timestamp of the current basic block is the same as the timestamp of the last basic block,
        # it means that the timestamp of the current basic block is not accurate, so we need to compensate it.
        if self.bb and bb.timestamp.ts == self.bb.timestamp.ts:
            self.timestamp.compensate(bb)

        for parser in self.parser:
            parser.parse(bb, self.bb)

        self.bb = bb

    def trace_address(self, pkt: ETMv4Packet):
        addr = pkt["address"]
        self.thumb_mode = pkt["instruction_set"]
        self.execute_address(addr | (0x1 if self.thumb_mode else 0))

    def trace_atom(self, pkt: ETMv4Packet):
        if self.bb:
            try:
                if pkt["atom_type"] == ATOM_TYPE_E:
                    next_addr = self.elf.get_block_next_address(self.bb.block)
                elif pkt["atom_type"] == ATOM_TYPE_N:
                    next_addr = self.bb.addr + self.bb.block.size

                if next_addr:
                    self.execute_address(next_addr | (0x1 if self.thumb_mode else 0))

            except Exception as e:
                log.error(f"trace_atom parse failed: {self.bb.addr:#08x} {e}")

    etm2trace_decoder = {
        ETMv4PacketType.TIMESTAMP: trace_timestamp,
        ETMv4PacketType.ADDRESS: trace_address,
        ETMv4PacketType.CYCLE_COUNT: trace_cycle,
        ETMv4PacketType.ATOM: trace_atom,
    }

    def decode_trace(self, pkt: ETMv4Packet):
        decoder = self.etm2trace_decoder.get(pkt.type)
        if decoder:
            decoder(self, pkt)


def parse_arguments():
    parser = argparse.ArgumentParser(description="ETM2Trace Command Line Arguments")
    parser.add_argument("-e", "--elf", type=str, required=True, help="ELF file path")
    parser.add_argument(
        "-i", "--input", type=str, required=True, help="Input file path"
    )
    parser.add_argument(
        "-o", "--output", type=str, default="perfetto_trace.pb", help="Output file path"
    )
    parser.add_argument(
        "-d",
        "--debug",
        type=str,
        choices=["debug", "info", "warning", "error", "critical"],
        default="warning",
        help="Set the debug level",
    )
    parser.add_argument(
        "-m",
        "--mode",
        type=str,
        nargs="+",
        choices=["timeline", "callstack"],
        help="Mode of operation",
    )
    parser.add_argument(
        "-f", "--freq", type=int, default=1000000000, help="Frequency of the trace"
    )

    return parser.parse_args()


def main():
    args = parse_arguments()

    logger = logging.getLogger()
    logger.setLevel(args.debug.upper())
    logger = logging.getLogger("angr")
    logger.setLevel(logging.WARNING)

    ptrace = PerfettoTrace(args.output)

    elf = AngrElf(args.elf)

    etm2trace = ETM2Trace(elf, freq=args.freq)
    if "timeline" in args.mode:
        etm2trace.register_parser(TimeLine(ptrace))
    if "callstack" in args.mode:
        etm2trace.register_parser(CallStack(ptrace))

    etm_decoder = ETMv4Decoder()
    stream = ETBStream(etm_decoder, args.input, etm2trace.decode_trace)
    decode_etb(stream, False)

    ptrace.flush()


if __name__ == "__main__":
    main()
