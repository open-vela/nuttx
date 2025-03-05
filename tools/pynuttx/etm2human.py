#!/usr/bin/env python3
############################################################################
# tools/pynuttx/etm2human.py
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
import sys

from nxtrace.etmv4_decoder import (
    ATOM_TYPE_E,
    ETBStream,
    ETMv4Decoder,
    ETMv4Packet,
    ETMv4PacketType,
    decode_etb,
)

log = logging.getLogger()


class ETM2Human:

    def emit(self, msg):
        print(msg)
        sys.stdout.flush()

    def trace_info(self, pkt: ETMv4Packet):
        cc_enabled = pkt["cycle_count_enabled"]
        cond_inst_enabled = pkt["conditional_non_branch_instruction_enabled"]
        load_inst_enabled = pkt["load_instruction_enabled"]
        store_inst_enabled = pkt["store_instruction_enabled"]

        self.emit(
            f"TraceInfo - Cycle count {'enabled' if cc_enabled else 'disabled'},\n"
            f"            Tracing of conditional non-branch instruction {'enabled' if cond_inst_enabled else 'disabled'},\n"
            f"            {'NO' if not load_inst_enabled else ''} Explicit tracing of load instructions,\n"
            f"            {'NO' if not store_inst_enabled else ''} Explicit tracing of store instructions,\n"
            f"            p0_key = 0x{pkt['p0_key']:x},\n"
            f"            curr_spec_depth = {pkt['curr_spec_depth']},\n"
            f"            cc_threshold = 0x{pkt['cc_threshold']:x}"
        )

    def trace_on(self, pkt: ETMv4Packet):
        self.emit("TraceOn - A discontinuity in the trace stream")

    def trace_off(self, pkt: ETMv4Packet):
        self.emit("TraceOff - A discontinuity in the trace stream")

    def trace_discard(self, pkt: ETMv4Packet):
        self.emit("Discard")

    def trace_overflow(self, pkt: ETMv4Packet):
        self.emit("Overflow")

    def trace_ts(self, pkt: ETMv4Packet):
        timestamp = hasattr(pkt, "timestamp") and pkt["timestamp"] or 0
        self.emit(f"Timestamp - {timestamp}")
        if hasattr(pkt, "cycle"):
            self.emit(
                f"            (number of cycles between the most recent Cycle Count element {pkt['cycle']})"
            )

    def trace_exception(self, pkt: ETMv4Packet):
        self.emit(
            f"Exception - exception type {pkt['exception_type_name']}, address 0x{pkt['address']:016x}"
        )

    def trace_exception_return(self, pkt: ETMv4Packet):
        self.emit("Exception return")

    def trace_cc(self, pkt: ETMv4Packet):
        if hasattr(pkt, "unknown"):
            self.emit("Cycle count - unknown")
        else:
            self.emit(f"Cycle count - {pkt['cycle']}")

    def trace_commit(self, pkt: ETMv4Packet):
        self.emit(f"Commit - {pkt['commit']}")

    def trace_cancel(self, pkt: ETMv4Packet):
        self.emit(f"Cancel - {pkt['cancel']}")

    def trace_mispredict(self, pkt: ETMv4Packet):
        self.emit("Mispredict")

    def trace_cond_inst(self, pkt: ETMv4Packet):
        log.warning("Conditional instruction is not implemented yet")

    def trace_cond_flush(self, pkt: ETMv4Packet):
        self.emit("Conditional flush")

    def trace_cond_result(self, pkt: ETMv4Packet):
        log.warning("Conditional result is not implemented yet")

    def trace_context(self, pkt: ETMv4Packet):
        setattr(self, "sixty_four_bit", pkt["sixty_four_bit"])
        setattr(self, "ex_level", pkt["ex_level"])
        setattr(self, "security", pkt["security"])

        self.emit(f"Context - Context ID = 0x{pkt['context_id']:x},")
        self.emit(f"          VMID = 0x{pkt['vmid']:x},")
        self.emit(f"          Exception level = EL{pkt['ex_level']},")
        self.emit(f"          Security = {'S' if pkt['security'] else 'NS'},")
        self.emit(f"          {64 if pkt['sixty_four_bit'] else 32}-bit instruction")

    def trace_address(self, pkt: ETMv4Packet):
        if self.sixty_four_bit:
            self.emit(
                f"Address - Instruction address 0x{pkt['address']:016x}, Instruction set Aarch64"
            )
        else:
            if pkt["instruction_set"]:
                self.emit(
                    f"Address - Instruction address 0x{pkt['address']:016x}, Instruction set Aarch32 (Thumb)"
                )
            else:
                self.emit(
                    f"Address - Instruction address 0x{pkt['address']:016x}, Instruction set Aarch32 (ARM)"
                )

    def trace_atom(self, pkt: ETMv4Packet):
        self.emit(f"ATOM - {'E' if pkt['atom_type'] == ATOM_TYPE_E else 'N'}")

    def trace_q(self, pkt: ETMv4Packet):
        if pkt["cycle"]:
            self.emit(f"Q - {pkt['cycle']} of instructions")
        else:
            self.emit("Q - UNKNOWN of instructions")

    etm2human_decoder = {
        ETMv4PacketType.TRACE_INFO: trace_info,
        ETMv4PacketType.TRACE_ON: trace_on,
        ETMv4PacketType.TRACE_OFF: trace_off,
        ETMv4PacketType.DISCARD: trace_discard,
        ETMv4PacketType.OVERFLOW: trace_overflow,
        ETMv4PacketType.TIMESTAMP: trace_ts,
        ETMv4PacketType.EXCEPTION: trace_exception,
        ETMv4PacketType.EXCEPTION_RETURN: trace_exception_return,
        ETMv4PacketType.CYCLE_COUNT: trace_cc,
        ETMv4PacketType.COMMIT: trace_commit,
        ETMv4PacketType.CANCEL: trace_cancel,
        ETMv4PacketType.MISPREDICT: trace_mispredict,
        ETMv4PacketType.COND_INST: trace_cond_inst,
        ETMv4PacketType.COND_FLUSH: trace_cond_flush,
        ETMv4PacketType.COND_RESULT: trace_cond_result,
        ETMv4PacketType.CONTEXT: trace_context,
        ETMv4PacketType.ADDRESS: trace_address,
        ETMv4PacketType.ATOM: trace_atom,
        ETMv4PacketType.Q: trace_q,
    }

    def decode_trace(self, pkt: ETMv4Packet):
        self.etm2human_decoder[pkt.type](self, pkt)


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "-i", "--input", required=True, type=str, help="Give the trace file"
    )
    parser.add_argument(
        "-u",
        "--unaligned",
        action="store_true",
        help="Trace is unaligned and needs to be aligned by frame synchronization packet",
    )
    parser.add_argument(
        "-d",
        "--debug",
        type=str,
        choices=["debug", "info", "warning", "error", "critical"],
        default="warning",
        help="Set the debug level",
    )

    return parser.parse_args()


def main():
    args = parse_args()
    logger = logging.getLogger()
    logger.setLevel(args.debug.upper())

    etm2human = ETM2Human()
    etm_decoder = ETMv4Decoder()
    stream = ETBStream(etm_decoder, args.input, etm2human.decode_trace)
    decode_etb(stream, args.unaligned)


if __name__ == "__main__":
    main()
