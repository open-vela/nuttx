############################################################################
# tools/pynuttx/nxtrace/etmv4_decoder.py
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

import logging
from dataclasses import dataclass
from enum import Enum, auto

log = logging.getLogger(__name__)

ADDR_REG_IS_UNKNOWN = -1
ADDR_REG_IS0 = 0
ADDR_REG_IS1 = 1

ATOM_TYPE_E = 1
ATOM_TYPE_N = 2


class StreamState(Enum):
    READING = 0
    SYNCING = 1
    INSYNC = 2
    OUTSYNC = 3
    DECODING = 4


class ETMv4PacketType(Enum):
    TRACE_INFO = auto()
    TRACE_ON = auto()
    TRACE_OFF = auto()
    DISCARD = auto()
    OVERFLOW = auto()
    ATOM = auto()
    Q = auto()
    EXCEPTION = auto()
    EXCEPTION_RETURN = auto()
    ADDRESS = auto()
    CONTEXT = auto()
    BRANCH_FLUSH = auto()
    FUNCTION_RETURN = auto()
    TIMESTAMP = auto()
    TIMESTAMP_MARK = auto()
    CYCLE_COUNT = auto()
    EVENT = auto()
    COMMIT = auto()
    CANCEL = auto()
    MISPREDICT = auto()
    COND_INST = auto()
    COND_RESULT = auto()
    COND_FLUSH = auto()
    DATA_SYNC_MARKER = auto()


@dataclass
class ETMv4AddressRegister:
    address: int
    instruction_set: int


class ETMv4Tracer:
    def __init__(self):
        self.info = 0
        self.condtype = 0
        self.commopt = 0
        self.timestamp = 0
        self.address_register = [ETMv4AddressRegister(0, 1) for _ in range(3)]
        self.context_id = 0
        self.vmid = 0
        self.ex_level = 0
        self.security = 0
        self.sixty_four_bit = 0
        self.curr_spec_depth = 0
        self.p0_key = 0
        self.cond_c_key = 0
        self.cond_r_key = 0
        self.p0_key_max = 0
        self.cond_key_max_incr = 0
        self.max_spec_depth = 0
        self.cc_threshold = 0

    def set_address_register(self, n, address):
        self.address_register[n].address = address

    def set_address_register_IS(self, n, instruction_set):
        self.address_register[n].instruction_set = instruction_set

    def reset_address_register(self):
        self.address_register[0].address = 0
        self.address_register[0].instruction_set = 1
        self.address_register[1].address = 0
        self.address_register[1].instruction_set = 1
        self.address_register[2].address = 0
        self.address_register[2].instruction_set = 1


class ETMv4Packet:
    def __init__(self, pkt_type, raw_data: bytes = b""):
        self.type = pkt_type
        self.rawdata = raw_data

    def __getitem__(self, key):
        if hasattr(self, key):
            return getattr(self, key)
        else:
            raise KeyError(f"Attribute '{key}' not found")


class ETBStream:
    def __init__(self, etm_decoder, input_file=None, trace_decode=None):
        self.state = StreamState.READING
        self.tracer = ETMv4Tracer()
        self.decoder = etm_decoder
        self.buff = bytearray()
        self.tracer.decoder = trace_decode
        if input_file:
            with open(input_file, "rb") as f:
                self.buff = memoryview(f.read())

    def copy(self):
        new_stream = ETBStream(self.decoder)
        new_stream.state = self.state
        new_stream.tracer = self.tracer
        new_stream.decoder = self.decoder
        return new_stream


class TracePacketMeta(type):
    def __new__(cls, name, bases, dct):
        # Automatically create decode function if not provided
        if "decode" not in dct:
            dct["decode"] = lambda self, data, stream: len(data)
        return super().__new__(cls, name, bases, dct)


class TracePacket(metaclass=TracePacketMeta):
    def __init__(self, mask, val, decode_func):
        self.mask = mask
        self.val = val
        self.decode_func = decode_func
        self.name = decode_func.__name__[7:]

    def decode(self, data, stream):
        return self.decode_func(data, stream)


def tracer_trace_info(
    tracer: ETMv4Tracer, plctl: int, info: int, key: int, spec: int, cyct: int
):
    tracer.reset_address_register()

    tracer.info = info
    tracer.p0_key = key
    tracer.curr_spec_depth = spec
    tracer.cc_threshold = cyct

    log.info(
        f"TraceInfo - Cycle count {'enabled' if tracer.info & 1 else 'disabled'},\n"
        f"            Tracing of conditional non-branch instruction {'enabled' if tracer.info & 0x0E else 'disabled'},\n"
        f"            {'NO' if not tracer.info & 0x10 else ''} Explicit tracing of load instructions,\n"
        f"            {'NO' if not tracer.info & 0x20 else ''} Explicit tracing of store instructions,\n"
        f"            p0_key = 0x{key:x},\n"
        f"            curr_spec_depth = {spec},\n"
        f"            cc_threshold = 0x{cyct:x}"
    )

    packet = ETMv4Packet(ETMv4PacketType.TRACE_INFO)
    setattr(packet, "cycle_count_enabled", tracer.info & 1)
    setattr(packet, "conditional_non_branch_instruction_enabled", tracer.info & 0x0E)
    setattr(packet, "load_instruction_enabled", tracer.info & 0x10)
    setattr(packet, "store_instruction_enabled", tracer.info & 0x20)
    setattr(packet, "p0_key", tracer.p0_key)
    setattr(packet, "curr_spec_depth", tracer.curr_spec_depth)
    setattr(packet, "cc_threshold", tracer.cc_threshold)
    tracer.decoder(packet)
    return tracer


def tracer_trace_on(tracer: ETMv4Tracer):
    packet = ETMv4Packet(ETMv4PacketType.TRACE_ON)
    tracer.decoder(packet)
    log.info("TraceOn - A discontinuity in the trace stream")


def tracer_trace_off(tracer: ETMv4Tracer):
    packet = ETMv4Packet(ETMv4PacketType.TRACE_OFF)
    tracer.decoder(packet)
    log.info("[TraceOff - A discontinuity in the trace stream]")
    return tracer


def tracer_discard(tracer: ETMv4Tracer):
    packet = ETMv4Packet(ETMv4PacketType.DISCARD)
    tracer.decoder(packet)
    log.info("Discard")
    tracer_cond_flush(tracer)
    tracer.curr_spec_depth = 0


def tracer_overflow(tracer: ETMv4Tracer):
    packet = ETMv4Packet(ETMv4PacketType.OVERFLOW)
    tracer.decoder(packet)
    log.info("Discard")
    tracer_cond_flush(tracer)
    tracer.curr_spec_depth = 0


def tracer_ts(
    tracer: ETMv4Tracer, timestamp: int, have_cc: int, cycle: int, nr_replace: int
):
    packet = ETMv4Packet(ETMv4PacketType.TIMESTAMP)
    if timestamp:
        tracer.timestamp &= ~((1 << nr_replace) - 1)
        tracer.timestamp |= timestamp
        setattr(packet, "timestamp", tracer.timestamp)

    log.info(f"Timestamp - {tracer.timestamp}")
    if have_cc:
        setattr(packet, "cycle", cycle)
        log.info(
            f"            (number of cycles between the most recent Cycle Count element {cycle})"
        )

    tracer.decoder(packet)


def tracer_exception(tracer: ETMv4Tracer, pkt_type: int):
    exp_name = [
        "PE reset",
        "Debug halt",
        "Call",
        "Trap",
        "System error",
        None,
        "Inst debug",
        "Data debug",
        None,
        None,
        "Alignment",
        "Inst fault",
        "Data fault",
        None,
        "IRQ",
        "FIQ",
    ]

    exp_name = (
        exp_name[pkt_type]
        if pkt_type < len(exp_name) and exp_name[pkt_type]
        else "Reserved"
    )
    log.info(
        f"Exception - exception type {exp_name}, address 0x{tracer.address_register[0].address:016x}"
    )
    packet = ETMv4Packet(ETMv4PacketType.EXCEPTION)
    setattr(packet, "exception_type", pkt_type)
    setattr(
        packet,
        "exception_type_name",
        (
            exp_name[pkt_type]
            if (pkt_type < len(exp_name) and exp_name[pkt_type])
            else "Reserved"
        ),
    )
    setattr(packet, "address", tracer.address_register[0].address)
    tracer.decoder(packet)
    tracer_cond_flush(tracer)

    if tracer.p0_key_max:
        tracer.p0_key += 1
        tracer.p0_key %= tracer.p0_key_max

    tracer.curr_spec_depth += 1
    if not tracer.max_spec_depth or (tracer.curr_spec_depth > tracer.max_spec_depth):
        tracer_commit(tracer, 1)


def tracer_exception_return(tracer: ETMv4Tracer):
    packet = ETMv4Packet(ETMv4PacketType.EXCEPTION_RETURN)
    tracer.decoder(packet)
    log.info("Exception return")
    # FIXME: for ARMv6-M and ARMv7-M PEs, exception_return is a P0 element


def tracer_cc(tracer: ETMv4Tracer, unknown: int, count: int):
    packet = ETMv4Packet(ETMv4PacketType.CYCLE_COUNT)
    if unknown:
        log.info("Cycle count - unknown")
        setattr(packet, "unknown", 1)
    else:
        cc = count + tracer.cc_threshold
        log.info(f"Cycle count - {cc}")
        setattr(packet, "cycle", cc)
    tracer.decoder(packet)


def tracer_commit(tracer: ETMv4Tracer, commit: int):
    log.info(f"Commit - {commit}")
    tracer.curr_spec_depth -= commit
    packet = ETMv4Packet(ETMv4PacketType.COMMIT)
    setattr(packet, "commit", commit)
    tracer.decoder(packet)


def tracer_cancel(tracer: ETMv4Tracer, mispredict: int, cancel: int):
    log.info(f"Cancel - {cancel}")
    tracer.curr_spec_depth -= cancel

    if tracer.p0_key_max:
        tracer.p0_key -= cancel
        tracer.p0_key %= tracer.p0_key_max

    if mispredict:
        tracer_mispredict(tracer, 0)
        tracer_cond_flush(tracer)


def tracer_mispredict(tracer: ETMv4Tracer, param: int):
    if param == 0:
        pass
    elif param == 1:
        tracer_atom(tracer, ATOM_TYPE_E)
    elif param == 2:
        tracer_atom(tracer, ATOM_TYPE_E)
        tracer_atom(tracer, ATOM_TYPE_E)
    elif param == 3:
        tracer_atom(tracer, ATOM_TYPE_N)
    else:
        log.info(f"Invalid param ({param})")

    packet = ETMv4Packet(ETMv4PacketType.MISPREDICT)
    setattr(packet, "param", param)
    tracer.decoder(packet)
    log.info("Mispredict")


def tracer_cond_inst(tracer: ETMv4Tracer, format: int, param1: int, param2: int):
    if tracer.cond_key_max_incr == 0:
        log.info(
            "cond_key_max_incr MUST NOT be zero for conditional instruction elements."
        )
        return

    if format == 1:
        key = param1
        if __is_cond_key_special(tracer, key):
            tracer.cond_c_key += 1
            tracer.cond_c_key %= tracer.cond_key_max_incr
        else:
            tracer.cond_c_key = key
        log.info(f"Conditional instruction - C key = {key}")

    elif format == 2:
        ci = param1
        if ci == 0:
            tracer.cond_c_key += 1
            tracer.cond_c_key %= tracer.cond_key_max_incr
            log.info(f"Conditional instruction - C key = {tracer.cond_c_key}")
        elif ci == 1:
            log.info(f"Conditional instruction - C key = {tracer.cond_c_key}")
        elif ci == 2:
            tracer.cond_c_key += 1
            tracer.cond_c_key %= tracer.cond_key_max_incr
            log.info(f"Conditional instruction - C key = {tracer.cond_c_key}")
        else:
            log.info(f"Invalid CI ({ci})")

    elif format == 3:
        z = param1
        num = param2
        for _ in range(num):
            tracer.cond_c_key += 1
            tracer.cond_c_key %= tracer.cond_key_max_incr
            log.info(f"Conditional instruction - C key = {tracer.cond_c_key}")
        if z:
            log.info(f"Conditional instruction - C key = {tracer.cond_c_key}")

    else:
        log.info(f"Invalid format ({format})")


def tracer_cond_flush(tracer: ETMv4Tracer):
    log.info("Conditional flush")
    packet = ETMv4Packet(ETMv4PacketType.COND_FLUSH)
    tracer.decoder(packet)
    return tracer


def tracer_atom(tracer: ETMv4Tracer, atom_type):

    if atom_type == ATOM_TYPE_E:
        log.info("ATOM - E")
    elif atom_type == ATOM_TYPE_N:
        log.info("ATOM - N")
    else:
        log.info(f"Invalid ATOM type ({atom_type})")
        return

    packet = ETMv4Packet(ETMv4PacketType.ATOM)
    setattr(packet, "atom_type", atom_type)
    tracer.decoder(packet)
    if tracer.p0_key_max:
        tracer.p0_key += 1
        tracer.p0_key %= tracer.p0_key_max

    tracer.curr_spec_depth += 1
    if not tracer.max_spec_depth or (tracer.curr_spec_depth > tracer.max_spec_depth):
        tracer_commit(tracer, 1)


def __is_cond_key_special(tracer: ETMv4Tracer, key: int):
    return key >= tracer.cond_key_max_incr


def tracer_q(tracer: ETMv4Tracer, count: int):
    if count:
        log.info(f"Q - {count} of instructions")
    else:
        log.info("Q - UNKNOWN of instructions")

    packet = ETMv4Packet(ETMv4PacketType.Q)
    setattr(packet, "cycle", count)
    tracer.decoder(packet)

    if tracer.p0_key_max:
        tracer.p0_key += 1
        tracer.p0_key %= tracer.p0_key_max

    tracer.curr_spec_depth += 1
    if not tracer.max_spec_depth or (tracer.curr_spec_depth > tracer.max_spec_depth):
        tracer_commit(tracer, 1)


def __interpret_tokens(tracer: ETMv4Tracer, tokens: int, pos: int):
    if pos % 2:
        log.info(f"Invalid pos ({pos})")
        return pos + 1

    if tracer.condtype:
        token = (tokens >> pos) & 0x0F
        if (token & 0x03) == 0x03:
            return pos + 2
        else:
            if token == 0x0F:
                return 0
            else:
                return pos + 4
    else:
        token = (tokens >> pos) & 0x03
        if token == 3:
            # /* NULL, no R element indicated */
            return 0
        else:
            return pos + 2


def tracer_cond_result(
    tracer: ETMv4Tracer, format: int, param1: int, param2: int, param3: int
):
    cond_result_token_apsr = [
        "C flag set",
        "N flag set",
        "Z and C flags set",
        "N and C flags set",
        "unknown",
        "unknown",
        "unknown",
        "No flag set",
        "unknown",
        "unknown",
        "unknown",
        "Z flag set",
        "unknown",
        "unknown",
        "unknown",
        "unknown",
    ]

    cond_result_token_pass_fail = [
        "failed the condition code check",
        "passed the condition code check",
        "don't know the result of the condition code check",
    ]

    MAX_TOKENS_POS = 12

    if tracer.cond_key_max_incr == 0:
        log.info(
            "cond_key_max_incr MUST NOT be zero for conditional instruction elements."
        )
        return

    if format == 1:
        key = param1
        ci = param2
        result = param3
        if __is_cond_key_special(tracer, key):
            tracer.cond_r_key += 1
            tracer.cond_r_key %= tracer.cond_key_max_incr
        else:
            tracer.cond_r_key = key
        if ci:
            while tracer.cond_c_key == tracer.cond_r_key:
                tracer_cond_inst(tracer, 3, 0, 1)
        if tracer.condtype:
            log.info(
                f"Conditional result - R key = {key}, APSR_V = {result & 0x01},"
                f"APSR_C = {result & 0x02}, APSR_Z = {result & 0x04}, APSR_N = {result & 0x08}"
            )
        else:
            log.info(
                f"Conditional result - R key = {key}, {'passed' if result else 'failed'} the condition code check"
            )

    elif format == 2:
        k = param1 & 0x01
        token = param2 & 0x03
        tracer.cond_r_key += 1 + k
        tracer.cond_r_key %= tracer.cond_key_max_incr
        while tracer.cond_c_key == tracer.cond_r_key:
            tracer_cond_inst(tracer, 3, 0, 1)
        if tracer.condtype:
            log.info(
                f"Conditional result - R key = {tracer.cond_r_key}, APSR indication: {cond_result_token_apsr[token]}"
            )
        else:
            log.info(
                f"Conditional result - R key = {tracer.cond_r_key}, {cond_result_token_pass_fail[token]}"
            )

    elif format == 3:
        tokens = param1 & 0x0FFF
        pos = 0
        while True:
            next_pos = __interpret_tokens(tracer, tokens, pos)
            if next_pos:
                token = (tokens & ((1 << next_pos) - 1)) >> pos
                if tracer.condtype:
                    log.info(
                        f"Conditional result - R key = {tracer.cond_r_key}, APSR indication: {cond_result_token_apsr[token]}"
                    )
                else:
                    log.info(
                        f"Conditional result - R key = {tracer.cond_r_key}, {cond_result_token_pass_fail[token]}"
                    )
                pos = next_pos
            if next_pos == 0 or pos >= MAX_TOKENS_POS:
                break

    elif format == 4:
        token = param1 & 0x03
        tracer.cond_r_key -= 1
        tracer.cond_r_key %= tracer.cond_key_max_incr
        if tracer.condtype:
            log.info(
                f"Conditional result - R key = {tracer.cond_r_key}, APSR indication: {cond_result_token_apsr[token]}"
            )
        else:
            log.info(
                f"Conditional result - R key = {tracer.cond_r_key}, {cond_result_token_pass_fail[token]}"
            )

    else:
        log.info(f"Invalid format ({format})")


def tracer_context(
    tracer: ETMv4Tracer,
    p: int,
    el: int,
    sf: int,
    ns: int,
    v: int,
    vmid: int,
    c: int,
    contextid: int,
):
    if p:
        tracer.ex_level = el
        tracer.sixty_four_bit = sf
        tracer.security = not ns
        if v:
            tracer.vmid = vmid
        if c:
            tracer.context_id = contextid

    packet = ETMv4Packet(ETMv4PacketType.CONTEXT)
    setattr(packet, "context_id", contextid)
    setattr(packet, "vmid", vmid)
    setattr(packet, "ex_level", el)
    setattr(packet, "security", not ns)
    setattr(packet, "sixty_four_bit", sf)
    tracer.decoder(packet)

    log.info(f"Context - Context ID = 0x{tracer.context_id:x},")
    log.info(f"          VMID = 0x{tracer.vmid:x},")
    log.info(f"          Exception level = EL{tracer.ex_level},")
    log.info(f"          Security = {'S' if tracer.security else 'NS'},")
    log.info(f"          {64 if tracer.sixty_four_bit else 32}-bit instruction")


def tracer_address(tracer: ETMv4Tracer):
    address = tracer.address_register[0].address
    instruction_set = tracer.address_register[0].instruction_set

    if tracer.sixty_four_bit:
        log.info(
            f"Address - Instruction address 0x{address:016x}, Instruction set Aarch64"
        )
    else:
        if instruction_set:
            log.info(
                f"Address - Instruction address 0x{address:016x}, Instruction set Aarch32 (ARM)"
            )
        else:
            log.info(
                f"Address - Instruction address 0x{address:016x}, Instruction set Aarch32 (ç)"
            )

    packet = ETMv4Packet(ETMv4PacketType.ADDRESS)
    setattr(packet, "address", address)
    setattr(packet, "instruction_set", instruction_set)
    tracer.decoder(packet)


# Define all trace packets using the metaclass
def decode_extension(data, stream):
    index = 1
    cnt = 0

    if len(data) <= 1:
        log.error("Not enough data to decode extension packet")
        return -1

    if data[index] == 0:
        if 11 > len(data):
            return -1

        # async
        while cnt < 11 and index < len(stream.buff):
            if cnt == 10 and data[index] != 0x80:
                break
            if cnt != 10 and data[index] != 0:
                break
            cnt += 1
            index += 1

        if cnt != 11:
            log.error("Payload bytes of async are not correct")
            log.error("Invalid async packet")
            return -1

        log.debug("[async]")

    elif data[index] == 3:
        # discard
        index += 1
        log.debug("[discard]")
        # Assuming tracer_discard is a method of the tracer object
        tracer_discard(stream.tracer)

    elif data[index] == 5:
        # overflow
        index += 1
        log.debug("[overflow]")
        # Assuming tracer_overflow is a method of the tracer object
        tracer_overflow(stream.tracer)

    else:
        log.error("First payload byte of async is not correct")
        log.error("Invalid async packet")
        return -1

    return index


def decode_trace_info(data, stream: ETBStream):
    index = 1
    plctl = 0
    info = 0
    key = 0
    spec = 0
    cyct = 0

    # Decode PLCTL
    for i in range(4):
        if index >= len(data):
            break
        byte = data[index]
        plctl |= (byte & ~0x80) << (7 * i)
        index += 1
        if not (byte & 0x80):
            break

    if i >= 1:
        log.error("More than 1 PLCTL field in the trace info packet")
        return -1

    # Decode INFO
    if plctl & 1:
        for i in range(4):
            byte = data[index]
            info |= (byte & ~0x80) << (7 * i)
            index += 1
            if not (byte & 0x80):
                break
        if i >= 1:
            log.error("More than 1 INFO field in the trace info packet")
            return -1

    # Decode KEY
    if plctl & 2:
        for i in range(4):
            byte = data[index]
            key |= (byte & ~0x80) << (7 * i)
            index += 1
            if not (byte & 0x80):
                break
        if i >= 4:
            log.error("More than 4 KEY fields in the trace info packet")
            return -1

    # Decode SPEC
    if plctl & 4:
        for i in range(4):
            byte = data[index]
            spec |= (byte & ~0x80) << (7 * i)
            index += 1
            if not (byte & 0x80):
                break
        if i >= 4:
            log.error("More than 4 SPEC fields in the trace info packet")
            return -1

    # Decode CYCT
    if plctl & 8:
        for i in range(2):
            byte = data[index]
            cyct |= (byte & ~0x80) << (7 * i)
            index += 1
            if not (byte & 0x80):
                break
        if i >= 2:
            log.error("More than 2 CYCT fields in the trace info packet")
            return -1

    log.debug(
        f"[trace info] plctl = 0x{plctl:X}, info = 0x{info:X}, key = 0x{key:X}, spec = {spec}, cyct = 0x{cyct:X}"
    )

    if stream.state.value >= StreamState.INSYNC.value:
        tracer_trace_info(stream.tracer, plctl, info, key, spec, cyct)

    return index


def decode_trace_on(data, stream):
    log.debug("[trace on]")
    tracer_trace_on(stream.tracer)
    return 1


def decode_timestamp(data, stream):
    nr_replace = 0
    index = 1
    ts = 0
    count = 0

    # Decode timestamp
    for i in range(10):
        byte = data[index]
        index += 1
        ts |= (byte & ~0x80) << (7 * i)
        if index != 9:
            nr_replace += 7
        else:
            nr_replace += 8
        if (index != 9) and not (byte & 0x80):
            break

    # Decode cycle count if present
    if data[0] & 1:
        for i in range(3):
            byte = data[index]
            index += 1
            count |= (byte & ~0x80) << (7 * i)
            if (index != 9) and not (byte & 0x80):
                break

    log.debug(f"[timestamp] timestamp = {ts}, cycle count = {count}")

    tracer_ts(stream.tracer, ts, data[0] & 1, count, nr_replace)

    return index


def decode_exception(data, stream: ETBStream):
    index = 0
    EE = 0
    TYPE = 0
    P = 0

    if data[index] & 1:
        # Exception return packet
        index += 1
        log.debug("[exception return]")
        tracer_exception_return(stream.tracer)
    else:
        # Exception packet
        index += 1
        data1 = data[index]
        index += 1
        data2 = 0

        if data1 & 0x80:  # Assuming c_bit is 0x80
            data2 = data[index]
            index += 1

        EE = ((data1 & 0x40) >> 5) | (data1 & 0x01)
        TYPE = ((data1 & 0x3E) >> 1) | (data2 & 0x1F)
        P = (data2 & 0x20) >> 5

        log.debug(f"[exception] E1:E0 = {EE}, TYPE = 0x{TYPE:02X}, P = {P}")

        if EE != 1 and EE != 2:
            log.error("Invalid EE in the exception packet")
            return -1
        elif EE == 2:
            # There is an address packet
            data1 = data[index]
            pkt = stream.decoder.etmv4_pkt_match(data1)
            if pkt:
                ret = pkt.decode(data[index:], stream)
                if ret > 0:
                    index += ret
                else:
                    log.error("Invalid address packet in the exception packet")
                    return -1
            else:
                log.error("No matching trace packet found for address packet")
                return -1

        tracer_exception(stream.tracer, TYPE)

    return index


def decode_cc_format_1(data, stream):
    index = 0
    u_bit = data[index]
    index += 1
    commit = 0
    count = 0

    if not stream.tracer.commopt:
        for i in range(4):
            byte = data[index]
            commit |= (byte & ~0x80) << (7 * i)
            index += 1
            if not (byte & 0x80):
                break
        if i >= 4:
            log.error(
                "More than 4 bytes of the commit section in the cycle count format 1 packet"
            )
            return -1
        tracer_commit(stream.tracer, commit)

    if not u_bit:
        for i in range(3):
            byte = data[index]
            count |= (byte & ~0x80) << (7 * i)
            index += 1
            if not (byte & 0x80):
                break
        if i >= 3:
            log.error(
                "More than 3 bytes of the cycle count section in the cycle count format 1 packet"
            )
            return -1

    log.debug(f"[cycle count format 1] U = {u_bit}, COMMIT = {commit}, COUNT = {count}")

    tracer_cc(stream.tracer, u_bit, count)
    return index


def decode_cc_format_2(data, stream):
    F = data[0] & 0x01
    AAAA = (data[1] & 0xF0) >> 4
    BBBB = data[1] & 0x0F
    log.debug(f"[cycle count format 2] F = {F}, AAAA = {AAAA}, BBBB = {BBBB}")

    if F:
        commit = stream.tracer.max_spec_depth + AAAA - 15
    else:
        commit = AAAA + 1
    if AAAA:
        tracer_commit(stream.tracer, commit)

    tracer_cc(stream.tracer, 0, BBBB)
    return 2


def decode_cc_format_3(data, stream):
    AA = (data[0] & 0x0C) >> 2
    BB = data[0] & 0x03
    log.debug(f"[cycle count format 3] AA = {AA}, BB = {BB}")

    if not stream.tracer.commopt:
        tracer_commit(stream.tracer, AA + 1)

    tracer_cc(stream.tracer, 0, BB)

    return 1


def decode_data_sync_marker(data, stream):
    if data[0] & 0x08:
        log.debug(f"[unnumbered data sync maker] A = {data[0] & 0x07}")
    else:
        log.debug(f"[numbered data sync maker] NUM = {data[0] & 0x07}")
    return 1


def decode_commit(data, stream):
    index = 0
    commit = 0
    for i in range(4):
        byte = data[index]
        commit |= (byte & ~0x80) << (7 * i)
        index += 1
        if not (byte & 0x80):
            break

    if i >= 4:
        log.error("More than 4 bytes of the commit section in the commit packet")
        return -1
    log.debug("[commit] COMMIT = %d", commit)
    tracer_commit(stream.tracer, commit)
    return index


def decode_cancel(data, stream: ETBStream):
    index = 0
    cancel = 0

    if not (data[index] & 0x10):
        # Cancel format 1
        index += 1
        for i in range(4):
            byte = data[index]
            cancel |= (byte & ~0x80) << (7 * i)
            index += 1
            if not (byte & 0x80):
                break
        if i >= 4:
            log.error(
                "More than 4 bytes of the cancel section in the cancel format 1 packet"
            )
            return -1
        log.debug(f"[cancel format 1] M = {data[0] & 0x01}, CANCEL = {cancel}")
        tracer_cancel(stream.tracer, data[0] & 0x01, cancel)
    elif not (data[index] & 0x80):
        # Cancel format 2
        index += 1
        log.debug(f"[cancel format 2] A = {data[index] & 0x03}")
        action = data[index] & 0x03
        if action == 1:
            tracer_atom(stream.tracer, ATOM_TYPE_E)
        elif action == 2:
            tracer_atom(stream.tracer, ATOM_TYPE_E)
            tracer_atom(stream.tracer, ATOM_TYPE_E)
        elif action == 3:
            tracer_atom(stream.tracer, ATOM_TYPE_N)
        else:
            log.error("Unexpected A in a cancel format 2 packet")
        tracer_cancel(stream.tracer, 1, 1)
    else:
        # Cancel format 3
        index += 1
        log.debug(
            f"[cancel format 3] CC = {(data[index] & 0x06) >> 1}, A = {data[index] & 0x01}"
        )
        if data[index] & 0x01:
            tracer_atom(stream.tracer, ATOM_TYPE_E)
        tracer_cancel(stream.tracer, 1, ((data[index] & 0x06) >> 1) + 2)

    return index


def decode_cancel_format_1(data, stream: ETBStream):
    return decode_cancel(data, stream)


def decode_cancel_format_2(data, stream: ETBStream):
    return decode_cancel(data, stream)


def decode_cancel_format_3(data, stream: ETBStream):
    return decode_cancel(data, stream)


def decode_mispredict(data, stream: ETBStream):
    log.debug(f"[mispredict] A = {data[0] & 0x03}")
    tracer_mispredict(stream.tracer, data[0] & 0x03)
    return 1


def decode_cond_inst_format_1(data, stream: ETBStream):
    index = 1
    key = 0

    for i in range(4):
        byte = data[index]
        key |= (byte & ~0x80) << (7 * i)
        index += 1
        if not (byte & 0x80):
            break
    if i >= 4:
        log.error("More than 4 bytes of the commit section in the commit packet")
        return -1

    log.debug(f"[conditional instruction format 1] key = {key}")

    tracer_cond_inst(stream.tracer, 1, key, 0)

    return index


def decode_cond_inst_format_2(data, stream: ETBStream):
    ci = data[0] & 0x03
    log.debug(f"[conditional instruction format 2] ci = {ci}")

    tracer_cond_inst(stream.tracer, 2, ci, 0)

    return 1


def decode_cond_inst_format_3(data, stream: ETBStream):
    z = data[1] & 0x01
    num = (data[1] & 0x7E) >> 1
    log.debug(f"[conditional instruction format 3] z = {z}, num = {num}")

    tracer_cond_inst(stream.tracer, 3, z, num)

    return 2


def decode_cond_flush(data, stream: ETBStream):
    log.debug("[conditional flush]")

    tracer_cond_flush(stream.tracer)

    return 1


def decode_cond_result_format_1(data, stream: ETBStream):
    index = 0
    nr_payloads = 1 if (data[index] & 0x4) else 2
    index += 1

    for payload in range(nr_payloads):
        CI = (data[0] & 0x1) if payload == 0 else ((data[1] & 0x2) >> 1)
        RESULT = data[index] & 0x0F
        KEY = (data[index] >> 4) & 0x7
        index += 1

        for i in range(5):
            byte = data[index]
            KEY |= (byte & ~0x80) << (7 * i + 3)
            index += 1
            if not (byte & 0x80):
                break
        if i >= 5:
            log.error(
                "More than 5 payload bytes in the conditional result format 1 packet"
            )
            return -1

        log.debug(
            f"[conditional result format 1] CI[{payload}] = {CI}, RESULT[{payload}] = 0x{RESULT:X}, KEY[{payload}] = {KEY}"
        )

        tracer_cond_result(stream.tracer, 1, KEY, CI, RESULT)

    return index


def decode_cond_result_format_2(data, stream: ETBStream):
    K = (data[0] >> 2) & 0x1
    T = data[0] & 0x3
    log.debug(f"[conditional result format 2] K = {K}, T = 0x{T:X}")

    tracer_cond_result(stream.tracer, 2, K, T, 0)

    return 1


def decode_cond_result_format_3(data, stream: ETBStream):
    TOKEN = data[1] | ((data[0] & 0x0F) << 8)
    log.debug(f"[conditional result format 3] TOKEN = 0x{TOKEN:X}")

    tracer_cond_result(stream.tracer, 3, TOKEN, 0, 0)

    return 2


def decode_cond_result_format_4(data, stream: ETBStream):
    T = data[0] & 0x3
    log.debug(f"[conditional result format 4] T = 0x{T:X}")

    tracer_cond_result(stream.tracer, 4, T, 0, 0)

    return 1


def decode_event(data, stream: ETBStream):
    event = data[0] & 0x0F
    log.debug(f"[event] EVENT = 0x{event:X}")
    return 1


def update_address_regs(stream: ETBStream, address, instruction_set):
    stream.tracer.set_address_register(2, stream.tracer.address_register[1].address)
    stream.tracer.set_address_register(1, stream.tracer.address_register[0].address)
    stream.tracer.set_address_register(0, address)
    stream.tracer.set_address_register_IS(0, instruction_set)


def decode_short_address(data, stream):
    index = 0
    address = stream.tracer.address_register[0].address
    instruction_set = ADDR_REG_IS0 if (data[index] & 0x01) else ADDR_REG_IS1
    index += 1
    if instruction_set == ADDR_REG_IS0:
        address &= ~0x000001FF
        address |= (data[index] & 0x7F) << 2
        index += 1
        if data[index] & 0x80:
            address &= ~0x0001FE00
            address |= (data[index] & 0x7F) << 9
            index += 1
    else:
        address &= ~0x000000FF
        address |= (data[index] & 0x7F) << 1
        index += 1
        if data[1] & 0x80:
            address &= ~0x0000FF00
            address |= (data[index]) << 8
            index += 1

    log.debug(
        f"[short address] address = 0x{address:016x}, instruction_set = {instruction_set}"
    )

    update_address_regs(stream, address, instruction_set)
    tracer_address(stream.tracer)
    return index


def decode_short_address_is0(data, stream):
    return decode_short_address(data, stream)


def decode_short_address_is1(data, stream):
    return decode_short_address(data, stream)


def decode_long_address(data, stream):
    index = 1
    address = stream.tracer.address_register[0].address

    if data[0] == 0x9A:
        instruction_set = ADDR_REG_IS0
        address &= ~0xFFFFFFFF
        address |= (data[index] & 0x7F) << 2
        address |= (data[index + 1] & 0x7F) << 9
        address |= data[index + 2] << 16
        address |= data[index + 3] << 24
        index += 4
    elif data[0] == 0x9B:
        instruction_set = ADDR_REG_IS1
        address &= ~0xFFFFFFFF
        address |= (data[index] & 0x7F) << 1
        address |= data[index + 1] << 8
        address |= data[index + 2] << 16
        address |= data[index + 3] << 24
        index += 4
    elif data[0] == 0x9D:
        instruction_set = ADDR_REG_IS0
        address = 0
        address |= (data[index] & 0x7F) << 2
        address |= (data[index + 1] & 0x7F) << 9
        address |= data[index + 2] << 16
        address |= data[index + 3] << 24
        address |= data[index + 4] << 32
        address |= data[index + 5] << 40
        address |= data[index + 6] << 48
        address |= data[index + 7] << 56
        index += 8
    elif data[0] == 0x9E:
        instruction_set = ADDR_REG_IS1
        address = 0
        address |= (data[index] & 0x7F) << 1
        address |= data[index + 1] << 8
        address |= data[index + 2] << 16
        address |= data[index + 3] << 24
        address |= data[index + 4] << 32
        address |= data[index + 5] << 40
        address |= data[index + 6] << 48
        address |= data[index + 7] << 56
        index += 8
    else:
        return -1

    log.debug(
        f"[long address] address = 0x{address:016x}, instruction_set = {instruction_set}"
    )

    update_address_regs(stream, address, instruction_set)
    tracer_address(stream.tracer)

    return index


def decode_long_address_32bit_is0(data, stream):
    return decode_long_address(data, stream)


def decode_long_address_32bit_is1(data, stream):
    return decode_long_address(data, stream)


def decode_long_address_64bit_is0(data, stream):
    return decode_long_address(data, stream)


def decode_long_address_64bit_is1(data, stream):
    return decode_long_address(data, stream)


def decode_exact_match_address(data, stream: ETBStream):
    QE = data[0] & 0x03

    address = stream.tracer.address_register[QE].address
    instruction_set = stream.tracer.address_register[QE].instruction_set

    log.debug(
        f"[Exact match address] QE = {QE}, address = 0x{address:016x}, instruction_set = {instruction_set}"
    )

    update_address_regs(stream, address, instruction_set)
    tracer_address(stream.tracer)

    return 1


def decode_context(data, stream):
    index = 0
    EL = SF = NS = 0
    V = C = 0
    VMID = 0
    CONTEXTID = 0

    if data[index] & 1:
        index += 1
        EL = data[index] & 0x3
        SF = (data[index] & 0x10) >> 4
        NS = (data[index] & 0x20) >> 5
        if data[index] & 0x40:
            V = 1
            index += 1
            VMID = data[index]
        if data[index] & 0x80:
            C = 1
            index += 1
            CONTEXTID = data[index]
            CONTEXTID |= data[index + 1] << 8
            CONTEXTID |= data[index + 2] << 16
            CONTEXTID |= data[index + 3] << 24
            index += 3
        log.debug(
            f"[context] P = 1'b1, EL = {EL}, SF = {SF}, NS = {NS}, V = {V}, \n"
            f"C = {C}, VMID = {VMID}, CONTEXTID = 0x{CONTEXTID:04X}"
        )
    else:
        log.debug("[context] P = 1'b0")

    tracer_context(stream.tracer, data[0] & 1, EL, SF, NS, V, VMID, C, CONTEXTID)

    return index + 1


def decode_address_context(data, stream: ETBStream):
    index = 1
    address = stream.tracer.address_register[0].address

    if data[0] == 0x82:
        instruction_set = ADDR_REG_IS0
        address &= ~0xFFFFFFFF
        address |= (data[index] & 0x7F) << 2
        address |= (data[index + 1] & 0x7F) << 9
        address |= data[index + 2] << 16
        address |= data[index + 3] << 24
        index += 4
    elif data[0] == 0x83:
        instruction_set = ADDR_REG_IS1
        address &= ~0xFFFFFFFF
        address |= (data[index] & 0x7F) << 1
        address |= data[index + 1] << 8
        address |= data[index + 2] << 16
        address |= data[index + 3] << 24
        index += 4
    elif data[0] == 0x85:
        instruction_set = ADDR_REG_IS0
        address = 0
        address |= (data[index] & 0x7F) << 2
        address |= (data[index + 1] & 0x7F) << 9
        address |= data[index + 2] << 16
        address |= data[index + 3] << 24
        address |= data[index + 4] << 32
        address |= data[index + 5] << 40
        address |= data[index + 6] << 48
        address |= data[index + 7] << 56
        index += 8
    elif data[0] == 0x86:
        instruction_set = ADDR_REG_IS1
        address = 0
        address |= (data[index] & 0x7F) << 1
        address |= data[index + 1] << 8
        address |= data[index + 2] << 16
        address |= data[index + 3] << 24
        address |= data[index + 4] << 32
        address |= data[index + 5] << 40
        address |= data[index + 6] << 48
        address |= data[index + 7] << 56
        index += 8
    else:
        return -1

    update_address_regs(stream, address, instruction_set)

    data_byte = data[index]
    EL = data_byte & 0x3
    SF = (data_byte & 0x10) >> 4
    NS = (data_byte & 0x20) >> 5
    V = C = 0
    VMID = 0
    CONTEXTID = 0

    if data_byte & 0x40:
        V = 1
        index += 1
        VMID = data[index]
    if data_byte & 0x80:
        C = 1
        index += 1
        CONTEXTID = data[index]
        CONTEXTID |= data[index + 1] << 8
        CONTEXTID |= data[index + 2] << 16
        CONTEXTID |= data[index + 3] << 24
        index += 3

    log.debug(
        f"[address with context] address = 0x{address:016x}, instruction_set = {instruction_set}, "
        f"EL = {EL}, SF = {SF}, NS = {NS}, V = {V}, C = {C}, VMID = {VMID}, CONTEXTID = 0x{CONTEXTID:04X}"
    )

    tracer_context(stream.tracer, 1, EL, SF, NS, V, VMID, C, CONTEXTID)
    tracer_address(stream.tracer)

    return index + 1


def decode_address_context_32bit_is0(data, stream):
    return decode_address_context(data, stream)


def decode_address_context_32bit_is1(data, stream):
    return decode_address_context(data, stream)


def decode_address_context_64bit_is0(data, stream):
    return decode_address_context(data, stream)


def decode_address_context_64bit_is1(data, stream):
    return decode_address_context(data, stream)


def decode_atom_format_1(data, stream):
    A = data[0] & 0x01
    log.debug(f"[atom format 1] A = {A}")
    tracer_atom(stream.tracer, ATOM_TYPE_E if A != 0 else ATOM_TYPE_N)
    return 1


def decode_atom_format_2(data, stream):
    A = data[0] & 0x03
    log.debug(f"[atom format 2] A = {A}")
    tracer_atom(stream.tracer, ATOM_TYPE_E if A & 1 != 0 else ATOM_TYPE_N)
    tracer_atom(stream.tracer, ATOM_TYPE_E if A & 2 != 0 else ATOM_TYPE_N)
    return 1


def decode_atom_format_3(data, stream):
    A = data[0] & 0x07
    log.debug(f"[atom format 3] A = {A}")
    tracer_atom(stream.tracer, ATOM_TYPE_E if A & 1 != 0 else ATOM_TYPE_N)
    tracer_atom(stream.tracer, ATOM_TYPE_E if A & 2 != 0 else ATOM_TYPE_N)
    tracer_atom(stream.tracer, ATOM_TYPE_E if A & 4 != 0 else ATOM_TYPE_N)
    return 1


def decode_atom_format_4(data, stream):
    A = data[0] & 0x03
    log.debug(f"[atom format 4] A = {A}")

    if A == 0:
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_E)
        tracer_atom(stream.tracer, ATOM_TYPE_E)
        tracer_atom(stream.tracer, ATOM_TYPE_E)
    elif A == 1:
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_N)
    elif A == 2:
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_E)
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_E)
    elif A == 3:
        tracer_atom(stream.tracer, ATOM_TYPE_E)
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_E)
        tracer_atom(stream.tracer, ATOM_TYPE_N)
    return 1


def decode_atom_format_5_1(data, stream):
    ABC = ((data[0] >> 3) & 0x04) | (data[0] & 0x3)
    log.debug(f"[atom format 5] ABC = {ABC}")

    if ABC == 5:
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_E)
        tracer_atom(stream.tracer, ATOM_TYPE_E)
        tracer_atom(stream.tracer, ATOM_TYPE_E)
        tracer_atom(stream.tracer, ATOM_TYPE_E)
    elif ABC == 1:
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_N)
    elif ABC == 2:
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_E)
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_E)
        tracer_atom(stream.tracer, ATOM_TYPE_N)
    elif ABC == 3:
        tracer_atom(stream.tracer, ATOM_TYPE_E)
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_E)
        tracer_atom(stream.tracer, ATOM_TYPE_N)
        tracer_atom(stream.tracer, ATOM_TYPE_E)
    else:
        log.error("Invalid ABC in an ATOM format 5 packet")

    return 1


def decode_atom_format_5_2(data, stream):
    return decode_atom_format_5_1(data, stream)


def decode_atom_format_5_3(data, stream):
    return decode_atom_format_5_1(data, stream)


def decode_atom_format_5_4(data, stream):
    return decode_atom_format_5_1(data, stream)


def decode_atom_format_6_1(data, stream):
    A = (data[0] >> 5) & 0x01
    COUNT = data[0] & 0x1F
    if COUNT > 20:
        log.error("Invalid COUNT in an ATOM format 6 packet")
        return -1
    log.debug(f"[atom format 6] A = {A}, COUNT = {COUNT}")

    for _ in range(COUNT + 3):
        tracer_atom(stream.tracer, ATOM_TYPE_E)
    tracer_atom(stream.tracer, ATOM_TYPE_N if A else ATOM_TYPE_E)

    return 1


def decode_atom_format_6_2(data, stream):
    return decode_atom_format_6_1(data, stream)


def decode_atom_format_6_3(data, stream):
    return decode_atom_format_6_1(data, stream)


def decode_atom_format_6_4(data, stream):
    return decode_atom_format_6_1(data, stream)


def decode_atom_format_6_5(data, stream):
    return decode_atom_format_6_1(data, stream)


def decode_atom_format_6_6(data, stream):
    return decode_atom_format_6_1(data, stream)


def decode_atom_format_6_7(data, stream):
    return decode_atom_format_6_1(data, stream)


def decode_atom_format_6_8(data, stream):
    return decode_atom_format_6_1(data, stream)


def decode_atom_format_6_9(data, stream):
    return decode_atom_format_6_1(data, stream)


def decode_atom_format_6_10(data, stream):
    return decode_atom_format_6_1(data, stream)


def decode_atom_format_6_11(data, stream):
    return decode_atom_format_6_1(data, stream)


def decode_atom_format_6_12(data, stream):
    return decode_atom_format_6_1(data, stream)


def decode_q(data, stream):
    index = 0
    type = data[index] & 0x0F
    index += 1
    count_unknown = False
    address = stream.tracer.address_register[0].address
    instruction_set = stream.tracer.address_register[0].instruction_set

    if type in [0, 1, 2]:
        address = stream.tracer.address_register[type].address
        instruction_set = stream.tracer.address_register[type].instruction_set
        update_address_regs(stream, address, instruction_set)

    elif type in [5, 6]:
        instruction_set = ADDR_REG_IS0 if type == 5 else ADDR_REG_IS1
        if instruction_set == ADDR_REG_IS0:
            address &= ~0x000001FF
            address |= (data[index] & 0x7F) << 2
            index += 1
            if data[1] & 0x80:
                address &= ~0x0001FE00
                address |= data[index] << 9
                index += 1
        else:
            address &= ~0x000000FF
            address |= (data[index] & 0x7F) << 1
            index += 1
            if data[1] & 0x80:
                address &= ~0x0000FF00
                address |= data[index] << 8
                index += 1
        update_address_regs(stream, address, instruction_set)

    elif type in [10, 11]:
        if type == 10:
            instruction_set = ADDR_REG_IS0
            address &= ~0xFFFFFFFF
            address |= (data[index] & 0x7F) << 2
            address |= (data[index + 1] & 0x7F) << 9
            address |= data[index + 2] << 16
            address |= data[index + 3] << 24
            index += 4
        else:
            instruction_set = ADDR_REG_IS1
            address &= ~0xFFFFFFFF
            address |= (data[index] & 0x7F) << 1
            address |= data[index + 1] << 8
            address |= data[index + 2] << 16
            address |= data[index + 3] << 24
            index += 4
        update_address_regs(stream, address, instruction_set)

    elif type == 12:
        pass

    elif type == 15:
        count_unknown = True

    else:
        return -1

    if not count_unknown:
        COUNT = 0
        for i in range(5):
            byte = data[index]
            COUNT |= (byte & ~0x80) << (7 * i + 3)
            index += 1
            if not (byte & 0x80):
                break

    if count_unknown:
        log.debug(
            f"[Q] type = {type}, address = 0x{address:016x}, instruction_set = {instruction_set}, count unknown"
        )
    else:
        log.debug(
            f"[Q] type = {type}, address = 0x{address:016x}, instruction_set = {instruction_set}, count = {COUNT}"
        )

    tracer_q(stream.tracer, 0 if count_unknown else COUNT)
    tracer_address(stream.tracer)

    return index


extension = TracePacket(0xFF, 0x00, decode_extension)
trace_info = TracePacket(0xFF, 0x01, decode_trace_info)
trace_on = TracePacket(0xFF, 0x04, decode_trace_on)
timestamp = TracePacket(0xFE, 0x02, decode_timestamp)
exception = TracePacket(0xFE, 0x06, decode_exception)
cc_format_1 = TracePacket(0xFE, 0x0E, decode_cc_format_1)
cc_format_2 = TracePacket(0xFE, 0x0C, decode_cc_format_2)
cc_format_3 = TracePacket(0xF0, 0x10, decode_cc_format_3)
data_sync_marker = TracePacket(0xF0, 0x20, decode_data_sync_marker)
commit = TracePacket(0xFF, 0x2D, decode_commit)
cancel_format_1 = TracePacket(0xFE, 0x2E, decode_cancel_format_1)
cancel_format_2 = TracePacket(0xFC, 0x34, decode_cancel_format_2)
cancel_format_3 = TracePacket(0xF8, 0x38, decode_cancel_format_3)
mispredict = TracePacket(0xFC, 0x30, decode_mispredict)
cond_inst_format_1 = TracePacket(0xFF, 0x6C, decode_cond_inst_format_1)
cond_inst_format_2 = TracePacket(0xFC, 0x40, decode_cond_inst_format_2)
cond_inst_format_3 = TracePacket(0xFF, 0x6D, decode_cond_inst_format_3)
cond_flush = TracePacket(0xFF, 0x43, decode_cond_flush)
cond_result_format_1 = TracePacket(0xF8, 0x68, decode_cond_result_format_1)
cond_result_format_2 = TracePacket(0xF8, 0x48, decode_cond_result_format_2)
cond_result_format_3 = TracePacket(0xF0, 0x50, decode_cond_result_format_3)
cond_result_format_4 = TracePacket(0xFC, 0x44, decode_cond_result_format_4)
event = TracePacket(0xF0, 0x70, decode_event)
short_address_is0 = TracePacket(0xFF, 0x95, decode_short_address_is0)
short_address_is1 = TracePacket(0xFF, 0x96, decode_short_address_is1)
long_address_32bit_is0 = TracePacket(0xFF, 0x9A, decode_long_address_32bit_is0)
long_address_32bit_is1 = TracePacket(0xFF, 0x9B, decode_long_address_32bit_is1)
long_address_64bit_is0 = TracePacket(0xFF, 0x9D, decode_long_address_64bit_is0)
long_address_64bit_is1 = TracePacket(0xFF, 0x9E, decode_long_address_64bit_is1)
exact_match_address = TracePacket(0xFC, 0x90, decode_exact_match_address)
context = TracePacket(0xFE, 0x80, decode_context)
address_context_32bit_is0 = TracePacket(0xFF, 0x82, decode_address_context_32bit_is0)
address_context_32bit_is1 = TracePacket(0xFF, 0x83, decode_address_context_32bit_is1)
address_context_64bit_is0 = TracePacket(0xFF, 0x85, decode_address_context_64bit_is0)
address_context_64bit_is1 = TracePacket(0xFF, 0x86, decode_address_context_64bit_is1)
atom_format_1 = TracePacket(0xFE, 0xF6, decode_atom_format_1)
atom_format_2 = TracePacket(0xFC, 0xD8, decode_atom_format_2)
atom_format_3 = TracePacket(0xF8, 0xF8, decode_atom_format_3)
atom_format_4 = TracePacket(0xFC, 0xDC, decode_atom_format_4)
atom_format_5_1 = TracePacket(0xFF, 0xF5, decode_atom_format_5_1)
atom_format_5_2 = TracePacket(0xFF, 0xD5, decode_atom_format_5_2)
atom_format_5_3 = TracePacket(0xFF, 0xD6, decode_atom_format_5_3)
atom_format_5_4 = TracePacket(0xFF, 0xD7, decode_atom_format_5_4)
atom_format_6_1 = TracePacket(0xFF, 0xD0, decode_atom_format_6_1)
atom_format_6_2 = TracePacket(0xFF, 0xD1, decode_atom_format_6_2)
atom_format_6_3 = TracePacket(0xFF, 0xD2, decode_atom_format_6_3)
atom_format_6_4 = TracePacket(0xFF, 0xD3, decode_atom_format_6_4)
atom_format_6_5 = TracePacket(0xFF, 0xD4, decode_atom_format_6_5)
atom_format_6_6 = TracePacket(0xFF, 0xF0, decode_atom_format_6_6)
atom_format_6_7 = TracePacket(0xFF, 0xF1, decode_atom_format_6_7)
atom_format_6_8 = TracePacket(0xFF, 0xF2, decode_atom_format_6_8)
atom_format_6_9 = TracePacket(0xFF, 0xF3, decode_atom_format_6_9)
atom_format_6_10 = TracePacket(0xFF, 0xF4, decode_atom_format_6_10)
atom_format_6_11 = TracePacket(0xF0, 0xC0, decode_atom_format_6_11)
atom_format_6_12 = TracePacket(0xF0, 0xE0, decode_atom_format_6_12)
q = TracePacket(0xF0, 0xA0, decode_q)

# Example usage
etmv4_tracepkts = [
    extension,
    trace_info,
    trace_on,
    timestamp,
    exception,
    cc_format_1,
    cc_format_2,
    cc_format_3,
    data_sync_marker,
    commit,
    cancel_format_1,
    cancel_format_2,
    cancel_format_3,
    mispredict,
    cond_inst_format_1,
    cond_inst_format_2,
    cond_inst_format_3,
    cond_flush,
    cond_result_format_1,
    cond_result_format_2,
    cond_result_format_3,
    cond_result_format_4,
    event,
    short_address_is0,
    short_address_is1,
    long_address_32bit_is0,
    long_address_32bit_is1,
    long_address_64bit_is0,
    long_address_64bit_is1,
    exact_match_address,
    context,
    address_context_32bit_is0,
    address_context_32bit_is1,
    address_context_64bit_is0,
    address_context_64bit_is1,
    atom_format_1,
    atom_format_2,
    atom_format_3,
    atom_format_4,
    atom_format_5_1,
    atom_format_5_2,
    atom_format_5_3,
    atom_format_5_4,
    atom_format_6_1,
    atom_format_6_2,
    atom_format_6_3,
    atom_format_6_4,
    atom_format_6_5,
    atom_format_6_6,
    atom_format_6_7,
    atom_format_6_8,
    atom_format_6_9,
    atom_format_6_10,
    atom_format_6_11,
    atom_format_6_12,
    q,
]


def etmv4_synchronization(stream: ETBStream):
    log.debug(f"MAX_SPEC_DEPTH = {stream.tracer.max_spec_depth}")
    log.debug(f"P0_KEY_MAX = {stream.tracer.p0_key_max}")
    log.debug(f"COND_KEY_MAX_INCR = {stream.tracer.cond_key_max_incr}")
    log.debug(f"CONDTYPE = {stream.tracer.condtype}")
    log.debug(f"COMMOPT = {stream.tracer.commopt}")
    PKT_SIZE = 12

    # Locate an async packet and search for a trace-info packet
    for i in range(len(stream.buff)):
        c = stream.buff[i]
        if (c & extension.mask) == extension.val:
            p = extension.decode(stream.buff[i:], stream)
            if p != PKT_SIZE:
                continue
            c = stream.buff[i + PKT_SIZE]
            if (c & trace_info.mask) == trace_info.val:
                p = trace_info.decode(stream.buff[i + PKT_SIZE :], stream)
                if p > 0:
                    # SYNCING -> INSYNC
                    stream.state = StreamState.INSYNC
                    stream.tracer.reset_address_register()
                    return i

            log.error("No trace info packet right after an a-sync packet")

    return -1


class ETMv4Decoder:
    def __init__(self):
        self.pkt_map = {}

        for c in range(256):
            for pkt in etmv4_tracepkts:
                if (c & pkt.mask) == pkt.val:
                    self.pkt_map[c] = pkt
                    break

    def etmv4_pkt_match(self, c):
        return self.pkt_map.get(c, None)

    def synchronization(self, stream: ETBStream):
        return etmv4_synchronization(stream)


def decode_stream(stream: ETBStream):
    if stream.state != StreamState.READING:
        log.error("Stream state is not correct")
        return -1

    stream.state = StreamState.SYNCING
    log.debug("Syncing the trace stream...")
    cur = stream.decoder.synchronization(stream)
    if cur < 0:
        log.error("Cannot find any synchronization packet")
        return -1
    else:
        log.debug(f"Trace starts from offset {cur}")

    stream.state = StreamState.DECODING

    log.debug("Decoding the trace stream...")
    buff_mv = memoryview(stream.buff)
    total_len = len(buff_mv)

    while cur < total_len:
        c = buff_mv[cur]
        pkt = stream.decoder.etmv4_pkt_match(c)
        if pkt:
            log.debug(f"Get a packet of type {pkt.name}")
        else:
            log.error(f"Cannot recognize a packet header 0x{c:02x}, offset: {cur}")
            log.error(f"Proceed on guesswork, offset: {cur}")
            cur += 1
            continue

        ret = pkt.decode(buff_mv[cur:], stream)

        if ret <= 0:
            log.error(f"Proceed on guesswork, offset: {cur}")
            cur += 1
        else:
            cur += ret

    log.debug("Complete decode of the trace stream")
    return 0


def decode_etb(stream: ETBStream, unaligned: bool):
    FSYNC = bytes([0xFF, 0xFF, 0xFF, 0x7F])
    ETB_PACKET_SIZE = 16
    NULL_TRACE_SOURCE = 0

    pkt_idx = 0
    cur_id = -1
    pre_id = -1
    nr_stream = 1
    streams = [stream.copy() for _ in range(nr_stream)]
    buff_mv = stream.buff
    total_len = len(buff_mv)

    pkt_idx = buff_mv.find(FSYNC) if unaligned else 0
    if pkt_idx == -1:
        log.error("No frame synchronization packet found.")
        return -1

    for s in streams:
        if not isinstance(s.buff, bytearray):
            s.buff = bytearray(s.buff)

    while pkt_idx + ETB_PACKET_SIZE <= total_len:
        sync_ofs = buff_mv[pkt_idx : pkt_idx + 4] == FSYNC
        if sync_ofs:
            pkt_idx += 4

        if pkt_idx + ETB_PACKET_SIZE >= total_len:
            break

        end = buff_mv[pkt_idx + ETB_PACKET_SIZE - 1]

        byte_idx = 0
        log.debug(f"ofs: {pkt_idx}, cur_id: {cur_id}")
        while byte_idx < ETB_PACKET_SIZE - 1:
            c = buff_mv[pkt_idx + byte_idx]

            if (byte_idx & 1) == 0:
                if c & 1:
                    id = (c >> 1) & 0x7F
                    if id == NULL_TRACE_SOURCE:
                        log.info("Found a NULL_TRACE_SOURCE ID in the ETB data packet")
                        break
                    pre_id = cur_id
                    cur_id = id - 1
                    if cur_id >= len(streams):
                        streams.extend(
                            stream.copy() for _ in range(cur_id - len(streams) + 1)
                        )
                else:
                    c |= (end & (1 << (byte_idx // 2))) and 1
                    if cur_id >= 0:
                        log.debug(
                            f"==Got a data byte {c:02x} at offset {len(streams[cur_id].buff)}, pkt_idx: {pkt_idx}"
                        )
                        streams[cur_id].buff.append(c)
            else:
                tmp = buff_mv[pkt_idx + byte_idx - 1]
                if (tmp & 1) and (end & (1 << (byte_idx // 2))):
                    if pre_id >= 0:
                        log.debug(
                            f"==Got a data byte {c:02x} at offset {len(streams[cur_id].buff)}, pkt_idx: {pkt_idx}"
                        )
                        streams[pre_id].buff.append(c)
                else:
                    if cur_id >= 0:
                        log.debug(
                            f"==Got a data byte {c:02x} at offset {len(streams[cur_id].buff)}, pkt_idx: {pkt_idx}"
                        )
                        streams[cur_id].buff.append(c)

            byte_idx += 1

        pkt_idx += ETB_PACKET_SIZE

    cnt = 0
    for i, s in enumerate(streams):
        log.debug(f"There are {len(s.buff)} bytes in the stream {i}")
        if len(s.buff) != 0:
            log.info(f"Decode trace stream of ID {i}")
            decode_stream(s)
            cnt += 1
        else:
            log.info(f"There is no valid data in the stream of ID {i}")

    print(f"Decode {cnt} streams")
    return 0
