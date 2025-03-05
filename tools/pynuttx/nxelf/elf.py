#! /usr/bin/env python3
############################################################################
# tools/pynuttx/nxelf/elf.py
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


import hashlib
import importlib
import logging
import os
import pickle
import sys
import time
from enum import IntEnum
from typing import Tuple

from .macros import Macro

try:
    import cxxfilt
    import lief
    from construct import (
        Array,
        Construct,
        Float16l,
        Float32l,
        Float64l,
        Int8sl,
        Int8ul,
        Int16sl,
        Int16ul,
        Int32sb,
        Int32sl,
        Int32ub,
        Int32ul,
        Int64sb,
        Int64sl,
        Int64ub,
        Int64ul,
        Struct,
    )
    from elftools.elf.elffile import ELFFile
    from elftools.elf.sections import SymbolTableSection
except ModuleNotFoundError as e:
    print(f"Error:{e}.\nPlease execute the following command to install dependencies:")
    print("pip install construct pyelftools cxxfilt lief ")

log = logging.getLogger(__name__)


class TypeConflictError(Exception):
    """
    Symbols have different definitions
    """

    pass


class Types:
    def __init__(self, tag):
        self.types = {}
        self.tag = tag
        self.result = dict()

    def set_type(self, die):
        if "DW_AT_name" not in die.attributes:
            return

        name = die.attributes["DW_AT_name"].value.decode("utf-8")
        if name not in self.types:
            self.types[name] = set()

        self.types[name].add(die)

    def get_types(self, type_name):
        if type_name in self.types:
            sets = self.types[type_name]
            return sets
        else:
            return None

    def set_result(self, type_name, result):
        if len(result) != 1:
            raise TypeConflictError(
                f"Multiple different definitions or values ​​exist for a symbol: {type_name} {result}"
            )

        result = result.pop()
        if type_name in self.types:
            self.result[type_name] = result
        return result

    def get_result(self, type_name):
        if type_name in self.result:
            return self.result[type_name]
        return None


class ELFParser:
    """
    ELF file parser class for extracting the following information from ELF files:
    - Symbol addresses
    - Structure definitions
    - Enumeration type definitions
    - Enumeration values

    Main functionality:
    1. Get structure definitions
        elf_parser = ELFParser("nuttx")
        struct = elf_parser.get_type("file_operations")  # Returns construct.Struct object
        print(struct._subcons)

        # result:
        Container:
            open = <Renamed open <FormatField>>
            close = <Renamed close <FormatField>>
            read = <Renamed read <FormatField>>
            write = <Renamed write <FormatField>>
            seek = <Renamed seek <FormatField>>
            ioctl = <Renamed ioctl <FormatField>>
            mmap = <Renamed mmap <FormatField>>
            truncate = <Renamed truncate <FormatField>>
            poll = <Renamed poll <FormatField>>
            unlink = <Renamed unlink <FormatField>>

    2. Get symbol addresses
       addr = elf_parser.symbol_addr("_SeggerRTT")  # Returns symbol address

    3. Get enumeration type definitions
       enum = elf_parser.get_type("tstate_e")  # Returns construct.IntEnum object

    4. Get enumeration values
        value = elf_parser.enum_value("TSTATE_TASK_RUNNING")  # Returns integer value

    """

    def __init__(self, elf_path):
        self.elf = ELFFile(open(elf_path, "rb"))
        self.types = dict()
        self.info = dict()
        self.symbol = dict()
        self.dwarf = self.elf.get_dwarf_info()

        t = time.time()
        print("Parsing ELF file...")
        self.parse_header()
        self.parse_types()
        self.macro = Macro(elf_path)
        print(f"ELF file parsed in {time.time() - t:.1f} seconds")

    def parse_header(self):
        header = self.elf.header
        self.info["bitwides"] = (
            32 if header["e_ident"]["EI_CLASS"] == "ELFCLASS32" else 64
        )
        self.info["byteorder"] = (
            "little" if header["e_ident"]["EI_DATA"] == "ELFDATA2LSB" else "big"
        )
        self.info["arch"] = header["e_machine"]
        self.info["size_t"] = "uint%d" % self.info["bitwides"]

    def parse_symbol(self):
        tables = [
            s
            for _, s in enumerate(self.elf.iter_sections())
            if isinstance(s, SymbolTableSection)
            and s.name == ".symtab"
            and s["sh_entsize"]
        ]

        for section in tables:
            for nsym, symbol in enumerate(section.iter_symbols()):
                try:
                    name = cxxfilt.demangle(symbol.name)
                except Exception:
                    name = symbol.name
                self.symbol[name] = symbol["st_value"]

    def symbol_addr(self, name):
        if len(self.symbol.keys()) == 0:
            self.parse_symbol()

        if name not in self.symbol:
            return None

        return self.symbol[name]

    def parse_types(self):
        for CU in self.elf.get_dwarf_info().iter_CUs():
            # Iterate all DIEs in CU and save them by tag
            for DIE in CU.iter_DIEs():
                # If the type is already in the dictionary, add the die to the set
                if DIE.tag in self.types.keys():
                    self.types[DIE.tag].set_type(DIE)
                else:
                    self.types[DIE.tag] = Types(DIE.tag)
                    self.types[DIE.tag].set_type(DIE)

    def parse_array(self, die):
        nums = 0
        for child in die.iter_children():
            nums = child.attributes["DW_AT_upper_bound"].value

        type_die = self.dwarf.get_DIE_from_refaddr(
            die.attributes["DW_AT_type"].value + die.cu.cu_offset
        )

        item_type = self.parse_die(type_die)
        array = Array(nums, item_type)
        return array

    def parse_die(self, die):
        if die.tag == "DW_TAG_structure_type":
            if "DW_AT_name" not in die.attributes:
                return None
            ret = self.struct(die.attributes["DW_AT_name"].value.decode("utf-8"))
            return ret
        elif die.tag == "DW_TAG_enumeration_type":
            if "DW_AT_name" not in die.attributes:
                return None
            return self.enum(die.attributes["DW_AT_name"].value.decode("utf-8"))
        elif die.tag == "DW_TAG_base_type":
            if "DW_AT_name" not in die.attributes:
                return None
            return self.base_type(die.attributes["DW_AT_name"].value.decode("utf-8"))
        elif die.tag == "DW_TAG_typedef":
            if "DW_AT_name" not in die.attributes:
                return None
            return self.typedef(die.attributes["DW_AT_name"].value.decode("utf-8"))
        elif die.tag == "DW_TAG_pointer_type":
            if self.info["bitwides"] == 32:
                return Int32ul
            elif self.info["bitwides"] == 64:
                return Int64ul
            else:
                raise ValueError("Unsupported ELF class")
        elif die.tag == "DW_TAG_array_type":
            return self.parse_array(die)
        else:
            raise ValueError(f"Unsupported type: {die.tag}")

    def get_type(self, type_name):
        if len(self.types.keys()) == 0:
            self.parse_types()

        for key, value in self.types.items():
            if type_name in value.types.keys():
                if key == "DW_TAG_structure_type":
                    return self.struct(type_name)
                elif key == "DW_TAG_enumeration_type":
                    return self.enum(type_name)
                elif key == "DW_TAG_base_type":
                    return self.base_type(type_name)
                elif key == "DW_TAG_typedef":
                    return self.typedef(type_name)
                elif key == "DW_TAG_enumerator":
                    return self.enum_value(type_name)
                else:
                    raise ValueError(f"Unsupported type: {key} {type_name}")

        return None

    def base_type(self, name):
        base_types = self.types["DW_TAG_base_type"]
        types = base_types.get_types(name)
        if types is None:
            return None

        for die in types:
            name = die.attributes["DW_AT_name"].value.decode("utf-8")
            size = die.attributes["DW_AT_byte_size"].value

            unsigned_map = {
                1: Int8ul,
                2: Int16ul,
                4: Int32ul,
                8: Int64ul,
            }
            signed_map = {
                1: Int8sl,
                2: Int16sl,
                4: Int32sl,
                8: Int64sl,
            }
            double_map = {
                2: Float16l,
                4: Float32l,
                8: Float64l,
            }
            if "unsigned" in name:
                return unsigned_map.get(size, None)
            elif "double" in name or "float" in name:
                return double_map.get(size, None)
            elif "_Bool" in name:
                return Int8ul
            elif "char" in name or "short" in name or "int" in name:
                return signed_map.get(size, None)

        raise ValueError(f"Unsupported base type: {name}")

    def typedef(self, name):
        typedefs = self.types["DW_TAG_typedef"]
        types = typedefs.get_types(name)
        if types is None:
            return None

        for die in types:
            name = die.attributes["DW_AT_name"].value.decode("utf-8")
            type_attr = die.attributes["DW_AT_type"]
            die = self.dwarf.get_DIE_from_refaddr(type_attr.value + die.cu.cu_offset)
            return self.parse_die(die)

    def struct(self, type_name):
        structs = self.types["DW_TAG_structure_type"]
        types = structs.get_types(type_name)

        if types is None:
            return None

        # If the type is a list, it means we have already obtained the enum value
        ret = structs.get_result(type_name)
        if ret is not None:
            return ret

        rets = set()
        for dies in types:
            members = dict()
            for die in dies.iter_children():
                member_name = die.attributes["DW_AT_name"].value.decode("utf-8")
                member_type = die.attributes["DW_AT_type"].value
                type_die = self.dwarf.get_DIE_from_refaddr(
                    member_type + die.cu.cu_offset
                )
                member_type = self.parse_die(type_die)
                members[member_name] = member_type

            struct = Struct(**members)
            if not rets or all(ret.sizeof() != struct.sizeof() for ret in rets):
                rets.add(struct)

        return structs.set_result(type_name, rets)

    def enum(self, type_name):
        if len(self.types.keys()) == 0:
            self.parse_types()

        enums = self.types["DW_TAG_enumeration_type"]
        types = enums.get_types(type_name)

        if types is None:
            return None

        # If the type is a list, it means we have already obtained the enum value
        ret = enums.get_result(type_name)
        if ret is not None:
            return ret

        rets = set()
        for dies in types:
            enum = dict()
            for die in dies.iter_children():
                name = die.attributes["DW_AT_name"].value.decode("utf-8")
                value = die.attributes["DW_AT_const_value"].value
                enum[name] = value

            ret = IntEnum(type_name, enum)

            # Remove duplicates
            if not any(
                all(item[key] == value for key, value in enum.items()) for item in rets
            ):
                rets.add(ret)

        return enums.set_result(type_name, rets)

    def enum_value(self, enum_name):
        if len(self.types.keys()) == 0:
            self.parse_types()

        enums = self.types["DW_TAG_enumerator"]
        types = enums.get_types(enum_name)

        if types is None:
            return None

        ret = enums.get_result(enum_name)
        if ret is not None:
            return ret

        rets = set()
        for dies in types:
            name = dies.attributes["DW_AT_name"].value.decode("utf-8")
            if name == enum_name:
                value = dies.attributes["DW_AT_const_value"].value
                rets.add(value)

        return enums.set_result(enum_name, rets)


class LiefELF:
    def __init__(self, filename):
        self.elf = lief.parse(filename)
        if not self.elf:
            raise BaseException(f"Failed to parse ELF file: {filename}")

        self.symbols = {sym.name.split(".")[0]: sym for sym in self.elf.symbols}
        self.endian = (
            "l"
            if self.elf.abstract.header.endianness == lief.Header.ENDIANNESS.LITTLE
            else "b"
        )
        self.architecture = self.elf.abstract.header.architecture
        self.bits = 64 if self.elf.abstract.header.is_64 else 32

    def get_symbol(self, symbol):
        return self.symbols.get(symbol)

    def read_symbol(
        self, symbol, struct: Construct = None
    ) -> Tuple[lief.Symbol, memoryview]:
        sym = self.get_symbol(symbol)
        if sym is None:
            return None

        data = self.read_from(sym.value, sym.size)
        if struct:
            data = struct.parse(data)
        return sym, data

    def read_from(self, addr, len=1) -> memoryview:
        for section in self.elf.sections:
            if section.type == lief.ELF.Section.TYPE.PROGBITS:
                off = addr - section.virtual_address
                if (
                    section.virtual_address
                    <= addr
                    < section.virtual_address + section.size
                    and section.size - off >= len
                ):
                    return section.content[off : off + len]

        for segment in self.elf.segments:
            if segment.type == lief.ELF.Segment.TYPE.LOAD:
                off = addr - segment.virtual_address
                if (
                    segment.virtual_address
                    <= addr
                    < segment.virtual_address + segment.virtual_size
                    and segment.virtual_size - off >= len
                ):
                    return segment.content[off : off + len]

        return None

    def read_string(self, addr) -> str:
        """Read const string from ELF file"""
        output = b""
        while True:
            c = self.read_from(addr, 1)
            if c == b"\0":
                break

            output += c.tobytes()
            addr += 1

        return output.decode("utf-8")

    def get_inttype(self) -> Construct:
        return {
            "32l": Int32sl,
            "32b": Int32sb,
            "64l": Int64sl,
            "64b": Int64sb,
        }.get(f"{self.bits}{self.endian}", Int32sl)

    def get_pointer_type(self) -> Construct:
        return {
            "32l": Int32ul,
            "32b": Int32ub,
            "64l": Int64ul,
            "64b": Int64ub,
        }.get(f"{self.bits}{self.endian}", Int32ul)

    def get_pointer_size(self):
        return 8 if self.elf.abstract.header.is_64 else 4


class AngrElf:
    def __init__(self, elf: str):
        try:
            self.angr = importlib.import_module("angr")
            self.capstone = importlib.import_module("capstone")
        except Exception as e:
            print(
                f"Error:{e}.\nPlease execute the following command to install dependencies:"
            )
            print("pip install angr capstone")
            sys.exit(1)

        with open(elf, "rb") as f:
            hash = hashlib.md5(f.read()).hexdigest()

        project_cache_path = f"{hash}.project"
        cfg_cache_path = f"{hash}.cfg"

        self.project = self.load_save_cache(
            project_cache_path, lambda: self.angr.Project(elf, auto_load_libs=False)
        )
        self.cfg = self.load_save_cache(
            cfg_cache_path, lambda: self.project.analyses.CFGFast()
        )

        for func in self.cfg.kb.functions.values():
            func._project = self.project

    def load_save_cache(self, file_path, func):
        cache_path = f"{file_path}.pkl"
        if os.path.exists(cache_path):
            with open(cache_path, "rb") as f:
                print(f"load cache {cache_path}")
                return pickle.load(f)
        else:
            print(f"not found cache {cache_path}")
            obj = func()
            with open(cache_path, "wb") as f:
                print(f"create cache {cache_path}")
                pickle.dump(obj, f)
            return obj

    def sym2addr(self, sym: str):
        sym = self.cfg.kb.functions.floor_func(sym)
        if sym:
            return sym.addr
        else:
            return None

    def addr2sym(self, addr: int):
        sym = self.cfg.kb.functions.floor_func(addr)
        if sym:
            return sym.name
        else:
            return f"0x{addr:08x}"

    def addr2func(self, addr: int):
        addr = addr
        sym = self.cfg.kb.functions.floor_func(addr)
        if sym:
            return self.cfg.kb.functions[sym.addr]
        else:
            return None

    def sym2func(self, sym: str):
        addr = self.sym2addr(sym)
        if addr:
            return self.cfg.kb.functions[addr]
        else:
            return None

    def addr2block(self, addr: int):
        addr = addr
        sym = self.cfg.kb.functions.floor_func(addr)
        if sym:
            func = self.cfg.kb.functions[sym.addr]
            for block in func._addr_to_block_node.values():
                if addr >= block.addr and addr < block.addr + block.size:
                    return block
        else:
            return None

    def print_block(self, block):
        print(f"basic block: {hex(block.addr)}, size: {block.size}")
        for insn in block.capstone.insns:
            print(f"    {hex(insn.address)}: {insn.mnemonic} {insn.op_str}")
            if insn.group(self.capstone.CS_GRP_CALL):
                print(f" call {hex(insn.operands[0].imm)}")
            elif insn.group(self.capstone.CS_GRP_RET):
                print(f" return {hex(insn.operands[0].imm)}")
            elif insn.group(self.capstone.CS_GRP_INT):
                print(f" interrupt {hex(insn.operands[0].imm)}")
            elif insn.group(self.capstone.CS_GRP_BRANCH_RELATIVE):
                target_addr = insn.operands[0].imm
                target_function = self.cfg.kb.functions.floor_func(target_addr)
                print(
                    f" conditional jump {hex(target_addr)} (target function: {target_function.name})"
                )
            elif insn.group(self.capstone.CS_GRP_JUMP):
                sub_function = self.cfg.kb.functions.floor_func(insn.operands[0].imm)
                print(
                    f" unconditional jump {hex(insn.operands[0].imm)} (target function: {sub_function.name})"
                )
        sys.stdout.flush()

    def print_function(self, function):
        print(f"function: {function.name} {function.addr} {function.size}")
        for block in function.blocks:
            self.print_block(block)

    def get_block_next_address(self, block):
        for insn in block.capstone.insns:
            if insn.group(self.capstone.CS_GRP_BRANCH_RELATIVE):
                try:
                    addr = insn.operands[0].imm
                    func = self.addr2func(addr)
                    if func:
                        next_block = func.get_block(addr)
                        return next_block.addr
                except Exception as e:
                    log.error(
                        f"get_block_next_address failed: addr: {block.addr:#08x} {e}"
                    )
                    return None
        return None
