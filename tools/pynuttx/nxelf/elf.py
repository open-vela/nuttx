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

import logging
import time
from enum import IntEnum

from .macros import Macro

try:
    import construct
    import cxxfilt
    from elftools.elf.elffile import ELFFile
    from elftools.elf.sections import SymbolTableSection
except ModuleNotFoundError:
    print("Please execute the following command to install dependencies:")
    print("pip install construct pyelftools cxxfilt")
    exit(1)

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger(__name__)


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
        array = construct.Array(nums, item_type)
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
                return construct.Int32ul
            elif self.info["bitwides"] == 64:
                return construct.Int64ul
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
                1: construct.Int8ul,
                2: construct.Int16ul,
                4: construct.Int32ul,
                8: construct.Int64ul,
            }
            signed_map = {
                1: construct.Int8sl,
                2: construct.Int16sl,
                4: construct.Int32sl,
                8: construct.Int64sl,
            }
            double_map = {
                2: construct.Float16l,
                4: construct.Float32l,
                8: construct.Float64l,
            }
            if "unsigned" in name:
                return unsigned_map.get(size, None)
            elif "double" in name or "float" in name:
                return double_map.get(size, None)
            elif "_Bool" in name:
                return construct.Int8ul
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

            struct = construct.Struct(**members)
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
