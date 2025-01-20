############################################################################
# tools/pynuttx/rpmsg.py
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

import gdb

from . import utils
from .lists import NxList


class RPMsgDump(gdb.Command):
    """Dump rpmsg service"""

    CALLBACK_HEADER = ["rpmsg_cb_s at", "ns_match", "ns_bind"]
    ENDPOINT_HEADER = [
        "endpoint_addr",
        "name",
        "addr",
        "dest_addr",
        "cb",
        "ns_bound_cb",
        "ns_unbind_cb",
    ]
    VQ_HEADER = [
        "vq",
        "name",
        "size",
        "free",
        "queued",
        "desc_head_idx",
        "available_idx",
        "avail.idx",
        "avail.flags",
        "used_cons_idx",
        "used.idx",
        "used.flags",
    ]

    CALLBACK_FORAMTTER = "{:<20} {:<40} {:<40}"
    ENDPOINT_FORMATTER = "{:<20} {:<20} {:<12} {:<12} {:<40} {:<40} {:<40}"
    VQ_FORMATTER = (
        "{:<12} {:<8} {:<5} {:<5} {:<7} {:<14} {:<14} {:<10} {:<12} {:<14} {:<9} {:<11}"
    )

    def __init__(self):
        if utils.get_symbol_value("CONFIG_RPMSG"):
            super(RPMsgDump, self).__init__("rpmsgdump", gdb.COMMAND_USER)

    def parse_args(self, arg):
        parser = argparse.ArgumentParser(description=self.__doc__)
        parser.add_argument(
            "-t",
            "--transport-only",
            action="store_true",
            help="Only dump the rpmsg transport layer",
        )

        try:
            return parser.parse_args(gdb.string_to_argv(arg))
        except SystemExit:
            return

    def dump_virtqueue(self, vq):
        vr = vq["vq_ring"]
        gdb.write(
            self.VQ_FORMATTER.format(
                f"{vq}",
                f"{vq['vq_name'].string()}",
                f"{vq['vq_nentries']}",
                f"{vq['vq_free_cnt']}",
                f"{vq['vq_queued_cnt']}",
                f"{vq['vq_desc_head_idx']}",
                f"{vq['vq_available_idx']}",
                f"{vr['avail']['idx']}",
                f"{vr['avail']['flags']}",
                f"{vq['vq_used_cons_idx']}",
                f"{vr['used']['idx']}",
                f"{vr['used']['flags']}",
            )
            + "\n"
        )

    def dump_rpmsg_virtio(self, rdev):
        real_addr = hex(
            gdb.lookup_symbol("rpmsg_virtio_get_tx_payload_buffer")[0].value().address
        )
        ops_addr = hex(int(utils.Value(rdev["ops"]["get_tx_payload_buffer"])) & ~1)
        if real_addr != ops_addr:
            return

        rvdev = rdev.cast(utils.lookup_type("struct rpmsg_virtio_device").pointer())

        gdb.write(
            f"Rpmsg Virtio Trasport: rvdev:{rvdev} h2r_buf_size:{rvdev['config']['h2r_buf_size']}"
            f"r2h_buf_size:{rvdev['config']['r2h_buf_size']}\n"
        )
        for vbuff in NxList(rvdev["reclaimer"], "struct vbuff_reclaimer_t", "node"):
            gdb.write(f"rvdev reclaimer vbuff:{vbuff} idx:{vbuff['idx']}")

        gdb.write("Rpmsg Virtqueues:\n")
        self.print_headers(self.VQ_HEADER, self.VQ_FORMATTER)
        self.dump_virtqueue(rvdev["svq"])
        self.dump_virtqueue(rvdev["rvq"])

        gdb.write("\n")

    def print_headers(self, headers, formatter):
        gdb.write(formatter.format(*headers) + "\n")
        gdb.write(formatter.format(*["-" * len(header) for header in headers]) + "\n")

    def dump_rdev_epts(self, endpoints_head):
        gdb.write(f"dump_rdev_epts:{endpoints_head}\n")
        self.print_headers(self.ENDPOINT_HEADER, self.ENDPOINT_FORMATTER)

        output = []
        for endpoint in NxList(endpoints_head, "struct rpmsg_endpoint", "node"):
            output.append(
                self.ENDPOINT_FORMATTER.format(
                    f"{endpoint}",
                    f"{endpoint['name'].string()}",
                    f"{endpoint['addr']}",
                    f"{endpoint['dest_addr']}",
                    f"{endpoint['cb']}",
                    f"{endpoint['ns_bound_cb']}",
                    f"{endpoint['ns_unbind_cb']}",
                )
            )

        gdb.write("\n".join(output) + "\n")

    def dump_rdev_bitmap(self, rdev):
        bitmap_values = [hex(bit) for bit in utils.ArrayIterator(rdev["bitmap"])]

        gdb.write(
            f"bitmap:{' '.join(bitmap_values):<20} bitmaplast: {rdev['bitmap']}\n"
        )

    def dump_rdev(self, rdev):
        self.dump_rdev_bitmap(rdev)
        self.dump_rdev_epts(rdev["endpoints"])

    def dump_rpmsg_cb(self):
        gdb.write("Rpmsg Callback:\n")
        self.print_headers(self.CALLBACK_HEADER, self.CALLBACK_FORAMTTER)

        output = []
        for cb in NxList(gdb.parse_and_eval("g_rpmsg_cb"), "struct rpmsg_cb_s", "node"):
            output.append(
                self.CALLBACK_FORAMTTER.format(
                    str(cb), str(cb["ns_match"]), str(cb["ns_bind"])
                )
            )
        gdb.write("\n".join(output) + "\n")

    def dump_rpmsg(self, transport_only):
        for rpmsg in NxList(gdb.parse_and_eval("g_rpmsg"), "struct rpmsg_s", "node"):
            gdb.write(f"Rpmsg Device: rpmsg:{rpmsg} rdev:{rpmsg['rdev']}\n")
            if not transport_only:
                self.dump_rdev(rpmsg["rdev"])
            self.dump_rpmsg_virtio(rpmsg["rdev"])

    def invoke(self, args, from_tty):
        if not (args := self.parse_args(args)):
            return

        if not args.transport_only:
            self.dump_rpmsg_cb()
        self.dump_rpmsg(args.transport_only)
