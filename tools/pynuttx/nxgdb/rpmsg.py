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

    def is_rpmsg_transport(self, rdev, transport):
        if transport == "virtio":
            tx_payload = "rpmsg_virtio_get_tx_payload_buffer"
        elif transport == "port":
            tx_payload = "rpmsg_port_get_tx_payload_buffer"
        else:
            return False

        symbol = gdb.lookup_symbol(tx_payload)
        if symbol[0] is None:
            return False

        real_addr = symbol[0].value().address
        ops_addr = int(utils.Value(rdev["ops"]["get_tx_payload_buffer"])) & ~1
        return real_addr == ops_addr

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
        if not self.is_rpmsg_transport(rdev, "virtio"):
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
        try:
            self.dump_virtqueue(rvdev["svq"])
            self.dump_virtqueue(rvdev["rvq"])
        except gdb.error as e:
            gdb.write(f"Error when dump virtqueues: {e}\n")

        gdb.write("\n")

    def rpmsg_get_ept_from_addr(self, rdev, addr):
        endpoints = rdev["endpoints"]
        for endpoint in NxList(endpoints, "struct rpmsg_endpoint", "node"):
            ept_addr = endpoint["addr"]
            if ept_addr == addr:
                return endpoint
        return None

    def rpmsg_port_node_to_buf(self, queue, node):
        node_offset = int(node) - int(queue["node"])
        buf_addr = queue["buf"] + (node_offset * queue["len"])
        return buf_addr

    def dump_rpmsg_port_buffer(self, rdev, queue, label):
        buffer_list = []
        head = queue["ready"]["head"]

        for node in NxList(head):
            hdr = self.rpmsg_port_node_to_buf(queue, node)
            if not hdr or not hdr["buf"]:
                continue

            rphdr = hdr["buf"].cast(utils.lookup_type("struct rpmsg_hdr").pointer())
            ept_addr = rphdr["dst"] if label == "RX" else rphdr["src"]
            ept = self.rpmsg_get_ept_from_addr(rdev, ept_addr)
            if ept:
                ept_name = ept["name"].string().split("\0", 1)[0]
                buffer_list.append(f"{label} buffer:{rphdr} held by {ept_name}\n")

        return buffer_list

    def dump_rpmsg_port(self, rdev):
        if not self.is_rpmsg_transport(rdev, "port"):
            return

        port = rdev.cast(utils.lookup_type("struct rpmsg_port_s").pointer())

        gdb.write(f"rxq nused:{port['rxq']['ready']['num']}\n")
        gdb.write(f"rxq navail:{port['rxq']['free']['num']}\n")
        gdb.write(f"txq nused:{port['txq']['ready']['num']}\n")
        gdb.write(f"txq navail:{port['txq']['free']['num']}\n")

        rx_buffers = self.dump_rpmsg_port_buffer(rdev, port["rxq"], "RX")
        tx_buffers = self.dump_rpmsg_port_buffer(rdev, port["txq"], "TX")
        for buffer in rx_buffers + tx_buffers:
            gdb.write(buffer + "\n")

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
            rdev = utils.Value(int(rpmsg) + utils.sizeof("struct rpmsg_s"))
            rdev = rdev.cast(utils.lookup_type("struct rpmsg_device").pointer())
            gdb.write(f"Rpmsg Device: rpmsg:{rpmsg} rdev:{rdev}\n")
            if not transport_only:
                self.dump_rdev(rdev)
            self.dump_rpmsg_virtio(rdev)
            self.dump_rpmsg_port(rdev)

    def invoke(self, args, from_tty):
        if not (args := self.parse_args(args)):
            return

        if not args.transport_only:
            self.dump_rpmsg_cb()
        self.dump_rpmsg(args.transport_only)
