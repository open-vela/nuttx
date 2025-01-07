############################################################################
# tools/gdb/nuttxgdb/memclassify.py
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
import sys

import gdb

from . import memdump, mm, utils


class Classifier:
    def __init__(self, info, category_name=""):
        self.stats = []
        self.judgers = []
        self.class_name = category_name

        for key, sub_info in info.items():
            if callable(sub_info):
                self.judgers.append(sub_info)
                sub_classifier = None
            else:  # sub_info is a dict
                self.judgers.append(sub_info["judger"])
                sub_classifier = (
                    Classifier(
                        sub_info["subcategories"], "_".join([category_name, key])
                    )
                    if "subcategories" in sub_info
                    else None
                )

            self.stats.append(MemoryCategory(key, sub_classifier))

    def __call__(self, mem_blocks):
        for mb in mem_blocks:
            cat = next(
                (idx for idx, method in enumerate(self.judgers) if method(mb)),
                None,
            )

            if cat is None:
                cat = len(self.stats)
                self.stats.append(MemoryCategory(category="unknown"))
                self.judgers.append(lambda x: True)
            self.stats[cat].append(mb)
        for stat in self.stats:
            stat.classify()
        return self.stats


class MemBlock:
    def __init__(self):
        pass

    def total_size(self):
        pass

    def total_size_without_overhead(self):
        pass

    def count(self):
        pass

    def block_size(self):
        pass

    def overhead_size(self):
        pass

    def pid(self):
        pass

    def backtrace(self):
        pass

    def print(self, f=sys.stdout):
        def pprint(*args):
            print(*args, file=f)

        pprint(
            f"pid: {self.pid()}, size: {self.total_size()} = {self.block_size()} x {self.count()}"
        )

        for name, pos in self.backtrace():
            pprint(f"{name:<50} {pos}")
        pprint("")


class MemoryCategory:
    WIDTH = 88

    def __init__(self, category="total", classifier=None):
        self.category = category
        self.mem_blocks = []
        self.classifier = classifier
        self.children = []

    def append(self, mem_block):
        self.mem_blocks.append(mem_block)

    def extend(self, mem_blocks):
        self.mem_blocks.extend(mem_blocks)

    def classify(self):
        if self.classifier is None:
            return
        self.children = self.classifier(self.mem_blocks)

    def summarize(self):
        for child in self.children:
            child.summarize()

        self.total_mem_size = sum(mb.total_size() for mb in self.mem_blocks)
        self.no_overhead_size = sum(
            mb.total_size_without_overhead() for mb in self.mem_blocks
        )
        self.mem_block_unique_cnt = len(self.mem_blocks)
        self.mem_block_cnt = sum(mb.count() for mb in self.mem_blocks)

    def print_statistics(self, categorys=[]):

        if not self.children:
            # no children means no categories, just print nothing
            return

        categorys.append(self.category)

        def formatter(
            name, total_mem_size, total_mem_cnt1, total_mem_cnt2, mem_rm_bt=0
        ):
            print(
                f"{name:<15}| {total_mem_size:<15}| {total_mem_cnt1:<15}| {total_mem_cnt2:<15}| {mem_rm_bt:<15}"
            )

        title = f"{'.'.join(categorys)} Mem Statistics".center(self.WIDTH, "-")
        print(title)
        formatter(
            "category",
            "total mem size",
            "memblk uniq cnt",
            "memblk tot cnt",
            "mem without overhead",
        )
        print("-" * self.WIDTH)
        for child in self.children:
            formatter(
                child.category,
                child.total_mem_size,
                child.mem_block_unique_cnt,
                child.mem_block_cnt,
                child.no_overhead_size,
            )

        formatter(
            "total",
            self.total_mem_size,
            self.mem_block_unique_cnt,
            self.mem_block_cnt,
            self.no_overhead_size,
        )
        print("-" * self.WIDTH)
        for child in self.children:
            child.print_statistics()
        categorys.pop()

    def collect_piedata(self, title_path=[]):
        title_path.append(self.category)
        res = []
        if self.children:
            res.append(
                (
                    ".".join(title_path),
                    [child.category for child in self.children],
                    [child.no_overhead_size for child in self.children],
                )
            )
            for child in self.children:
                res.extend(child.collect_piedata(title_path))
        title_path.pop()
        return res

    def dump_category(self, output_dir, parentCat=None):
        os.makedirs(output_dir, exist_ok=True)

        for child in self.children:
            arr = sorted(child.mem_blocks, key=lambda x: x.total_size(), reverse=True)
            outputfile = (
                parentCat + f"_{child.category}" if parentCat else child.category
            )
            with open(os.path.join(output_dir, outputfile + ".bt"), "w") as f:
                print(f"{child.category}, count={len(child.mem_blocks)}", file=f)
                for mb in arr:
                    mb.print(f)
            child.dump_category(output_dir, outputfile)


def draw_pie(stat):
    datasets = stat.collect_piedata()
    plt = utils.import_check(
        "matplotlib.pyplot", errmsg="Please pip install matplotlib"
    )
    if plt is None:
        print("matplotlib is not installed")
        return

    num_plots = len(datasets)
    fig, axs = plt.subplots(
        1, num_plots, figsize=(12, 6), subplot_kw=dict(aspect="equal")
    )
    if num_plots == 1:
        axs = [axs]

    for i, (title, labels, sizes) in enumerate(datasets):
        temp = [x for x in filter(lambda x: x[1], zip(labels, sizes))]
        if len(temp) > 0:
            labels, sizes = zip(*temp)
        else:
            continue
        axs[i].pie(sizes, labels=labels, autopct="%1.1f%%")
        axs[i].set_title(title)
    plt.tight_layout()
    plt.show()


class MemBlockCoredump(MemBlock):
    def __init__(self, node, cnt):
        super().__init__()
        self.mem_node = node
        self.cnt = cnt
        # deal with call_stack
        self.call_stack = []
        for addr, func, file in utils.Backtrace(node.backtrace).backtrace:
            func = func.strip("<>")
            if func.find("+"):  # remove the offset affter '+'
                func = func[: func.find("+")]
            self.call_stack.append((func, file))

    def total_size(self):
        return self.mem_node.nodesize * self.cnt

    def total_size_without_overhead(self):
        return (self.mem_node.nodesize - self.mem_node.overhead) * self.cnt

    def count(self):
        return self.cnt

    def block_size(self):
        return self.mem_node.nodesize

    def overhead_size(self):
        return self.mem_node.overhead

    def pid(self):
        return self.mem_node.pid

    def backtrace(self):
        return self.call_stack


class MMClassify(gdb.Command):
    """classify memory by callstack"""

    def __init__(self):
        super().__init__("mm classify", gdb.COMMAND_USER)
        utils.alias("memclassify", "mm classify")

    def parse_args(self, argv):
        parser = argparse.ArgumentParser(description="Memory Classify")
        parser.add_argument(
            "-o",
            "--output-dir",
            default="memclass.output",
            help="Specify the directory to save the the call stack files after categorization",
        )

        parser.add_argument(
            "-p", "--pid", type=int, default=None, help="Thread PID, -1 for mempool"
        )

        parser.add_argument(
            "-c",
            "--classifier-file",
            default="default",
            help="Specify the config file. Default is 'script_path/default.py'",
        )

        parser.add_argument(
            "-l",
            "--log",
            default=None,
            help="Specify the memdump log file.",
        )

        try:
            args = parser.parse_args(argv)
        except SystemExit:
            return False

        return args

    def invoke(self, arg: str, from_tty: bool) -> None:
        if not (args := self.parse_args(gdb.string_to_argv(arg))):
            print("memoryclassify: parse args error")
            return

        def import_classify_config(classifier_file):
            classifier_dir = os.path.join(os.path.dirname(__file__), "memclassifier")
            sys.path.append(classifier_dir)
            return utils.import_reload(
                classifier_file,
                errmsg=f"Please provide {classifier_file}.py in {classifier_dir}\n",
            )

        classify_config = import_classify_config(args.classifier_file)
        if not classify_config:
            return

        memblocks = []
        if args.log:  # get mem block from memdump log
            memblocks.extend(
                MemBlockCoredump(node, 1)
                for node in memdump.parse_memdump_log(args.log)
            )
        else:
            if not mm.CONFIG_MM_BACKTRACE or mm.CONFIG_MM_BACKTRACE < 8:
                print("memoryclassify: no backtrace")
                return

            filters = {
                "pid": args.pid,
                "nodesize": None,
                "used": None,
                "free": None,
                "seqmin": None,
                "seqmax": None,
                "orphan": None,
                "no_heap": None,
                "no_pool": None,
                "no_pid": None,
            }

            memblocks.extend(
                MemBlockCoredump(node, len(nodes))
                for node, nodes in memdump.group_nodes(
                    memdump.dump_nodes(mm.get_heaps(), filters)
                ).items()
            )

        stat = MemoryCategory(
            "total", Classifier(getattr(classify_config, "categories"))
        )
        stat.extend(memblocks)
        stat.classify()
        stat.summarize()
        stat.print_statistics()
        stat.dump_category(args.output_dir)
        draw_pie(stat)
