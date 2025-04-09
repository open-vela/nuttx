############################################################################
# tools/pynuttx/nxgdb/profile.py
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

import gdb

from .utils import dont_repeat_decorator, import_check


class Profile(gdb.Command):
    """Profile a gdb command

    Usage: profile <gdb command>
    """

    def __init__(self):
        self.cProfile = import_check(
            "cProfile", errmsg="cProfile module not found, try gdb-multiarch.\n"
        )
        self.pstats = import_check(
            "pstats", errmsg="pstats module not found, try gdb-multiarch.\n"
        )
        if not self.cProfile or not self.pstats:
            return

        super().__init__("profile", gdb.COMMAND_USER)

    @dont_repeat_decorator
    def invoke(self, args, from_tty):
        self.cProfile.run(f"gdb.execute('{args}')", "results.prof")

        stats = self.pstats.Stats("results.prof")
        stats.strip_dirs().sort_stats("cumulative").print_stats(20)
        print(
            "Generate profile file: results.prof, Execute 'snakeviz results.prof' to view the report."
        )


class Time(gdb.Command):
    """Time a gdb command

    Usage: time <gdb command>
    """

    def __init__(self):
        super().__init__("time", gdb.COMMAND_USER)

    @dont_repeat_decorator
    def invoke(self, args, from_tty):
        import time

        start = time.time()
        gdb.execute(args)
        end = time.time()
        gdb.write(f"Time elapsed: {end - start:.6f}s\n")
