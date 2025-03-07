############################################################################
# tools/pynuttx/nxgdb/thread.py
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
from enum import Enum, auto
from typing import Union

import gdb
from nxelf.elf import LiefELF
from nxreg.register import Registers, g_reg_table

from . import utils
from .stack import Stack

UINT16_MAX = 0xFFFF
TSTATE_TASK_RUNNING = utils.get_symbol_value("TSTATE_TASK_RUNNING")
CONFIG_SMP_NCPUS = utils.get_symbol_value("CONFIG_SMP_NCPUS") or 1


def is_thread_command_supported():
    # Check if the native thread command is available by compare the number of threads.
    # It should have at least CONFIG_SMP_NCPUS of idle threads.
    return len(gdb.selected_inferior().threads()) > CONFIG_SMP_NCPUS


class NxRegisters:
    saved_regs = None

    def __init__(self):
        elf = gdb.objfiles()[0]
        elf = LiefELF(elf.filename)

        def read_memory(addr, size):
            return bytes(gdb.selected_inferior().read_memory(addr, size))

        arch = gdb.selected_inferior().architecture()
        mapped_arch_name = self.map_gdbarch_name(arch.name())
        if mapped_arch_name not in g_reg_table:
            raise ValueError(
                f"Architecture {mapped_arch_name} is not found in g_reg_table.\n"
            )

        self.registers = Registers(elf, arch=mapped_arch_name, readmem=read_memory)

    def map_gdbarch_name(self, arch: str):
        for arch_key, arch_info in g_reg_table.items():
            if arch in arch_info["architecture"]:
                return arch_key
        return None

    def load(self, regs: Union[int, gdb.Value] = None):
        """Load registers from context register address"""
        self.registers.load(regs)
        for reg in self.registers:
            gdb.execute(f"set ${reg.name} = {reg.value}")

    def switch(self, pid):
        """Switch to the specified thread"""
        tcb = utils.get_tcb(pid)
        if not tcb:
            gdb.write(f"Thread {pid} not found\n")
            return

        if tcb["task_state"] == TSTATE_TASK_RUNNING:
            # If the thread is running, then register is not in context but saved temporarily
            self.restore()
            return

        # Save current if this is the running thread, which is the case we never saved it before
        if not self.saved_regs:
            self.save()

        self.load(tcb["xcp"]["regs"])

    def save(self):
        """Save current registers"""
        if NxRegisters.saved_regs:
            # Already saved
            return

        registers = {}
        frame = gdb.newest_frame()
        for reg in self.registers:
            value = frame.read_register(reg.name)
            registers[reg.name] = value

        NxRegisters.saved_regs = registers

    def restore(self):
        if not NxRegisters.saved_regs:
            return

        for name, value in NxRegisters.saved_regs.items():
            gdb.execute(f"set ${name}={int(value)}")

        NxRegisters.saved_regs = None


g_registers = NxRegisters()


class SetRegs(gdb.Command):
    """Load registers from TCB context memory address.
    Usage: setregs [regs]

    Etc: setregs
         setregs tcb->xcp.regs
         setregs g_pidhash[0]->xcp.regs

    Default to load from g_running_tasks if no args are provided.
    If the memory address is NULL, it will not set registers.
    """

    def __init__(self):
        super().__init__("setregs", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        parser = argparse.ArgumentParser(
            description="Set registers to the specified values"
        )

        parser.add_argument(
            "regs",
            nargs="?",
            default="",
            help="The memory address to load register values, use g_running_tasks.xcp.regs if not specified",
        )

        try:
            args = parser.parse_args(gdb.string_to_argv(arg))
        except SystemExit:
            return

        if args and args.regs:
            regs = gdb.parse_and_eval(f"{args.regs}").cast(
                utils.lookup_type("char").pointer()
            )
        else:
            try:
                current_regs = gdb.parse_and_eval("g_running_tasks[0].xcp.regs")
            except gdb.error as e:
                gdb.write(f"Failed to parse running tasks: {e}\n")
                return

            regs = current_regs.cast(utils.lookup_type("char").pointer())

        if regs == 0:
            gdb.write("regs is NULL\n")
            return

        g_registers.save()
        g_registers.load(regs)


class Nxinfothreads(gdb.Command):
    """Display information of all threads"""

    def __init__(self):
        super().__init__("info nxthreads", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        npidhash = gdb.parse_and_eval("g_npidhash")
        pidhash = gdb.parse_and_eval("g_pidhash")
        statenames = gdb.parse_and_eval("g_statenames")

        if utils.is_target_smp():
            gdb.write(
                "%-5s %-4s %-4s %-4s %-21s %-80s %-30s\n"
                % ("Index", "Tid", "Pid", "Cpu", "Thread", "Info", "Frame")
            )
        else:
            gdb.write(
                "%-5s %-4s %-4s %-21s %-80s %-30s\n"
                % ("Index", "Tid", "Pid", "Thread", "Info", "Frame")
            )

        for i, tcb in enumerate(utils.ArrayIterator(pidhash, npidhash)):
            if not tcb:
                continue

            pid = tcb["group"]["tg_pid"]
            tid = tcb["pid"]

            if tcb["task_state"] == gdb.parse_and_eval("TSTATE_TASK_RUNNING"):
                index = f"*{i}"
                pc = utils.get_pc()
            else:
                index = f" {i}"
                pc = utils.get_pc(tcb=tcb)

            thread = f"Thread {hex(tcb)}"

            statename = statenames[tcb["task_state"]].string()
            statename = f'\x1b{"[32;1m" if statename == "Running" else "[33;1m"}{statename}\x1b[m'

            if tcb["task_state"] == gdb.parse_and_eval("TSTATE_WAIT_SEM"):
                mutex = tcb["waitobj"].cast(utils.lookup_type("sem_t").pointer())
                if utils.sem_is_mutex(mutex["flags"]):
                    mutex = tcb["waitobj"].cast(utils.lookup_type("mutex_t").pointer())
                    statename = f"Waiting,Mutex:{mutex['holder']}"

            try:
                """Maybe tcb not have name member, or name is not utf-8"""
                info = (
                    "(Name: \x1b[31;1m%s\x1b[m, State: %s, Priority: %d, Stack: %d)"
                    % (
                        utils.get_task_name(tcb),
                        statename,
                        tcb["sched_priority"],
                        tcb["adj_stack_size"],
                    )
                )
            except gdb.error and UnicodeDecodeError:
                info = "(Name: Not utf-8, State: %s, Priority: %d, Stack: %d)" % (
                    statename,
                    tcb["sched_priority"],
                    tcb["adj_stack_size"],
                )

            line = gdb.find_pc_line(pc)
            if line.symtab:
                func = gdb.execute(f"info symbol {pc} ", to_string=True)
                frame = "\x1b[34;1m0x%x\x1b[\t\x1b[33;1m%s\x1b[m at %s:%d" % (
                    pc,
                    func.split()[0] + "()",
                    line.symtab,
                    line.line,
                )
            else:
                frame = "No symbol with pc"

            if utils.is_target_smp():
                cpu = f"{tcb['cpu']}"
                gdb.write(
                    "%-5s %-4s %-4s %-4s %-21s %-80s %-30s\n"
                    % (index, tid, pid, cpu, thread, info, frame)
                )
            else:
                gdb.write(
                    "%-5s %-4s %-4s %-21s %-80s %-30s\n"
                    % (index, tid, pid, thread, info, frame)
                )


class Nxthread(gdb.Command):
    """Switch to a specified thread"""

    def __init__(self):
        super().__init__("nxthread", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        npidhash = gdb.parse_and_eval("g_npidhash")
        pidhash = gdb.parse_and_eval("g_pidhash")
        arg = args.split(" ")
        arglen = len(arg)

        if arg[0] == "":
            pass
        elif arg[0] == "apply":
            if arglen <= 1:
                gdb.write("Please specify a thread ID list\n")
            elif arglen <= 2:
                gdb.write("Please specify a command following the thread ID list\n")

            elif arg[1] == "all":
                for i, tcb in enumerate(utils.ArrayIterator(pidhash, npidhash)):
                    if tcb == 0:
                        continue
                    try:
                        gdb.write(f"Thread {i} {tcb['name'].string()}\n")
                    except gdb.error and UnicodeDecodeError:
                        gdb.write(f"Thread {i}\n")

                    gdb.execute(f"setregs g_pidhash[{i}]->xcp.regs")
                    cmd_arg = ""
                    for cmd in arg[2:]:
                        cmd_arg += cmd + " "

                    gdb.execute(f"{cmd_arg}\n")
                    g_registers.restore()
            else:
                threadlist = []
                i = 0
                cmd = ""
                for i in range(1, arglen):
                    if arg[i].isnumeric():
                        threadlist.append(int(arg[i]))
                    else:
                        cmd += arg[i] + " "

                if len(threadlist) == 0 or cmd == "":
                    gdb.write("Please specify a thread ID list and command\n")
                else:
                    for i in threadlist:
                        if i >= npidhash:
                            break

                        if pidhash[i] == 0:
                            continue

                        try:
                            gdb.write(f"Thread {i} {pidhash[i]['name'].string()}\n")
                        except gdb.error and UnicodeDecodeError:
                            gdb.write(f"Thread {i}\n")

                        gdb.execute(f"setregs g_pidhash[{i}]->xcp.regs")
                        gdb.execute(f"{cmd}\n")
                        g_registers.restore()

        else:
            if (
                arg[0].isnumeric()
                and int(arg[0]) < npidhash
                and pidhash[int(arg[0])] != 0
            ):
                if pidhash[int(arg[0])]["task_state"] == gdb.parse_and_eval(
                    "TSTATE_TASK_RUNNING"
                ):
                    g_registers.restore()
                else:
                    gdb.execute("setregs g_pidhash[%s]->xcp.regs" % arg[0])
            else:
                gdb.write(f"Invalid thread id {arg[0]}\n")


class Nxcontinue(gdb.Command):
    """Restore the registers and continue the execution"""

    def __init__(self):
        super().__init__("nxcontinue", gdb.COMMAND_USER)
        if not is_thread_command_supported():
            gdb.execute("define c\n nxcontinue \n end\n")
            gdb.write(
                "\n\x1b[31;1m if use thread command, please don't use 'continue', use 'c' instead !!!\x1b[m\n"
            )

    def invoke(self, args, from_tty):
        g_registers.restore()
        gdb.execute("continue")


class Nxstep(gdb.Command):
    """Restore the registers and step the execution"""

    def __init__(self):
        super().__init__("nxstep", gdb.COMMAND_USER)
        if not is_thread_command_supported():
            gdb.execute("define s\n nxstep \n end\n")
            gdb.write(
                "\x1b[31;1m if use thread command, please don't use 'step', use 's' instead !!!\x1b[m\n"
            )

    def invoke(self, args, from_tty):
        g_registers.restore()
        gdb.execute("step")


class TaskType(Enum):
    TASK = 0
    PTHREAD = 1
    KTHREAD = 2


class TaskSchedPolicy(Enum):
    FIFO = 0
    RR = 1
    SPORADIC = 2


class TaskState(Enum):
    Invalid = 0
    Waiting_Unlock = auto()
    Ready = auto()
    if utils.get_symbol_value("CONFIG_SMP"):
        Assigned = auto()
    Running = auto()
    Inactive = auto()
    Waiting_Semaphore = auto()
    Waiting_Signal = auto()
    if not utils.get_symbol_value(
        "CONFIG_DISABLE_MQUEUE"
    ) or not utils.get_symbol_value("CONFIG_DISABLE_MQUEUE_SYSV"):
        Waiting_MQEmpty = auto()
        Waiting_MQFull = auto()
    if utils.get_symbol_value("CONFIG_PAGING"):
        Waiting_PagingFill = auto()
    if utils.get_symbol_value("CONFIG_SIG_SIGSTOP_ACTION"):
        Stopped = auto()


class Ps(gdb.Command):
    def __init__(self):
        super().__init__("ps", gdb.COMMAND_USER)
        self._fmt_wxl = "{0: <{width}}"
        # By default we align to the right, whcih respects the nuttx foramt
        self._fmt_wx = "{0: >{width}}"

    def parse_and_show_info(self, tcb):
        def get_macro(x):
            return utils.get_symbol_value(x)

        def eval2str(cls, x):
            return cls(int(x)).name

        def cast2ptr(x, t):
            return x.cast(utils.lookup_type(t).pointer())

        pid = int(tcb["pid"])
        group = int(tcb["group"]["tg_pid"])
        priority = int(tcb["sched_priority"])

        policy = eval2str(
            TaskSchedPolicy,
            (tcb["flags"] & get_macro("TCB_FLAG_POLICY_MASK"))
            >> get_macro("TCB_FLAG_POLICY_SHIFT"),
        )

        task_type = eval2str(
            TaskType,
            (tcb["flags"] & get_macro("TCB_FLAG_TTYPE_MASK"))
            >> get_macro("TCB_FLAG_TTYPE_SHIFT"),
        )

        npx = "P" if (tcb["flags"] & get_macro("TCB_FLAG_EXIT_PROCESSING")) else "-"

        waiter = (
            str(int(cast2ptr(tcb["waitobj"], "mutex_t")["holder"]))
            if tcb["waitobj"]
            and utils.sem_is_mutex(cast2ptr(tcb["waitobj"], "sem_t")["flags"])
            else ""
        )
        state_and_event = eval2str(TaskState, (tcb["task_state"])) + (
            "@Mutex_Holder: " + waiter if waiter else ""
        )
        state_and_event = state_and_event.split("_")

        # Append a null str here so we don't need to worry
        # about the number of elements as we only want the first two
        state, event = (
            state_and_event if len(state_and_event) > 1 else state_and_event + [""]
        )

        sigmask = "{0:#0{1}x}".format(
            sum(
                int(tcb["sigprocmask"]["_elem"][i] << i)
                for i in range(get_macro("_SIGSET_NELEM"))
            ),
            get_macro("_SIGSET_NELEM") * 8 + 2,
        )[
            2:
        ]  # exclude "0x"

        st = Stack(
            utils.get_task_name(tcb),
            hex(tcb["entry"]["pthread"]),  # should use main?
            int(tcb["stack_base_ptr"]),
            int(tcb["stack_alloc_ptr"]),
            int(tcb["adj_stack_size"]),
            utils.get_sp(tcb if tcb["task_state"] != TSTATE_TASK_RUNNING else None),
            4,
        )

        stacksz = st._stack_size
        used = st.max_usage()
        filled = "{0:.2%}".format(st.max_usage() / st._stack_size)

        cpu = int(tcb["cpu"]) if get_macro("CONFIG_SMP") else 0

        # For a task we need to display its cmdline arguments, while for a thread we display
        # pointers to its entry and argument
        cmd = ""
        name = utils.get_task_name(tcb)

        if int(tcb["flags"] & get_macro("TCB_FLAG_TTYPE_MASK")) == int(
            get_macro("TCB_FLAG_TTYPE_PTHREAD")
        ):
            entry = tcb["entry"]["main"]
            ptcb = cast2ptr(tcb, "struct pthread_tcb_s")
            arg = ptcb["arg"]
            cmd = " ".join((name, hex(entry), hex(arg)))
        elif tcb["pid"] < get_macro("CONFIG_SMP_NCPUS"):
            # This must be the Idle Tasks, hence we just get its name
            cmd = name
        else:
            # For tasks other than pthreads, hence need to get its command line
            # arguments from
            argv = (
                tcb["stack_alloc_ptr"]
                + cast2ptr(tcb["stack_alloc_ptr"], "struct tls_info_s")["tl_size"]
            )
            args = []
            parg = argv.cast(gdb.lookup_type("char").pointer().pointer()) + 1
            while parg.dereference():
                args.append(parg.dereference().string())
                parg += 1

            cmd = " ".join([name] + args)

        if not utils.get_symbol_value("CONFIG_SCHED_CPULOAD_NONE"):
            load = "{0:.1%}".format(
                int(tcb["ticks"]) / int(gdb.parse_and_eval("g_cpuload_total"))
            )
        else:
            load = "Dis."

        gdb.write(
            " ".join(
                (
                    self._fmt_wx.format(pid, width=5),
                    self._fmt_wx.format(group, width=5),
                    self._fmt_wx.format(cpu, width=3),
                    self._fmt_wx.format(priority, width=3),
                    self._fmt_wxl.format(policy, width=8),
                    self._fmt_wxl.format(task_type, width=7),
                    self._fmt_wx.format(npx, width=3),
                    self._fmt_wxl.format(state, width=8),
                    self._fmt_wxl.format(event, width=9),
                    self._fmt_wxl.format(sigmask, width=8),
                    self._fmt_wx.format(stacksz, width=7),
                    self._fmt_wx.format(used, width=7),
                    self._fmt_wx.format(filled, width=6),
                    self._fmt_wx.format(load, width=6),
                    cmd,
                )
            )
        )
        gdb.write("\n")

    def invoke(self, args, from_tty):
        gdb.write(
            " ".join(
                (
                    self._fmt_wx.format("PID", width=5),
                    self._fmt_wx.format("GROUP", width=5),
                    self._fmt_wx.format("CPU", width=3),
                    self._fmt_wx.format("PRI", width=3),
                    self._fmt_wxl.format("POLICY", width=8),
                    self._fmt_wxl.format("TYPE", width=7),
                    self._fmt_wx.format("NPX", width=3),
                    self._fmt_wxl.format("STATE", width=8),
                    self._fmt_wxl.format("EVENT", width=9),
                    self._fmt_wxl.format(
                        "SIGMASK", width=utils.get_symbol_value("_SIGSET_NELEM") * 8
                    ),
                    self._fmt_wx.format("STACK", width=7),
                    self._fmt_wx.format("USED", width=7),
                    self._fmt_wx.format("FILLED", width=3),
                    self._fmt_wx.format("LOAD", width=6),
                    "COMMAND",
                )
            )
        )
        gdb.write("\n")

        for tcb in utils.get_tcbs():
            self.parse_and_show_info(tcb)
