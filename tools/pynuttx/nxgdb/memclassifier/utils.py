############################################################################
# tools/pynuttx/nxgdb/memclassifier/utils.py
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


def backtrace_function_name_judge(funcList, cmp):
    def judger(mb):
        for funcname, filename in mb.backtrace():
            if any(cmp(funcname, func) for func in funcList):
                return True
        return False

    return judger


def backtrace_function_name_startswith(funcList):
    return backtrace_function_name_judge(funcList, lambda x, y: x.startswith(y))


def backtrace_function_name_equal(funcList):
    return backtrace_function_name_judge(funcList, lambda x, y: x == y)


def backtrace_function_name_contain(funcList):
    return backtrace_function_name_judge(funcList, lambda x, y: y in x)
