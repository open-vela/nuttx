############################################################################
# tools/pynuttx/nxgdb/memclassifier/size.py
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

categories = {
    "l0-100": lambda mb: mb.block_size() <= 100,
    "l100-500": lambda mb: mb.block_size() <= 500,
    "l500_1000": lambda mb: mb.block_size() <= 1000,
    "l1000_2000": lambda mb: mb.block_size() <= 2000,
    "l2000_4000": lambda mb: mb.block_size() <= 4000,
    "l4000+": lambda mb: True,
}
