#!/usr/bin/env python3
# tools/kconfiglib_olddefconfig.py
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
# Resolve a NuttX defconfig into a full .config using kconfiglib
# (olddefconfig semantics), replacing kconfig-frontends' `make olddefconfig`.
#
# Usage: python3 tools/kconfiglib_olddefconfig.py <defconfig>

import os
import sys
from pathlib import Path

TOPDIR = Path(__file__).resolve().parents[1]
VELA_ROOT = TOPDIR.parent

sys.path.insert(
    0,
    str(VELA_ROOT / "prebuilts" / "tools" / "python" / "dist-packages" / "kconfiglib"),
)

import kconfiglib  # noqa: E402

os.environ["APPSDIR"] = str(VELA_ROOT / "apps")
os.environ["APPSBINDIR"] = str(VELA_ROOT / "apps")
os.environ["EXTERNALDIR"] = "dummy"
os.environ["BINDIR"] = str(TOPDIR)
os.environ["KCONFIG_CONFIG"] = str(TOPDIR / ".config")

os.chdir(TOPDIR)

kconf = kconfiglib.Kconfig("Kconfig", warn_to_stderr=False)
kconf.load_config(sys.argv[1])
kconf.write_config()
print("Wrote", os.environ["KCONFIG_CONFIG"])
