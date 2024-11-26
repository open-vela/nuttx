#! /bin/bash
############################################################################
# tools/pynuttx/tests/run_tests.sh
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

set -xeo pipefail

GDB_TOOLS=$(dirname $0)/../
NUTTX_EXE=$(dirname $0)/../../../nuttx

# The default target
elffile=${NUTTX_EXE}
use_qemu=true

qemu_gdb_srv_pid=

GDB_SOCKET_NAME=

# Pls uncomment this for local testing

# start_qemu_gdb_srv()
# {
#     qemu-system-arm \
#     -M mps3-an547 \
#     -nographic \
#     -kernel ${elffile} \
#     -chardev socket,path=$GDB_SOCKET_NAME,server=on,wait=off,id=gdb0 \
#     -gdb chardev:gdb0 \
#     -S & 1>/dev/null 2>&1

#     qemu_gdb_srv_pid=$!

#     if ! ps -p $qemu_gdb_srv_pid > /dev/null; then
#         echo "Failed to start qemu due to: $?"
#         exit 1
#     fi

#     while [ ! -e "$GDB_SOCKET_NAME" ]
#     do
#         echo "Waiting for $GDB_SOCKET_NAME to exist..."
#         sleep 0.5 # Wait for 0.5 second before checking again
#     done
# }

usage() {
  echo "USAGE: ${0} [options]"
  echo ""
  echo "Options:"
  echo "-h"
  echo "-e the target ELF file, an absolute path should be given. By default, we will look for one in nuttx/"
  echo "-t path to the gdb scritps, by default we will look for the relative path to nuttx/tools/gdb"
  echo "-s if the target ELF is the nuttx simulator"
  echo "-c the socket to connect to"

  exit $@
}

while [ ! -z "$1" ]; do
  case "$1" in
  -h )
    usage 0
    ;;
  -e )
    shift
    elffile=$1
    ;;
  -t )
    shift
    GDB_TOOLS=$1
    ;;
  -s )
    use_qemu=
    ;;
  -c )
    shift
    GDB_SOCKET_NAME=$1
    ;;
  * )
    break
    ;;
  esac
  shift
done

if ! [ -f ${elffile} ]; then
    echo "Failed to find the target ELF file"
    exit 1
fi

if ! [ -d ${GDB_TOOLS} ]; then
    echo "The required GDB tools not exist"
    exit 1
fi

EXTRA_CMD=
if [ -z "$use_qemu" ]; then
    EXTRA_CMD="r"
else
    EXTRA_CMD="target remote $GDB_SOCKET_NAME"
fi

if ! [ -z "$use_qemu" ]; then
    # Pls uncomment this for local testing
    # start_qemu_gdb_srv
    echo "Using the qemu as the testing platform"
fi

echo "##############################"
# figure out where we are running at?

echo `whoami`
echo `id`
echo `ifconfig`
echo `which python`
echo $(realpath `which python`)
python -c "import sys; print(sys.version); print(sys.path)"

echo "##############################"


# Test requires running the ELF without Qemu
gdb-multiarch \
-batch \
${elffile} \
-return-child-result \
-ex="b up_idle" \
-ex="${EXTRA_CMD}" \
-ex="c" \
-ex="python import sys; print(sys.version); print(sys.executable); print(sys.path)" \
-ex="source ${GDB_TOOLS}/gdbinit.py" \
-ex="source ${GDB_TOOLS}/tests/runner.py" \

result=$?

# Don't fail with an error code, all we need is
# to proceed Qemu without letting it hang.
# A bit tricky here, we attach the gdb again
# and quit
yes | gdb-multiarch \
-batch \
${elffile} \
-ex="${EXTRA_CMD}" \
-ex="q" || true

# After everything is done, don't bother just kill the Qemu process
if ! [ -z $qemu_gdb_srv_pid ]; then
    kill -9 $qemu_gdb_srv_pid
fi

if [ -f $GDB_SOCKET_NAME ]; then
    unlink $GDB_SOCKET_NAME
fi

exit $result
