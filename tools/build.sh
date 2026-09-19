#!/usr/bin/env bash
# tools/build.sh
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

function cleanup()
{
  # keep the mapping but change to the link since:
  # 1.root can't access mount point created by normal user
  # 2.debugger could find the source code without manual setting
  fusermount -u ${MOUNTDIR}
  rmdir ${MOUNTDIR}
  ln -s ${ROOTDIR} ${MOUNTDIR}
}

function mount_unionfs()
{
  echo -e "Mount command line:"
  echo -e "  unionfs-fuse -o cow ${OUTDIR}=RW:${ROOTDIR}=RO ${MOUNTDIR}"

  rm -f ${MOUNTDIR}
  mkdir -p ${MOUNTDIR}
  unionfs-fuse -o cow ${OUTDIR}=RW:${ROOTDIR}=RO ${MOUNTDIR}
}


function build_board()
{
  echo -e "Build command line:"
  echo -e "  ${TOOLSDIR}/configure.sh -e -S $1"
  echo -e "  make -C ${NUTTXDIR} EXTRAFLAGS="$EXTRA_FLAGS" ${@:2}"
  echo -e "  make -C ${NUTTXDIR} savedefconfig"

  KCONFIG_ARGS="--enable-mconf --disable-nconf --disable-gconf --disable-qconf"
  if [ `uname` == "Darwin" ]; then
    KCONFIG_ARGS+=" --disable-shared --enable-static"
  fi

  if [ ! -f "${ROOTDIR}/prebuilts/kconfig-frontends/bin/kconfig-conf" ] &&
     [ ! -x "$(command -v kconfig-conf)" ]; then
    pushd ${ROOTDIR}/prebuilts/kconfig-frontends
    ./configure --prefix=${ROOTDIR}/prebuilts/kconfig-frontends ${KCONFIG_ARGS} 1>/dev/null
    touch aclocal.m4 Makefile.in
    make install 1>/dev/null
    popd
  fi
  export PATH=${ROOTDIR}/prebuilts/kconfig-frontends/bin:$PATH

  if ! ${TOOLSDIR}/configure.sh -e -S $1; then
    echo "Error: ############# config ${1} fail ##############"
    exit 1
  fi

  # The minimal L0 and the GPIO-only L1 configurations have no
  # board_app_initialize() implementation.  Keep boardctl disabled for
  # these configurations until the board-control API is implemented.
  if [[ "$1" == */configs/l0 || "$1" == */configs/l1_gpio || \
        "$1" == */configs/l1_pwm || "$1" == */configs/l2_psram || \
        "$1" == */configs/l2_kernel_tests ]]; then
    kconfig-tweak --file ${NUTTXDIR}/.config --disable CONFIG_BOARDCTL \
      --disable CONFIG_BOARDCTL_MKRD
  fi

  if [[ ${CUSTOM_COMPILER} == "tasking" ]]; then
    kconfig-tweak --file ${NUTTXDIR}/.config --enable CONFIG_TRICORE_TOOLCHAIN_TASKING
  fi

  GHS_OPTS_STRING="CONFIG_ARCH_TOOLCHAIN_GHS=y"
  TASKING_OPTS_STRING="CONFIG_TRICORE_TOOLCHAIN_TASKING=y"
  if grep -qE "^(${GHS_OPTS_STRING}|${TASKING_OPTS_STRING})$" "${NUTTXDIR}/.config"; then
    echo "EXTRA_FLAGS are required to update the when using the GHS or Tasking toolchain."
    EXTRA_FLAGS=$(echo "$EXTRA_FLAGS" | sed 's/-Wno-cpp//' | xargs)
  fi

  if ! ${BEAR} make -C ${NUTTXDIR} EXTRAFLAGS="$EXTRA_FLAGS" ${@:2}; then
    echo "Error: ############# build ${1} fail ##############"
    exit 2
  else
    if [ -f "${COMPILE_COMMANDS}" ]; then
      cp ${COMPILE_COMMANDS} ${COMPILE_COMMANDS_BACKUP}
    fi
  fi

  if echo "${@:2}" | grep -q "distclean"; then
    if [ -f "${COMPILE_COMMANDS}" ]; then
      rm -rf ${COMPILE_COMMANDS}
    fi
    return;
  fi

  if ! make -C ${NUTTXDIR} savedefconfig; then
    echo "Error: ############# save ${1} fail ##############"
    exit 3
  fi

  if [ ! -d $1 ]; then
    cp ${NUTTXDIR}/defconfig ${ROOTDIR}/nuttx/boards/*/*/${1/[:|\/]//configs/}
  else
    if grep -q "#include" "$1/defconfig"; then
      echo "Note: skipping savedefconfig for debug defconfig."
    else
      cp ${NUTTXDIR}/defconfig $1
    fi
  fi
}

function build_board_cmake()
{
  # first check if the command target is `distclean`
  # cmake is built for out-of-tree, so delete the CMAKE_BINARY_DIR directory directly
  if echo "${@:2}" | grep -q "distclean"; then
    echo -e "Build target distclean:"
    echo -e "  there is no need to distclean in cmake, delete '${CMAKE_BINARY_DIR}' directly"
    if [ -d "${CMAKE_BINARY_DIR}" ]; then
      rm -rf $CMAKE_BINARY_DIR
    fi
    return 0
  fi
  # check parallelism
  j_arg=$(echo ${@:2} |grep -oP '\-j[0-9]+')
  # cmake verbose
  v_arg=""

  # remove the -Wno-cpp build option from ghs build options
  GHS_OPTS_STRING="CONFIG_ARM_TOOLCHAIN_GHS=y"
  TASKING_OPTS_STRING="CONFIG_TRICORE_TOOLCHAIN_TASKING=y"
  defconfig_path=$1/defconfig
  valid_defconfig_path=$(echo ${defconfig_path} | sed 's/^.\{3\}//')
  if grep -qE "^(${GHS_OPTS_STRING}|${TASKING_OPTS_STRING})$" "${valid_defconfig_path}"; then
    echo "EXTRA_FLAGS are required to update the when using the GHS toolchain."
    EXTRA_FLAGS=$(echo "$EXTRA_FLAGS" | sed 's/-Wno-cpp//' | xargs)
  fi
  export VELA_EXTRA_FLAGS="$EXTRA_FLAGS"
  # let use lunch directily
  echo " lunch $1 ${CMAKE_BINARY_DIR} "
  lunch $1 ${CMAKE_BINARY_DIR}
  echo
  # check if the command target is `Xconfig`
  for arg in "${@:2}"
  do
    if [[ $arg == *config ]]; then
      if ! m $arg; then
        echo "Error: #############  m $arg fail ##############"
        exit 2
      else
        return 0
      fi
    fi
    if [[ "$arg" =~ ^V=1$ ]]; then
      v_arg+="V=1"
    fi
  done
  # do cmake build
  if ! m $j_arg $v_arg; then
    echo "Error: ############# build ${1} fail ##############"
    exit 2
  fi
}


function setup_cmake_binary_dir()
{
  local boardconfig=$1
  if [ -d ${ROOTDIR}/${boardconfig} ]; then
    # parse path config
    config_name=$(basename "$boardconfig")
    board_name=$(basename $(dirname $(dirname "$boardconfig")))
  else
    # parse nuttx config pair
    config_name=`echo ${boardconfig} | cut -s -d':' -f2`
    if [ -z "${config_name}" ]; then
      board_name=`echo ${boardconfig} | cut -d'/' -f1`
      config_name=`echo ${boardconfig} | cut -d'/' -f2`
    else
      board_name=`echo ${boardconfig} | cut -d':' -f1`
    fi
  fi
  CMAKE_BINARY_DIR+="/${board_name}_${config_name}"
}

if [ $# == 0 ]; then
  echo "Usage: $0 [-m] <board-name>:<config-name> [-e <extraflags>] [--cmake] [-b <cmake_binary_dir>] [--dis-ninja] [make options]"
  echo ""
  echo "Where:"
  echo "  -m: out of tree build. Or default in tree build without it."
  echo "  -e: pass extra c/c++ flags such as -Werror via make command line"
  echo "  --cmake: switch the build mode to CMake compilation."
  echo "  -b: set custom binary directory for CMake."
  echo "  --dis-ninja: disable CMake Ninja generator fo default."
  echo "  -c: set custom toolchain."
  exit 1
fi

ROOTDIR=$(dirname $(readlink -f ${0}))
ROOTDIR=$(realpath ${ROOTDIR}/../..)

CONFIGPATH=$2

if [ $1 == "-m" ]; then
  # out of tree build
  confparams=(${CONFIGPATH//:/ })
  configdir=${confparams[1]}

  if [ -z "${configdir}" ]; then
    # handle cases where the end is a "/"
    if [ "${CONFIGPATH:(-1)}" = "/" ]; then
      CONFIGPATH=${CONFIGPATH:0:-1}
    fi
    boarddir=`echo ${CONFIGPATH} | rev | cut -d'/' -f3 | rev`
    configdir=`echo ${CONFIGPATH} | rev | cut -d'/' -f1 | rev`
  else
    boarddir=${confparams[0]}
  fi

  OUTDIR=${ROOTDIR}/out/${boarddir}/${configdir}
  MOUNTDIR=${OUTDIR}/.unionfs
  NUTTXDIR=${MOUNTDIR}/nuttx

  trap cleanup EXIT
  mount_unionfs
  shift
else
  # in tree build
  OUTDIR=${ROOTDIR}
  NUTTXDIR=${ROOTDIR}/nuttx
fi

TOOLSDIR=${NUTTXDIR}/tools
board_config=$1
shift

source ${ROOTDIR}/build/envsetup.sh

EXTRA_FLAGS="-Wno-cpp -Wno-deprecated-declarations"
while [[ "$1" == "-e" ]]; do
  shift
  EXTRA_FLAGS+=" $1"
  echo "extraflags: $EXTRA_FLAGS"
  shift
done

if [ "$1" == "--cmake" ]; then
  CMAKE_BINARY_DIR="cmake_out"
  CMAKE_GENERATOR="-GNinja"
  CMAKE_BUILD="cmake"
  setup_cmake_binary_dir $board_config
  shift
fi

if [ "$1" == "-b" ]; then
  shift
  CMAKE_BINARY_DIR="$1"
  echo "custom CMake binary dir: $CMAKE_BINARY_DIR"
  shift
fi

if [ "$1" == "--dis-ninja" ]; then
  CMAKE_GENERATOR=""
  shift
fi

if [ "$1" == "-c" ]; then
  shift
  CUSTOM_COMPILER="$1"
  echo "custom toolchain: $CUSTOM_COMPILER"
  shift
fi

# Determine the config path
if [ -d ${ROOTDIR}/${board_config} ]; then
  config_path="${ROOTDIR}/${board_config}"
else
  config_path="${board_config}"
fi

# Build with appropriate method
if [ -z "$CMAKE_BUILD" ]; then
  build_board ${config_path} $*
else
  build_board_cmake ${board_config} $*
fi
