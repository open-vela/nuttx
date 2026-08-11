#!/usr/bin/env bash
# tools/ci/buildsystems/vela.sh
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

# OpenVela build-system plugin for tools/testbuild.sh --buildsystem.
#
# Routes each target through the workspace ./build.sh, accepting both
# board:config and vendor/<...>/configs/<cfg> entries, and adding --cmake
# for CMake, entries. Blacklists use the testbuild grammar with the first
# slash replaced by a colon: -Linux,vendor:openvela/boards/.../foo/
#
# Environment:
#   VELA_REQUIRE_RUN=1          with -R, fail targets lacking a run script
#   VELA_TOOLCHAIN_SUBSTITUTE=0 fail instead of substituting a toolchain

VELA_ROOT=$(cd "$nuttx/.." && pwd)

if [ ! -x "$VELA_ROOT/build.sh" ]; then
  echo "ERROR: $VELA_ROOT/build.sh not found or not executable."
  echo "       The vela build system requires an OpenVela manifest workspace."
  exit 1
fi

VELA_EXTRA="${EXTRA_FLAGS#EXTRAFLAGS=}"
VELA_JOBS="${JOPTION#-j }"

function resolveconfig {
  if [[ "$config" == vendor/* ]]; then
    path=$VELA_ROOT/${config%/}
    if [ ! -r $path/defconfig ]; then
      echo "ERROR: no configuration found at $path"
      showusage
    fi
  else
    resolveconfig_default
  fi
}

vela_defconfig=""
vela_defconfig_saved=""
vela_substituted=""

function vela_restore_defconfig {
  if [ -n "$vela_defconfig" ] && [ -r "$vela_defconfig_saved" ]; then
    cp -f "$vela_defconfig_saved" "$vela_defconfig"
    rm -f "$vela_defconfig_saved"
  fi
  vela_defconfig=""
  vela_defconfig_saved=""
}

# The manifest ships no bare-metal clang; substitute GNU_EABI.

function vela_resolve_toolchain {
  case "$toolchain" in
    *_TOOLCHAIN_CLANG)
      command -v clang >/dev/null 2>&1 && return 0
      if [ "${VELA_TOOLCHAIN_SUBSTITUTE:-1}" -ne 1 ]; then
        return 0
      fi
      vela_substituted="$toolchain"
      toolchain="${toolchain%_CLANG}_GNU_EABI"
      echo "  Note: ${vela_substituted} is unavailable (no clang in PATH);" \
           "substituting ${toolchain}"
      ;;
  esac
  return 0
}

function vela_apply_toolchain {
  local defconfig original

  vela_substituted=""
  vela_resolve_toolchain

  defconfig=$(ls -d $path/defconfig 2>/dev/null | head -1)
  if [ ! -w "$defconfig" ]; then
    echo "  ERROR: cannot apply $toolchain, no writable defconfig at $path"
    fail=1
    return $fail
  fi

  if grep -qx "$toolchain=y" "$defconfig"; then
    echo "  Toolchain $toolchain already selected"
    return $fail
  fi

  vela_defconfig=$defconfig
  vela_defconfig_saved=$(mktemp)
  cp -f "$defconfig" "$vela_defconfig_saved"

  # Same rule as configure_default: the enabled _TOOLCHAIN_ symbol.
  original=$(grep '_TOOLCHAIN_' "$defconfig" | grep -v 'CONFIG_ARCH_TOOLCHAIN_' \
             | grep '=y$' | head -1 | cut -d'=' -f1)
  if [ -n "$original" ]; then
    echo "  Disabling $original"
    sed -i "/^$original=y\$/d" "$defconfig"
  fi

  echo "  Enabling $toolchain"
  echo "$toolchain=y" >> "$defconfig"

  return $fail
}

# repo checks other manifest projects out inside the nuttx and apps trees
# and links their .git as a symlink, which git clean deletes. Exclude those
# paths, then clean inside each project so no output crosses targets.

function vela_clean_tree {
  local dir=$1 excludes=() nested rel protect parent
  [ -e "$dir/.git" ] || return 0

  while IFS= read -r nested; do
    rel=${nested#$dir/}

    # git clean -d removes the whole untracked ancestor, so exclude the
    # shallowest untracked one, not the project path.
    protect=$rel
    parent=$(dirname "$rel")
    while [ "$parent" != "." ] && [ "$parent" != "/" ]; do
      [ -z "$(git -C "$dir" ls-files -- "$parent" | head -1)" ] || break
      protect=$parent
      parent=$(dirname "$parent")
    done

    excludes+=(-e "/$protect/")
  done < <(find "$dir" -mindepth 2 -name .git -printf '%h\n' 2>/dev/null | sort)

  git -C "$dir" clean -xfdq "${excludes[@]}"

  while IFS= read -r nested; do
    git -C "$nested" clean -xfdq 2>/dev/null
    git -C "$nested" checkout -- . 2>/dev/null
  done < <(find "$dir" -mindepth 2 -name .git -printf '%h\n' 2>/dev/null | sort)
}

function distclean {
  # A target that died mid-build leaves its defconfig patched.
  vela_restore_defconfig

  echo "  Cleaning (OpenVela)..."
  rm -rf "$VELA_ROOT/cmake_out"
  vela_clean_tree "$nuttx"
  vela_clean_tree "$APPSDIR"
  return $fail
}

function configure {
  # build.sh configures and builds in one step; only the toolchain
  # override is done here.
  vela_substituted=""
  if [ "X$toolchain" != "X" ]; then
    vela_apply_toolchain
  fi
  return $fail
}

# Failure reporting: annotation, full build log in a collapsed group, and a
# row in the job summary.

function vela_summary_row {
  local status=$1 target=$2 detail=$3

  [ -n "$GITHUB_STEP_SUMMARY" ] || return 0

  if [ ! -e "$VELA_ROOT/.vela-summary-started" ]; then
    : > "$VELA_ROOT/.vela-summary-started"
    {
      echo "## OpenVela build results: $(basename ${testfile:-targets} .dat)"
      echo
      echo "| Result | Target | Detail |"
      echo "| --- | --- | --- |"
    } >> "$GITHUB_STEP_SUMMARY"
  fi

  echo "| $status | \`$target\` | ${detail//|/\\|} |" >> "$GITHUB_STEP_SUMMARY"
}

function vela_report_failure {
  local target=$1 log=$2 first

  # make's "*** [rule] Error N" wrapper names nothing, so use it last.
  first=$(grep -m1 -E "error:|undefined reference|No such file or directory|No rule to make target|cannot find|fatal" "$log")
  : "${first:=$(grep -m1 -E "Error [0-9]+" "$log")}"
  : "${first:=see the build log below}"

  echo "::error title=build failed: ${target}::${first}"

  echo "  FAIL: ${target}; errors from build.sh output:"
  grep -E "error:|Error [0-9]+|undefined reference|fatal" "$log" | tail -n 40

  echo "::group::full build log: ${target}"
  cat "$log"
  echo "::endgroup::"

  vela_summary_row ":x: FAIL" "$target" "$first"
}

function build {
  echo "  Building with OpenVela build.sh..."
  local target="${config%/}"
  if [[ "$target" != vendor/* ]]; then
    target="${target/\//:}"
  fi
  local args=("$target")
  if [ ! -z ${cmake} ]; then
    args+=(--cmake)
  fi
  if [ ! -z "$VELA_EXTRA" ]; then
    args+=(-e "$VELA_EXTRA")
  fi
  if [ ! -z "$VELA_JOBS" ]; then
    args+=(-j$VELA_JOBS)
  fi
  local outdir=$ARTIFACTDIR/$(echo $config | sed "s/:/\//")
  mkdir -p $outdir
  if ! (cd "$VELA_ROOT" && ./build.sh "${args[@]}" >$outdir/build.log 2>&1); then
    vela_report_failure "$target" "$outdir/build.log"
    fail=1
  else
    echo "  PASS: ${target}"
    local detail="${toolchain:+toolchain ${toolchain}}"
    if [ -n "$vela_substituted" ]; then
      detail="$detail (substituted for unavailable ${vela_substituted})"
    fi
    vela_summary_row ":white_check_mark: pass" "$target" "$detail"
  fi
  return $fail
}

function refresh {
  # build.sh runs savedefconfig and copies the result back, so a toolchain
  # override has to be reverted here.
  vela_restore_defconfig
  return $fail
}

function run {
  if [ ${RUN} -ne 0 ] && [ -z ${cmake} ]; then
    run_script=""
    for candidate in "$path/run.sh" "$path/run"; do
      if [ -x $candidate ]; then
        run_script=$candidate
        break
      fi
    done
    if [ ! -z "$run_script" ]; then
      echo "  Running NuttX..."
      export CURRENTCONFDIR=$(cd $path && pwd)
      export ARTIFACTCONFDIR=$ARTIFACTDIR/$(echo $config | sed "s/:/\//")
      mkdir -p $ARTIFACTCONFDIR
      if ! $run_script; then
        fail=1
      fi
    elif [ "${VELA_REQUIRE_RUN:-0}" -eq 1 ]; then
      echo "ERROR: no executable run script at $path"
      fail=1
    fi
  fi
  return $fail
}
