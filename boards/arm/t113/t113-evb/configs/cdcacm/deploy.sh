#!/usr/bin/env bash
# Phase 1 deploy driver: rsync nuttx.bin + bench scripts to opt7050,
# trigger remote bench, pull results back.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# repo root = scripts/../../../../../..  (configs/cdcacm -> ws root, 6 levels)
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../../../.." && pwd)"
NUTTX_BIN="${REPO_ROOT}/build_cdcacm/nuttx.bin"
REMOTE_HOST="opt7050"
REMOTE_DIR="~/cdcopt-bench"
LOCAL_RESULTS_PARENT="${REPO_ROOT}/cdcbench"

if [[ ! -f "${NUTTX_BIN}" ]]; then
    echo "ERROR: nuttx.bin not found at ${NUTTX_BIN}" >&2
    echo "       run cmake+ninja first (see Phase 1 mission notes)" >&2
    exit 1
fi

echo "=== build artifact ==="
sha256sum "${NUTTX_BIN}"
stat -c '%y %s bytes' "${NUTTX_BIN}"
echo

echo "=== rsync to ${REMOTE_HOST}:${REMOTE_DIR} ==="
ssh "${REMOTE_HOST}" "mkdir -p ${REMOTE_DIR}"
rsync -av \
    "${NUTTX_BIN}" \
    "${SCRIPT_DIR}/bench.py" \
    "${SCRIPT_DIR}/remote_bench.sh" \
    "${REMOTE_HOST}:${REMOTE_DIR}/"
echo

echo "=== running remote bench (foreground) ==="
ssh "${REMOTE_HOST}" "bash ${REMOTE_DIR}/remote_bench.sh"
echo

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LOCAL_RESULTS="${LOCAL_RESULTS_PARENT}/${STAMP}"
mkdir -p "${LOCAL_RESULTS}"
echo "=== rsync results back to ${LOCAL_RESULTS} ==="
rsync -av "${REMOTE_HOST}:${REMOTE_DIR}/results/" "${LOCAL_RESULTS}/"
echo
echo "results: ${LOCAL_RESULTS}"
