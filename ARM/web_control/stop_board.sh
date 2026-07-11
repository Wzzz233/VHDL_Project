#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
    exec sudo -- "$0" "$@"
fi

RUNTIME_DIR=/run/pplcnet-board
for service in web plate mediamtx; do
    pidfile="${RUNTIME_DIR}/${service}.pid"
    [[ -f ${pidfile} ]] || continue
    pid="$(cat "${pidfile}")"
    if [[ ${pid} =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
        kill "${pid}"
        echo "已停止 ${service} (PID ${pid})"
    fi
    rm -f "${pidfile}"
done
