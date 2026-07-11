#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
    exec sudo -- "$0" "$@"
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ARM_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
RUNTIME_DIR=/run/pplcnet-board
LOG_DIR=/var/log/pplcnet-board
MODEL_DIR="${MODEL_DIR:-/userdata/model}"
BOARD_HOST="${BOARD_HOST:-pg2l50h.home.arpa}"
MEDIAMTX_BIN="${MEDIAMTX_BIN:-/opt/mediamtx-v1.19.2/bin/mediamtx}"

detect_ip() {
    local route token previous=""
    route="$(ip -4 route get 1.1.1.1 2>/dev/null | head -n 1)"
    for token in ${route}; do
        if [[ ${previous} == src ]]; then
            printf '%s\n' "${token}"
            return 0
        fi
        previous="${token}"
    done
    return 1
}

BOARD_IP="${BOARD_IP:-$(detect_ip)}"
if [[ ! ${BOARD_IP} =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "无法自动确定局域网 IPv4；请使用 BOARD_IP=地址 sudo -E $0" >&2
    exit 1
fi

mkdir -p "${RUNTIME_DIR}" "${LOG_DIR}"
chmod 0755 "${RUNTIME_DIR}" "${LOG_DIR}"

required_files=(
    "${ARM_ROOT}/pplcnet_bgp_live"
    "${MEDIAMTX_BIN}"
    "${MODEL_DIR}/best_fp16.rknn"
    "${MODEL_DIR}/pplcnet_blue_v3_rk3568_fp16.rknn"
    "${MODEL_DIR}/pplcnet_green_v2_b1plus_rk3568_fp16.rknn"
    "${MODEL_DIR}/pplcnet_yellow_all_single_v1_rk3568_fp16.rknn"
    "${MODEL_DIR}/pplcnet_police_v4_warmblue_rk3568_fp16.rknn"
    "${MODEL_DIR}/pplcnet_embassy_v1_rk3568_fp16.rknn"
    "${MODEL_DIR}/plate_type_classifier_6cls_resnet18_warped_nocrop_rk3568_fp16_opt0.rknn"
    "${MODEL_DIR}/special_keys.txt"
    "${MODEL_DIR}/pplcnet_green_keys.txt"
    "${MODEL_DIR}/yellow_keys.txt"
    "${MODEL_DIR}/police_keys.txt"
    "${MODEL_DIR}/embassy_keys.txt"
)
for file in "${required_files[@]}"; do
    if [[ ! -r ${file} ]]; then
        echo "缺少文件: ${file}" >&2
        exit 1
    fi
done

if [[ ! -x ${ARM_ROOT}/cplus-rk3568-driver ]]; then
    echo "警告: ${ARM_ROOT}/cplus-rk3568-driver 不存在，行人图片模式暂不可用" >&2
fi

sed "s/^webrtcAdditionalHosts:.*/webrtcAdditionalHosts: [${BOARD_HOST}, ${BOARD_IP}]/" \
    "${SCRIPT_DIR}/mediamtx.yml" >"${RUNTIME_DIR}/mediamtx.yml"

CERT_DIR="${SCRIPT_DIR}/tls/generated"
if [[ ! -r ${CERT_DIR}/server.crt ]] ||
   ! openssl x509 -in "${CERT_DIR}/server.crt" -noout -ext subjectAltName 2>/dev/null |
       grep -Fq "IP Address:${BOARD_IP}"; then
    "${SCRIPT_DIR}/tls/create_local_ca.sh" "${BOARD_IP}" "${BOARD_HOST}"
fi
chown root:root "${CERT_DIR}/server.key"
chmod 0600 "${CERT_DIR}/server.key"

stop_pidfile() {
    local name=$1 pidfile="${RUNTIME_DIR}/$1.pid" pid
    [[ -f ${pidfile} ]] || return 0
    pid="$(cat "${pidfile}")"
    if [[ ${pid} =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
        kill "${pid}"
        for _ in {1..30}; do
            kill -0 "${pid}" 2>/dev/null || break
            sleep 0.1
        done
    fi
    rm -f "${pidfile}"
    echo "已停止旧进程: ${name}"
}

stop_pidfile web
stop_pidfile plate
stop_pidfile mediamtx

nohup "${MEDIAMTX_BIN}" "${RUNTIME_DIR}/mediamtx.yml" \
    >"${LOG_DIR}/mediamtx.log" 2>&1 &
echo $! >"${RUNTIME_DIR}/mediamtx.pid"

echo plate >"${RUNTIME_DIR}/mode"
MODEL_DIR="${MODEL_DIR}" nohup "${SCRIPT_DIR}/run_plate_live.sh" \
    >"${LOG_DIR}/plate.log" 2>&1 &
echo $! >"${RUNTIME_DIR}/plate.pid"

web_command=(
    python3 "${SCRIPT_DIR}/server.py"
    --host "${BOARD_IP}"
    --port 8443
    --cert-file "${CERT_DIR}/server.crt"
    --key-file "${CERT_DIR}/server.key"
    --control-socket /run/pplcnet-bgp-live/control.sock
    --mediamtx-host 127.0.0.1
    --mediamtx-port 8889
    --arm-root "${ARM_ROOT}"
    --model-root "${MODEL_DIR}"
    --plate-image-driver "${ARM_ROOT}/pplcnet_bgp_live"
    --pedestrian-image-driver "${ARM_ROOT}/cplus-rk3568-driver"
    --driver-runtime-dir "${RUNTIME_DIR}"
    --driver-log-dir "${LOG_DIR}"
    --plate-live-script "${SCRIPT_DIR}/run_plate_live.sh"
)
nohup "${web_command[@]}" >"${LOG_DIR}/web.log" 2>&1 &
echo $! >"${RUNTIME_DIR}/web.pid"

sleep 2
for service in mediamtx plate web; do
    pid="$(cat "${RUNTIME_DIR}/${service}.pid")"
    if ! kill -0 "${pid}" 2>/dev/null; then
        echo "${service} 启动失败，日志如下:" >&2
        tail -n 80 "${LOG_DIR}/${service}.log" >&2
        exit 1
    fi
done

cat >"${RUNTIME_DIR}/current.env" <<EOF
BOARD_IP=${BOARD_IP}
BOARD_HOST=${BOARD_HOST}
HTTPS_URL=https://${BOARD_IP}:8443
EOF

echo "启动完成"
echo "当前 IP: ${BOARD_IP}"
echo "手机访问: https://${BOARD_IP}:8443"
echo "日志目录: ${LOG_DIR}"
echo "停止命令: sudo ${SCRIPT_DIR}/stop_board.sh"
