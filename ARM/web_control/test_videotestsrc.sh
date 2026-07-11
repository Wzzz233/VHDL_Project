#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MEDIAMTX_BIN="${1:-${MEDIAMTX_BIN:-${SCRIPT_DIR}/vendor/mediamtx-v1.19.2/bin/mediamtx}}"

echo "This board-only test validates synthetic H.264 through MediaMTX to the"
echo "RTSP/GStreamer/MPP/appsink downstream path. It does not validate WHIP,"
echo "Safari camera permission, iPhone backgrounding, or recovery behavior."

for command in gst-launch-1.0 gst-inspect-1.0 mktemp timeout; do
    if ! command -v "${command}" >/dev/null 2>&1; then
        echo "missing required command: ${command}" >&2
        exit 1
    fi
done
if [[ ! -x "${MEDIAMTX_BIN}" ]]; then
    echo "MediaMTX binary not executable: ${MEDIAMTX_BIN}" >&2
    exit 1
fi

required_elements=(videotestsrc videoconvert videoscale h264parse flvmux rtmpsink rtspsrc rtph264depay appsink)
for element in "${required_elements[@]}"; do
    if ! gst-inspect-1.0 "${element}" >/dev/null 2>&1; then
        echo "missing GStreamer element: ${element}" >&2
        exit 1
    fi
done

encoder=""
for candidate in mpph264enc x264enc; do
    if gst-inspect-1.0 "${candidate}" >/dev/null 2>&1; then
        encoder="${candidate}"
        break
    fi
done
if [[ -z "${encoder}" ]]; then
    echo "missing H.264 encoder (mpph264enc or x264enc)" >&2
    exit 1
fi

if ! gst-inspect-1.0 mppvideodec >/dev/null 2>&1; then
    echo "missing production Rockchip MPP decoder mppvideodec" >&2
    exit 1
fi

work_dir="$(mktemp -d)"
config="${work_dir}/mediamtx-test.yml"
mediamtx_log="${work_dir}/mediamtx.log"
publisher_log="${work_dir}/publisher.log"
mediamtx_pid=""
publisher_pid=""

cleanup() {
    if [[ -n "${publisher_pid}" ]] && kill -0 "${publisher_pid}" 2>/dev/null; then
        kill "${publisher_pid}" 2>/dev/null || true
        wait "${publisher_pid}" 2>/dev/null || true
    fi
    if [[ -n "${mediamtx_pid}" ]] && kill -0 "${mediamtx_pid}" 2>/dev/null; then
        kill "${mediamtx_pid}" 2>/dev/null || true
        wait "${mediamtx_pid}" 2>/dev/null || true
    fi
    rm -rf -- "${work_dir}"
}
trap cleanup EXIT

cat >"${config}" <<'EOF'
logLevel: warn
logDestinations: [stdout]
api: no
metrics: no
pprof: no
playback: no
hls: no
srt: no
webrtc: no
moq: no
rtsp: yes
rtspAddress: 127.0.0.1:8554
rtspTransports: [tcp]
rtmp: yes
rtmpAddress: 127.0.0.1:1935
paths:
  phone:
    source: publisher
EOF

"${MEDIAMTX_BIN}" "${config}" >"${mediamtx_log}" 2>&1 &
mediamtx_pid=$!
sleep 1
if ! kill -0 "${mediamtx_pid}" 2>/dev/null; then
    echo "temporary MediaMTX failed to start; stop any service using ports 8554 or 1935" >&2
    cat "${mediamtx_log}" >&2
    exit 1
fi

if [[ "${encoder}" == "x264enc" ]]; then
    encoder_args=(x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 key-int-max=15)
else
    encoder_args=(mpph264enc)
fi

gst-launch-1.0 -q \
    videotestsrc is-live=true pattern=ball num-buffers=450 \
    ! video/x-raw,width=1280,height=720,framerate=15/1 \
    ! videoconvert \
    ! "${encoder_args[@]}" \
    ! h264parse config-interval=-1 \
    ! flvmux streamable=true \
    ! rtmpsink location="rtmp://127.0.0.1:1935/phone live=1" \
    >"${publisher_log}" 2>&1 &
publisher_pid=$!
sleep 2
if ! kill -0 "${publisher_pid}" 2>/dev/null; then
    echo "synthetic H.264 publisher failed" >&2
    cat "${publisher_log}" >&2
    exit 1
fi

if ! timeout 20 gst-launch-1.0 -q \
    rtspsrc location=rtsp://127.0.0.1:8554/phone protocols=tcp latency=100 \
    ! rtph264depay \
    ! h264parse \
    ! mppvideodec \
    ! videoconvert \
    ! videoscale add-borders=true \
    ! video/x-raw,format=BGRx,width=1280,height=720,pixel-aspect-ratio=1/1 \
    ! appsink max-buffers=1 drop=true sync=false num-buffers=45; then
    echo "downstream RTSP/H.264/MPP/appsink pipeline failed" >&2
    cat "${mediamtx_log}" >&2
    exit 1
fi

echo "[PASS] videotestsrc -> H.264 -> MediaMTX -> RTSP -> MPP -> BGRx appsink"
