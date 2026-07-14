#!/usr/bin/env bash
# SD card inference API smoke test. Runs against a live board web server.
# It only checks routing and validation; real photo/video inference requires
# actual files on the SD card (see DEPLOYMENT_ZH.md section 21).
set -euo pipefail

BASE="${BASE:-https://127.0.0.1:8443}"
# Prefer the local CA when present, otherwise allow self-signed certs.
CA="${CA:-./web_control/tls/generated/ca.crt}"
if [[ -f "${CA}" ]]; then
    CURL=(curl --cacert "${CA}")
else
    CURL=(curl -k)
fi

get_code() {
    "${CURL[@]}" -s -o /dev/null -w '%{http_code}' "$@"
}

echo "== sd/list =="
code=$(get_code "${BASE}/api/v1/sd/list")
[[ "${code}" == "200" ]] || { echo "FAIL sd/list: ${code}"; exit 1; }
echo "ok (200)"

echo "== sd/photo-inference rejects bad mode =="
code=$(get_code -X POST -H 'Content-Type: application/json' \
    -d '{"path":"x.jpg","mode":"bogus"}' "${BASE}/api/v1/sd/photo-inference")
[[ "${code}" == "400" ]] || { echo "FAIL photo bad mode: ${code}"; exit 1; }
echo "ok (400)"

echo "== sd/video-inference rejects bad fps =="
code=$(get_code -X POST -H 'Content-Type: application/json' \
    -d '{"path":"x.mp4","sample_fps":99}' "${BASE}/api/v1/sd/video-inference")
[[ "${code}" == "400" ]] || { echo "FAIL video bad fps: ${code}"; exit 1; }
echo "ok (400)"

echo "== sd/video-jobs list =="
code=$(get_code "${BASE}/api/v1/sd/video-jobs")
[[ "${code}" == "200" ]] || { echo "FAIL video-jobs: ${code}"; exit 1; }
echo "ok (200)"

echo
echo "SD inference API smoke passed."
echo "For real photo/video inference, place files on the SD card and follow"
echo "DEPLOYMENT_ZH.md section 21."
