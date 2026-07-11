#!/usr/bin/env bash
set -euo pipefail

VERSION="1.19.2"
ASSET="mediamtx_v1.19.2_linux_arm64.tar.gz"
URL="https://github.com/bluenviron/mediamtx/releases/download/v1.19.2/mediamtx_v1.19.2_linux_arm64.tar.gz"
EXPECTED_SHA256="562f419912a8668c18216a9e8c95359ec82fbb754e4a44e2953ef62b98eec688"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PREFIX="${1:-${SCRIPT_DIR}/vendor/mediamtx-v${VERSION}}"

for command in curl sha256sum tar install mktemp; do
    if ! command -v "${command}" >/dev/null 2>&1; then
        echo "missing required command: ${command}" >&2
        exit 1
    fi
done

work_dir="$(mktemp -d)"
trap 'rm -rf -- "${work_dir}"' EXIT
archive="${work_dir}/${ASSET}"
extract_dir="${work_dir}/extract"
mkdir -p -- "${extract_dir}" "${PREFIX}/bin"

echo "Downloading MediaMTX v${VERSION} Linux ARM64"
curl --fail --location --proto '=https' --tlsv1.2 --output "${archive}" "${URL}"
read -r actual_sha256 _ < <(sha256sum "${archive}")
if [[ "${actual_sha256}" != "${EXPECTED_SHA256}" ]]; then
    echo "MediaMTX archive SHA256 mismatch" >&2
    echo "expected: ${EXPECTED_SHA256}" >&2
    echo "actual:   ${actual_sha256}" >&2
    exit 1
fi

tar -xzf "${archive}" -C "${extract_dir}"
if [[ ! -f "${extract_dir}/mediamtx" ]]; then
    echo "MediaMTX archive does not contain the mediamtx binary" >&2
    exit 1
fi

install -m 0755 "${extract_dir}/mediamtx" "${PREFIX}/bin/mediamtx"
if [[ -f "${extract_dir}/LICENSE" ]]; then
    install -m 0644 "${extract_dir}/LICENSE" "${PREFIX}/LICENSE"
fi

echo "Installed: ${PREFIX}/bin/mediamtx"
echo "Verified SHA256: ${EXPECTED_SHA256}"
