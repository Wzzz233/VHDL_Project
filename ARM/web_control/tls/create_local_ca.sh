#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "usage: $0 <board-fixed-ip> [hostname]" >&2
    exit 2
fi

BOARD_IP="$1"
HOSTNAME="${2:-pg2l50h.home.arpa}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="${SCRIPT_DIR}/generated"
CONFIG="${OUTPUT_DIR}/server-ext.cnf"

if ! command -v openssl >/dev/null 2>&1; then
    echo "missing required command: openssl" >&2
    exit 1
fi

umask 077
mkdir -p -- "${OUTPUT_DIR}"
trap 'rm -f -- "${CONFIG}"' EXIT

if [[ -f "${OUTPUT_DIR}/ca.key" && -f "${OUTPUT_DIR}/ca.crt" ]]; then
    echo "Reusing existing local CA."
elif [[ -e "${OUTPUT_DIR}/ca.key" || -e "${OUTPUT_DIR}/ca.crt" ]]; then
    echo "incomplete local CA: ca.key and ca.crt must both exist or both be absent" >&2
    exit 1
else
    openssl genrsa -out "${OUTPUT_DIR}/ca.key" 3072
    openssl req -x509 -new -sha256 -days 3650 \
        -key "${OUTPUT_DIR}/ca.key" \
        -out "${OUTPUT_DIR}/ca.crt" \
        -subj "/CN=PPLCNet Local CA/O=PPLCNet Live"
fi

openssl genrsa -out "${OUTPUT_DIR}/server.key" 3072
openssl req -new -sha256 \
    -key "${OUTPUT_DIR}/server.key" \
    -out "${OUTPUT_DIR}/server.csr" \
    -subj "/CN=${HOSTNAME}/O=PPLCNet Live"

{
    echo "basicConstraints=critical,CA:FALSE"
    echo "keyUsage=critical,digitalSignature,keyEncipherment"
    echo "extendedKeyUsage=serverAuth"
    echo "subjectAltName=DNS:${HOSTNAME},IP:${BOARD_IP}"
    echo "subjectKeyIdentifier=hash"
    echo "authorityKeyIdentifier=keyid,issuer"
} >"${CONFIG}"

openssl x509 -req -sha256 -days 397 \
    -in "${OUTPUT_DIR}/server.csr" \
    -CA "${OUTPUT_DIR}/ca.crt" \
    -CAkey "${OUTPUT_DIR}/ca.key" \
    -CAcreateserial \
    -extfile "${CONFIG}" \
    -out "${OUTPUT_DIR}/server.crt"

chmod 0600 "${OUTPUT_DIR}/ca.key" "${OUTPUT_DIR}/server.key"
chmod 0644 "${OUTPUT_DIR}/ca.crt" "${OUTPUT_DIR}/server.crt"

echo "Created HTTPS certificate in ${OUTPUT_DIR}"
echo "Install ca.crt on the iPhone and enable full trust for the root certificate."
echo "Open https://${HOSTNAME}:8443 after DNS maps the hostname to ${BOARD_IP}."
