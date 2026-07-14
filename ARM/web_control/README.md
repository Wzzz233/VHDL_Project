# PPLCNet Live Web Control

详细中文部署步骤见 [`DEPLOYMENT_ZH.md`](DEPLOYMENT_ZH.md)。

This directory contains the LAN-only HTTPS control panel for the modular
`pplcnet_bgp_live` pipeline. The browser publishes an iPhone camera through
WHIP/WebRTC. MediaMTX routes the encoded stream without transcoding, and the C
pipeline reads `rtsp://127.0.0.1:8554/phone` through GStreamer.

For board startup on changing LANs, run `sudo ./web_control/start_board.sh`.
It detects the current IPv4 address, renews only the server certificate when
needed, writes a runtime MediaMTX configuration, and starts MediaMTX, the live
plate pipeline, and this web server. `stop_board.sh` stops those managed
processes.

The control panel also accepts JPEG/PNG uploads at
`POST /api/v1/image-inference` with `X-Inference-Mode: plate` or
`X-Inference-Mode: pedestrian`. Plate uploads use the live driver's
`--input-bgrx` one-image source. Pedestrian uploads use the CPlus driver's
`--input-bgrx` mode and its web extension `--output-mask-bgrx`. The returned
preview contains the same green road, blue sidewalk, and red zebra mask blend
as the HDMI path.

`GET /api/v1/mode` and `PUT /api/v1/mode` manage exclusive driver modes.
Plate mode keeps the OV5640 live plate process running. Pedestrian mode stops
that process and runs CPlus only for an uploaded still image. A plate still
image temporarily pauses the live process and restores it after inference.

Still-image plate inference uses a 0.25 detector threshold, 0.45 NMS, and a
16-plate result capacity for collage-style QA images. The live OV5640 command
keeps its existing 0.45 / 0.65 / 9 settings. Police OCR uses the v5 white-expand
model; the embassy route now loads the unified black-plate model and keys.

The web server never opens the FPGA DMA device, starts the live binary, changes
model paths, or overrides inference/display defaults. Runtime commands go to an
already-running live process through a Unix domain socket.

## Components

- `server.py`: HTTPS static/API server, live-process Unix socket proxy, and
  same-origin MediaMTX WHIP proxy.
- `static/`: iPhone-ready control panel and 480x270 JPEG/canvas preview.
- `mediamtx.yml`: one publisher path named `phone`; RTSP and WHIP signaling are
  loopback-only, while WebRTC media uses UDP port 8189 on the LAN.
- `install_mediamtx.sh`: installs MediaMTX v1.19.2 Linux ARM64 after checking
  SHA256 `562f419912a8668c18216a9e8c95359ec82fbb754e4a44e2953ef62b98eec688`.
- `tls/create_local_ca.sh`: creates the project-local CA and HTTPS certificate.
- `check_board_deps.sh`: checks required GStreamer and Rockchip MPP elements.
- `test_videotestsrc.sh`: board-only synthetic H.264/RTSP/MPP/appsink test.
- `test_phone_letterbox_pipeline.sh`: pixel-level BGRx border normalization test.
- `test_whip_videotestsrc.py`: real Chromium WHIP/WebRTC publication test.
- `test_whip_videotestsrc_unit.py`: hardware-free command/configuration tests for
  the WHIP harness.
- `test_server.py`: hardware-free API, Unix socket, JPEG, and WHIP proxy tests.

## Install MediaMTX

Run this on the ARM64 board. The optional first argument is the installation
prefix; without it the binary is placed below `web_control/vendor/`.

```bash
./web_control/install_mediamtx.sh /opt/mediamtx-v1.19.2
```

The installer uses this exact release asset:

```text
https://github.com/bluenviron/mediamtx/releases/download/v1.19.2/mediamtx_v1.19.2_linux_arm64.tar.gz
```

Add the board's fixed LAN IP to `webrtcAdditionalHosts` in `mediamtx.yml` if the
router does not provide `pg2l50h.home.arpa` to WebRTC clients. Open UDP port 8189
between the phone and board. MediaMTX signaling remains on `127.0.0.1:8889` and
is exposed only through the HTTPS server's `/whip/phone` proxy.

Start MediaMTX:

```bash
/opt/mediamtx-v1.19.2/bin/mediamtx web_control/mediamtx.yml
```

## HTTPS

Map `pg2l50h.home.arpa` to the board's fixed IP in the router, then create the
local CA and server certificate:

```bash
./web_control/tls/create_local_ca.sh 192.168.1.50 pg2l50h.home.arpa
```

Transfer `web_control/tls/generated/ca.crt` to the iPhone, install the profile,
then enable full trust for that root certificate in iOS certificate settings.
Private keys and generated certificates are ignored by Git.

Start the server with the board's specific LAN address. Wildcard binds are
rejected deliberately.

```bash
python3 web_control/server.py \
  --host 192.168.1.50 \
  --port 8443 \
  --control-socket /run/pplcnet-bgp-live/control.sock
```

Open `https://pg2l50h.home.arpa:8443`. There is no login; firewall the port to
the trusted LAN. For a systemd deployment, give the live process service
`RuntimeDirectory=pplcnet-bgp-live` and make the socket group-readable/writable
by the web service account.

## Unix Socket Contract

Each connection carries one newline-terminated JSON request and one response.
The supported requests are:

```json
{"op":"status"}
{"op":"results"}
{"op":"frame"}
{"op":"source","source":"fpga"}
{"op":"source","source":"phone"}
{"op":"pipeline","action":"pause"}
{"op":"pipeline","action":"resume"}
{"op":"pipeline","action":"restart"}
```

Status, results, source, and pipeline responses are one newline-terminated JSON
object. A frame response is a JSON header followed immediately by the exact JPEG
bytes:

```text
{"ok":true,"content_type":"image/jpeg","content_length":12345,"source_generation":42}\n
<12345 JPEG bytes>
```

The status object exposes `desired_source`, `active_source`,
`failover_reason`, `source_generation`, `pipeline`, source health,
`frame.{width,height,age_ms}`, `fps.{input,decode,infer}`, frame counters,
and `dropped.{input,decode,infer,display}`. Results expose
`source_generation`, sequence/timing fields, `frame.{width,height}`, and flat
`detections[]` entries with coordinates, detector/OCR confidence, route, and
text. The browser ignores detections from an older source generation.
The frame endpoint includes the same generation in `X-Source-Generation`.
On a generation change, the server invalidates its short JPEG cache and the
browser clears the prior image, results, and overlay before accepting a frame
from the new source.

The C pipeline produces the preview in memory at 480x270, no more than 8fps,
with `jpegenc quality=55`. The web server also caches each JPEG for 125ms, so
concurrent browser requests cannot raise the producer rate. No BMP or continuous
raw-frame upload is used.

Set `PLATE_TYPE_MODEL` to a full RKNN path to select an older or experimental
plate-type classifier without changing the police, black, or yellow OCR models.

## HTTP API

```text
GET  /api/v1/status
GET  /api/v1/results
GET  /api/v1/frame.jpg
PUT  /api/v1/source                         {"source":"fpga"|"phone"}
POST /api/v1/pipeline/pause                 {}
POST /api/v1/pipeline/resume                {}
POST /api/v1/pipeline/restart               {}
POST /whip/phone                            WHIP SDP offer
PATCH/DELETE /whip/phone/<session>          WHIP session lifecycle
```

WHIP `Location`, `ETag`, `If-Match`, `Link`, and `Accept-Patch` headers are
forwarded. MediaMTX session locations are rewritten under the HTTPS origin.
State-changing API requests are same-origin only and use
`Content-Type: application/json`; WHIP methods enforce their standard SDP
media types. When codec preferences are available, the browser offer contains
H.264 only so it cannot negotiate a codec that the fixed MPP path cannot decode.

## Browser WHIP Test

The end-to-end browser harness needs Playwright with Chromium, GStreamer
`videotestsrc` and `y4menc`, FFmpeg, OpenSSL, and the exact MediaMTX v1.19.2
binary. With those commands available natively, run:

```bash
./web_control/test_whip_videotestsrc.py \
  --mediamtx-command /opt/mediamtx-v1.19.2/bin/mediamtx
```

The command options also accept quoted command prefixes, so the ARM64
MediaMTX and SDK GStreamer binaries can be exercised through QEMU on a
development host. The test generates an exact 1280x720, 15fps synthetic
camera stream, launches a temporary HTTPS control server and MediaMTX, starts
the real page publisher, and decodes at least 30 H.264 frames back from RTSP.
It also forces peer and page lifecycle recovery and confirms those recoveries
do not override an explicit OV5640 selection.

WebRTC can adapt the transmitted resolution under test load. The test requires
a non-empty 16:9 H.264 stream; the board source remains responsible for the
final aspect-preserving 1280x720 normalization. All temporary processes,
certificates, sockets, and video files are removed afterward. This Chromium
test does not substitute for the iPhone Safari checks.

## Board MPP Downstream Test

The network source must decode H.264 in hardware and normalize to read-only
1280x720 BGRx with aspect-preserving borders. The required shape is equivalent
to:

```text
rtspsrc location=rtsp://127.0.0.1:8554/phone protocols=tcp
! rtph264depay ! h264parse ! mppvideodec
! videoconvert ! videoscale add-borders=true
! video/x-raw,format=BGRx,width=1280,height=720
! appsink max-buffers=1 drop=true sync=false
```

Confirm the installed plugins on the board:

```bash
./web_control/check_board_deps.sh
```

With the production MediaMTX service stopped, run the synthetic downstream test:

```bash
./web_control/test_videotestsrc.sh /opt/mediamtx-v1.19.2/bin/mediamtx
```

The script starts a temporary loopback-only RTMP publisher port solely because
the SDK has no WHIP test publisher. It sends `videotestsrc` as H.264, then reads
the fixed `rtsp://127.0.0.1:8554/phone` path through the complete
`rtspsrc/rtph264depay/h264parse/MPP/videoconvert/videoscale/appsink` chain.
Production `mediamtx.yml` keeps RTMP disabled. This proves the downstream
network-source path, not browser WHIP or any iPhone behavior.

Run the pixel-level normalization test in both aspect-ratio directions:

```bash
./web_control/test_phone_letterbox_pipeline.sh
LETTERBOX_CASE=top ./web_control/test_phone_letterbox_pipeline.sh
```

The test inspects a complete 1280x720 BGRx frame. It verifies that the active
image remains proportional and the resulting left/right or top/bottom padding
is black. Its default temporary raw frame is removed automatically; set
`OUTPUT` explicitly to retain a frame for inspection.

## Hardware-Free Tests

The tests create temporary Unix socket and MediaMTX signaling simulators. They
do not require DMA, RKNN, GStreamer, MediaMTX, TLS, or network access.

```bash
cd web_control
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest -v test_server.py
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest -v test_whip_videotestsrc_unit.py
node --check static/app.js
bash -n install_mediamtx.sh tls/create_local_ca.sh check_board_deps.sh test_videotestsrc.sh test_phone_letterbox_pipeline.sh
python3 -m py_compile server.py test_server.py test_whip_videotestsrc.py test_whip_videotestsrc_unit.py
```

The unit tests validate server routing, framing, and WHIP harness construction.
The separate Chromium harness validates actual browser WHIP publication through
MediaMTX and H.264 RTSP readback. The board-native RK3568 MediaMTX/MPP
deployment, 30-minute OV5640 run, and iPhone Safari camera,
lock/background/recovery behavior still require board and phone testing.
