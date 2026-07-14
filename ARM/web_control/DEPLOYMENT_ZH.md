# PPLCNet 双摄像头板端部署指南

本文说明如何把当前 `pplcnet_bgp_live` 部署到 RK3568 板卡，并通过 iPhone Safari 使用手机后摄像头。

```text
iPhone Safari -> HTTPS 页面 -> WHIP/WebRTC -> MediaMTX
  -> rtsp://127.0.0.1:8554/phone -> GStreamer/MPP
  -> 1280x720 BGRx -> RKNN 推理 -> HDMI 与网页预览
```

OV5640 继续使用现有 FPGA PCIe DMA。手机 2 秒没有新帧时回退 OV5640；手机连续恢复 3 秒后，如果期望来源仍为手机，则自动切回手机。

## 1. 示例环境与原则

本文使用以下示例，请替换成实际值：

```bash
BOARD_USER=linaro
BOARD_IP=192.168.1.50
BOARD_HOST=pg2l50h.home.arpa
BOARD_DIR=/home/linaro/ARM
```

部署原则：

- 板卡使用固定局域网 IP，路由器 DNS 将 `pg2l50h.home.arpa` 指向该 IP。
- FPGA DMA 驱动、bitstream、RKNN 运行库和模型沿用当前已验证版本。
- 本次只更新 ARM 用户态程序和 `web_control`，不修改 FPGA RTL、PCIe DMA ABI 或 RKNN 模型。
- 控制面板没有登录认证，只允许可信局域网访问，禁止映射到公网。
- 首次部署先手动启动，确认正常后再配置 systemd。

## 2. 开发机使用 Docker 编译

板端程序必须在指定 Docker 镜像中交叉编译。当前最新源码位于独立 worktree，因此应挂载
`/home/wzzz/VHDL_Project/.worktrees/phone-camera-input/ARM`，不要误挂载仍包含旧源码的
`/home/wzzz/ARM`。

先确认镜像存在：

```bash
sudo docker image inspect cdc81f835218 >/dev/null
```

进入交互式编译容器：

```bash
sudo docker run --rm --privileged -it -u root \
  -v /home/wzzz/VHDL_Project/.worktrees/phone-camera-input/ARM:/app \
  -v /home/wzzz/RK3568J_SDK:/home/hjf/SDK \
  -w /app \
  cdc81f835218 /bin/bash
```

进入容器后执行：

```bash
# 先运行全部主机侧单元测试
make test

# -B 强制重新编译，避免误用旧二进制
make -B pplcnet-bgp-live cplus-rk3568-driver

# 确认输出是 ARM64，而不是开发机的 x86_64 程序
aarch64-linux-gnu-readelf -h pplcnet_bgp_live | sed -n '1,20p'
sha256sum pplcnet_bgp_live
exit
```

`readelf` 输出必须包含 `Class: ELF64` 和 `Machine: AArch64`。由于 `/app` 是宿主机目录的
绑定挂载，退出容器后，生成的 `pplcnet_bgp_live` 已直接位于 worktree 的 `ARM` 目录，
不需要再从容器复制。

不需要进入交互式终端时，也可以直接执行一次性编译：

```bash
sudo docker run --rm --privileged -u root \
  -v /home/wzzz/VHDL_Project/.worktrees/phone-camera-input/ARM:/app \
  -v /home/wzzz/RK3568J_SDK:/home/hjf/SDK \
  -w /app \
  cdc81f835218 \
  make -B pplcnet-bgp-live cplus-rk3568-driver
```

本次实际使用上述镜像完成了强制全量编译。当前产物为 165936 字节，SHA256 为：

```text
40b3bc67f5e1fdc107737f37abba1f846f0fd028b34f1f28aefe83a3a6bd1a3b
```

网页行人 mask 需要本次重新编译的 `cplus-rk3568-driver`。当前验证产物为 71472 字节，
SHA256 为：

```text
2ab1fd8190906c8eca19305f74103806bccbdfd3754a6c182cccc3aeba551b98
```

同一容器内的 `make test` 已实际执行：C 测试、CPlus 测试和 9 项网页/驱动管理测试全部
通过，其中姿态解码测试汇总为 `21 tests, 0 failures`。编译姿态解码测试时有未使用参数
警告，但没有编译错误或测试失败。

源代码、编译器或依赖变化后校验值也会变化，这是正常的；部署时应记录实际生成文件的值。

## 3. 复制到板卡

开发机执行：

```bash
# 按实际环境修改；即使从本节开始操作也要先设置这些变量
BOARD_USER=linaro
BOARD_IP=192.168.1.50
BOARD_DIR=/home/linaro/ARM

cd /home/wzzz/VHDL_Project/.worktrees/phone-camera-input/ARM
ssh "${BOARD_USER}@${BOARD_IP}" "mkdir -p '${BOARD_DIR}'"
scp pplcnet_bgp_live cplus-rk3568-driver \
  "${BOARD_USER}@${BOARD_IP}:/tmp/"
rsync -av --exclude 'tls/generated/' web_control/ \
  "${BOARD_USER}@${BOARD_IP}:${BOARD_DIR}/web_control/"

ssh "${BOARD_USER}@${BOARD_IP}" \
  "sudo install -m 0755 /tmp/pplcnet_bgp_live '${BOARD_DIR}/pplcnet_bgp_live' && \
   sudo install -m 0755 /tmp/cplus-rk3568-driver '${BOARD_DIR}/cplus-rk3568-driver'"
```

板卡执行：

```bash
ssh "${BOARD_USER}@${BOARD_IP}"
cd /home/linaro/ARM
chmod +x pplcnet_bgp_live
chmod +x cplus-rk3568-driver
chmod +x web_control/*.sh web_control/*.py web_control/tls/*.sh
```

当前启动脚本使用板端已验证的 `/userdata/model`。不要重新转换模型。

## 4. 板端依赖检查

### 4.1 设备和动态库

```bash
ls -l /dev/fpga_dma0 /dev/dri/card0
sudo dmesg -T | tail -200

cd /home/linaro/ARM
ldd ./pplcnet_bgp_live | sort
ldd ./pplcnet_bgp_live | grep "not found" || true
```

第二条 `ldd` 命令不应输出缺失库。重点确认 `librknnrt`、GStreamer、GLib 和 DRM。

如果 RKNN 库不在系统搜索路径，可临时设置：

```bash
export LD_LIBRARY_PATH=/path/to/rknn/lib:${LD_LIBRARY_PATH:-}
```

正式部署时应把路径写入启动脚本或系统动态库配置。

### 4.2 GStreamer、MPP 和 HDMI

```bash
./web_control/check_board_deps.sh
gst-inspect-1.0 kmssink >/dev/null && echo "[PASS] kmssink"
```

脚本每一项都应为 `[PASS]`，特别是：

```text
rtspsrc  rtph264depay  h264parse  appsink  jpegenc
videoconvert  videoscale  mppvideodec
```

生产链路固定使用 `mppvideodec`。只有 `mpph264dec` 仍然不满足要求。

## 5. 安装 MediaMTX v1.19.2

```bash
cd /home/linaro/ARM
sudo ./web_control/install_mediamtx.sh /opt/mediamtx-v1.19.2
```

预期最后输出：

```text
Installed: /opt/mediamtx-v1.19.2/bin/mediamtx
Verified SHA256: 562f419912a8668c18216a9e8c95359ec82fbb754e4a44e2953ef62b98eec688
```

检查：

```bash
file /opt/mediamtx-v1.19.2/bin/mediamtx
/opt/mediamtx-v1.19.2/bin/mediamtx --version
```

版本必须为 `v1.19.2`，架构必须为 ARM64。

## 6. 配置 MediaMTX 与网络

编辑 `/home/linaro/ARM/web_control/mediamtx.yml`，把主机名和固定 IP 都放入候选地址：

```yaml
webrtcAdditionalHosts: [pg2l50h.home.arpa, 192.168.1.50]
```

关键配置保持：

```yaml
rtspAddress: 127.0.0.1:8554
rtspTransports: [tcp]
webrtcAddress: 127.0.0.1:8889
webrtcLocalUDPAddress: :8189
webrtcLocalTCPAddress: ""

rtmp: no
hls: no
srt: no
moq: no
```

端口用途：

- `8443/tcp`：HTTPS 页面和同源 WHIP 代理。
- `8889/tcp`：MediaMTX WHIP，只监听本机。
- `8554/tcp`：手机 RTSP，只监听本机。
- `8189/udp`：手机与板卡之间的 WebRTC 视频。

路由器添加：

```text
pg2l50h.home.arpa -> 192.168.1.50
```

局域网验证：

```bash
getent hosts pg2l50h.home.arpa
ping -c 2 pg2l50h.home.arpa
```

只向可信局域网开放 `8443/tcp` 和 `8189/udp`。不要开放到公网，也不要开放 `8554` 或 `8889`。

UFW 示例：

```bash
sudo ufw allow from 192.168.1.0/24 to any port 8443 proto tcp
sudo ufw allow from 192.168.1.0/24 to any port 8189 proto udp
```

把网段改成实际局域网。

手动验证配置：

```bash
cd /home/linaro/ARM
/opt/mediamtx-v1.19.2/bin/mediamtx ./web_control/mediamtx.yml
```

看到 RTSP 和 WebRTC 启动日志后按 `Ctrl+C` 停止。

## 7. 生成 HTTPS 本地 CA

```bash
cd /home/linaro/ARM
./web_control/tls/create_local_ca.sh 192.168.1.50 pg2l50h.home.arpa
```

生成：

```text
web_control/tls/generated/ca.crt
web_control/tls/generated/ca.key
web_control/tls/generated/server.crt
web_control/tls/generated/server.key
```

注意：

- iPhone 只需要 `ca.crt`。
- `ca.key` 是根 CA 私钥，不能传到手机或公开。
- Web 服务使用 `server.crt` 和 `server.key`。
- IP 或主机名变化后需要重新生成服务器证书。
- 重新生成 CA 后，手机需要重新安装和信任新 CA。

检查证书：

```bash
openssl verify \
  -CAfile web_control/tls/generated/ca.crt \
  web_control/tls/generated/server.crt

openssl x509 -in web_control/tls/generated/server.crt \
  -noout -subject -issuer -ext subjectAltName
```

第一条应输出 `OK`，第二条应同时包含域名和固定 IP。

## 8. iPhone 安装 CA

只把 `web_control/tls/generated/ca.crt` 传到 iPhone，不要传任何 `.key` 文件。

1. 打开 `ca.crt`，允许下载描述文件。
2. 在“设置 -> 通用 -> VPN 与设备管理”安装描述文件。
3. 在“设置 -> 通用 -> 关于本机 -> 证书信任设置”完全信任 `PPLCNet Local CA`。
4. 完全关闭并重新打开 Safari。

固定使用：

```text
https://pg2l50h.home.arpa:8443
```

## 9. 运行权限

识别程序通常需要 root 访问 FPGA DMA 和 DRM，Web 服务不需要 root。建立共享组：

```bash
sudo groupadd -f pplcnet
sudo usermod -aG pplcnet linaro
sudo install -d -o root -g pplcnet -m 0770 /run/pplcnet-bgp-live

cd /home/linaro/ARM
sudo chown root:pplcnet web_control/tls/generated/server.key
sudo chmod 0640 web_control/tls/generated/server.key
sudo chmod 0644 web_control/tls/generated/server.crt
chmod 0600 web_control/tls/generated/ca.key
```

## 10. 模型参数

当前板端模型目录与必需文件：

```bash
MODEL_DIR=/userdata/model
test -r "${MODEL_DIR}/best_int8_5color_scorex256_rk3568.rknn"
test -r "${MODEL_DIR}/pplcnet_blue_v3_rk3568_fp16.rknn"
test -r "${MODEL_DIR}/pplcnet_green_v2_b1plus_rk3568_fp16.rknn"
test -r "${MODEL_DIR}/special_keys.txt"
test -r "${MODEL_DIR}/pplcnet_green_keys.txt"
```

完整路由还使用：

```text
pplcnet_police_v5_whiteexpand_rk3568_fp16.rknn + police_keys.txt
pplcnet_black_unified_v1_rk3568_fp16.rknn + black_unified_keys.txt
pplcnet_yellow_all_single_v1_rk3568_fp16.rknn + yellow_keys.txt
plate_type_classifier_5color_large_green_v2_rk3568_fp16_opt0.rknn
```

黑牌统一模型通过现有 embassy 路由参数加载。黄色牌继续使用 Yellow7，包括末位“学/挂”。

### 10.1 临时指定其他五色分类器

启动脚本默认使用上面的 `large_green_v2` 分类器。需要对比其他兼容五色分类器时，
可通过完整路径覆盖，实时车牌和网页单图识别会使用同一个文件：

```bash
cd /home/linaro/ARM
sudo env PLATE_TYPE_MODEL=/userdata/model/other_compatible_5color.rknn \
  ./web_control/start_board.sh
```

覆盖模型必须保持 `blue, green, yellow, white, black` 的五类顺序。取消该变量后恢复
`large_green_v2` 默认分类器。OCR 专家不会因这个变量发生变化。

## 11. 首次手动启动

打开三个 SSH 终端，按顺序启动。

### 11.1 终端一：MediaMTX

```bash
cd /home/linaro/ARM
sudo -u linaro /opt/mediamtx-v1.19.2/bin/mediamtx \
  ./web_control/mediamtx.yml
```

没有手机发布时，`phone` 路径离线或 RTSP 返回 404 是正常的。

### 11.2 终端二：识别程序

网页服务与手动运行共用 `run_plate_live.sh`。单独验证最新五色车牌链路时执行：

```bash
cd /home/linaro/ARM
sudo env MODEL_DIR=/userdata/model ./web_control/run_plate_live.sh
```

脚本固定加载 `best_int8_5color_scorex256_rk3568.rknn` 和
`plate_type_classifier_5color_large_green_v2_rk3568_fp16_opt0.rknn`，并使用 30 fps、
分数缩放 256、检测阈值 0.35、NMS 0.35、最多 9 个检测框。五路 OCR 模型与 keys
也都由该脚本统一传入。

完整网页服务不要先手动启动该命令，直接运行 `start_board.sh`，它会调用相同脚本并创建
控制 socket、MediaMTX 和 HTTPS 服务。

检查 socket：

```bash
ls -l /run/pplcnet-bgp-live/control.sock
```

预期类似：

```text
srw-rw---- root pplcnet ... control.sock
```

### 11.3 终端三：HTTPS 服务

```bash
cd /home/linaro/ARM
sudo -u linaro -g pplcnet python3 ./web_control/server.py \
  --host 192.168.1.50 \
  --port 8443 \
  --cert-file ./web_control/tls/generated/server.crt \
  --key-file ./web_control/tls/generated/server.key \
  --control-socket /run/pplcnet-bgp-live/control.sock \
  --mediamtx-host 127.0.0.1 \
  --mediamtx-port 8889
```

`--host` 必须是板卡具体 IP，程序会拒绝 `0.0.0.0` 和 `::`。

## 12. API 检查

局域网电脑执行：

```bash
BASE=https://pg2l50h.home.arpa:8443
CA=/path/to/ca.crt

curl --cacert "$CA" "$BASE/api/v1/status"
curl --cacert "$CA" "$BASE/api/v1/results"
curl --cacert "$CA" -o frame.jpg "$BASE/api/v1/frame.jpg"
```

切换来源：

```bash
curl --cacert "$CA" -X PUT "$BASE/api/v1/source" \
  -H 'Content-Type: application/json' -d '{"source":"phone"}'
curl --cacert "$CA" -X PUT "$BASE/api/v1/source" \
  -H 'Content-Type: application/json' -d '{"source":"fpga"}'
```

控制流水线：

```bash
curl --cacert "$CA" -X POST "$BASE/api/v1/pipeline/pause" \
  -H 'Content-Type: application/json' -d '{}'
curl --cacert "$CA" -X POST "$BASE/api/v1/pipeline/resume" \
  -H 'Content-Type: application/json' -d '{}'
curl --cacert "$CA" -X POST "$BASE/api/v1/pipeline/restart" \
  -H 'Content-Type: application/json' -d '{}'
```

## 13. iPhone 与故障切换测试

打开 `https://pg2l50h.home.arpa:8443`：

1. 页面应显示“已连接”，初始画面为 OV5640。
2. 点击“启动手机摄像头”并允许权限。
3. 页面只请求视频，不应请求麦克风。
4. 状态变为“正在发送”后选择“手机”。
5. `desired source` 和 `active source` 应先后变为手机。
6. HDMI 与网页画面都应切换到手机。

页面请求后摄、理想 1280x720、最高 15 fps。WebRTC 可能降低实际分辨率；板端会保持比例并补黑边到 1280x720。

至少重复 5 次：

```text
OV5640 -> 启动并选择手机 -> 停止手机/锁屏
 -> 约 2 秒后回退 OV5640 -> 恢复 Safari
 -> 连续稳定约 3 秒后切回手机
```

重点观察：

- 切换后没有旧来源检测框或旧 OCR 闪回。
- desired、active 和故障原因正确。
- 手机必须连续恢复 3 秒才切回。
- 用户明确选择 OV5640 后，Safari 恢复不会擅自选手机。
- FPS、帧龄和各类丢帧持续更新。

iOS 在锁屏或后台暂停 Safari 是正常的；板端应自动回退，回到前台后页面会尝试恢复发布。

## 14. 板端专项测试

运行 MPP 测试前先停止正式 MediaMTX，避免端口冲突：

```bash
sudo systemctl stop mediamtx-phone 2>/dev/null || true
pkill -f '/opt/mediamtx-v1.19.2/bin/mediamtx' 2>/dev/null || true

cd /home/linaro/ARM
./web_control/test_videotestsrc.sh \
  /opt/mediamtx-v1.19.2/bin/mediamtx
```

预期：

```text
[PASS] videotestsrc -> H.264 -> MediaMTX -> RTSP -> MPP -> BGRx appsink
```

补边像素测试：

```bash
./web_control/test_phone_letterbox_pipeline.sh
LETTERBOX_CASE=top ./web_control/test_phone_letterbox_pipeline.sh
```

两条都应输出 `[PASS]`。

## 15. systemd 开机服务

手动运行通过后再配置。

### 15.1 模型环境文件

创建 `/etc/pplcnet-bgp-live.env`：

```bash
sudo tee /etc/pplcnet-bgp-live.env >/dev/null <<'EOF'
PLATE_MODEL=/home/linaro/models/plate_pose.rknn
OCR_BLUE_MODEL=/home/linaro/models/ocr_blue.rknn
OCR_GREEN_MODEL=/home/linaro/models/ocr_green.rknn
BLUE_KEYS=/home/linaro/models/keys_blue.txt
GREEN_KEYS=/home/linaro/models/keys_green.txt
OCR_POLICE_MODEL=
POLICE_KEYS=
OCR_EMBASSY_MODEL=
EMBASSY_KEYS=
OCR_YELLOW_MODEL=
YELLOW_KEYS=
PTYPE_MODEL=
EOF
sudo chmod 0600 /etc/pplcnet-bgp-live.env
```

### 15.2 识别程序启动脚本

创建 `/home/linaro/ARM/start_pplcnet_bgp_live.sh`：

```bash
sudo tee /home/linaro/ARM/start_pplcnet_bgp_live.sh >/dev/null <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
set -a
source /etc/pplcnet-bgp-live.env
set +a

cmd=(/home/linaro/ARM/pplcnet_bgp_live
  --plate-model "$PLATE_MODEL"
  --ocr-blue-model "$OCR_BLUE_MODEL"
  --ocr-green-model "$OCR_GREEN_MODEL"
  --ocr-blue-keys "$BLUE_KEYS"
  --ocr-green-keys "$GREEN_KEYS"
  --source fpga
  --phone-rtsp rtsp://127.0.0.1:8554/phone
  --control-socket /run/pplcnet-bgp-live/control.sock)

if [[ -n "${OCR_POLICE_MODEL:-}" || -n "${POLICE_KEYS:-}" ]]; then
  [[ -n "${OCR_POLICE_MODEL:-}" && -n "${POLICE_KEYS:-}" ]]
  cmd+=(--ocr-police-model "$OCR_POLICE_MODEL" --ocr-police-keys "$POLICE_KEYS")
fi
if [[ -n "${OCR_EMBASSY_MODEL:-}" || -n "${EMBASSY_KEYS:-}" ]]; then
  [[ -n "${OCR_EMBASSY_MODEL:-}" && -n "${EMBASSY_KEYS:-}" ]]
  cmd+=(--ocr-embassy-model "$OCR_EMBASSY_MODEL" --ocr-embassy-keys "$EMBASSY_KEYS")
fi
if [[ -n "${OCR_YELLOW_MODEL:-}" || -n "${YELLOW_KEYS:-}" ]]; then
  [[ -n "${OCR_YELLOW_MODEL:-}" && -n "${YELLOW_KEYS:-}" ]]
  cmd+=(--ocr-yellow-model "$OCR_YELLOW_MODEL" --ocr-yellow-keys "$YELLOW_KEYS")
fi
if [[ -n "${PTYPE_MODEL:-}" ]]; then
  cmd+=(--plate-type-classifier-model "$PTYPE_MODEL")
fi
exec "${cmd[@]}"
EOF
sudo chown root:pplcnet /home/linaro/ARM/start_pplcnet_bgp_live.sh
sudo chmod 0750 /home/linaro/ARM/start_pplcnet_bgp_live.sh
```

### 15.3 MediaMTX 服务

`/etc/systemd/system/mediamtx-phone.service`：

```ini
[Unit]
Description=MediaMTX phone camera router
Wants=network-online.target
After=network-online.target
[Service]
Type=simple
User=linaro
Group=pplcnet
WorkingDirectory=/home/linaro/ARM
ExecStart=/opt/mediamtx-v1.19.2/bin/mediamtx /home/linaro/ARM/web_control/mediamtx.yml
Restart=on-failure
RestartSec=2
[Install]
WantedBy=multi-user.target
```

### 15.4 识别服务

`/etc/systemd/system/pplcnet-bgp-live.service`：

```ini
[Unit]
Description=PPLCNet dual-source live recognition
Requires=mediamtx-phone.service
After=mediamtx-phone.service
[Service]
Type=simple
User=root
Group=pplcnet
WorkingDirectory=/home/linaro/ARM
RuntimeDirectory=pplcnet-bgp-live
RuntimeDirectoryMode=0770
ExecStart=/home/linaro/ARM/start_pplcnet_bgp_live.sh
Restart=on-failure
RestartSec=2
TimeoutStopSec=15
[Install]
WantedBy=multi-user.target
```

### 15.5 Web 服务

`/etc/systemd/system/pplcnet-web-control.service`，把 IP 改成实际值：

```ini
[Unit]
Description=PPLCNet LAN HTTPS control panel
Requires=mediamtx-phone.service pplcnet-bgp-live.service
After=mediamtx-phone.service pplcnet-bgp-live.service
[Service]
Type=simple
User=linaro
Group=pplcnet
WorkingDirectory=/home/linaro/ARM
ExecStart=/usr/bin/python3 /home/linaro/ARM/web_control/server.py --host 192.168.1.50 --port 8443 --cert-file /home/linaro/ARM/web_control/tls/generated/server.crt --key-file /home/linaro/ARM/web_control/tls/generated/server.key --control-socket /run/pplcnet-bgp-live/control.sock --mediamtx-host 127.0.0.1 --mediamtx-port 8889
Restart=on-failure
RestartSec=2
NoNewPrivileges=true
PrivateTmp=true
[Install]
WantedBy=multi-user.target
```

启用与检查：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now mediamtx-phone.service
sudo systemctl enable --now pplcnet-bgp-live.service
sudo systemctl enable --now pplcnet-web-control.service

sudo systemctl --no-pager --full status mediamtx-phone.service
sudo systemctl --no-pager --full status pplcnet-bgp-live.service
sudo systemctl --no-pager --full status pplcnet-web-control.service
```

实时日志：

```bash
sudo journalctl -f -u mediamtx-phone.service \
  -u pplcnet-bgp-live.service -u pplcnet-web-control.service
```

## 16. 常见问题

### 页面打不开

```bash
getent hosts pg2l50h.home.arpa
ss -lntp | grep 8443
sudo systemctl status pplcnet-web-control.service
```

确认 CA 已完全信任、地址为 HTTPS、手机与板卡在同一局域网。

### Safari 不询问摄像头权限

- 页面必须是受信任 HTTPS。
- 检查 Safari 站点摄像头权限。
- 完全关闭 Safari 后重新打开。
- 当前页面不请求麦克风；出现麦克风请求说明页面版本不对。

### 手机“正在发送”，active 仍为 OV5640

确认 desired 已选择手机，然后查看：

```bash
sudo journalctl -u mediamtx-phone.service -n 100 --no-pager
sudo journalctl -u pplcnet-bgp-live.service -n 100 --no-pager
```

常见原因是 `8189/udp` 被阻断、候选地址错误、Wi-Fi 客户端隔离、MediaMTX 尚未收到 H.264，或缺少 `mppvideodec`。

### 状态接口返回 503

```bash
ls -l /run/pplcnet-bgp-live/control.sock
id linaro
sudo -u linaro -g pplcnet test -r /run/pplcnet-bgp-live/control.sock
```

socket 不存在表示识别程序未启动；不可访问时检查组是否为 `pplcnet`。

### 网页画面卡顿

当前平衡档为 480x270、JPEG 质量 55、最高 8 帧/秒。它用更小的单帧换取更连续的画面，通常不会增加局域网流量。更新程序后必须同时替换 `pplcnet_bgp_live`、`web_control/server.py` 和 `web_control/static/app.js`，然后重启；只更新网页文件或只更新 C 程序都仍会受到旧的 5 帧/秒限制。

可在板端确认新预览已启动：

```bash
grep -F "[preview] started 480x270 JPEG quality=55 max_fps=8" \
  /var/log/pplcnet-board/plate.log
```

### 网页没有预览图

```bash
gst-inspect-1.0 jpegenc
curl --cacert /path/to/ca.crt -o /tmp/frame.jpg \
  https://pg2l50h.home.arpa:8443/api/v1/frame.jpg
file /tmp/frame.jpg
```

切换、暂停或重启时旧 JPEG 会被清除，短暂显示“等待新源画面”是正常的。

### HDMI 无显示

```bash
gst-inspect-1.0 kmssink
ls -l /dev/dri/card0
sudo journalctl -u pplcnet-bgp-live.service -n 100 --no-pager
```

检查 DRM 是否被桌面或其他程序占用。

### NPU 或模型加载失败

```bash
ldd /home/linaro/ARM/pplcnet_bgp_live | grep "not found" || true
ls -l /home/linaro/models
sudo journalctl -u pplcnet-bgp-live.service -n 200 --no-pager
```

确认路径、权限、keys、RKNN 运行库和驱动版本，不要用重新转换模型掩盖部署错误。

### 端口冲突

```bash
sudo ss -lntup | grep -E '(:8443|:8554|:8889|:8189)'
```

正常情况：8443 绑定板卡局域网 IP；8554 和 8889 绑定 `127.0.0.1`；8189 为 WebRTC UDP。

## 17. 上板验收清单

- [ ] OV5640 上电后持续显示。
- [ ] RKNN 检测和多路 OCR 正常。
- [ ] 黄色牌末位“学/挂”可识别和显示。
- [ ] HDMI 保持 1280x720 BGRx。
- [ ] HTTPS 域名访问无证书警告。
- [ ] Safari 只请求摄像头并优先后摄。
- [ ] 手机发布后能成为 active source。
- [ ] 手机断流约 2 秒后回退 OV5640。
- [ ] 手机连续恢复约 3 秒后切回。
- [ ] 连续切换至少 5 次无旧框或旧 OCR 闪回。
- [ ] FPS、帧龄和丢帧指标更新。
- [ ] 暂停、继续和重启可用。
- [ ] iPhone 锁屏、后台、恢复符合预期。
- [ ] OV5640 连续运行 30 分钟无新增错误。
- [ ] 三个 systemd 服务重启后可自动恢复。

## 18. 停止与回滚

```bash
sudo systemctl disable --now pplcnet-web-control.service
sudo systemctl disable --now pplcnet-bgp-live.service
sudo systemctl disable --now mediamtx-phone.service
```

本次部署没有修改 FPGA RTL、PCIe DMA ABI 或 RKNN 模型。回滚时恢复旧的 `pplcnet_bgp_live` 二进制并停止新增服务即可。不要覆盖现有 bitstream、内核模块或模型文件。

## 19. 动态局域网通用启动

板卡切换到其他局域网后，不需要手工修改 IP。先停止之前在终端中手动运行的 MediaMTX、
`pplcnet_bgp_live` 和 `server.py`，然后执行：

```bash
cd /home/linaro/ARM
sudo ./web_control/start_board.sh
```

脚本会自动完成：

- 从默认路由识别当前板卡 IPv4。
- 把当前 IP 写入运行时 MediaMTX WebRTC 候选地址。
- 检查 HTTPS 证书是否包含当前 IP。
- IP 变化时只续签服务器证书，保留原来的项目根 CA。
- 启动 MediaMTX、实时车牌程序和 HTTPS 网页服务。
- 把 PID 写入 `/run/pplcnet-board`，日志写入 `/var/log/pplcnet-board`。

成功时会直接打印手机访问地址，例如：

```text
启动完成
当前 IP: 192.168.137.247
手机访问: https://192.168.137.247:8443
```

查看当前地址和日志：

```bash
sudo cat /run/pplcnet-board/current.env
sudo tail -f /var/log/pplcnet-board/mediamtx.log \
  /var/log/pplcnet-board/plate.log \
  /var/log/pplcnet-board/web.log
```

停止脚本管理的进程：

```bash
cd /home/linaro/ARM
sudo ./web_control/stop_board.sh
```

如果板卡有多个出口，或者自动识别的 IP 不对，可明确指定：

```bash
cd /home/linaro/ARM
sudo env BOARD_IP=192.168.10.50 ./web_control/start_board.sh
```

首次生成根 CA 后，仍需在 iPhone 安装并完全信任一次 `ca.crt`。以后 IP 变化只重新签发
服务器证书，根 CA 不变，iPhone 不需要重复安装。板卡 IP 变化后仍需按实际网段开放
`8443/tcp` 和 `8189/udp`。

## 20. 驱动模式切换与手机上传图片识别

通用启动脚本启动网页后，Safari 打开脚本打印的 HTTPS 地址。在“识别模式与图片”区域：

1. 选择“车牌”或“行人”，该选择会实际切换板端运行状态。
2. 从手机相册选择 JPEG 或 PNG 图片。
3. 点击“上传并识别”。
4. 等待板端返回结果，页面会在原图上绘制识别框。
5. 点击“返回实时画面”继续查看 OV5640。

上传限制为 15 MiB，一次只执行一个图片任务。板端会保持原图比例，把图片补边转换为
1280x720 BGRx，不会拉伸原图。

两种模式的资源行为如下：

- 车牌模式：常驻运行 `pplcnet_bgp_live`，OV5640 继续实时识别和 HDMI 显示。
- 车牌图片：短暂停止实时车牌进程，使用同一套模型识别上传图片，完成后自动恢复实时进程。
- 行人模式：停止实时车牌进程，不打开 OV5640，也不启动常驻 CPlus 进程。
- 行人图片：收到上传请求后单次运行 CPlus，返回结果后立即退出。

因此两个驱动不会同时占用 FPGA、NPU 或 HDMI。行人模式只处理手机上传的单张图片，
不使用板端摄像头，也不使用手机实时摄像头。

车牌模式使用当前 `pplcnet_bgp_live` 的单图入口，并加载 `/userdata/model` 下的现有车牌
检测、板型分类和五路 OCR 模型。行人模式使用：

实时视频与网页单图使用同一套五色检测参数：30 fps、分数缩放 256、检测阈值 0.35、
NMS 0.35、最多保留 9 个检测框。这样同一张图片在本地命令和网页入口中的后处理行为一致。

```text
/home/linaro/ARM/cplus-rk3568-driver
```

以及 `/userdata/model` 下现有的 CPlus 检测和分割模型。网页使用
`--output-mask-bgrx` 导出与 HDMI 相同配色的半透明 mask，再编码为 JPEG 返回手机：
道路为绿色、人行道为蓝色、斑马线为红色。CPlus 可执行文件不存在或仍是旧版本时，实时
车牌功能仍可启动，但网页行人图片模式会返回明确错误。

接口验证示例：

```bash
curl --cacert ./web_control/tls/generated/ca.crt \
  -H 'Content-Type: image/jpeg' \
  -H 'X-Inference-Mode: plate' \
  --data-binary @/path/to/test.jpg \
  https://板卡当前IP:8443/api/v1/image-inference
```

行人模式把请求头改为：

```text
X-Inference-Mode: pedestrian
```

图片模式会按请求临时调用对应推理程序，不常驻启动第二个 HDMI/DMA 流程，避免两个实时
程序同时争用显示和采集设备。图片任务与实时摄像头画面相互独立。

查询与切换模式：

```bash
BASE=https://板卡当前IP:8443
CA=./web_control/tls/generated/ca.crt

curl --cacert "${CA}" "${BASE}/api/v1/mode"

curl --cacert "${CA}" -X PUT \
  -H 'Content-Type: application/json' \
  -d '{"mode":"pedestrian"}' \
  "${BASE}/api/v1/mode"

curl --cacert "${CA}" -X PUT \
  -H 'Content-Type: application/json' \
  -d '{"mode":"plate"}' \
  "${BASE}/api/v1/mode"
```

车牌模式正常状态应包含 `"plate_running":true`；行人模式应包含
`"plate_running":false` 和 `"pedestrian_on_demand":true`。

## 21. SD 卡照片/视频行人违法推理

网页新增“SD 卡文件”区域，可直接浏览板端 SD 卡上的照片和视频并就地推理，无需通过
手机上传。照片走与手机上传相同的单图入口；视频离线逐帧推理，输出违规事件时间线、
关键违规帧和统计摘要。

### 21.1 挂载 SD 卡并指定根目录

板端默认从 `/mnt/sdcard` 读取。先确认 SD 卡已挂载：

```bash
lsblk        # 找到 SD 卡设备，例如 /dev/mmcblk1p1
sudo mkdir -p /mnt/sdcard
sudo mount /dev/mmcblk1p1 /mnt/sdcard
ls /mnt/sdcard
```

若实际挂载点不同，用 `SD_ROOT` 覆盖（`start_board.sh` 会透传为 `--sd-root`）：

```bash
sudo SD_ROOT=/media/sd start_board.sh
```

`--sd-root` 决定浏览器可见的根目录；所有照片/视频路径都被强制限制在该根下，跨目录
穿越会被拒绝。识别的照片后缀为 `.jpg/.jpeg/.png`，视频为
`.mp4/.mov/.avi/.mkv/.m4v/.h264/.ts/.webm`。

### 21.2 浏览 SD 卡

```bash
curl --cacert "${CA}" "${BASE}/api/v1/sd/list"
curl --cacert "${CA}" "${BASE}/api/v1/sd/list?path=sub"
```

返回 `entries` 列表，每项含 `name`、`path`、`type`（`photo`/`video`/`dir`/`parent`）、
`size`、`mtime`。目录条目可继续下钻，`..` 返回上一级。单次最多返回 2000 项，超出会
标记 `truncated`。

### 21.3 SD 卡照片推理

```bash
curl --cacert "${CA}" -X POST \
  -H 'Content-Type: application/json' \
  -d '{"path":"photo.jpg","mode":"pedestrian"}' \
  "${BASE}/api/v1/sd/photo-inference"
```

与手机上传一致：返回 `results`，行人模式额外返回 `rendered_image`（带 mask 的标注
JPEG）和 `source_image`（1280x720 letterbox 源图）。`mode` 可为 `plate` 或
`pedestrian`。任务与实时流程互斥，照片推理期间会临时停止实时车牌进程。

### 21.4 SD 卡视频推理

视频推理是异步任务。提交后返回 `job_id`，轮询状态直至完成：

```bash
JOB=$(curl --cacert "${CA}" -sX POST \
  -H 'Content-Type: application/json' \
  -d '{"path":"clip.mp4","sample_fps":1}' \
  "${BASE}/api/v1/sd/video-inference" | python3 -c 'import sys,json;print(json.load(sys.stdin)["job_id"])')

curl --cacert "${CA}" "${BASE}/api/v1/sd/video-jobs/${JOB}"
```

`sample_fps` 取值 0.1–10，默认 1。低于 1 时按比例隔帧抽样（例如 0.5 即每 2 秒 1 帧），
适合长视频。任务状态 `running` 时持续轮询；`done` 后 `summary` 给出采样帧数、违规帧
数、违规事件数、跳过帧数和决策分布，`events` 列出每个违规事件的时间、原因和关联的
关键帧索引；`error` 给出失败原因。

关键帧为违规时刻的 mask 标注图：

```bash
curl --cacert "${CA}" "${BASE}/api/v1/sd/video-jobs/${JOB}/keyframes/0.jpg" -o kf0.jpg
```

### 21.5 限制与注意事项

- 视频逐帧推理复用 CPlus 单帧离线入口，每帧都会重启 driver 并重新加载模型，属于离线
  批处理路径，不是实时。30 秒 1fps 视频约需数十秒。
- 同时只允许一个视频推理任务，且与照片/实时推理互斥。
- 单任务最多采样 600 帧，超出会截断并在 `summary.truncated` 标记；最多保留 24 张关键
  帧。需要更长视频可调高 `sample_fps` 或分段。
- 抽帧使用 `decodebin ! videoconvert ! videoscale ! videorate ! multifilesink`，依赖板端
  GStreamer 解码插件；硬解是否启用取决于 `mppvideodec` 是否被 `decodebin` 选中。
- 临时帧文件在任务结束的临时目录中自动清理。

