"use strict";

const FRAME_PERIOD_MS = 125;
const STATUS_PERIOD_MS = 1000;
const RESULTS_PERIOD_MS = 400;
const STALE_FRAME_ERROR = "stale_frame_generation";

const elements = {
  connection: document.getElementById("connectionState"),
  frame: document.getElementById("framePreview"),
  frameState: document.getElementById("frameState"),
  stage: document.getElementById("cameraStage"),
  canvas: document.getElementById("detectionOverlay"),
  desiredSource: document.getElementById("desiredSource"),
  activeSource: document.getElementById("activeSource"),
  failoverReason: document.getElementById("failoverReason"),
  sourceSelector: document.getElementById("sourceSelector"),
  phoneToggle: document.getElementById("phoneCameraToggle"),
  phoneState: document.getElementById("phoneCameraState"),
  publisherVideo: document.getElementById("publisherVideo"),
  imageModeSelector: document.getElementById("imageModeSelector"),
  imageFile: document.getElementById("imageFile"),
  runImageInference: document.getElementById("runImageInference"),
  returnLiveView: document.getElementById("returnLiveView"),
  imageInferenceState: document.getElementById("imageInferenceState"),
  driverModeState: document.getElementById("driverModeState"),
  inputFps: document.getElementById("inputFps"),
  decodeFps: document.getElementById("decodeFps"),
  inferFps: document.getElementById("inferFps"),
  frameAge: document.getElementById("frameAge"),
  inputDrops: document.getElementById("inputDrops"),
  decodeDrops: document.getElementById("decodeDrops"),
  inferDrops: document.getElementById("inferDrops"),
  displayDrops: document.getElementById("displayDrops"),
  count: document.getElementById("detectionCount"),
  resultList: document.getElementById("resultList"),
  sdCurrentPath: document.getElementById("sdCurrentPath"),
  sdFileList: document.getElementById("sdFileList"),
  sdBrowseState: document.getElementById("sdBrowseState"),
  sdRefresh: document.getElementById("sdRefresh"),
  sdSelectedName: document.getElementById("sdSelectedName"),
  runSdPhotoInference: document.getElementById("runSdPhotoInference"),
  runSdVideoInference: document.getElementById("runSdVideoInference"),
  sdVideoFps: document.getElementById("sdVideoFps"),
  sdVideoResult: document.getElementById("sdVideoResult"),
  sdVideoProgressLabel: document.getElementById("sdVideoProgressLabel"),
  sdVideoProgress: document.getElementById("sdVideoProgress"),
  sdVideoSummary: document.getElementById("sdVideoSummary"),
  sdVideoEvents: document.getElementById("sdVideoEvents"),
  sdVideoEventCount: document.getElementById("sdVideoEventCount"),
  sdFrameList: document.getElementById("sdFrameList"),
  sdFrameCount: document.getElementById("sdFrameCount"),
};

const overlayContext = elements.canvas.getContext("2d");
let latestStatus = null;
let latestResults = null;
let latestStatusGeneration = null;
let visualEpoch = 0;
let currentFrameUrl = null;
let publisher = null;
let cameraStarting = false;
let phoneCameraIntent = false;
let imageViewActive = false;
let imageInferenceRunning = false;
let currentDriverMode = "plate";
let driverSwitching = false;
let sdCurrentSubpath = "";
let sdSelectedEntry = null;
let sdVideoJobId = null;
let sdVideoPolling = false;

function sleep(ms) {
  return new Promise((resolve) => window.setTimeout(resolve, ms));
}

function firstValue(object, keys, fallback = null) {
  for (const key of keys) {
    const parts = key.split(".");
    let value = object;
    for (const part of parts) {
      if (value == null || typeof value !== "object" || !(part in value)) {
        value = undefined;
        break;
      }
      value = value[part];
    }
    if (value !== undefined && value !== null) return value;
  }
  return fallback;
}

function unwrapPayload(data, key) {
  if (data && typeof data[key] === "object" && data[key] !== null) return data[key];
  if (data && typeof data.data === "object" && data.data !== null) return data.data;
  return data;
}

function normalizedGeneration(value) {
  if (value === null || value === undefined || value === "") return null;
  const text = String(value);
  return /^[1-9][0-9]*$/.test(text) ? text : null;
}

function statusGeneration(status) {
  return normalizedGeneration(firstValue(status || {}, ["source_generation", "source.generation"]));
}

function resultsGeneration(results) {
  return normalizedGeneration(firstValue(results || {}, ["source_generation", "generation"]));
}

async function fetchWithTimeout(
  path,
  options = {},
  timeoutMs = 4000,
  consumeResponse = async (response) => response,
) {
  const controller = new AbortController();
  const timeout = window.setTimeout(() => controller.abort(), timeoutMs);
  try {
    const response = await fetch(path, {cache: "no-store", ...options, signal: controller.signal});
    if (!response.ok) {
      let message = `${response.status} ${response.statusText}`;
      try {
        const problem = await response.json();
        message = problem?.error?.message || problem?.message || message;
      } catch (_) {
        // Keep the HTTP status when an upstream protocol returns non-JSON.
      }
      throw new Error(message);
    }
    return await consumeResponse(response);
  } finally {
    window.clearTimeout(timeout);
  }
}

async function fetchJson(path, options = {}) {
  return fetchWithTimeout(path, options, 4000, (response) => response.json());
}

function setConnection(connected, label) {
  elements.connection.dataset.state = connected ? "connected" : "error";
  elements.connection.textContent = label;
}

function sourceLabel(source) {
  if (source === "fpga" || source === "ov5640") return "OV5640";
  if (source === "phone") return "手机";
  if (source === "none") return "不使用";
  return source || "--";
}

function failoverLabel(reason) {
  const labels = {
    startup: "正在启动",
    desired_fpga: "已选择 OV5640",
    desired_phone: "已选择手机",
    none: "正常",
    desired_active: "正常",
    phone_unavailable: "手机输入不可用，已回退",
    phone_stale: "手机画面中断，已回退",
    phone_unhealthy: "手机画面异常，已回退",
    phone_recovering: "手机画面恢复确认中",
    phone_recovered: "手机画面已恢复",
    fpga_unhealthy: "OV5640 输入异常",
    pipeline_paused: "流程已暂停",
    pedestrian_image_mode: "行人图片模式",
  };
  return labels[reason] || reason || "正常";
}

function numberText(value, digits = 1) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : "--";
}

function integerText(value) {
  const number = Number(value);
  return Number.isFinite(number) ? String(Math.max(0, Math.round(number))) : "--";
}

function updateSourceSelector(desired) {
  for (const button of elements.sourceSelector.querySelectorAll("button[data-source]")) {
    const selected = button.dataset.source === desired || (desired === "ov5640" && button.dataset.source === "fpga");
    button.setAttribute("aria-checked", selected ? "true" : "false");
  }
}

function updateStatusView(status) {
  const desired = firstValue(status, ["desired_source", "source.desired"], null);
  const active = firstValue(status, ["active_source", "source.active"], null);
  const reason = firstValue(status, ["failover_reason", "source.failover_reason"], "none");
  elements.desiredSource.textContent = sourceLabel(desired);
  elements.activeSource.textContent = sourceLabel(active);
  elements.failoverReason.textContent = failoverLabel(reason);
  elements.failoverReason.dataset.alert = reason && !["none", "desired_active"].includes(reason) ? "true" : "false";
  updateSourceSelector(desired);

  elements.inputFps.textContent = numberText(firstValue(status, ["input_fps", "fps.input", "metrics.input_fps"]));
  elements.decodeFps.textContent = numberText(firstValue(status, ["decode_fps", "fps.decode", "metrics.decode_fps"]));
  elements.inferFps.textContent = numberText(firstValue(status, ["infer_fps", "fps.infer", "metrics.infer_fps"]));
  const age = firstValue(status, ["frame.age_ms", "frame_age_ms", "active_frame_age_ms", "metrics.frame_age_ms"]);
  const ageNumber = Number(age);
  elements.frameAge.textContent = age !== null && age !== "" &&
    Number.isFinite(ageNumber) && ageNumber >= 0
    ? `${Math.round(ageNumber)} ms`
    : "--";
  elements.inputDrops.textContent = integerText(firstValue(status, ["dropped.input", "input_drops", "dropped_frames", "metrics.input_drops"]));
  elements.decodeDrops.textContent = integerText(firstValue(status, ["dropped.decode", "decode_drops", "metrics.decode_drops"]));
  elements.inferDrops.textContent = integerText(firstValue(status, ["dropped.infer", "infer_drops", "metrics.infer_drops"]));
  elements.displayDrops.textContent = integerText(firstValue(status, ["dropped.display", "display_drops", "display_drop_total", "metrics.display_drops"]));
}

const PED_REASON_TEXT = {
  road_dominant_without_zebra: "路面横穿",
  road_dominant_zebra_noise_ignored: "路面横穿",
  sidewalk_suppressed: "人行道",
  zebra_suppressed: "斑马线上",
  weak_ground_evidence: "地面不明",
  road_not_dominant: "非路面",
  rider_filtered: "骑行者",
  two_wheel_filtered: "骑行者",
};

function isPedestrianTarget(target) {
  return target != null && typeof target === "object" && "decision" in target;
}

function isSuspectedDecision(decision) {
  return String(decision || "").startsWith("suspected");
}

function pedestrianReasonText(reason, suspected) {
  return PED_REASON_TEXT[String(reason || "")] || (suspected ? "疑似违规" : "正常");
}

function normalizedPlates(results) {
  const candidates = firstValue(results || {}, ["plates", "detections", "targets", "results"], []);
  return Array.isArray(candidates) ? candidates : [];
}

function plateBox(plate) {
  const box = plate && typeof plate.box === "object" ? plate.box : plate;
  if (!box) return null;
  const x1 = Number(Array.isArray(box) ? box[0] : box.x1);
  const y1 = Number(Array.isArray(box) ? box[1] : box.y1);
  const x2 = Number(Array.isArray(box) ? box[2] : box.x2);
  const y2 = Number(Array.isArray(box) ? box[3] : box.y2);
  if (![x1, y1, x2, y2].every(Number.isFinite) || x2 <= x1 || y2 <= y1) return null;
  return {x1, y1, x2, y2};
}

function resultIsCurrent() {
  if (imageViewActive) return Boolean(latestResults);
  if (!latestResults || latestStatusGeneration === null) return false;
  return resultsGeneration(latestResults) === latestStatusGeneration;
}

function drawOverlay() {
  const rect = elements.stage.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  elements.canvas.width = Math.max(1, Math.round(rect.width * dpr));
  elements.canvas.height = Math.max(1, Math.round(rect.height * dpr));
  overlayContext.setTransform(dpr, 0, 0, dpr, 0, 0);
  overlayContext.clearRect(0, 0, rect.width, rect.height);
  if (!latestResults || !resultIsCurrent()) return;

  const frameWidth = Number(firstValue(latestResults, ["frame.width", "frame_w", "frame_width", "width"], 1280));
  const frameHeight = Number(firstValue(latestResults, ["frame.height", "frame_h", "frame_height", "height"], 720));
  if (!(frameWidth > 0 && frameHeight > 0)) return;
  const scale = Math.min(rect.width / frameWidth, rect.height / frameHeight);
  const offsetX = (rect.width - frameWidth * scale) / 2;
  const offsetY = (rect.height - frameHeight * scale) / 2;

  const colors = {blue: "#38bdf8", green: "#48bf84", yellow: "#f2c14e", police: "#f4f5f6", embassy: "#df6464"};
  overlayContext.lineWidth = 2;
  overlayContext.font = "600 13px system-ui, sans-serif";
  overlayContext.textBaseline = "middle";

  for (const plate of normalizedPlates(latestResults)) {
    const box = plateBox(plate);
    if (!box) continue;
    const x = offsetX + box.x1 * scale;
    const y = offsetY + box.y1 * scale;
    const width = (box.x2 - box.x1) * scale;
    const height = (box.y2 - box.y1) * scale;
    let color;
    let label;
    if (isPedestrianTarget(plate)) {
      const suspected = isSuspectedDecision(plate.decision);
      color = suspected ? "#df6464" : "#48bf84";
      const reason = pedestrianReasonText(plate.reason, suspected);
      const score = Number(plate.score);
      const scoreText = Number.isFinite(score) ? ` ${Math.round(score * 100)}%` : "";
      label = `${suspected ? "⚠ 违法" : "正常"} ${reason}${scoreText}`;
    } else {
      const route = String(plate.route || plate.route_name || plate.type || "").toLowerCase();
      color = colors[route] || "#21b4c6";
      const confidence = Number(plate.ocr_conf ?? plate.conf ?? plate.confidence);
      const confidenceText = Number.isFinite(confidence) ? ` ${confidence.toFixed(2)}` : "";
      label = `${plate.text || plate.type || "目标"}${confidenceText}`;
    }

    const suspectedPed = isPedestrianTarget(plate) && isSuspectedDecision(plate.decision);
    overlayContext.strokeStyle = color;
    overlayContext.lineWidth = suspectedPed ? 4 : 2;
    overlayContext.strokeRect(x, y, width, height);
    overlayContext.lineWidth = 2;
    const labelWidth = Math.min(rect.width, overlayContext.measureText(label).width + 12);
    const labelY = y >= 25 ? y - 23 : Math.min(rect.height - 23, y + height);
    overlayContext.fillStyle = "rgba(9, 11, 13, 0.88)";
    overlayContext.fillRect(Math.max(0, x), labelY, labelWidth, 22);
    overlayContext.fillStyle = color;
    overlayContext.fillText(label, Math.max(0, x) + 6, labelY + 11);
  }
}

function updateResultsView(results) {
  const plates = resultIsCurrent() ? normalizedPlates(results) : [];
  const hasPed = plates.some(isPedestrianTarget);
  let ordered = plates;
  let violationCount = 0;
  if (hasPed) {
    violationCount = plates.filter((t) => isSuspectedDecision(t.decision)).length;
    ordered = [...plates].sort((a, b) => {
      const sa = isSuspectedDecision(a.decision) ? 0 : 1;
      const sb = isSuspectedDecision(b.decision) ? 0 : 1;
      return sa - sb;
    });
  }
  elements.count.textContent = hasPed ? `违法 ${violationCount}/${plates.length}` : String(plates.length);
  elements.resultList.replaceChildren();
  if (!ordered.length) {
    const empty = document.createElement("p");
    empty.className = "empty-result";
    empty.textContent = imageViewActive ? "未检测到目标" : "暂无车牌";
    elements.resultList.append(empty);
  } else {
    for (const plate of ordered) {
      const item = document.createElement("div");
      const suspected = isPedestrianTarget(plate) && isSuspectedDecision(plate.decision);
      item.className = suspected ? "result-row violation-row" : "result-row";
      const text = document.createElement("strong");
      const meta = document.createElement("span");
      if (isPedestrianTarget(plate)) {
        const reason = pedestrianReasonText(plate.reason, suspected);
        text.textContent = suspected ? "⚠ 违法横穿" : "正常";
        const score = Number(plate.score);
        meta.textContent = `${reason}${Number.isFinite(score) ? `  ${Math.round(score * 100)}%` : ""}`;
      } else {
        text.textContent = plate.text || plate.type || "未识别";
        const route = plate.route || plate.route_name || plate.decision || "--";
        const confidence = Number(plate.ocr_conf ?? plate.conf ?? plate.confidence ?? plate.score);
        const reason = plate.reason ? ` · ${plate.reason}` : "";
        meta.textContent = Number.isFinite(confidence) ? `${route}  ${confidence.toFixed(3)}${reason}` : `${route}${reason}`;
      }
      item.append(text, meta);
      elements.resultList.append(item);
    }
  }
  drawOverlay();
}

function hidePreview(label) {
  if (currentFrameUrl) URL.revokeObjectURL(currentFrameUrl);
  currentFrameUrl = null;
  elements.frame.removeAttribute("src");
  elements.frame.dataset.ready = "false";
  elements.frameState.textContent = label;
  elements.frameState.hidden = false;
  const rect = elements.stage.getBoundingClientRect();
  overlayContext.clearRect(0, 0, rect.width, rect.height);
}

function resetVisualEpoch() {
  visualEpoch += 1;
  latestResults = null;
  updateResultsView({detections: []});
  hidePreview("等待新源画面");
}

async function statusLoop() {
  while (true) {
    const started = performance.now();
    if (currentDriverMode === "pedestrian") {
      updateStatusView({
        desired_source: "none",
        active_source: "none",
        failover_reason: "pedestrian_image_mode",
        fps: {input: 0, decode: 0, infer: 0},
        frame: {age_ms: -1},
        dropped: {input: 0, decode: 0, infer: 0, display: 0},
      });
      setConnection(true, "已连接");
      await sleep(Math.max(100, STATUS_PERIOD_MS - (performance.now() - started)));
      continue;
    }
    try {
      const data = await fetchJson("/api/v1/status");
      const nextStatus = unwrapPayload(data, "status");
      const nextGeneration = statusGeneration(nextStatus);
      const generationChanged = (
        latestStatusGeneration !== null &&
        nextGeneration !== null &&
        nextGeneration !== latestStatusGeneration
      );
      latestStatus = nextStatus;
      if (nextGeneration !== null) latestStatusGeneration = nextGeneration;
      if (generationChanged && !imageViewActive) resetVisualEpoch();
      updateStatusView(latestStatus);
      setConnection(true, "已连接");
    } catch (error) {
      setConnection(false, "连接中断");
    }
    await sleep(Math.max(100, STATUS_PERIOD_MS - (performance.now() - started)));
  }
}

async function resultsLoop() {
  while (true) {
    const started = performance.now();
    if (imageViewActive) {
      await sleep(RESULTS_PERIOD_MS);
      continue;
    }
    try {
      const data = await fetchJson("/api/v1/results");
      latestResults = unwrapPayload(data, "results");
      updateResultsView(latestResults);
    } catch (_) {
      latestResults = null;
      updateResultsView({plates: []});
    }
    await sleep(Math.max(100, RESULTS_PERIOD_MS - (performance.now() - started)));
  }
}

async function frameLoop() {
  while (true) {
    const started = performance.now();
    if (!document.hidden && !imageViewActive) {
      const requestEpoch = visualEpoch;
      try {
        const frameResponse = await fetchWithTimeout(
          "/api/v1/frame.jpg?t=" + Date.now(),
          {},
          3000,
          async (response) => ({
            blob: await response.blob(),
            generation: normalizedGeneration(
              response.headers.get("X-Source-Generation"),
            ),
          }),
        );
        const blob = frameResponse.blob;
        if (blob.type && blob.type !== "image/jpeg") throw new Error("preview is not JPEG");
        if (
          requestEpoch !== visualEpoch ||
          latestStatusGeneration === null ||
          frameResponse.generation !== latestStatusGeneration
        ) {
          hidePreview("等待新源画面");
          throw new Error(STALE_FRAME_ERROR);
        }
        const nextUrl = URL.createObjectURL(blob);
        try {
          await new Promise((resolve, reject) => {
            const image = new Image();
            image.onload = resolve;
            image.onerror = reject;
            image.src = nextUrl;
          });
        } catch (error) {
          URL.revokeObjectURL(nextUrl);
          throw error;
        }
        if (
          requestEpoch !== visualEpoch ||
          frameResponse.generation !== latestStatusGeneration
        ) {
          URL.revokeObjectURL(nextUrl);
          hidePreview("等待新源画面");
          throw new Error(STALE_FRAME_ERROR);
        }
        const previousUrl = currentFrameUrl;
        elements.frame.src = nextUrl;
        currentFrameUrl = nextUrl;
        if (previousUrl) URL.revokeObjectURL(previousUrl);
        elements.frame.dataset.ready = "true";
        elements.frameState.hidden = true;
        drawOverlay();
      } catch (error) {
        elements.frameState.hidden = false;
        if (error?.message !== STALE_FRAME_ERROR) {
          elements.frameState.textContent = "画面暂不可用";
        }
      }
    }
    await sleep(Math.max(20, FRAME_PERIOD_MS - (performance.now() - started)));
  }
}

async function setDesiredSource(source) {
  for (const button of elements.sourceSelector.querySelectorAll("button")) button.disabled = true;
  try {
    await fetchJson("/api/v1/source", {
      method: "PUT",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({source}),
    });
    updateSourceSelector(source);
  } catch (error) {
    setConnection(false, "切换失败");
  } finally {
    for (const button of elements.sourceSelector.querySelectorAll("button")) button.disabled = false;
  }
}

function setPhoneState(state, label) {
  elements.phoneState.dataset.state = state;
  elements.phoneState.textContent = label;
  elements.phoneToggle.textContent = publisher ? "停止手机摄像头" : "启动手机摄像头";
  elements.phoneToggle.disabled = cameraStarting || currentDriverMode !== "plate";
}

function preferH264(transceiver) {
  if (!transceiver || typeof transceiver.setCodecPreferences !== "function") return;
  if (!window.RTCRtpSender || typeof RTCRtpSender.getCapabilities !== "function") {
    throw new Error("浏览器无法确认 H.264 支持");
  }
  const capabilities = RTCRtpSender.getCapabilities("video");
  if (!capabilities?.codecs?.length) throw new Error("浏览器无法确认 H.264 支持");
  const h264 = capabilities.codecs.filter((codec) => codec.mimeType.toLowerCase() === "video/h264");
  if (!h264.length) throw new Error("浏览器不支持 H.264 视频");
  try {
    transceiver.setCodecPreferences(h264);
  } catch (_) {
    throw new Error("无法启用 H.264 视频");
  }
}

function waitForIceGathering(peer, timeoutMs = 8000) {
  if (peer.iceGatheringState === "complete") return Promise.resolve();
  return new Promise((resolve) => {
    let settled = false;
    const finish = () => {
      if (settled) return;
      settled = true;
      peer.removeEventListener("icegatheringstatechange", check);
      window.clearTimeout(timeout);
      resolve();
    };
    const check = () => {
      if (peer.iceGatheringState === "complete") finish();
    };
    const timeout = window.setTimeout(finish, timeoutMs);
    peer.addEventListener("icegatheringstatechange", check);
  });
}

async function deleteWhipSession(session, timeoutMs = 2500) {
  if (!session?.sessionUrl) return;
  const headers = {};
  if (session.etag) headers["If-Match"] = session.etag;
  const controller = new AbortController();
  const timeout = window.setTimeout(() => controller.abort(), timeoutMs);
  try {
    await fetch(session.sessionUrl, {
      method: "DELETE",
      headers,
      body: null,
      keepalive: true,
      signal: controller.signal,
    });
  } catch (_) {
    // Closing tracks and the peer connection remains the local source of truth.
  } finally {
    window.clearTimeout(timeout);
  }
}

function closeLocalPublisher(session) {
  if (!session) return;
  session.peer?.close();
  for (const track of session.stream?.getTracks() || []) track.stop();
  elements.publisherVideo.srcObject = null;
}

function recoverFailedPublisher(session) {
  if (publisher !== session) return;
  const preserveIntent = phoneCameraIntent;
  void stopPhoneCamera({preserveIntent}).then(() => {
    if (preserveIntent && !document.hidden) {
      void startPhoneCamera({recovery: true});
    }
  });
}

async function startPhoneCamera({recovery = false} = {}) {
  if (publisher || cameraStarting) return;
  if (!window.isSecureContext || !navigator.mediaDevices?.getUserMedia) {
    phoneCameraIntent = false;
    setPhoneState("error", "需要 HTTPS 摄像头权限");
    return;
  }
  phoneCameraIntent = true;
  cameraStarting = true;
  setPhoneState("starting", recovery ? "正在恢复" : "正在启动");
  let session = null;
  try {
    const stream = await navigator.mediaDevices.getUserMedia({
      video: {
        facingMode: {ideal: "environment"},
        width: {ideal: 1280},
        height: {ideal: 720},
        frameRate: {ideal: 15, max: 15},
      },
      audio: false,
    });
    session = {stream, peer: null, sessionUrl: null, etag: null};
    const track = stream.getVideoTracks()[0];
    if (!track) throw new Error("没有可用的视频轨道");
    track.contentHint = "motion";
    elements.publisherVideo.srcObject = stream;
    await elements.publisherVideo.play().catch(() => {});

    const peer = new RTCPeerConnection({bundlePolicy: "max-bundle"});
    session.peer = peer;
    const sender = peer.addTrack(track, stream);
    preferH264(peer.getTransceivers().find((item) => item.sender === sender));
    peer.addEventListener("connectionstatechange", () => {
      if (publisher !== session) return;
      if (peer.connectionState === "connected") setPhoneState("live", "正在发送");
      if (peer.connectionState === "disconnected") {
        setPhoneState("error", "视频连接中断");
      }
      if (["failed", "closed"].includes(peer.connectionState)) {
        recoverFailedPublisher(session);
      }
    });
    track.addEventListener("ended", () => {
      if (publisher !== session) return;
      const preserveIntent = document.hidden && phoneCameraIntent;
      void stopPhoneCamera({preserveIntent});
      if (!preserveIntent) setPhoneState("error", "摄像头需要重新启动");
    });
    track.addEventListener("mute", () => {
      if (publisher === session) setPhoneState("paused", "摄像头已暂停");
    });
    track.addEventListener("unmute", () => {
      if (publisher === session) setPhoneState("live", "正在发送");
    });

    const offer = await peer.createOffer();
    await peer.setLocalDescription(offer);
    await waitForIceGathering(peer);
    const whip = await fetchWithTimeout(
      "/whip/phone",
      {
        method: "POST",
        headers: {"Content-Type": "application/sdp", "Accept": "application/sdp"},
        body: peer.localDescription.sdp,
      },
      10000,
      async (response) => ({
        sessionUrl: response.headers.get("Location"),
        etag: response.headers.get("ETag"),
        answer: await response.text(),
      }),
    );
    session.sessionUrl = whip.sessionUrl;
    session.etag = whip.etag;
    await peer.setRemoteDescription({type: "answer", sdp: whip.answer});
    publisher = session;
    if (peer.connectionState === "connected") {
      setPhoneState("live", "正在发送");
    } else {
      setPhoneState("starting", "正在连接");
    }
    if (!recovery) await setDesiredSource("phone");
  } catch (error) {
    phoneCameraIntent = false;
    closeLocalPublisher(session);
    await deleteWhipSession(session);
    setPhoneState("error", error?.message || "启动失败");
  } finally {
    cameraStarting = false;
    elements.phoneToggle.disabled = false;
    elements.phoneToggle.textContent = publisher ? "停止手机摄像头" : "启动手机摄像头";
  }
}

async function stopPhoneCamera({preserveIntent = false} = {}) {
  if (!preserveIntent) phoneCameraIntent = false;
  if (!publisher) {
    if (!preserveIntent) setPhoneState("idle", "未启动");
    return;
  }
  const session = publisher;
  publisher = null;
  closeLocalPublisher(session);
  setPhoneState(
    preserveIntent ? "paused" : "idle",
    preserveIntent ? "等待恢复" : "未启动",
  );
  await deleteWhipSession(session);
}

async function pipelineAction(action, button) {
  button.disabled = true;
  try {
    await fetchJson(`/api/v1/pipeline/${action}`, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: "{}",
    });
  } catch (_) {
    setConnection(false, "操作失败");
  } finally {
    button.disabled = false;
  }
}

function selectedImageMode() {
  const selected = elements.imageModeSelector.querySelector('button[aria-checked="true"]');
  return selected?.dataset.imageMode || "plate";
}

function updateDriverModeView(status) {
  const mode = status?.mode === "pedestrian" ? "pedestrian" : "plate";
  currentDriverMode = mode;
  for (const button of elements.imageModeSelector.querySelectorAll("button")) {
    button.setAttribute("aria-checked", button.dataset.imageMode === mode ? "true" : "false");
    button.disabled = driverSwitching || imageInferenceRunning;
  }
  const plateRunning = Boolean(status?.plate_running);
  elements.driverModeState.dataset.state = mode === "plate" && plateRunning ? "live" : "idle";
  elements.driverModeState.textContent = mode === "plate"
    ? (plateRunning ? "车牌实时运行" : "车牌驱动正在启动")
    : "行人图片模式";
  for (const button of elements.sourceSelector.querySelectorAll("button")) {
    button.disabled = mode !== "plate";
  }
  elements.phoneToggle.disabled = mode !== "plate" || cameraStarting;
}

async function setDriverMode(mode) {
  if (driverSwitching || imageInferenceRunning || mode === currentDriverMode) return;
  driverSwitching = true;
  updateDriverModeView({mode, plate_running: false});
  elements.driverModeState.dataset.state = "starting";
  elements.driverModeState.textContent = "正在切换";
  try {
    if (publisher && mode === "pedestrian") await stopPhoneCamera();
    const status = await fetchWithTimeout(
      "/api/v1/mode",
      {
        method: "PUT",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({mode}),
      },
      15000,
      (response) => response.json(),
    );
    updateDriverModeView(status);
    if (mode === "plate") {
      returnToLiveView();
    } else {
      imageViewActive = true;
      latestResults = {frame: {width: 1280, height: 720}, detections: []};
      updateResultsView(latestResults);
      hidePreview("行人模式：请选择图片");
      elements.returnLiveView.hidden = true;
      setImageInferenceState("idle", "等待选择行人图片");
    }
  } catch (error) {
    setImageInferenceState("error", error?.message || "模式切换失败");
    try {
      const status = await fetchJson("/api/v1/mode");
      updateDriverModeView(status);
    } catch (_) {
      elements.driverModeState.dataset.state = "error";
      elements.driverModeState.textContent = "模式状态不可用";
    }
  } finally {
    driverSwitching = false;
    for (const button of elements.imageModeSelector.querySelectorAll("button")) {
      button.disabled = imageInferenceRunning;
    }
    for (const button of elements.sourceSelector.querySelectorAll("button")) {
      button.disabled = currentDriverMode !== "plate";
    }
    elements.phoneToggle.disabled = currentDriverMode !== "plate" || cameraStarting;
  }
}

async function driverModeLoop() {
  while (true) {
    if (!driverSwitching && !imageInferenceRunning) {
      try {
        updateDriverModeView(await fetchJson("/api/v1/mode"));
      } catch (_) {
        elements.driverModeState.dataset.state = "error";
        elements.driverModeState.textContent = "模式状态不可用";
      }
    }
    await sleep(1000);
  }
}

function setImageInferenceState(state, label) {
  elements.imageInferenceState.dataset.state = state;
  elements.imageInferenceState.textContent = label;
}

async function showSelectedImage(file) {
  if (currentFrameUrl) URL.revokeObjectURL(currentFrameUrl);
  currentFrameUrl = URL.createObjectURL(file);
  imageViewActive = true;
  latestResults = {frame: {width: 1280, height: 720}, detections: []};
  elements.frame.src = currentFrameUrl;
  elements.frame.dataset.ready = "true";
  elements.frameState.hidden = true;
  elements.returnLiveView.hidden = false;
  updateResultsView(latestResults);
  await elements.frame.decode().catch(() => {});
  drawOverlay();
}

async function showRenderedImage(rendered) {
  if (!rendered || rendered.content_type !== "image/jpeg" || typeof rendered.base64 !== "string") {
    return;
  }
  const decoded = window.atob(rendered.base64);
  const bytes = new Uint8Array(decoded.length);
  for (let index = 0; index < decoded.length; index += 1) {
    bytes[index] = decoded.charCodeAt(index);
  }
  const nextUrl = URL.createObjectURL(new Blob([bytes], {type: "image/jpeg"}));
  if (currentFrameUrl) URL.revokeObjectURL(currentFrameUrl);
  currentFrameUrl = nextUrl;
  elements.frame.src = nextUrl;
  elements.frame.dataset.ready = "true";
  elements.frameState.hidden = true;
  await elements.frame.decode().catch(() => {});
}

async function showBase64Image(image) {
  if (!image || typeof image.base64 !== "string") return false;
  const contentType = image.content_type || "image/jpeg";
  if (!contentType.startsWith("image/")) return false;
  const decoded = window.atob(image.base64);
  const bytes = new Uint8Array(decoded.length);
  for (let index = 0; index < decoded.length; index += 1) {
    bytes[index] = decoded.charCodeAt(index);
  }
  const nextUrl = URL.createObjectURL(new Blob([bytes], {type: contentType}));
  if (currentFrameUrl) URL.revokeObjectURL(currentFrameUrl);
  currentFrameUrl = nextUrl;
  elements.frame.src = nextUrl;
  elements.frame.dataset.ready = "true";
  elements.frameState.hidden = true;
  elements.returnLiveView.hidden = false;
  imageViewActive = true;
  await elements.frame.decode().catch(() => {});
  drawOverlay();
  return true;
}

async function runImageInference() {
  const file = elements.imageFile.files?.[0];
  if (!file || imageInferenceRunning) return;
  if (!["image/jpeg", "image/png"].includes(file.type)) {
    setImageInferenceState("error", "只支持 JPEG 或 PNG");
    return;
  }
  imageInferenceRunning = true;
  elements.runImageInference.disabled = true;
  elements.imageFile.disabled = true;
  for (const button of elements.imageModeSelector.querySelectorAll("button")) button.disabled = true;
  setImageInferenceState("starting", "正在上传并识别");
  try {
    await showSelectedImage(file);
    const response = await fetchWithTimeout(
      "/api/v1/image-inference",
      {
        method: "POST",
        headers: {
          "Content-Type": file.type,
          "X-Inference-Mode": selectedImageMode(),
        },
        body: file,
      },
      60000,
      (result) => result.json(),
    );
    latestResults = {
      ...(response.results || {}),
      frame: response.frame || {width: 1280, height: 720},
    };
    await showRenderedImage(response.rendered_image);
    updateResultsView(latestResults);
    const count = normalizedPlates(latestResults).length;
    setImageInferenceState("live", count ? `识别完成，发现 ${count} 个目标` : "识别完成，未发现目标");
  } catch (error) {
    latestResults = {frame: {width: 1280, height: 720}, detections: []};
    updateResultsView(latestResults);
    setImageInferenceState("error", error?.message || "图片识别失败");
  } finally {
    imageInferenceRunning = false;
    elements.imageFile.disabled = false;
    elements.runImageInference.disabled = !elements.imageFile.files?.length;
    for (const button of elements.imageModeSelector.querySelectorAll("button")) button.disabled = false;
  }
}

function returnToLiveView() {
  if (currentDriverMode !== "plate") return;
  imageViewActive = false;
  latestResults = null;
  elements.returnLiveView.hidden = true;
  elements.imageFile.value = "";
  elements.runImageInference.disabled = true;
  setImageInferenceState("idle", "未选择图片");
  resetVisualEpoch();
}

function setSdBrowseState(state, label) {
  elements.sdBrowseState.dataset.state = state;
  elements.sdBrowseState.textContent = label;
}

function sdIconText(type) {
  if (type === "parent") return "↩";
  if (type === "dir") return "📁";
  if (type === "photo") return "🖼";
  if (type === "video") return "🎬";
  return "📄";
}

function formatFileSize(bytes) {
  if (!bytes) return "0 B";
  const units = ["B", "KB", "MB", "GB"];
  let value = bytes;
  let unit = 0;
  while (value >= 1024 && unit < units.length - 1) {
    value /= 1024;
    unit += 1;
  }
  return `${value < 10 && unit > 0 ? value.toFixed(1) : Math.round(value)} ${units[unit]}`;
}

function renderSdList(listing) {
  elements.sdCurrentPath.textContent = "/" + (listing.path || "");
  elements.sdFileList.replaceChildren();
  const entries = listing.entries || [];
  if (!entries.length) {
    const empty = document.createElement("p");
    empty.className = "empty-result";
    empty.textContent = "目录为空";
    elements.sdFileList.append(empty);
    return;
  }
  for (const entry of entries) {
    const row = document.createElement("button");
    row.type = "button";
    row.className = "sd-file-row";
    row.dataset.type = entry.type;
    row.dataset.path = entry.path;
    if (sdSelectedEntry && sdSelectedEntry.path === entry.path) {
      row.dataset.selected = "true";
    }
    const icon = document.createElement("span");
    icon.className = "sd-file-icon";
    icon.textContent = sdIconText(entry.type);
    const name = document.createElement("span");
    name.className = "sd-file-name";
    name.textContent = entry.name;
    const meta = document.createElement("span");
    meta.className = "sd-file-meta";
    meta.textContent = entry.type === "dir" || entry.type === "parent" ? "" : formatFileSize(entry.size);
    row.append(icon, name, meta);
    elements.sdFileList.append(row);
  }
}

async function loadSdList(subpath = "") {
  setSdBrowseState("loading", "正在读取 SD 卡");
  try {
    const data = await fetchJson("/api/v1/sd/list?path=" + encodeURIComponent(subpath));
    sdCurrentSubpath = data.path || "";
    renderSdList(data);
    setSdBrowseState(
      "live",
      data.truncated ? `已截断，显示前 ${data.entries.length} 项` : `共 ${data.entries.length} 项`,
    );
  } catch (error) {
    elements.sdFileList.replaceChildren();
    const empty = document.createElement("p");
    empty.className = "empty-result";
    empty.textContent = error?.message || "读取失败";
    elements.sdFileList.append(empty);
    setSdBrowseState("error", error?.message || "读取失败");
  }
}

function selectSdEntry(entry) {
  if (entry.type === "parent" || entry.type === "dir") {
    void loadSdList(entry.path);
    return;
  }
  if (entry.type !== "photo" && entry.type !== "video") {
    setSdBrowseState("error", "该文件类型不支持推理");
    return;
  }
  sdSelectedEntry = entry;
  elements.sdSelectedName.textContent = entry.name;
  for (const row of elements.sdFileList.querySelectorAll(".sd-file-row")) {
    row.dataset.selected = row.dataset.path === entry.path ? "true" : "false";
  }
  setSdBrowseState("selected", `已选 ${entry.type === "photo" ? "照片" : "视频"}`);
  updateSdActionButtons();
}

function updateSdActionButtons() {
  const photo = Boolean(sdSelectedEntry && sdSelectedEntry.type === "photo");
  const video = Boolean(sdSelectedEntry && sdSelectedEntry.type === "video");
  const busy = imageInferenceRunning || sdVideoPolling;
  elements.runSdPhotoInference.disabled = !photo || busy;
  elements.runSdVideoInference.disabled = !video || busy;
}

function setSdVideoProgressLabel(state, label) {
  elements.sdVideoProgressLabel.dataset.state = state;
  elements.sdVideoProgressLabel.textContent = label;
}

function renderSdVideoSummary(summary) {
  if (!summary) {
    elements.sdVideoSummary.replaceChildren();
    return;
  }
  const rows = [
    ["采样帧数", summary.total_frames ?? "--"],
    ["违规帧数", summary.violation_frames ?? 0],
    ["违规事件", summary.violation_count ?? 0],
    ["跳过帧数", summary.skipped_frames ?? 0],
    ["采样率", `${summary.sample_fps ?? "--"} fps`],
  ];
  if (summary.truncated) rows.push(["提示", "已达帧数上限，已截断"]);
  elements.sdVideoSummary.replaceChildren();
  for (const [label, value] of rows) {
    const row = document.createElement("div");
    const dt = document.createElement("span");
    dt.className = "sd-summary-label";
    dt.textContent = label;
    const dd = document.createElement("strong");
    dd.textContent = value;
    row.append(dt, dd);
    elements.sdVideoSummary.append(row);
  }
}

function renderSdVideoEvents(events) {
  elements.sdVideoEventCount.textContent = String(events.length);
  elements.sdVideoEvents.replaceChildren();
  if (!events.length) {
    const empty = document.createElement("p");
    empty.className = "empty-result";
    empty.textContent = "未发现违规事件";
    elements.sdVideoEvents.append(empty);
    return;
  }
  for (const event of events) {
    const row = document.createElement("button");
    row.type = "button";
    row.className = "sd-video-event";
    if (event.frame_index == null) row.disabled = true;
    else row.dataset.frame = String(event.frame_index);
    const time = document.createElement("span");
    time.className = "ev-time";
    time.textContent = `${Number(event.time_sec ?? 0).toFixed(1)}s`;
    const reason = document.createElement("span");
    reason.className = "ev-reason";
    reason.textContent = pedestrianReasonText(event.reason, true);
    row.append(time, reason);
    elements.sdVideoEvents.append(row);
  }
}

function renderSdVideoFrames(frames) {
  elements.sdFrameCount.textContent = String(frames ? frames.length : 0);
  elements.sdFrameList.replaceChildren();
  if (!frames || !frames.length) {
    const empty = document.createElement("p");
    empty.className = "empty-result";
    empty.textContent = "无帧";
    elements.sdFrameList.append(empty);
    return;
  }
  for (const frame of frames) {
    const row = document.createElement("button");
    row.type = "button";
    row.className = "sd-frame-thumb";
    if (frame.violation) row.dataset.violation = "true";
    if (!frame.has_image) row.disabled = true;
    else row.dataset.frame = String(frame.frame_index);
    const time = document.createElement("span");
    time.className = "ev-time";
    time.textContent = `${Number(frame.time_sec ?? 0).toFixed(1)}s`;
    const mark = document.createElement("span");
    mark.className = "ev-reason";
    mark.textContent = frame.violation ? "⚠ 违法" : "正常";
    row.append(time, mark);
    elements.sdFrameList.append(row);
  }
}

function renderSdVideoJob(job) {
  elements.sdVideoProgress.max = job.total || 1;
  elements.sdVideoProgress.value = job.processed || 0;
  if (job.status === "running") {
    setSdVideoProgressLabel("loading", job.message || "进行中");
  } else if (job.status === "done") {
    setSdVideoProgressLabel("live", "完成");
  } else if (job.status === "error") {
    setSdVideoProgressLabel("error", job.error || "失败");
  }
  renderSdVideoEvents(job.events || []);
  renderSdVideoFrames(job.frames || []);
  renderSdVideoSummary(job.summary);
}

async function showSdVideoFrame(index) {
  if (!sdVideoJobId) return;
  const base = `/api/v1/sd/video-jobs/${encodeURIComponent(sdVideoJobId)}/frames/${index}`;
  try {
    // Fetch the result JSON first (small), then the JPEG. Serial avoids one
    // slow request aborting the other, and both get a generous timeout since
    // the server reads frame data from an in-memory job under a lock.
    const result = await fetchWithTimeout(`${base}/result`, {}, 15000, (r) => r.json());
    const imgResp = await fetchWithTimeout(
      `${base}.jpg`,
      {},
      15000,
      async (res) => new Uint8Array(await res.arrayBuffer()),
    );
    const url = URL.createObjectURL(new Blob([imgResp], {type: "image/jpeg"}));
    if (currentFrameUrl) URL.revokeObjectURL(currentFrameUrl);
    currentFrameUrl = url;
    elements.frame.src = url;
    elements.frame.dataset.ready = "true";
    elements.frameState.hidden = true;
    elements.returnLiveView.hidden = false;
    imageViewActive = true;
    latestResults = {
      frame: {width: 1280, height: 720},
      targets: result.targets || [],
    };
    updateResultsView(latestResults);
    drawOverlay();
  } catch (error) {
    const msg = error?.name === "AbortError"
      ? `帧加载超时（请求被中止）`
      : error?.message || "帧加载失败";
    setSdVideoProgressLabel("error", msg);
  }
}

async function pollSdVideoJob() {
  if (!sdVideoJobId) return;
  try {
    const job = await fetchJson(`/api/v1/sd/video-jobs/${encodeURIComponent(sdVideoJobId)}`);
    renderSdVideoJob(job);
    if (job.status === "running") {
      window.setTimeout(pollSdVideoJob, 1000);
    } else {
      sdVideoPolling = false;
      updateSdActionButtons();
    }
  } catch (error) {
    setSdVideoProgressLabel("error", error?.message || "查询失败");
    sdVideoPolling = false;
    updateSdActionButtons();
  }
}

async function runSdVideoInference() {
  if (!sdSelectedEntry || sdSelectedEntry.type !== "video" || sdVideoPolling) return;
  const fps = Number(elements.sdVideoFps.value);
  if (!Number.isFinite(fps) || fps < 0.5 || fps > 10) {
    setSdVideoProgressLabel("error", "采样率需在 0.5–10 之间");
    return;
  }
  elements.sdVideoResult.hidden = false;
  setSdVideoProgressLabel("loading", "正在提交任务");
  elements.sdVideoProgress.value = 0;
  renderSdVideoEvents([]);
  renderSdVideoFrames([]);
  renderSdVideoSummary(null);
  try {
    const data = await fetchJson("/api/v1/sd/video-inference", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({path: sdSelectedEntry.path, sample_fps: fps}),
    });
    sdVideoJobId = data.job_id;
    sdVideoPolling = true;
    updateSdActionButtons();
    void pollSdVideoJob();
  } catch (error) {
    setSdVideoProgressLabel("error", error?.message || "视频推理提交失败");
  }
}

async function runSdPhotoInference() {
  if (!sdSelectedEntry || sdSelectedEntry.type !== "photo" || imageInferenceRunning) return;
  const mode = selectedImageMode();
  imageInferenceRunning = true;
  updateSdActionButtons();
  for (const button of elements.imageModeSelector.querySelectorAll("button")) button.disabled = true;
  setImageInferenceState("starting", "正在识别 SD 照片");
  try {
    const response = await fetchWithTimeout(
      "/api/v1/sd/photo-inference",
      {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({path: sdSelectedEntry.path, mode}),
      },
      60000,
      (result) => result.json(),
    );
    latestResults = {
      ...(response.results || {}),
      frame: response.frame || {width: 1280, height: 720},
    };
    const shown = (await showBase64Image(response.rendered_image)) || (await showBase64Image(response.source_image));
    if (!shown) {
      imageViewActive = true;
      elements.returnLiveView.hidden = false;
      hidePreview("仅结果，无预览图");
    }
    updateResultsView(latestResults);
    const count = normalizedPlates(latestResults).length;
    setImageInferenceState("live", count ? `识别完成，发现 ${count} 个目标` : "识别完成，未发现目标");
  } catch (error) {
    latestResults = {frame: {width: 1280, height: 720}, detections: []};
    updateResultsView(latestResults);
    setImageInferenceState("error", error?.message || "SD 照片识别失败");
  } finally {
    imageInferenceRunning = false;
    updateSdActionButtons();
    for (const button of elements.imageModeSelector.querySelectorAll("button")) button.disabled = false;
  }
}

elements.sourceSelector.addEventListener("click", (event) => {
  const button = event.target.closest("button[data-source]");
  if (button) void setDesiredSource(button.dataset.source);
});

elements.phoneToggle.addEventListener("click", () => {
  if (publisher) void stopPhoneCamera();
  else void startPhoneCamera();
});

elements.imageModeSelector.addEventListener("click", (event) => {
  const button = event.target.closest("button[data-image-mode]");
  if (!button || imageInferenceRunning) return;
  void setDriverMode(button.dataset.imageMode);
});

elements.imageFile.addEventListener("change", () => {
  const file = elements.imageFile.files?.[0];
  elements.runImageInference.disabled = !file;
  if (!file) {
    setImageInferenceState("idle", "未选择图片");
    return;
  }
  setImageInferenceState("idle", file.name);
  void showSelectedImage(file);
});

elements.runImageInference.addEventListener("click", () => void runImageInference());
elements.returnLiveView.addEventListener("click", returnToLiveView);

elements.sdRefresh.addEventListener("click", () => {
  void loadSdList(sdCurrentSubpath);
});

elements.sdFileList.addEventListener("click", (event) => {
  const row = event.target.closest("button.sd-file-row");
  if (!row) return;
  const nameEl = row.querySelector(".sd-file-name");
  selectSdEntry({
    name: nameEl ? nameEl.textContent : row.dataset.path,
    path: row.dataset.path,
    type: row.dataset.type,
  });
});

elements.runSdPhotoInference.addEventListener("click", () => void runSdPhotoInference());
elements.runSdVideoInference.addEventListener("click", () => void runSdVideoInference());

elements.sdVideoEvents.addEventListener("click", (event) => {
  const row = event.target.closest("button.sd-video-event");
  if (!row || row.disabled) return;
  const index = Number(row.dataset.frame);
  if (Number.isFinite(index)) void showSdVideoFrame(index);
});

elements.sdFrameList.addEventListener("click", (event) => {
  const row = event.target.closest("button.sd-frame-thumb");
  if (!row || row.disabled) return;
  const index = Number(row.dataset.frame);
  if (Number.isFinite(index)) void showSdVideoFrame(index);
});

for (const [id, action] of [["pausePipeline", "pause"], ["resumePipeline", "resume"], ["restartPipeline", "restart"]]) {
  const button = document.getElementById(id);
  button.addEventListener("click", () => void pipelineAction(action, button));
}

window.addEventListener("resize", drawOverlay);
new ResizeObserver(drawOverlay).observe(elements.stage);
document.addEventListener("visibilitychange", () => {
  if (!document.hidden && publisher) {
    const track = publisher.stream.getVideoTracks()[0];
    if (!track || track.readyState !== "live") {
      void stopPhoneCamera({preserveIntent: phoneCameraIntent}).then(() => {
        if (phoneCameraIntent) void startPhoneCamera({recovery: true});
      });
    }
  } else if (!document.hidden && phoneCameraIntent && !publisher && !cameraStarting) {
    void startPhoneCamera({recovery: true});
  }
});
window.addEventListener("pagehide", () => {
  if (publisher) void stopPhoneCamera({preserveIntent: phoneCameraIntent});
});
window.addEventListener("pageshow", () => {
  if (phoneCameraIntent && !publisher && !cameraStarting) {
    void startPhoneCamera({recovery: true});
  } else if (!publisher && !cameraStarting) {
    setPhoneState("idle", "未启动");
  }
});
window.setInterval(() => {
  const session = publisher;
  if (session && ["failed", "closed"].includes(session.peer.connectionState)) {
    recoverFailedPublisher(session);
  }
}, 500);

if (!window.isSecureContext) {
  elements.phoneToggle.disabled = true;
  setPhoneState("error", "需要 HTTPS");
}

void statusLoop();
void resultsLoop();
void frameLoop();
void driverModeLoop();
void loadSdList("");
