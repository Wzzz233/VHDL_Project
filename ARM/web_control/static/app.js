const frame = document.getElementById("frame");
const canvas = document.getElementById("overlay");
const ctx = canvas.getContext("2d");
const summary = document.getElementById("summary");
const resultsEl = document.getElementById("results");
const logEl = document.getElementById("log");

const displayEvery = document.getElementById("displayEvery");
const fps = document.getElementById("fps");
const previewFps = document.getElementById("previewFps");
const dmaDelay = document.getElementById("dmaDelay");
const noInfer = document.getElementById("noInfer");

let latestResults = null;

function formBody(values) {
  return new URLSearchParams(values).toString();
}

async function post(path, values = {}) {
  const res = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/x-www-form-urlencoded" },
    body: formBody(values),
  });
  return res.json();
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(rect.width * dpr));
  canvas.height = Math.max(1, Math.round(rect.height * dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function drawOverlay() {
  resizeCanvas();
  const rect = canvas.getBoundingClientRect();
  ctx.clearRect(0, 0, rect.width, rect.height);
  if (!latestResults || !latestResults.plates || !latestResults.frame_w || !latestResults.frame_h) return;

  const scale = Math.min(rect.width / latestResults.frame_w, rect.height / latestResults.frame_h);
  const drawW = latestResults.frame_w * scale;
  const drawH = latestResults.frame_h * scale;
  const ox = (rect.width - drawW) / 2;
  const oy = (rect.height - drawH) / 2;

  ctx.lineWidth = 2;
  ctx.font = "13px system-ui, sans-serif";
  for (const plate of latestResults.plates) {
    const x = ox + plate.x1 * scale;
    const y = oy + plate.y1 * scale;
    const w = (plate.x2 - plate.x1) * scale;
    const h = (plate.y2 - plate.y1) * scale;
    const label = `${plate.text || "OCR"} ${plate.route || ""} ${(plate.conf || 0).toFixed(2)}`;
    ctx.strokeStyle = "#4cc9f0";
    ctx.fillStyle = "rgba(10, 14, 18, 0.75)";
    ctx.strokeRect(x, y, w, h);
    const labelW = ctx.measureText(label).width + 10;
    ctx.fillRect(x, Math.max(0, y - 22), labelW, 20);
    ctx.fillStyle = "#eef3f8";
    ctx.fillText(label, x + 5, Math.max(14, y - 7));
  }
}

async function refreshStatus() {
  try {
    const data = await fetch("/api/status", { cache: "no-store" }).then(r => r.json());
    const s = data.server;
    displayEvery.value = s.display_every;
    fps.value = s.fps;
    previewFps.value = s.preview_fps;
    dmaDelay.value = s.dma_pre_delay_us;
    noInfer.checked = !!s.no_infer;
    summary.textContent = `${s.running ? "running" : "stopped"} | display_every=${s.display_every} fps=${s.fps} preview=${s.preview_fps}`;
  } catch (err) {
    summary.textContent = `status error: ${err}`;
  }
}

async function refreshResults() {
  try {
    latestResults = await fetch(`/api/results?t=${Date.now()}`, { cache: "no-store" }).then(r => r.json());
    const plates = latestResults.plates || [];
    resultsEl.innerHTML = plates.length ? plates.map(p => (
      `<div class="plate"><strong>${p.text || "OCR"}</strong> ${p.route || ""} conf=${Number(p.conf || 0).toFixed(3)} box=[${p.x1},${p.y1},${p.x2},${p.y2}]</div>`
    )).join("") : "no plate";
    drawOverlay();
  } catch {
    latestResults = null;
    resultsEl.textContent = "no result";
    drawOverlay();
  }
}

async function refreshFrame() {
  frame.src = `/api/frame.bmp?t=${Date.now()}`;
}

async function refreshLog() {
  try {
    const data = await fetch("/api/log", { cache: "no-store" }).then(r => r.json());
    logEl.textContent = data.log || "";
    logEl.scrollTop = logEl.scrollHeight;
  } catch {
    logEl.textContent = "";
  }
}

document.getElementById("startBtn").addEventListener("click", async () => {
  await post("/api/start");
  await refreshStatus();
});

document.getElementById("stopBtn").addEventListener("click", async () => {
  await post("/api/stop");
  await refreshStatus();
});

document.getElementById("applyBtn").addEventListener("click", async () => {
  await post("/api/config", {
    display_every: displayEvery.value,
    fps: fps.value,
    preview_fps: previewFps.value,
    dma_pre_delay_us: dmaDelay.value,
    no_infer: noInfer.checked ? "1" : "0",
    restart: "1",
  });
  await refreshStatus();
});

frame.addEventListener("load", drawOverlay);
window.addEventListener("resize", drawOverlay);

setInterval(refreshStatus, 1000);
setInterval(refreshFrame, 250);
setInterval(refreshResults, 500);
setInterval(refreshLog, 1500);

refreshStatus();
refreshFrame();
refreshResults();
refreshLog();
