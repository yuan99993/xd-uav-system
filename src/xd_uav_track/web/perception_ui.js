/* Standalone page controller. Targets always come from the displayed video bundle. */
"use strict";
(() => {
  const el = id => document.getElementById(id), slots = new Map();
  let client = null, busy = false, connectionGeneration = 0, pageSession = null, displayedModelKey = null;
  const host = (location.hostname || "127.0.0.1").replace(/^\[|\]$/g, ""), protocol = location.protocol === "https:" ? "wss:" : "ws:";
  el("url").value = protocol + "//" + (host.includes(":") ? "[" + host + "]" : host) + ":9090";
  const stamp = value => value ? String(value.secs) + "." + String(value.nsecs).padStart(9, "0") : "—";
  const numbers = values => (values || []).map(value => Number(value).toFixed(1)).join(", ");
  function result(value) { el("result").textContent = typeof value === "string" ? value : JSON.stringify(value, null, 2); }
  async function action(operation) {
    try { const outcome = await operation(); result(outcome); return outcome; }
    catch (error) { result(error.message); return null; }
    finally { controls(); }
  }
  function currentSlot() { return slots.get(el("source").value); }
  function controls() {
    const live = client && client.online, slot = currentSlot();
    for (const button of document.querySelectorAll("[data-command]")) button.disabled = !live || busy;
    el("models").disabled = !live || busy || !client.models.length;
    el("switch").disabled = !live || busy || !el("models").value || el("models").selectedOptions[0]?.disabled;
    el("freeze").disabled = !live || busy || !slot?.current;
    el("freeze").textContent = slot?.frozen ? "当前帧已冻结" : "冻结当前帧";
    el("resume").disabled = !slot?.frozen;
    for (const button of document.querySelectorAll("[data-target]"))
      button.disabled = !live || busy || button.dataset.unavailable === "true";
    for (const button of document.querySelectorAll("[data-mode]"))
      button.classList.toggle("active", live && client.state.mode === button.dataset.mode);
  }
  function closeBitmap(bitmap) { if (bitmap && bitmap.close) bitmap.close(); }
  function cancelDrag(slot) {
    const drag = slot.drag;
    slot.drag = null;
    if (drag && slot.canvas.hasPointerCapture(drag.pointerId)) slot.canvas.releasePointerCapture(drag.pointerId);
  }
  function lose(slot, reason) {
    slot.token++; cancelDrag(slot); slot.frozen = false; slot.latest = null;
    closeBitmap(slot.current?.bitmap); slot.current = null;
    slot.canvas.getContext("2d").clearRect(0, 0, slot.canvas.width, slot.canvas.height);
    slot.notice.hidden = false; slot.notice.textContent = "LOST · " + reason;
    slot.meta.textContent = "未显示有效原帧"; renderTargets(); controls();
  }
  function resetSlots() {
    for (const slot of slots.values()) {
      slot.token++; cancelDrag(slot); slot.latest = null; slot.frozen = false;
      closeBitmap(slot.current?.bitmap); slot.current = null;
    }
    slots.clear(); el("videos").replaceChildren(); el("source").replaceChildren(); el("targets").replaceChildren();
    el("empty").hidden = false;
  }
  function sourceSlot(source) {
    if (slots.has(source)) return slots.get(source);
    if (slots.size >= (client?.maxSources || 8)) return null;
    const card = document.createElement("article"); card.className = "video-card";
    const top = document.createElement("div"); top.className = "video-top";
    const name = document.createElement("strong"); name.textContent = source;
    const choose = document.createElement("button"); choose.textContent = "在此画面选目标";
    choose.onclick = () => { el("source").value = source; chooseSource(); }; top.append(name, choose);
    const area = document.createElement("div"); area.className = "video-area";
    const canvas = document.createElement("canvas"); canvas.width = 1; canvas.height = 1;
    canvas.setAttribute("aria-label", source + " 检测视频；点击或拖拽选择追踪目标");
    const notice = document.createElement("div"); notice.className = "video-notice"; notice.textContent = "等待同帧视频";
    area.append(canvas, notice);
    const meta = document.createElement("div"); meta.className = "video-meta muted";
    card.append(top, area, meta); el("videos").append(card); el("empty").hidden = true;
    const option = document.createElement("option"); option.value = source; option.textContent = source; el("source").append(option);
    const slot = {source, card, canvas, notice, meta, current:null, latest:null, frozen:false, drag:null, decoding:false, token:0};
    slots.set(source, slot);
    canvas.addEventListener("pointerdown", event => beginDrag(slot, event));
    canvas.addEventListener("pointermove", event => moveDrag(slot, event));
    canvas.addEventListener("pointerup", event => endDrag(slot, event));
    canvas.addEventListener("pointercancel", () => resume(slot));
    canvas.addEventListener("lostpointercapture", () => { if (slot.drag) resume(slot); });
    chooseSource(); return slot;
  }
  function chooseSource() {
    for (const slot of slots.values()) slot.card.classList.toggle("chosen", slot.source === el("source").value);
    controls();
  }
  function draw(slot) {
    if (!slot.current) return;
    const {frame, bitmap} = slot.current, canvas = slot.canvas;
    if (canvas.width !== frame.width) canvas.width = frame.width;
    if (canvas.height !== frame.height) canvas.height = frame.height;
    const context = canvas.getContext("2d"); context.clearRect(0, 0, canvas.width, canvas.height);
    context.drawImage(bitmap, 0, 0); // PNG is already annotated; do not redraw independent target boxes.
    if (slot.drag) {
      const start = slot.drag.start, end = slot.drag.end;
      context.strokeStyle = "#58efff"; context.lineWidth = Math.max(2, frame.width / 600);
      context.setLineDash([8, 5]);
      context.strokeRect(Math.min(start[0], end[0]), Math.min(start[1], end[1]), Math.abs(end[0] - start[0]), Math.abs(end[1] - start[1]));
      context.setLineDash([]);
    }
    const target = frame.targets;
    slot.meta.textContent = frame.width + " × " + frame.height + " 原像素 · 帧 " + target.frame_seq +
      " · stamp " + stamp(target.header.stamp) + " · revision " + frame.frame_revision +
      " · " + target.mode + " / " + target.model_id + (slot.frozen ? " · 已冻结原帧" : " · 实时");
  }
  async function bitmapFor(frame) {
    const blob = new Blob([frame.pngBytes], {type:"image/png"});
    if (typeof createImageBitmap === "function") return createImageBitmap(blob);
    const url = URL.createObjectURL(blob);
    try { return await new Promise((resolve, reject) => {
      const image = new Image(); image.onload = () => resolve(image); image.onerror = () => reject(new Error("浏览器无法解码 PNG")); image.src = url;
    }); } finally { URL.revokeObjectURL(url); }
  }
  async function play(slot) {
    if (slot.decoding || slot.frozen || !slot.latest || !client?.online) return;
    const frame = slot.latest, token = slot.token, owner = client;
    slot.decoding = true;
    try {
      const bitmap = await bitmapFor(frame);
      if (token !== slot.token || slot.frozen || client !== owner || !owner.matches(frame.targets)) { closeBitmap(bitmap); return; }
      closeBitmap(slot.current?.bitmap); slot.current = {frame, bitmap}; slot.notice.hidden = true;
      draw(slot); renderTargets(); controls();
    } catch (error) { if (token === slot.token) { result(error.message); lose(slot, "PNG 解码失败"); } }
    finally {
      slot.decoding = false;
      if (!slot.frozen && slot.latest && slot.latest !== frame) play(slot);
    }
  }
  function resume(slot) {
    if (!slot) return;
    cancelDrag(slot); slot.frozen = false; draw(slot); controls();
    if (slot.latest && slot.latest !== slot.current?.frame) play(slot);
  }
  function beginDrag(slot, event) {
    if (event.button !== 0 || !client?.online || busy || !slot.current || slot.drag) return;
    const frame = slot.current.frame;
    if (!client.matches(frame.targets)) return;
    const point = PerceptionVideo.point(slot.canvas.getBoundingClientRect(), frame.width, frame.height, event.clientX, event.clientY);
    if (!point) return; // Black bars are outside the original image.
    el("source").value = slot.source; chooseSource(); slot.frozen = true;
    slot.drag = {pointerId:event.pointerId, frame, context:PerceptionVideo.identity(frame), start:point, end:point,
      clientStart:[event.clientX, event.clientY]};
    slot.canvas.setPointerCapture(event.pointerId); event.preventDefault(); draw(slot); controls();
  }
  function moveDrag(slot, event) {
    if (!slot.drag || slot.drag.pointerId !== event.pointerId) return;
    const frame = slot.drag.frame;
    slot.drag.end = PerceptionVideo.point(slot.canvas.getBoundingClientRect(), frame.width, frame.height, event.clientX, event.clientY, true);
    draw(slot);
  }
  async function endDrag(slot, event) {
    const drag = slot.drag;
    if (!drag || drag.pointerId !== event.pointerId) return;
    moveDrag(slot, event); const end = drag.end; cancelDrag(slot); draw(slot);
    if (!client || !client.matches(drag.context)) { result("模式或模型状态已改变，请在新画面重新选择"); resume(slot); return; }
    const point = Math.hypot(event.clientX - drag.clientStart[0], event.clientY - drag.clientStart[1]) < 5;
    const selection = point ? [end[0] / drag.frame.width, end[1] / drag.frame.height] :
      PerceptionVideo.region(drag.start, end, drag.frame.width, drag.frame.height);
    const header = {...drag.frame.targets.header, seq:drag.frame.targets.frame_seq};
    try { await action(() => client.region(slot.source, header, selection, point, drag.context)); }
    finally { resume(slot); }
  }
  function renderTargets() {
    el("targets").replaceChildren();
    for (const slot of slots.values()) {
      if (!slot.current) continue;
      const frame = slot.current.frame, message = frame.targets;
      for (const target of message.targets) {
        const fresh = client?.online && client.matches(message), measured = fresh && target.position_valid &&
          !["lost", "stale"].includes(String(target.lifecycle_state).toLowerCase()) &&
          ["detector", "visual_tracker"].includes(target.measurement_source);
        const position = measured ? numbers(target.position_xyz) + " · " + target.position_frame +
          " · t=" + stamp(target.position_stamp) : "位置无效 / 无当前实测";
        const row = document.createElement("tr");
        for (const value of [message.image_source + " / " + message.frame_seq,
          target.class_valid ? target.class_name + " (" + target.class_id + ")" : "类别待确认",
          target.track_id + " / " + target.target_uid, numbers(target.center_px) + " / [" + (target.bbox_xyxy || []).join(", ") + "]",
          position, target.lifecycle_state + " / " + target.measurement_source + (target.selected ? " · 已选中" : "")]) {
          const cell = document.createElement("td"); cell.textContent = value; row.append(cell);
        }
        const cell = document.createElement("td"), button = document.createElement("button"); button.textContent = "按 UID 追踪";
        button.dataset.target = target.target_uid; button.dataset.unavailable = String(!fresh ||
          ["lost", "stale"].includes(String(target.lifecycle_state).toLowerCase()) || target.measurement_source === "stale");
        button.onclick = () => action(() => client.select(message.image_source, target.target_uid, PerceptionVideo.identity(frame)));
        cell.append(button); row.append(cell); el("targets").append(row);
      }
    }
    controls();
  }
  function models(response) {
    const desired = el("models").value, active = client?.state?.active_model_id;
    el("models").replaceChildren();
    for (const item of response.models) {
      const option = document.createElement("option"); option.value = item.model_id;
      option.textContent = item.model_id;
      option.disabled = !item.engine_prepared; el("models").append(option);
    }
    if (Array.from(el("models").options).some(option => option.value === desired)) el("models").value = desired;
    else if (active) el("models").value = active;
    controls();
  }
  function runtimeState(state) {
    if (pageSession && pageSession !== state.session_id) resetSlots();
    pageSession = state.session_id;
    el("state").textContent = JSON.stringify(state, null, 2);
    el("actual").textContent = "实际模式：" + state.mode + " · 实际模型：" + state.active_model_id +
      " · epoch " + state.model_epoch + " · 状态 " + state.operating_state + " · 版本 " + state.state_version +
      (state.selected_target_uid ? " · 追踪 " + state.selected_target_uid + " / " + state.selected_image_source : " · 未选择目标") +
      " · 事务 " + state.transaction_state;
    const modelKey = JSON.stringify([state.session_id, state.active_model_id, state.model_epoch]);
    // Heartbeats and mode/selection updates must not overwrite a user's
    // unsubmitted choice. Synchronize only an actual model/session change.
    // Defer while busy so the next idle state can finish synchronization.
    if (modelKey !== displayedModelKey && state.active_model_id &&
        Array.from(el("models").options).some(option => option.value === state.active_model_id) && !busy) {
      el("models").value = state.active_model_id;
      displayedModelKey = modelKey;
    }
    // A state/epoch change invalidates a frozen selection immediately.
    for (const slot of slots.values()) if (slot.drag && !client.matches(slot.drag.context)) {
      result("状态已改变，已取消原帧框选；请在新画面重新选择"); resume(slot);
    }
    controls();
  }
  el("connect").onclick = () => {
    ++connectionGeneration;
    if (client) client.close(); resetSlots(); pageSession = null; displayedModelKey = null; busy = false; el("trajectory").textContent = "等待选中目标轨迹";
    el("connection").textContent = "连接中，等待 state…";
    const generation = connectionGeneration;
    const active = callback => (...args) => { if (generation === connectionGeneration) callback(...args); };
    client = new PerceptionClient(el("url").value, el("ns").value, {
      onConnection:active(online => {
        el("connection").textContent = online ? "已连接 · 状态已同步" : "LOST · 状态或连接中断，等待恢复";
        el("connection").className = online ? "status" : "lost"; controls();
      }),
      onBusy:active(value => { busy = value; controls(); }), onError:active(error => result(error.message)),
      onState:active(runtimeState), onModels:active(models), onResult:active(result),
      onVideo:active(frame => { const slot = sourceSlot(frame.targets.image_source); if (!slot) return; slot.latest = frame; if (!slot.frozen) play(slot); }),
      onVideoLost:active((source, reason) => { const slot = slots.get(source); if (slot) lose(slot, reason); }),
      onTrajectory:active(trajectory => {
        el("trajectory").textContent = "轨迹：" + (trajectory.target_uid || "未选择") + " / " + trajectory.status +
          " / " + trajectory.points.length + " 点 · 轨迹源时间戳 " + stamp(trajectory.header.stamp);
      })
    });
    controls();
  };
  for (const button of document.querySelectorAll("[data-mode]")) button.onclick = () => action(() => client.mode(button.dataset.mode));
  el("switch").onclick = () => action(() => client.model(el("models").value));
  el("clear").onclick = () => action(() => client.clear());
  el("source").onchange = chooseSource;
  el("freeze").onclick = () => { const slot = currentSlot(); if (slot?.current) { slot.frozen = true; draw(slot); controls(); } };
  el("resume").onclick = () => resume(currentSlot());
  window.addEventListener("beforeunload", () => client?.close());
  window.addEventListener("keydown", event => { if (event.key === "Escape") for (const slot of slots.values()) resume(slot); });
  // The launch-served runtime config adapts a copied page to namespace and port.
  // Plain file/http.server hosting has no config and uses the editable defaults.
  if (location.protocol !== "file:") {
    const abort = new AbortController(), timer = setTimeout(() => abort.abort(), 2000);
    fetch("perception_runtime.json", {signal:abort.signal, cache:"no-store"}).then(response => {
      if (!response.ok) return null; return response.json();
    }).then(config => {
      if (!config || client) return;
      if (typeof config.uav_name === "string") el("ns").value = config.uav_name;
      if (Number.isInteger(config.websocket_port) && config.websocket_port > 0 && config.websocket_port < 65536)
        el("url").value = protocol + "//" + (host.includes(":") ? "[" + host + "]" : host) + ":" + config.websocket_port;
    }).catch(() => {}).finally(() => clearTimeout(timer));
  }
})();
