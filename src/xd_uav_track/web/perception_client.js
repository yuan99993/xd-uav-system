/* Native rosbridge client. No CDN, frame rebasing, or concurrent transactions. */
"use strict";
class PerceptionClient {
  constructor(url, namespace, options = {}) {
    this.url = url;
    const scope = namespace.replace(/^\/+|\/+$/g, "");
    this.root = (scope ? "/" + scope : "") + "/perception/";
    this.options = options;
    this.maxSources = Number.isInteger(options.maxSources) ? Math.max(1, Math.min(32, options.maxSources)) : 8;
    this.Socket = options.WebSocketClass || globalThis.WebSocket;
    this.videoTools = options.videoTools || globalThis.PerceptionVideo ||
      (typeof require !== "undefined" ? require("./perception_video") : null);
    this.pending = new Map();
    this.videos = new Map();
    this.revisions = new Map();
    this.buffer = new Map();
    this.retiredSessions = new Set();
    this.state = null;
    this.models = [];
    this.online = false;
    this.closed = false;
    this.sequence = 0;
    this.generation = 0;
    this.queued = 0;
    this.serial = Promise.resolve();
    this.lastStateAt = 0;
    this.lastModelsQuery = 0;
    this.retryTimer = null;
    this.watchdog = setInterval(() => this.checkHealth(), options.watchdogIntervalMs || 500);
    if (this.watchdog.unref) this.watchdog.unref();
    this.connect();
  }
  now() { return this.options.now ? this.options.now() : Date.now(); }
  static compare(a, b) {
    try { const x = BigInt(a || 0), y = BigInt(b || 0); return x < y ? -1 : x > y ? 1 : 0; }
    catch (_) { return Number(a || 0) - Number(b || 0); }
  }
  emit(name, ...args) { if (this.options[name]) this.options[name](...args); }
  setOnline(value) {
    if (this.online === value) return;
    this.online = value; this.emit("onConnection", value);
  }
  send(data) {
    if (!this.socket || this.socket.readyState !== 1) return false;
    this.socket.send(JSON.stringify(data)); return true;
  }
  connect() {
    if (this.closed) return;
    clearTimeout(this.retryTimer);
    const old = this.socket, generation = ++this.generation;
    if (old) this.connectionLost("reconnecting");
    if (old && old.readyState < 2) old.close();
    const socket = this.socket = new this.Socket(this.url);
    const current = () => !this.closed && this.socket === socket && this.generation === generation;
    socket.onopen = () => {
      if (!current()) return;
      for (const [suffix, type] of Object.entries({"mode/set":"SetPerceptionModeCommand",
        "target/select":"SelectPerceptionTargetCommand", "model/set":"SetDetectorModelCommand"}))
        this.send({op:"advertise", topic:this.root + suffix, type:"xd_uav_track/" + type});
      const topics = {state:"PerceptionState", command_result:"PerceptionCommandResult",
        targets:"PerceptionTargetArray", trajectory:"TargetTrajectoryArray"};
      if (this.options.subscribeVideo !== false) topics.video = "PerceptionVideoFrame";
      for (const [suffix, type] of Object.entries(topics))
        this.send({op:"subscribe", topic:this.root + suffix, type:"xd_uav_track/" + type,
          queue_length:suffix === "command_result" ? 32 : (suffix === "video" || suffix === "targets" ? 2 : 1),
          throttle_rate:suffix === "trajectory" ? 500 : 0});
      this.requestState(); this.requestModels();
    };
    socket.onmessage = event => {
      if (!current()) return;
      try { this.receive(JSON.parse(event.data)); }
      catch (error) { this.emit("onError", error); }
    };
    socket.onclose = () => {
      if (!current()) return;
      this.connectionLost("connection closed");
      this.retryTimer = setTimeout(() => this.connect(), this.options.reconnectMs || 500);
    };
    socket.onerror = () => { if (current()) this.connectionLost("connection error"); };
  }
  requestState() { this.send({op:"call_service", id:"state_get", service:this.root + "state/get", args:{}}); }
  requestModels() {
    this.send({op:"call_service", id:"models_list", service:this.root + "models/list", args:{}});
    this.lastModelsQuery = this.now();
  }
  loseVideo(source, reason) {
    if (!this.videos.has(source)) return;
    this.videos.delete(source); this.emit("onVideoLost", source, reason);
  }
  clearVideos(reason) { for (const source of this.videos.keys()) this.loseVideo(source, reason); }
  connectionLost(reason) {
    this.setOnline(false); this.clearVideos(reason); this.buffer.clear();
    for (const item of this.pending.values()) item.needsReplay = true;
  }
  checkHealth() {
    if (this.closed) return;
    const now = this.now();
    if (this.online && now - this.lastStateAt > (this.options.stateTimeoutMs || 3500)) {
      this.connectionLost("state heartbeat expired"); this.requestState();
    }
    for (const [source, frame] of this.videos)
      if (now - frame.receivedAt > (this.options.videoTimeoutMs || 2500)) this.loseVideo(source, "video heartbeat expired");
  }
  updateState(state) {
    if (!state || !state.session_id || this.retiredSessions.has(state.session_id)) return;
    const previous = this.state;
    if (previous && previous.session_id === state.session_id &&
        (PerceptionClient.compare(state.state_version, previous.state_version) < 0 ||
         PerceptionClient.compare(state.model_epoch, previous.model_epoch) < 0)) return;
    const changedSession = previous && previous.session_id !== state.session_id;
    if (changedSession) {
      this.retiredSessions.add(previous.session_id);
      if (this.retiredSessions.size > 8) this.retiredSessions.delete(this.retiredSessions.values().next().value);
      this.revisions.clear();
    }
    this.state = state; this.lastStateAt = this.now(); this.setOnline(true);
    for (const [source, frame] of this.videos) if (!this.matches(frame.targets)) this.loseVideo(source, "perception state changed");
    this.emit("onState", state);
    if (!this.models.length && this.now() - this.lastModelsQuery > 1000) this.requestModels();
    for (const [id, item] of this.pending) {
      if (item.command.session_id !== state.session_id)
        this.finish(id, {stage:"FAILED", error_code:"STALE_SESSION", message:"server restarted; select again"});
      else if (item.applied && PerceptionClient.compare(state.state_version, item.applied.state_version) >= 0)
        this.finish(id, item.applied);
      else if (item.needsReplay) {
        // Preserve the original request id, version, image frame, ROI, and TTL.
        item.needsReplay = !this.send({op:"publish", topic:item.topic, msg:item.command});
      }
    }
    this.flushData();
  }
  matches(msg) {
    const state = this.state;
    return this.online && state && msg.session_id === state.session_id &&
      PerceptionClient.compare(msg.state_version, state.state_version) === 0 &&
      PerceptionClient.compare(msg.model_epoch, state.model_epoch) === 0 &&
      (msg.mode === undefined || state.mode === undefined || msg.mode === state.mode);
  }
  data(suffix, msg) {
    const metadata = suffix === "video" ? msg.targets : msg;
    if (!metadata || this.retiredSessions.has(metadata.session_id)) return;
    if (this.matches(metadata)) { this.deliver(suffix, msg); return; }
    if (this.state && metadata.session_id === this.state.session_id &&
        (PerceptionClient.compare(metadata.state_version, this.state.state_version) < 0 ||
         PerceptionClient.compare(metadata.model_epoch, this.state.model_epoch) < 0)) return;
    // A committed frame may beat its state publication over rosbridge.
    const key = [suffix, metadata.image_source || "", metadata.session_id, metadata.state_version, metadata.model_epoch].join("|");
    const previous = this.buffer.get(key);
    if (suffix === "video" && previous) {
      const before = previous.msg.targets.header.stamp, after = metadata.header.stamp;
      const order = PerceptionClient.compare(after.secs, before.secs) || PerceptionClient.compare(after.nsecs, before.nsecs);
      if (order < 0 || (order === 0 && PerceptionClient.compare(msg.frame_revision, previous.msg.frame_revision) <= 0)) return;
    }
    this.buffer.delete(key); this.buffer.set(key, {suffix, msg});
    while (this.buffer.size > (this.options.maxBufferedData || 8)) this.buffer.delete(this.buffer.keys().next().value);
  }
  flushData() {
    for (const [key, item] of this.buffer) {
      const meta = item.suffix === "video" ? item.msg.targets : item.msg;
      if (this.matches(meta)) { this.buffer.delete(key); this.deliver(item.suffix, item.msg); }
      else if (this.retiredSessions.has(meta.session_id) ||
        (meta.session_id === this.state.session_id && PerceptionClient.compare(meta.state_version, this.state.state_version) <= 0))
        this.buffer.delete(key);
    }
  }
  deliver(suffix, msg) {
    if (suffix === "video") {
      if (!this.videoTools) throw new Error("load perception_video.js before the video client");
      const source = msg.targets.image_source, previous = this.revisions.get(source);
      if (typeof source !== "string" || !source) throw new Error("video must name its image source");
      if (!previous && this.revisions.size >= this.maxSources) return;
      const capture = msg.targets.header.stamp;
      if (previous && previous.session === msg.targets.session_id) {
        const order = PerceptionClient.compare(capture.secs, previous.stamp.secs) ||
          PerceptionClient.compare(capture.nsecs, previous.stamp.nsecs);
        // An overlay restart may reset revision without restarting perception.
        // A newer capture is then allowed; the same capture still needs a newer revision.
        if (order < 0 || (order === 0 && PerceptionClient.compare(msg.frame_revision, previous.revision) <= 0)) return;
      }
      const frame = this.videoTools.decode(msg);
      frame.receivedAt = this.now();
      this.revisions.set(source, {session:msg.targets.session_id, revision:msg.frame_revision, stamp:{...capture}});
      this.videos.set(source, frame); this.emit("onVideo", frame); return;
    }
    this.emit(suffix === "targets" ? "onTargets" : "onTrajectory", msg);
  }
  receive(packet) {
    if (packet.op === "service_response") {
      if (!packet.result || !packet.values || !packet.values.success) {
        this.emit("onError", new Error("service unavailable: " + packet.service)); return;
      }
      if (packet.id === "state_get") this.updateState(packet.values.state);
      if (packet.id === "models_list") {
        this.models = packet.values.models || []; this.emit("onModels", packet.values);
      }
      return;
    }
    if (packet.op !== "publish" || !packet.topic.startsWith(this.root)) return;
    const suffix = packet.topic.slice(this.root.length), msg = packet.msg;
    if (suffix === "state") this.updateState(msg);
    else if (suffix === "command_result") {
      this.emit("onResult", msg);
      const item = this.pending.get(msg.request_id);
      if (!item || msg.session_id !== item.command.session_id || msg.session_id !== (this.state || {}).session_id) return;
      if (msg.stage === "APPLIED") {
        item.applied = msg;
        if (this.online && PerceptionClient.compare(this.state.state_version, msg.state_version) >= 0) this.finish(msg.request_id, msg);
        else this.requestState();
      } else if (["REJECTED", "FAILED"].includes(msg.stage)) {
        if (["STALE_STATE", "STALE_SESSION"].includes(msg.error_code)) {
          this.setOnline(false); this.clearVideos("state changed; resynchronizing"); this.requestState();
        }
        this.finish(msg.request_id, msg);
      }
    } else if (["targets", "trajectory", "video"].includes(suffix)) this.data(suffix, msg);
  }
  finish(id, result) {
    const item = this.pending.get(id);
    if (!item) return;
    clearTimeout(item.timer); this.pending.delete(id); item.resolve(result);
  }
  command(suffix, fields, frameContext = null) {
    if (!this.online || !this.state) return Promise.reject(new Error("wait for live state after reconnect"));
    if (this.queued >= 16) return Promise.reject(new Error("client command limit reached"));
    fields = JSON.parse(JSON.stringify(fields));
    frameContext = frameContext ? JSON.parse(JSON.stringify(frameContext)) : null;
    const session = this.state.session_id;
    const immediate = this.queued === 0;
    this.queued++; this.emit("onBusy", true);
    const run = () => {
      if (this.closed || !this.online || !this.state) throw new Error("wait for live state after reconnect");
      if (this.state.session_id !== session) throw new Error("server restarted; select again");
      if (frameContext && !this.matches(frameContext)) throw new Error("displayed frame belongs to a previous perception state; select again");
      return this.sendCommand(suffix, fields);
    };
    let operation;
    try { operation = immediate ? run() : this.serial.then(run); }
    catch (error) { operation = Promise.reject(error); }
    this.serial = operation.catch(() => {});
    return operation.finally(() => { this.queued--; this.emit("onBusy", this.queued > 0); });
  }
  sendCommand(suffix, fields) {
    const id = this.state.session_id + ":web:" + this.now() + ":" + (++this.sequence);
    const command = Object.assign({request_id:id, session_id:this.state.session_id,
      expected_state_version:this.state.state_version, issued_at:this.state.header.stamp, ttl_sec:20}, fields);
    return new Promise(resolve => {
      const item = {topic:this.root + suffix, command:JSON.parse(JSON.stringify(command)), resolve, needsReplay:false};
      item.timer = setTimeout(() => {
        this.setOnline(false); this.clearVideos("command timeout; resynchronizing"); this.requestState();
        this.finish(id, {stage:"FAILED", error_code:"CLIENT_TIMEOUT", message:"refresh state before retrying"});
      }, this.options.commandTimeoutMs || 35000);
      this.pending.set(id, item);
      item.needsReplay = !this.send({op:"publish", topic:item.topic, msg:item.command});
    });
  }
  mode(mode) { return this.command("mode/set", {mode}); }
  model(model_id) { return this.command("model/set", {model_id}); }
  selection(image_source) {
    return {selection_type:"clear", track_id:-1, target_uid:"", image_source,
      frame_seq:0, capture_stamp:{secs:0,nsecs:0}, point_norm:[0,0], roi_norm:[0,0,0,0], tracker_algorithm:"auto"};
  }
  clear() { return this.command("target/select", this.selection("")); }
  select(image_source, target_uid, frameContext = null) {
    return this.command("target/select", {...this.selection(image_source), selection_type:"track_id", target_uid}, frameContext);
  }
  region(image_source, header, roi_norm, point = false, frameContext = null) {
    return this.command("target/select", {...this.selection(image_source), selection_type:point ? "point" : "roi",
      frame_seq:frameContext ? frameContext.frame_seq : header.seq, capture_stamp:header.stamp,
      point_norm:point ? roi_norm : [0,0], roi_norm:point ? [0,0,0,0] : roi_norm}, frameContext);
  }
  withCurrentFrame(image_source, callback) {
    const frame = this.videos.get(image_source);
    if (!frame || !this.matches(frame.targets) ||
        this.now() - frame.receivedAt > (this.options.videoTimeoutMs || 2500))
      throw new Error("no live video frame for this source");
    return callback ? callback(frame) : frame;
  }
  disconnect() { if (this.socket) this.socket.close(); }
  close() {
    this.closed = true; ++this.generation; clearTimeout(this.retryTimer); clearInterval(this.watchdog);
    this.connectionLost("client closed");
    for (const id of Array.from(this.pending.keys())) this.finish(id, {stage:"FAILED", error_code:"CLIENT_CLOSED"});
    if (this.socket) this.socket.close();
  }
}
if (typeof module !== "undefined") module.exports = PerceptionClient;
else globalThis.PerceptionClient = PerceptionClient;
