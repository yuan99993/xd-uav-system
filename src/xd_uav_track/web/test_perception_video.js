"use strict";
const assert = require("assert"), zlib = require("zlib");
const Client = require("./perception_client"), Video = require("./perception_video");
class Socket {
  constructor() { this.readyState = 0; this.sent = []; }
  open() { this.readyState = 1; this.onopen(); }
  send(raw) { this.sent.push(JSON.parse(raw)); }
  receive(packet) { this.onmessage({data:JSON.stringify(packet)}); }
  close() { this.readyState = 3; if (this.onclose) this.onclose(); }
}
function crc32(buffer) {
  let crc = 0xffffffff;
  for (const byte of buffer) { crc ^= byte; for (let i = 0; i < 8; i++) crc = (crc >>> 1) ^ ((crc & 1) ? 0xedb88320 : 0); }
  return (crc ^ 0xffffffff) >>> 0;
}
function png(width = 4, height = 2) {
  const chunk = (type, payload) => {
    const name = Buffer.from(type), result = Buffer.alloc(payload.length + 12);
    result.writeUInt32BE(payload.length, 0); name.copy(result, 4); payload.copy(result, 8);
    result.writeUInt32BE(crc32(Buffer.concat([name, payload])), payload.length + 8); return result;
  };
  const ihdr = Buffer.alloc(13); ihdr.writeUInt32BE(width, 0); ihdr.writeUInt32BE(height, 4); ihdr[8] = 8; ihdr[9] = 2;
  return Buffer.concat([Buffer.from([137,80,78,71,13,10,26,10]), chunk("IHDR", ihdr),
    chunk("IDAT", zlib.deflateSync(Buffer.alloc((width * 3 + 1) * height))), chunk("IEND", Buffer.alloc(0))]);
}
function state(version = 1, epoch = 1, mode = "detect", session = "s") {
  return {session_id:session, state_version:version, model_epoch:epoch, mode, active_model_id:"train7",
    header:{seq:99, stamp:{secs:100, nsecs:0}, frame_id:"camera"}};
}
function bundle(source = "fixed_rgb", revision = 1, version = 1, epoch = 1, mode = "detect", session = "s", secs = 100) {
  const header = {seq:7, stamp:{secs, nsecs:12}, frame_id:"camera"};
  return {header:{...header, seq:1000}, frame_revision:revision,
    image:{header:{...header}, format:"bgr8; png compressed bgr8", data:png().toString("base64")},
    targets:{header:{...header, seq:1001}, frame_seq:7, image_source:source, image_width:4, image_height:2,
      session_id:session, state_version:version, model_epoch:epoch, mode, model_id:"train7", targets:[]}};
}
function setup(options = {}) {
  let now = 1000;
  const frames = [], lost = [], errors = [], targets = [], trajectories = [];
  const client = new Client("ws://test", "uav1", {WebSocketClass:Socket, now:() => now,
    onVideo:frame => frames.push(frame), onVideoLost:(source, reason) => lost.push([source, reason]),
    onError:error => errors.push(error), onTargets:frame => targets.push(frame), onTrajectory:frame => trajectories.push(frame), ...options});
  client.socket.open();
  const publish = (suffix, msg, socket = client.socket) => socket.receive({op:"publish", topic:"/uav1/perception/" + suffix, msg});
  publish("state", state());
  return {client, frames, lost, errors, targets, trajectories, publish, time:value => { now = value; }};
}
const turn = () => new Promise(resolve => setImmediate(resolve));
function published(socket) { return socket.sent.filter(packet => packet.op === "publish"); }

function testGeometryAndPng() {
  const payload = png(1920, 1080);
  assert.equal(Video.pngMetadata(payload.toString("base64")).width, 1920);
  assert.equal(Video.pngMetadata(Array.from(payload)).height, 1080);
  assert.throws(() => Video.pngMetadata([1,2,3]), /PNG/);
  assert.throws(() => Video.bytes([300]), /invalid bytes/);
  const oversized = []; oversized.length = Video.MAX_BYTES + 1;
  assert.throws(() => Video.bytes(oversized), /byte limit/); // Sparse array: no huge payload allocation/copy.
  const hugeHeader = Buffer.from(png()); hugeHeader.writeUInt32BE(Video.MAX_PIXELS,16); hugeHeader.writeUInt32BE(2,20);
  assert.throws(() => Video.pngMetadata(hugeHeader), /pixel limit/);
  const encodedLimit = Math.ceil(Video.MAX_BYTES / 3) * 4;
  assert.throws(() => Video.bytes("A".repeat(encodedLimit)), /byte limit/); // No atob/byte allocation occurs.
  const frame = bundle(); assert.equal(Video.decode(frame).height, 2);
  frame.image.data = Array.from(png()); assert.equal(Video.decode(frame).width, 4);
  frame.targets.image_width = 5; assert.throws(() => Video.decode(frame), /dimensions/);
  frame.targets.image_width = 4; frame.image.header.seq = 6; assert.throws(() => Video.decode(frame), /different frames/);
  const rect = {left:10, top:20, width:800, height:600};
  assert.equal(Video.point(rect, 1920, 1080, 410, 30), null); // Top letterbox.
  assert.deepEqual(Video.point(rect, 1920, 1080, 410, 320), [960,540]);
  assert.deepEqual(Video.point(rect, 1920, 1080, 910, 720, true), [1920,1080]);
  assert.deepEqual(Video.region([960,540], [480,270], 1920,1080), [.25,.25,.25,.25]);
  const vertical = {left:0,top:0,width:1000,height:400};
  assert.equal(Video.point(vertical, 200,400,10,200), null); // Side letterbox.
  assert.deepEqual(Video.point(vertical, 200,400,500,200), [100,200]);
}
function testFrameGates() {
  const t = setup();
  try {
    const subscription = t.client.socket.sent.find(packet => packet.op === "subscribe" && packet.topic.endsWith("/video"));
    assert.equal(subscription.throttle_rate, 0); assert(subscription.queue_length > 1);
    t.publish("video", bundle()); t.publish("video", bundle("gimbal_rgb")); assert.equal(t.frames.length, 2);
    t.publish("video", bundle()); assert.equal(t.frames.length, 2); // Exact revision duplicate.
    const cleared = bundle("fixed_rgb", 2); cleared.targets.targets = [];
    t.publish("video", cleared); assert.equal(t.frames.length, 3); // Same capture, new LOST/clear revision.
    t.publish("video", bundle("fixed_rgb", 1)); assert.equal(t.frames.length, 3); // Old same-capture overlay.
    t.publish("video", bundle("fixed_rgb", 1, 1, 1, "detect", "s", 101));
    assert.equal(t.frames.length, 4); // Overlay restart: newer capture beats reset revision.
    t.publish("video", bundle("fixed_rgb", 100, 1, 1, "detect", "s", 100)); assert.equal(t.frames.length, 4);
    t.publish("video", bundle("fixed_rgb", 3, 2, 2, "fusion", "s", 102)); assert.equal(t.frames.length, 4);
    t.publish("state", state(2, 2, "fusion")); assert.equal(t.frames.length, 5); // Future data was cached.
    assert(t.lost.some(([source]) => source === "gimbal_rgb"));
    t.publish("video", bundle("fixed_rgb", 4, 2, 1, "fusion", "s", 103)); assert.equal(t.frames.length, 5); // Old epoch.
    t.publish("video", bundle("fixed_rgb", 4, 2, 2, "track", "s", 103)); assert.equal(t.frames.length, 5); // Wrong mode.
    t.publish("video", bundle("fixed_rgb", 4, 1, 2, "fusion", "s", 103)); assert.equal(t.frames.length, 5); // Old version.
    const broken = bundle("fixed_rgb", 4, 2, 2, "fusion", "s", 103); broken.targets.image_width = 50;
    t.publish("video", broken); assert.equal(t.errors.length, 1); assert.equal(t.frames.length, 5);
    t.publish("video", bundle("fixed_rgb", 4, 2, 2, "fusion", "s", 103)); assert.equal(t.frames.length, 6);
    t.publish("targets", {session_id:"s", state_version:3, model_epoch:2, mode:"fusion",image_source:"fixed_rgb"});
    t.publish("trajectory", {session_id:"s", state_version:3, model_epoch:2, mode:"fusion"});
    assert.equal(t.targets.length, 0); assert.equal(t.trajectories.length, 0);
    t.publish("state", state(3,2,"fusion")); assert.equal(t.targets.length,1); assert.equal(t.trajectories.length,1);
    t.publish("video", bundle("fixed_rgb",1,1,1,"detect","new",104));
    t.publish("state", state(1,1,"detect","new")); assert.equal(t.frames.length,7);
    t.publish("state", state(100,9,"track","s")); assert.equal(t.client.state.session_id,"new");
    t.publish("video", bundle("fixed_rgb",500,100,9,"track","s",105)); assert.equal(t.frames.length,7);
    for (let version=2; version<30; version++) t.publish("video", bundle("fixed_rgb",version,version,1,"detect","new",106));
    assert(t.client.buffer.size <= 8);
    t.publish("state", state(30,1,"detect","new"));
    for (let source=0; source<30; source++) t.publish("video", bundle("unexpected_" + source,1,30,1,"detect","new",107));
    assert(t.client.revisions.size <= 8); assert(t.client.videos.size <= 8);
  } finally { t.client.close(); }
}
async function testSerialTransactions() {
  const t = setup();
  try {
    const first = t.client.mode("fusion"), second = t.client.model("train4");
    let sent = published(t.client.socket); assert.equal(sent.length,1);
    const id = sent[0].msg.request_id;
    t.publish("command_result", {session_id:"s", request_id:id, stage:"ACCEPTED",state_version:1});
    await turn(); assert.equal(published(t.client.socket).length,1);
    t.publish("command_result", {session_id:"s", request_id:id, stage:"APPLIED",state_version:2});
    await turn(); assert.equal(published(t.client.socket).length,1); // APPLIED alone is insufficient.
    t.publish("state", state(2,1,"fusion")); assert.equal((await first).stage,"APPLIED");
    await turn(); sent = published(t.client.socket); assert.equal(sent.length,2); assert.equal(sent[1].msg.expected_state_version,2);
    t.publish("state", state(3,2,"fusion"));
    t.publish("command_result", {session_id:"s", request_id:sent[1].msg.request_id,stage:"APPLIED",state_version:3});
    assert.equal((await second).stage,"APPLIED");
    const same = t.client.mode("fusion"), sameRequest = published(t.client.socket).at(-1);
    t.publish("command_result", {session_id:"s", request_id:sameRequest.msg.request_id,stage:"APPLIED",state_version:3});
    assert.equal((await same).stage,"APPLIED"); // Idempotent operation may not increase version.
    const frozen = bundle("fixed_rgb",1,3,2,"fusion"); t.publish("video",frozen);
    const frame = t.client.withCurrentFrame("fixed_rgb"), context = Video.identity(frame);
    const selection = t.client.region("fixed_rgb",frame.targets.header,[.1,.2,.3,.4],false,context);
    const original = published(t.client.socket).at(-1);
    assert.equal(original.msg.frame_seq,7); assert.deepEqual(original.msg.capture_stamp,{secs:100,nsecs:12});
    const old = t.client.socket; t.client.disconnect(); clearTimeout(t.client.retryTimer); t.client.connect(); t.client.socket.open();
    t.publish("state",state(4,2,"fusion"));
    const replay = published(t.client.socket).at(-1); assert.deepEqual(replay,original);
    old.receive({op:"publish",topic:"/uav1/perception/state",msg:state(100,9,"track","old-socket")});
    assert.equal(t.client.state.session_id,"s"); // Old connection callback cannot mutate the new session.
    old.onclose(); assert(t.client.online); // Old close callback cannot schedule another connection.
    t.publish("command_result",{session_id:"s",request_id:original.msg.request_id,stage:"APPLIED",state_version:4});
    assert.equal((await selection).stage,"APPLIED");
    await assert.rejects(t.client.region("fixed_rgb",frame.targets.header,[.1,.2],true,context),/previous perception state/);
    const clear = t.client.clear(), clearRequest = published(t.client.socket).at(-1);
    assert.equal(clearRequest.msg.selection_type,"clear");
    t.publish("command_result",{session_id:"s",request_id:clearRequest.msg.request_id,stage:"REJECTED",error_code:"NOT_READY",state_version:4});
    assert.equal((await clear).error_code,"NOT_READY");
  } finally { t.client.close(); }
}
function testHeartbeatAndVideoOptOut() {
  const t = setup();
  try {
    t.publish("video",bundle()); assert.equal(t.client.withCurrentFrame("fixed_rgb", frame => frame.width),4);
    t.time(3600); t.client.checkHealth(); assert.equal(t.client.videos.size,0); assert(t.client.online);
    assert.throws(() => t.client.withCurrentFrame("fixed_rgb"),/no live/);
    t.time(5000); t.client.checkHealth(); assert(!t.client.online);
    assert(t.lost.length); t.publish("state",state()); assert(t.client.online);
    t.publish("video",bundle("fixed_rgb",2)); assert.equal(t.client.videos.size,1);
    t.client.disconnect(); assert.equal(t.client.videos.size,0); assert(!t.client.online);
  } finally { t.client.close(); }
  const quiet = setup({subscribeVideo:false});
  try { assert(!quiet.client.socket.sent.some(packet => packet.op === "subscribe" && packet.topic.endsWith("/video"))); }
  finally { quiet.client.close(); }
}
async function main() {
  testGeometryAndPng(); testFrameGates(); await testSerialTransactions(); testHeartbeatAndVideoOptOut();
  console.log("native video geometry/PNG/frame-gating/serial-transaction/heartbeat/reconnect tests passed");
}
main().catch(error => { console.error(error); process.exit(1); });
