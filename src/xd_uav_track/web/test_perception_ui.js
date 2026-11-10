/* No browser/GPU required: exercise page frame ownership and pointer transactions. */
"use strict";
const assert = require("assert"), fs = require("fs"), vm = require("vm");
const Video = require("./perception_video");
class Element {
  constructor(tag = "div") {
    this.tag = tag; this.children = []; this.value = ""; this.dataset = {}; this.events = {};
    this.classList = {toggle() {}}; this.pointers = new Set(); this.context = {clearRect() {}, drawImage() {}, setLineDash() {}, strokeRect() {}};
  }
  append(...children) { this.children.push(...children); if (this.tag === "select" && !this.value) this.value = children[0].value; }
  replaceChildren(...children) { this.children = children; }
  get options() { return this.children; }
  get selectedOptions() { return this.children.filter(child => child.value === this.value); }
  setAttribute() {}
  addEventListener(name, callback) { this.events[name] = callback; }
  getContext() { return this.context; }
  getBoundingClientRect() { return {left:0, top:0, width:800, height:600}; }
  setPointerCapture(id) { this.pointers.add(id); }
  hasPointerCapture(id) { return this.pointers.has(id); }
  releasePointerCapture(id) { this.pointers.delete(id); }
}
class StubClient {
  constructor(url, namespace, options) {
    this.url = url; this.namespace = namespace; this.options = options; this.online = true;
    this.models = [{model_id:"train7"}]; this.state = state(); this.calls = [];
    StubClient.instances.push(this);
  }
  matches(metadata) { return this.online && ["session_id","state_version","model_epoch","mode"].every(key => metadata[key] === this.state[key]); }
  region(...args) { this.calls.push(args); return Promise.resolve({stage:"APPLIED"}); }
  model(modelId) { this.calls.push(["model",modelId]); return Promise.resolve({stage:"APPLIED"}); }
  close() { this.online = false; }
}
StubClient.instances = [];
function state(version = 1, epoch = 1) {
  return {session_id:"s",state_version:version,model_epoch:epoch,mode:"detect",active_model_id:"train7",operating_state:"ACTIVE",transaction_state:"IDLE"};
}
function frame(seq, version = 1, epoch = 1) {
  return {width:1920,height:1080,pngBytes:new Uint8Array([1]),frame_revision:seq,
    targets:{...state(version,epoch),frame_seq:seq,image_source:"fixed_rgb",model_id:"train7",
      header:{seq:999,stamp:{secs:seq,nsecs:123},frame_id:"camera"},targets:[]}};
}
const turn = () => new Promise(resolve => setImmediate(resolve));
async function main() {
  const elements = new Map(), created = [], modeButtons = ["detect","track","fusion"].map(mode => {
    const element = new Element("button"); element.dataset.mode = mode; return element;
  });
  for (const id of ["url","ns","result","models","source","videos","targets","trajectory","connect","connection","actual","state","empty","freeze","resume","switch","clear"])
    elements.set(id,new Element(["models","source"].includes(id) ? "select" : "div"));
  elements.get("ns").value = "uav1";
  const document = {
    getElementById:id => elements.get(id), createElement:tag => { const element = new Element(tag); created.push(element); return element; },
    querySelectorAll:selector => selector === "[data-mode]" ? modeButtons : selector === "[data-command]" ?
      [...modeButtons,...["switch","freeze","clear"].map(id=>elements.get(id))] : []
  };
  const bitmaps = [];
  const context = vm.createContext({document,location:{protocol:"file:",hostname:"remote-host"},window:{addEventListener() {}},
    PerceptionClient:StubClient,PerceptionVideo:Video,Blob,URL,console,setTimeout,clearTimeout,
    createImageBitmap:async () => { const bitmap = {closed:false,close() { this.closed = true; }}; bitmaps.push(bitmap); return bitmap; }});
  vm.runInContext(fs.readFileSync(__dirname + "/perception_ui.js","utf8"),context);
  assert.equal(elements.get("url").value,"ws://remote-host:9090");
  elements.get("connect").onclick(); const client = StubClient.instances.at(-1);
  client.options.onState(client.state); client.options.onVideo(frame(10)); await turn();
  const canvas = created.find(element => element.tag === "canvas");
  assert.equal(canvas.width,1920); assert.equal(canvas.height,1080); // Native storage; CSS rectangle is 800 x 600.
  const pointer = (x,y) => ({button:0,pointerId:1,clientX:x,clientY:y,preventDefault() {}});
  canvas.events.pointerdown(pointer(400,20)); assert(!canvas.hasPointerCapture(1)); // Black letterbox cannot select.
  canvas.events.pointerdown(pointer(200,200)); assert(canvas.hasPointerCapture(1));
  client.options.onVideo(frame(11)); await turn(); // Newest video arrives while original frame stays frozen.
  await canvas.events.pointerup(pointer(600,400)); await turn();
  assert.equal(client.calls.length,1); const [source,header,roi,point,identity] = client.calls[0];
  assert.equal(source,"fixed_rgb"); assert.equal(header.seq,10); assert.equal(header.stamp.secs,10);
  assert.equal(identity.frame_seq,10); assert(!point); assert.deepEqual(Array.from(roi),[.25,5/18,.5,4/9]);
  elements.get("freeze").onclick();
  client.options.onVideoLost("fixed_rgb","perception state changed"); // Frozen, with no active pointer gesture.
  assert(created.find(element=>element.className==="video-notice").textContent.startsWith("LOST"));
  assert(elements.get("freeze").disabled);
  client.options.onVideo(frame(11)); await turn();
  canvas.events.pointerdown(pointer(400,300));
  client.state = state(2,2); client.options.onState(client.state);
  await canvas.events.pointerup(pointer(500,350)); assert.equal(client.calls.length,1); // Epoch switch cancels the gesture.
  client.options.onVideoLost("fixed_rgb","connection closed");
  assert(created.find(element=>element.className==="video-notice").textContent.startsWith("LOST"));
  assert(bitmaps.every(bitmap=>bitmap.closed));
  client.options.onVideo(frame(12,2,2)); await turn();
  canvas.events.pointerdown(pointer(400,300)); await canvas.events.pointerup(pointer(400,300));
  assert.equal(client.calls.length,2); assert.equal(client.calls[1][3],true); assert.equal(client.calls[1][1].seq,12);
  for (let source=0;source<20;source++) {
    const extra = frame(13,2,2); extra.targets.image_source = "unexpected_" + source; client.options.onVideo(extra);
  }
  await turn(); assert(elements.get("videos").children.length <= 8);
  // A camera/state heartbeat must not silently change the requested model.
  client.models=["train7","train4","train"].map(model_id=>({model_id,engine_prepared:true}));
  client.options.onModels({models:client.models}); client.options.onState(client.state);
  elements.get("models").value="train4";
  for (let index=0;index<20;index++) client.options.onState({...client.state,state_version:3+index});
  assert.equal(elements.get("models").value,"train4");
  await elements.get("switch").onclick();
  assert.deepEqual(client.calls.at(-1),["model","train4"]);
  // Mode changes are not model commits; failed requests keep the choice.
  client.options.onState({...client.state,mode:"fusion",state_version:25});
  assert.equal(elements.get("models").value,"train4");
  // A committed model/epoch synchronizes only after the transaction is idle.
  client.options.onBusy(true);
  const committed={...client.state,active_model_id:"train",model_epoch:3,state_version:26};
  client.options.onState(committed); assert.equal(elements.get("models").value,"train4");
  client.options.onBusy(false); client.state=committed; client.options.onState(committed);
  assert.equal(elements.get("models").value,"train");
  elements.get("models").value="train4";
  client.options.onState({...committed,session_id:"new-session"});
  assert.equal(elements.get("models").value,"train");
  console.log("page native canvas/frozen-frame ROI/black-bar/epoch-cancel/LOST/click tests passed");
}
main().catch(error=>{console.error(error);process.exit(1);});
