"use strict";
// Controller smoke test with a simulated DOM/server; not a browser/GPU test.
const assert = require("assert"), fs = require("fs"), vm = require("vm");
class Element {
  constructor() { this.value = ""; this.textContent = ""; this.children = []; this.dataset = {}; }
  replaceChildren() { this.children = []; this.value = ""; }
  append(item) { this.children.push(item); if (!this.value) this.value = item.value; }
  get options() { return this.children; }
  get selectedOptions() { return this.children.filter(item => item.value === this.value); }
}
const elements = Object.fromEntries(["url", "namespace", "models", "connect", "disconnect", "status",
  "actual", "result", "log", "state", "model-info", "switch", "refresh"].map(id => [id, new Element()]));
elements.namespace.value = "uav1";
const modes = ["detect", "track", "fusion"].map(mode => Object.assign(new Element(), {dataset:{mode}}));
class Client {
  constructor(url, namespace, options) { this.options = options; this.models = []; this.online = false; Client.current = this; }
  static compare(a, b) { return a - b; }
  ready() {
    this.online = true; this.state = {session_id:"s", state_version:1, model_epoch:1, mode:"detect", active_model_id:"train7"};
    this.options.onConnection(true); this.options.onState(this.state);
    this.models = [{model_id:"train7", engine_prepared:true}, {model_id:"train4", engine_prepared:true}, {model_id:"train", engine_prepared:false}];
    this.options.onModels({models:this.models});
  }
  async change(field, value) {
    this.options.onBusy(true);
    const result = this.reject ? {stage:"REJECTED", error_code:"INSUFFICIENT_MEMORY", message:"old model retained"} :
      {stage:"APPLIED", session_id:"s", state_version:++this.state.state_version};
    if (!this.reject) this.state[field] = value;
    this.options.onState(this.state); this.options.onBusy(false); return result;
  }
  mode(value) { return this.change("mode", value); }
  model(value) { return this.change("active_model_id", value); }
  requestModels() { this.refreshed = true; }
  close() { this.online = false; this.closed = true; }
}
vm.runInNewContext(fs.readFileSync(__dirname + "/switch_test.js", "utf8"), {
  document:{getElementById:id => elements[id], querySelectorAll:() => modes, createElement:() => new Element()},
  location:{hostname:"localhost", protocol:"file:"}, window:{addEventListener:() => {}},
  PerceptionClient:Client, URL, AbortController, setTimeout, clearTimeout, console
});
const tick = () => new Promise(resolve => setImmediate(resolve));
async function main() {
  assert(modes[0].disabled); elements.connect.onclick(); const client = Client.current;
  assert.strictEqual(client.options.subscribeVideo, false); client.ready();
  assert(!modes[0].disabled); assert(elements.models.options[2].disabled);
  modes[1].onclick(); await tick(); assert(elements.result.textContent.startsWith("PASS"));
  assert.strictEqual(client.state.mode, "track");
  elements.models.value = "train4"; elements.models.onchange(); elements.switch.onclick(); await tick();
  assert(elements.result.textContent.startsWith("PASS")); assert.strictEqual(client.state.active_model_id, "train4");
  client.reject = true; elements.models.value = "train7"; elements.switch.onclick(); await tick();
  assert(elements.result.textContent.includes("INSUFFICIENT_MEMORY")); assert.strictEqual(client.state.active_model_id, "train4");
  elements.refresh.onclick(); assert(client.refreshed); elements.disconnect.onclick();
  assert(client.closed); assert(modes.every(button => button.disabled));
  console.log("simple switch page mode/model success, rejection, no-video and disconnect tests passed");
}
main().catch(error => { console.error(error); process.exitCode = 1; });
