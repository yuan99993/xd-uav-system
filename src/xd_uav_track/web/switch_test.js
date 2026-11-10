"use strict";
(() => {
  const el = id => document.getElementById(id), modes = [...document.querySelectorAll("[data-mode]")];
  let client = null, busy = false, generation = 0;
  const history = [];
  const host = (location.hostname || "127.0.0.1").replace(/^\[|\]$/g, "");
  const socketBase = (location.protocol === "https:" ? "wss://" : "ws://") +
    (host.includes(":") ? "[" + host + "]" : host) + ":";
  el("url").value = socketBase + "9090";
  function controls() {
    const live = Boolean(client && client.online);
    for (const button of modes) button.disabled = !live || busy;
    el("refresh").disabled = !live || busy;
    el("models").disabled = !live || busy || !client.models.length;
    const option = el("models").selectedOptions[0];
    el("switch").disabled = !live || busy || !option || option.disabled;
    el("disconnect").disabled = !client;
  }
  function log(value) {
    history.push(typeof value === "string" ? value : JSON.stringify(value));
    if (history.length > 60) history.shift();
    el("log").textContent = history.join("\n");
  }
  function modelInfo() {
    const item = client?.models.find(model => model.model_id === el("models").value);
    el("model-info").textContent = item ? JSON.stringify({model_id:item.model_id,
      engine_prepared:item.engine_prepared, validated:item.validated, class_names:item.class_names}) : "无可用模型列表";
    controls();
  }
  async function change(kind, desired) {
    const owner = client, currentGeneration = generation;
    if (!owner?.online || busy) return;
    el("result").className = ""; el("result").textContent = "执行中，请等待命令终态及实际 state…";
    try {
      const result = await (kind === "mode" ? owner.mode(desired) : owner.model(desired));
      if (currentGeneration !== generation) return;
      const state = owner.state;
      const matches = owner.online && result.stage === "APPLIED" && state &&
        state.session_id === result.session_id &&
        PerceptionClient.compare(state.state_version, result.state_version) >= 0 &&
        (kind === "mode" ? state.mode === desired : state.active_model_id === desired);
      el("result").className = matches ? "good" : "bad";
      el("result").textContent = matches ? "PASS：" + kind + " 已实际切换为 " + desired +
        "（版本 " + state.state_version + "，epoch " + state.model_epoch + "）" :
        "未通过：" + (result.error_code || result.stage) + " · " + (result.message || "实际状态不符或连接失效");
      log(result);
    } catch (error) {
      if (currentGeneration !== generation) return;
      el("result").className = "bad"; el("result").textContent = "失败：" + error.message;
      log(error.message);
    } finally { if (currentGeneration === generation) controls(); }
  }
  el("connect").onclick = () => {
    try {
      const url = new URL(el("url").value.trim());
      if (!["ws:", "wss:"].includes(url.protocol)) throw new Error("地址必须使用 ws:// 或 wss://");
      const namespace = el("namespace").value.trim().replace(/^\/+|\/+$/g, "");
      if (!/^[A-Za-z_][A-Za-z0-9_]*(\/[A-Za-z_][A-Za-z0-9_]*)*$/.test(namespace)) throw new Error("请输入有效 UAV 命名空间");
      ++generation; if (client) client.close(); client = null; busy = false;
      el("models").replaceChildren(); el("actual").textContent = "等待新的服务端状态";
      el("state").textContent = ""; el("status").textContent = "连接中…";
      const ownGeneration = generation;
      const guarded = callback => (...args) => { if (generation === ownGeneration) callback(...args); };
      client = new PerceptionClient(url.href, namespace, {
        subscribeVideo:false,
        onConnection:guarded(live => { el("status").textContent = live ? "已连接，state 新鲜" : "连接 / state 已失效，等待重连"; controls(); }),
        onBusy:guarded(value => { busy = value; controls(); }),
        onState:guarded(state => {
          el("actual").textContent = "实际模式：" + state.mode + " | 实际模型：" + state.active_model_id +
            " | epoch：" + state.model_epoch + " | model_ready：" + state.model_ready + " | 事务：" + state.transaction_state;
          el("state").textContent = JSON.stringify(state, null, 2); controls();
        }),
        onModels:guarded(response => {
          const desired = el("models").value || client?.state?.active_model_id;
          el("models").replaceChildren();
          for (const model of response.models) {
            const option = document.createElement("option"); option.value = model.model_id;
            option.textContent = model.model_id;
            option.disabled = !model.engine_prepared; el("models").append(option);
          }
          if ([...el("models").options].some(option => option.value === desired)) el("models").value = desired;
          modelInfo();
        }),
        onResult:guarded(log), onError:guarded(error => log(error.message))
      });
      controls();
    } catch (error) { el("result").className = "bad"; el("result").textContent = error.message; }
  };
  el("disconnect").onclick = () => { ++generation; client?.close(); client = null; busy = false;
    el("status").textContent = "已主动断开"; el("actual").textContent = "旧状态无效，请重新连接"; controls(); };
  for (const button of modes) button.onclick = () => change("mode", button.dataset.mode);
  el("switch").onclick = () => change("model", el("models").value);
  el("refresh").onclick = () => client?.requestModels();
  el("models").onchange = modelInfo;
  window.addEventListener("beforeunload", () => client?.close());
  const abort = new AbortController(), timer = setTimeout(() => abort.abort(), 2000);
  if (location.protocol !== "file:") fetch("perception_runtime.json", {signal:abort.signal, cache:"no-store"})
    .then(response => response.ok ? response.json() : null).then(config => {
      if (!config || client) return;
      if (typeof config.uav_name === "string") el("namespace").value = config.uav_name;
      if (Number.isInteger(config.websocket_port) && config.websocket_port > 0 && config.websocket_port < 65536)
        el("url").value = socketBase + config.websocket_port;
    }).catch(() => {}).finally(() => clearTimeout(timer));
  else clearTimeout(timer);
  controls();
})();
