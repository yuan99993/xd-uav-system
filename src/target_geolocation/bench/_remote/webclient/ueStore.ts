// ============================================================================
// UE 仿真视窗 —— 共享控制 store
// 顶栏「仿真视窗」菜单与 UeView(iframe) 通过 useSyncExternalStore 共享同一份状态，
// 避免两处各自维护地址/运行态导致不一致。
// ============================================================================
import { useSyncExternalStore } from 'react';
import { UE_HOST, UE_HTTP_PORT, UE_CONTROL_PORT, UE_PLAYER_URL, ueControlBase, UE_ORCH_PORT } from './config';

export interface UeAddr {
  host: string;
  httpPort: number;
  ctlPort: number;
}

interface UeState {
  addr: UeAddr;
  /** 像素流编码器：VP8（Jetson kiosk Chromium 可解码，推荐）/ H264（需 H.264 能力的浏览器） */
  codec: string;
  /** 像素流帧率（fps）。启动仿真时以 -PixelStreamingFrameRate 下发给 UE。
   *  帧率直接决定 RTP 包率，而 Jetson 端 rtpjitterbuffer 的开销与包率成正比，
   *  所以降帧率比降分辨率更能省解码端的 CPU。60 -> 30 约可省一半包处理量。 */
  frameRate: number;
  running: boolean;
  busy: boolean;
  msg: string;
  timer: number; // 启动就绪倒计时（秒），>0 时提示“约 N s 后画面就绪”
}

const LS_KEY = 'ue-addr';

function readAddr(): UeAddr {
  const def: UeAddr = { host: UE_HOST, httpPort: UE_HTTP_PORT, ctlPort: UE_CONTROL_PORT };
  try {
    // URL 参数优先：?ue=<host>:<httpPort>&uectl=<ctlPort>
    const p = new URLSearchParams(window.location.search);
    const ue = p.get('ue');
    if (ue) {
      const [h, port] = ue.trim().split(':');
      if (h) {
        def.host = h;
        if (port) def.httpPort = parseInt(port, 10) || def.httpPort;
      }
    }
    const uc = p.get('uectl');
    if (uc && /^\d+$/.test(uc.trim())) def.ctlPort = parseInt(uc.trim(), 10);
    // localStorage 覆盖
    const saved = localStorage.getItem(LS_KEY);
    if (saved) {
      const o = JSON.parse(saved);
      if (o.host) def.host = o.host;
      if (o.httpPort) def.httpPort = o.httpPort;
      if (o.ctlPort) def.ctlPort = o.ctlPort;
    }
  } catch {
    /* ignore */
  }
  return def;
}

/** 像素流编码器选择：与地址共用 LS_KEY 持久化，缺省 H264
 *（Jetson 端只有 H264 能走 NVDEC 硬解；VP8 无硬件解码器，仅 PC 浏览器 iframe 可用） */
function readCodec(): string {
  try {
    const saved = localStorage.getItem(LS_KEY);
    if (saved) {
      const o = JSON.parse(saved);
      if (o.codec === 'H264' || o.codec === 'VP8') return o.codec;
    }
  } catch {
    /* ignore */
  }
  return 'H264';
}

/** 像素流帧率：与地址/编码器共用 LS_KEY 持久化，缺省 30。
 *  60 是 UE 默认值，但 Jetson 端解码开销也最高；30 在遥操作手感上仍可接受。
 *  只接受 10~120 的整数，避免 localStorage 里被写进脏值导致 UE 起不来。 */
export const DEFAULT_FRAME_RATE = 30;

function readFrameRate(): number {
  try {
    const saved = localStorage.getItem(LS_KEY);
    if (saved) {
      const o = JSON.parse(saved);
      const n = Number(o.frameRate);
      if (Number.isFinite(n) && n >= 10 && n <= 120) return Math.round(n);
    }
  } catch {
    /* ignore */
  }
  return DEFAULT_FRAME_RATE;
}

let state: UeState = {
  addr: readAddr(),
  codec: readCodec(),
  frameRate: readFrameRate(),
  running: false,
  busy: false,
  msg: '',
  timer: 0,
};
const listeners = new Set<() => void>();
function set(p: Partial<UeState>) {
  state = { ...state, ...p };
  listeners.forEach((l) => l());
}

export function subscribeUe(cb: () => void): () => void {
  listeners.add(cb);
  return () => listeners.delete(cb);
}
export function getUe(): UeState {
  return state;
}
export function useUeStore(): UeState {
  return useSyncExternalStore(subscribeUe, getUe, getUe);
}

/** 更新地址并持久化（localStorage）。注意合并原有字段，否则会把 codec 一起覆盖掉。 */
export function updateUeAddr(a: UeAddr) {
  try {
    const saved = localStorage.getItem(LS_KEY);
    const o = saved ? JSON.parse(saved) : {};
    localStorage.setItem(LS_KEY, JSON.stringify({ ...o, ...a }));
  } catch {
    /* ignore */
  }
  state = { ...state, addr: a };
  listeners.forEach((l) => l());
}

/** 更新像素流编码器并持久化（与地址同 LS_KEY，不覆盖地址字段） */
export function updateUeCodec(codec: string) {
  try {
    const saved = localStorage.getItem(LS_KEY);
    const o = saved ? JSON.parse(saved) : {};
    o.codec = codec;
    localStorage.setItem(LS_KEY, JSON.stringify(o));
  } catch {
    /* ignore */
  }
  state = { ...state, codec };
  listeners.forEach((l) => l());
}

/** 更新像素流帧率并持久化（与地址/编码器同 LS_KEY，不覆盖其它字段） */
export function updateUeFrameRate(fps: number) {
  const n = Math.round(Number(fps));
  if (!Number.isFinite(n) || n < 10 || n > 120) return;
  try {
    const saved = localStorage.getItem(LS_KEY);
    const o = saved ? JSON.parse(saved) : {};
    o.frameRate = n;
    localStorage.setItem(LS_KEY, JSON.stringify(o));
  } catch {
    /* ignore */
  }
  state = { ...state, frameRate: n };
  listeners.forEach((l) => l());
}

export function uePlayerUrl(): string {
  return UE_PLAYER_URL(state.addr.host, state.addr.httpPort);
}
function base(): string {
  return ueControlBase(state.addr.host, state.addr.ctlPort);
}

async function refreshRunning(): Promise<boolean> {
  try {
    const res = await fetch(`${base()}/status`);
    if (res.ok) {
      const j = await res.json().catch(() => ({}));
      set({ running: !!j.running });
      return true;
    }
  } catch {
    /* UE 控制服务未启动 */
  }
  return false;
}

/** 启动运行态轮询，返回清理函数 */
export function startStatusPolling(): () => void {
  refreshRunning();
  const t = setInterval(refreshRunning, 3000);
  return () => clearInterval(t);
}

/** 启动 / 停止仿真关卡 */
export async function uePost(path: '/launch' | '/stop') {
  set({ busy: true, msg: '' });
  try {
    const res = await fetch(`${base()}${path}`, { method: 'POST' });
    const j = await res.json().catch(() => ({}));
    if (res.ok) {
      set({
        msg: j.message || j.result || (path === '/launch' ? '已触发启动' : '已停止'),
        timer: path === '/launch' ? 15 : 0,
      });
      await refreshRunning();
    } else {
      set({ msg: `失败：${j.result || j.error || res.status}`, timer: 0 });
    }
  } catch {
    set({ msg: '无法连接 UE 控制服务，请检查地址，确认 ue_controller.py 已启动', timer: 0 });
  } finally {
    set({ busy: false });
  }
}

/** 启动仿真：把当前机队 manifest 发给 orchestrator:8070/launch，由它生成场景/载具并反调 ue_controller 拉起引擎。返回最终结果文案（供调用方展示进度）。 */
export async function orchLaunch(host: string, manifest: Record<string, unknown>): Promise<string> {
  set({ busy: true, msg: '' });
  let finalMsg = '';
  try {
    const res = await fetch(`${ueControlBase(host, UE_ORCH_PORT)}/launch`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(manifest),
    });
    const j = await res.json().catch(() => ({}));
    if (res.ok) {
      finalMsg = j.status === 'ok' ? ('已编排启动：' + (j.scene_rel || '')) : (j.result || j.error || '已触发');
      set({ msg: finalMsg, timer: 12 });
      await refreshRunning();
    } else {
      finalMsg = `编排失败：${j.error || j.result || res.status}`;
      set({ msg: finalMsg, timer: 0 });
    }
  } catch {
    finalMsg = '无法连接 orchestrator(8070)，请确认 Windows 侧 orchestrator.py 已启动';
    set({ msg: finalMsg, timer: 0 });
  } finally {
    set({ busy: false });
  }
  return finalMsg;
}

/** 停止仿真：请求 orchestrator:8070/stop，由它结束 tmux(PX4/mavros/桥接/场景)并请求 UE 停止。
 * 注意：不要用 uePost('/stop')——那只打 ue_controller:8090/stop（仅停 UE 场景），
 * 不会 stop_tmux，导致 WSL 端进程残留。 */
export async function orchStop(host: string): Promise<string> {
  set({ busy: true, msg: '' });
  let finalMsg = '';
  try {
    const res = await fetch(`${ueControlBase(host, UE_ORCH_PORT)}/stop`, { method: 'POST' });
    const j = await res.json().catch(() => ({}));
    if (res.ok) {
      finalMsg = j.result || j.message || '已编排停止';
      set({ msg: finalMsg, timer: 0 });
    } else {
      finalMsg = `编排停止失败：${j.result || j.error || res.status}`;
      set({ msg: finalMsg, timer: 0 });
    }
  } catch {
    finalMsg = '无法连接 orchestrator(8070)，请确认 Windows 侧 orchestrator.py 已启动';
    set({ msg: finalMsg, timer: 0 });
  } finally {
    set({ busy: false });
  }
  return finalMsg;
}

/** 就绪倒计时递减（每秒调用） */
export function decrementUeTimer() {
  if (state.timer <= 0) return;
  set({ timer: state.timer - 1 });
}
