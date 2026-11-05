import Sel from './Sel';
import { useState, useRef, useEffect, useCallback } from 'react';
import { useRosState } from '../ros/useRos';
import { useMsgCounts } from '../ros/useRos';
import { useBridgeCounts } from '../ros/wsBridge';
import { TOPICS, resolveTopic, TopicKind, BASEMAPS, IMAGE_TOPICS } from '../config';
import MappingControlPanel from './MappingControlPanel';
import PlyExportPanel from './PlyExportPanel';
import { useUeStore, startStatusPolling, updateUeAddr, updateUeCodec, updateUeFrameRate, uePost, decrementUeTimer, orchStop } from '../ueStore';
import { updateHostConfig, useHostConfig, getRosMasterIp } from '../hostConfig';
import { pushSystemConfig, getSystemStatus, SystemControlStatus, startAll, restartAll, restartSelkies, getClaheConfig, setClaheConfig, ClaheConfig, getRecord, setRecord, listRecordings, sendPlayback, RecordingItem, getPlaybackState } from '../systemControl';
import { rosManager } from '../ros/useRos';
import { wsBridge } from '../ros/wsBridge';
import { usePerf, updatePerf } from '../perf';

import type { PerfSettings } from '../perf';

const LABELS: Record<string, string> = {
  idle: '未连接',
  connecting: '连接中…',
  connected: '已连接',
  error: '连接失败',
};

const KIND_COLOR: Record<TopicKind, string> = {
  mesh: '#38e1c4',
  pointcloud: '#60a5fa',
  ortho: '#ffb454',
  traj: '#22d3ee',
  image: '#c084fc',
  video: '#ff7bac',
  base: '#9ca3af',
};
const KEY_COLOR: Record<string, string> = {
  traj_visual: '#19ff5a',
  traj_gnss: '#ff3b30',
};
function colorOf(t: { key: string; kind: TopicKind }): string {
  return KEY_COLOR[t.key] || KIND_COLOR[t.kind];
}

/** 性能开关项：低性能电脑可手动关闭的重负载功能 */
const PERF_ITEMS: { key: keyof PerfSettings; label: string; hint: string }[] = [
  { key: 'globe', label: '3D 地球（Cesium）', hint: '关闭后隐藏标签，不再加载 Cesium' },
  { key: 'ue', label: '仿真视窗（UE）', hint: '关闭后隐藏标签，不加载 UE 像素流' },
  { key: 'video', label: '视频面板', hint: '关闭后隐藏菜单并关闭已开面板' },
  { key: 'task', label: '任务', hint: '关闭后隐藏「任务」标签' },
  { key: 'potree', label: '测绘', hint: '关闭后隐藏「测绘」标签' },
  { key: 'raster', label: '栅格影像', hint: '关闭后隐藏「栅格影像」标签' },
  { key: 'log', label: '日志面板', hint: '关闭后隐藏左下角日志（每条日志会触发整页重渲染）' },
];

/** 通用下拉菜单。className 透传到菜单面板，便于控制宽度等。
 *  传了 open/onOpenChange 即为受控模式（由父级驱动开合，便于外部按钮触发打开）。 */
function Dropdown({ label, children, className, open: openProp, onOpenChange }: {
  label: string; children: React.ReactNode; className?: string;
  open?: boolean; onOpenChange?: (v: boolean) => void;
}) {
  const [innerOpen, setInnerOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
  const btnRef = useRef<HTMLButtonElement>(null);
  const menuRef = useRef<HTMLDivElement>(null);
  const [flip, setFlip] = useState(false);
  const open = openProp !== undefined ? openProp : innerOpen;
  const openRef = useRef(open);
  openRef.current = open;
  const setOpen = (v: boolean) => {
    // 菜单面板带 data-ue-hole：原生视频窗口会在这个矩形上挖洞，
    // 让菜单显示在视频之上（见 ueNative.ts 顶部说明）。
    if (openProp !== undefined) { onOpenChange?.(v); }
    else setInnerOpen(v);
  };

  // 菜单靠近屏幕右缘时，改为右对齐按钮右缘（向左展开），保证完整显示在窗口内
  useEffect(() => {
    if (!open) { setFlip(false); return; }
    const id = requestAnimationFrame(() => {
      const menu = menuRef.current;
      const btn = btnRef.current;
      if (!menu || !btn) return;
      const mr = menu.getBoundingClientRect();
      const br = btn.getBoundingClientRect();
      // 菜单右缘超出视口右边界（留 8px 边距）→ 翻转：菜单右缘对齐按钮右缘
      setFlip(mr.right > window.innerWidth - 8);
      void br;
    });
    return () => cancelAnimationFrame(id);
  }, [open]);

  useEffect(() => {
    const handler = (e: MouseEvent) => {
      const t = e.target as Node | null;
      if (ref.current && t && ref.current.contains(t)) return;
      // ⚠️ Sel 的选项面板是 createPortal 挂到 document.body 的，DOM 上并不在本菜单
      // 的子树里。只按 contains 判断会把它当成「点了外部」，于是**点一个下拉选项
      // 就把整个菜单关掉**（「视频 → 回放」里选个录制文件菜单就消失，就是这么来的）。
      // 落在 .sel-pop 里的点击一律不算外部：Sel 自己会在 pick() 里关掉面板。
      // 同一处修复对所有菜单里的 Sel 都生效（图层→底图、性能、统计等）。
      if (t instanceof Element && t.closest('.sel-pop')) return;
      // 只在当前处于展开状态时才触发关闭：受控模式下 onOpenChange 可能是「切换」语义，
      // 若菜单本已收起，点击任意处不应再次触发（否则会莫名重新打开）。
      if (openRef.current) setOpen(false);
    };
    document.addEventListener('mousedown', handler);
    return () => document.removeEventListener('mousedown', handler);
  }, []);

  return (
    <div className="tb-dropdown" ref={ref}>
      <button ref={btnRef} className={`tb-dd-btn ${open ? 'open' : ''}`} onClick={() => setOpen(!open)}>
        {label} <span className="tb-dd-arrow">{open ? '▴' : '▾'}</span>
      </button>
      {open && <div ref={menuRef} data-ue-hole="" className={`tb-dd-menu ${className || ''} ${flip ? 'tb-dd-flip' : ''}`}>{children}</div>}
    </div>
  );
}

/** 可配置 Jetson 主机（IP + 各服务端口），保存后立即应用到所有后端连接 */
function HostConfigDropdown({ onApply }: { onApply: () => void }) {
  const cfg = useHostConfig();
  const [host, setHost] = useState(cfg.host);
  const [rosMaster, setRosMaster] = useState(cfg.rosMasterIp || cfg.host);
  const [rp, setRp] = useState(String(cfg.rosbridgePort));
  const [rhp, setRhp] = useState(String(cfg.rosbridgeHeavyPort));
  const [wsp, setWsp] = useState(String(cfg.wsBridgePort));
  const [sp, setSp] = useState(String(cfg.saveServerPort));

  const save = () => {
    const nextHost = host.trim() || cfg.host;
    const nextMaster = rosMaster.trim() || nextHost;
    updateHostConfig({
      host: nextHost,
      rosMasterIp: nextMaster,
      rosbridgePort: parseInt(rp, 10) || cfg.rosbridgePort,
      rosbridgeHeavyPort: parseInt(rhp, 10) || cfg.rosbridgeHeavyPort,
      wsBridgePort: parseInt(wsp, 10) || cfg.wsBridgePort,
      saveServerPort: parseInt(sp, 10) || cfg.saveServerPort,
    });
    onApply();
    // 只把 ROS master 地址推送给 save_server（写 ROS_MASTER_URI），
    // 不触碰 WebClient 后端连接地址（host），避免改 master 导致前端连不上后端。
    pushSystemConfig(nextMaster);
  };

  return (
    <Dropdown label="主机配置" className="tb-dd-narrow">
      <div className="cfg-row">
        <span className="cfg-label">前端后端 IP</span>
        <input className="cfg-input cfg-ip" value={host} onChange={(e) => setHost(e.target.value)} placeholder="如 192.168.2.99" />
      </div>
      <div className="cfg-row">
        <span className="cfg-label">ROS master IP</span>
        <input className="cfg-input cfg-ip" value={rosMaster} onChange={(e) => setRosMaster(e.target.value)} placeholder="默认同上" />
      </div>
      <div className="cfg-row">
        <span className="cfg-label">ROS 端口</span>
        <input className="cfg-input cfg-port" value={rp} onChange={(e) => setRp(e.target.value)} />
      </div>
      <div className="cfg-row">
        <span className="cfg-label">ROS重 端口</span>
        <input className="cfg-input cfg-port" value={rhp} onChange={(e) => setRhp(e.target.value)} />
      </div>
      <div className="cfg-row">
        <span className="cfg-label">WS桥 端口</span>
        <input className="cfg-input cfg-port" value={wsp} onChange={(e) => setWsp(e.target.value)} />
      </div>
      <div className="cfg-row">
        <span className="cfg-label">保存 端口</span>
        <input className="cfg-input cfg-port" value={sp} onChange={(e) => setSp(e.target.value)} />
      </div>
      <div className="cfg-row" style={{ marginTop: 8 }}>
        <button className="dd-btn cfg-btn" onClick={save}>保存并重连</button>
      </div>
      <div className="cfg-status">后端 {cfg.host} · master {cfg.rosMasterIp || cfg.host}</div>
    </Dropdown>
  );
}

/**
 * 系统服务控制：展示 Jetson 各常驻服务端口状态，并提供「启动服务 / 重启服务」，
 * 由 save_server 的 /system/* 接口执行 start_all_asn.sh / start_all.sh。
 */
function SystemServiceDropdown() {
  const [status, setStatus] = useState<SystemControlStatus | null>(null);
  const [busy, setBusy] = useState('');
  const [msg, setMsg] = useState('');
  const [phase, setPhase] = useState<'idle' | 'restart'>('idle');
  /** 远程桌面重启的两步确认（这一下会切断自己正在看的画面，不能一键直发） */
  const [confirmSelkies, setConfirmSelkies] = useState(false);

  const refresh = useCallback(async () => {
    try {
      setStatus(await getSystemStatus());
    } catch {
      setStatus(null); // 后端不可达，下拉保持可点但状态未知
    }
  }, []);

  const sleep = (ms: number) => new Promise<void>((r) => setTimeout(r, ms));

  useEffect(() => {
    refresh();
    const t = setInterval(refresh, 3000); // 打开时每 3s 轮询一次
    return () => clearInterval(t);
  }, [refresh]);

  const run = async (kind: 'start' | 'restart') => {
    setBusy(kind);
    setMsg('');
    try {
      // 推送 ROS master 地址给 save_server，确保 ROS_MASTER_URI 指向正确的 roscore（默认可与后端同 IP）
      try { await pushSystemConfig(getRosMasterIp()); } catch { /* 忽略，save_server 兜底 */ }
      const r = await (kind === 'start' ? startAll() : restartAll());
      if (!r.ok) throw new Error(r.error || '操作失败');
      if (kind === 'restart') {
        setPhase('restart');
        setMsg('系统服务重启中，正在等待恢复…');
        const deadline = Date.now() + 60000;
        while (Date.now() < deadline) {
          try {
            const s = await getSystemStatus();
            const list = s?.services ?? {};
            if (Object.keys(list).length && Object.values(list).every((v) => v.running)) {
              setStatus(s);
              break;
            }
          } catch {
            /* save_server 重启期间不可达 */
          }
          await sleep(4000);
        }
        setPhase('idle');
        // 重启结束即清除临时提示；真正失败由下方 catch 分支的「失败：」提示。
        setMsg('');
      } else {
        setMsg('已触发启动，服务正在补齐…');
        setTimeout(refresh, 3000);
      }
    } catch (e: any) {
      setMsg(`失败：${e?.message ?? e}`);
    } finally {
      setBusy('');
    }
  };

  /**
   * 重启远程桌面（systemd unit selkies）。
   *
   * 为什么单独一条、还要两步确认：当前页面本身就是 :9080 推过来的流，
   * 点下去 = 切断自己的画面。只能靠 unit 的 Restart=always 拉回来，
   * 所以先要用户确认一次，再下发（后端延迟 2s 执行，先把提示显示出来）。
   */
  const runSelkies = async () => {
    if (!confirmSelkies) {
      setConfirmSelkies(true);
      setMsg('重启远程桌面会断开当前画面约 5~10 秒（systemd 会自动拉起），再点一次确认');
      window.setTimeout(() => setConfirmSelkies(false), 8000);
      return;
    }
    setConfirmSelkies(false);
    setBusy('selkies');
    setMsg('');
    try {
      const r = await restartSelkies();
      setMsg(r.note || '已下发重启；画面断开后等 5~10 秒再刷新客户端页面');
    } catch (e: any) {
      setMsg(`失败：${e?.message ?? e}`);
    } finally {
      setBusy('');
    }
  };

  const services = status?.services;
  const entries = services ? Object.entries(services) : [];
  const runningCount = entries.filter(([, v]) => v.running).length;

  return (
    <Dropdown label="系统服务" className="tb-dd-wide">
      <div className="dd-row">
        <span className="dd-name">ROS IP</span>
        <span className="dd-name" style={{ flex: 1, textAlign: 'right', fontVariantNumeric: 'tabular-nums' }}>
          {status?.rosIp ?? '—'}
        </span>
      </div>
      <div className="dd-row" style={{ fontSize: 11, color: 'var(--text-dim)' }}>
        脚本 {status?.script?.split('/').pop() ?? '未找到'} · 服务 {runningCount}/{entries.length}
      </div>

      <div style={{ margin: '8px', display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '4px 10px' }}>
        {entries.length === 0 && (
          <span style={{ fontSize: 12, color: 'var(--text-dim)', gridColumn: '1 / -1' }}>后端不可达</span>
        )}
        {entries.map(([port, svc]) => (
          <div className="dd-row" key={port} style={{ gridColumn: '1 / 2' }}>
            <span className="dd-swatch" style={{ background: phase === 'restart' ? '#eab308' : svc.running ? '#22c55e' : '#ef4444' }} />
            <span className="dd-name">{svc.name}</span>
            <span className="dd-cnt">{phase === 'restart' ? '重启中' : svc.running ? '运行中' : '停止'}</span>
          </div>
        ))}
      </div>

      <div className="archive-actions" style={{ margin: '0 8px 8px' }}>
        <button className="btn active" disabled={!!busy} onClick={() => run('start')}>
          {busy === 'start' ? '启动中…' : '▶ 启动服务'}
        </button>
        <button className="btn danger" disabled={!!busy} onClick={() => run('restart')}>
          {busy === 'restart' ? '重启中…' : '⟳ 重启服务'}
        </button>
      </div>

      {/* 远程桌面单独一行：它不在 start_all 的管辖范围内（systemd unit），
          而且重启会切断当前这条流，所以不跟上面两个按钮并排。 */}
      <div className="archive-actions" style={{ margin: '0 8px 8px' }}>
        <button
          className={`btn ${confirmSelkies ? 'danger' : ''}`}
          disabled={!!busy}
          title="重启 Selkies 远程桌面（systemd unit selkies）；会断开当前画面约 5~10 秒"
          onClick={runSelkies}
        >
          {busy === 'selkies' ? '重启中…' : confirmSelkies ? '确认重启远程桌面？' : '⟳ 远程桌面'}
        </button>
      </div>

      {msg && <div className="ue-msg-in" style={{ margin: '0 8px 8px' }}>{msg}</div>}
    </Dropdown>
  );
}

export default function TopBar({
  activeView,
  receive, setReceive, display, setDisplay,
  stats, trajPtCounts,
  basemapKey, onBasemapChange,
  showBase, setShowBase,
  showOverlay, setShowOverlay,
  rasterOverlayLabel,
  followGnss, setFollowGnss,
  videoActiveKeys, onToggleVideo, onVideoPreset,
  plyOpen, onTogglePly,
  fileMode, onViewPly, onViewRaster,
  controlMode, onControlModeChange,
}: {
  activeView: '2d' | '3d' | 'task' | 'potree' | 'raster' | 'ue';
  receive: Record<string, boolean>;
  setReceive: (k: string, v: boolean) => void;
  display: Record<string, boolean>;
  setDisplay: (k: string, v: boolean) => void;
  stats: any;
  trajPtCounts?: Record<string, number>;
  basemapKey: string;
  onBasemapChange: (key: string) => void;
  showBase: boolean;
  setShowBase: (v: boolean) => void;
  showOverlay: boolean;
  setShowOverlay: (v: boolean) => void;
  rasterOverlayLabel: string | null;
  followGnss: boolean;
  setFollowGnss: (v: boolean) => void;
  videoActiveKeys: string[];
  onToggleVideo: (key: string) => void;
  onVideoPreset: (keys: string[]) => void;
  plyOpen: boolean;
  onTogglePly: () => void;
  fileMode?: 'potree' | 'raster' | 'other';
  onViewPly: (url: string) => void;
  onViewRaster: (p: { srv: string; name: string; rel: string; label: string; ext: string; size?: number; mtime?: number; type?: string }) => void;
  controlMode: 'mapping' | 'free';
  onControlModeChange: (m: 'mapping' | 'free') => void;
}) {
  const conn = useRosState();
  const counts = useMsgCounts();
  const bridgeCounts = useBridgeCounts();
  const perf = usePerf();

  // ---- CLAHE 图像增强配置（「视频」菜单下发 save_server /system/clahe，receiver 秒级生效）----
  const [clahe, setClahe] = useState<ClaheConfig>({ enhance: 0, clahe_clip: 2.0, clahe_grid: 8, clahe_space: 'ycrcb' });
  const claheDebRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const applyClahe = useCallback((next: ClaheConfig) => {
    setClahe(next);
    if (claheDebRef.current) clearTimeout(claheDebRef.current);
    claheDebRef.current = setTimeout(() => {
      setClaheConfig({ enhance: next.enhance, clahe_clip: next.clahe_clip, clahe_grid: next.clahe_grid, clahe_space: next.clahe_space })
        .catch(() => { /* 下发失败静默，保留本地态 */ });
    }, 300);
  }, []);
  // 挂载时读取后端当前 CLAHE 配置
  useEffect(() => {
    let off = false;
    getClaheConfig()
      .then((cfg) => {
        if (off) return;
        setClahe({
          enhance: cfg.enhance || 0,
          clahe_clip: typeof cfg.clahe_clip === 'number' ? cfg.clahe_clip : 2.0,
          clahe_grid: ([4, 8, 16] as const).includes(cfg.clahe_grid as 4 | 8 | 16) ? (cfg.clahe_grid as 4 | 8 | 16) : 8,
          clahe_space: cfg.clahe_space === 'lab' ? 'lab' : 'ycrcb',
        });
      })
      .catch(() => { /* 后端不可达时沿用默认 */ });
    return () => { off = true; };
  }, []);
  useEffect(() => () => { if (claheDebRef.current) clearTimeout(claheDebRef.current); }, []);

  // ---- H264 裸流 + GPS 录制控制（「视频」菜单下发 save_server /system/record，receiver 秒级生效）----
  const [recording, setRecording] = useState<0 | 1>(0);
  const [recordBusy, setRecordBusy] = useState(false);
  // 挂载时读取后端当前录制状态
  useEffect(() => {
    let off = false;
    getRecord()
      .then((r) => { if (!off) setRecording(r.recording === 1 ? 1 : 0); })
      .catch(() => { /* 后端不可达时沿用默认 */ });
    return () => { off = true; };
  }, []);
  const toggleRecord = async () => {
    setRecordBusy(true);
    try {
      const r = await setRecord(recording ? 'stop' : 'start');
      setRecording(r.recording === 1 ? 1 : 0);
    } catch (e: any) {
      window.alert(`录制操作失败：${e?.message ?? e}`);
    } finally {
      setRecordBusy(false);
    }
  };

  // ---- 录制回放控制（「视频」菜单；下发 save_server /system/playback，由 playback_replayer.py 生效）----
  const [replayFiles, setReplayFiles] = useState<RecordingItem[]>([]);
  const [replayFile, setReplayFile] = useState('');
  const [replayState, setReplayState] = useState<'idle' | 'playing' | 'paused' | 'finished'>('idle');
  const [replaySpeed, setReplaySpeed] = useState(1);
  const [replayBusy, setReplayBusy] = useState(false);
  const [replayMsg, setReplayMsg] = useState('');
  const [replayProgress, setReplayProgress] = useState(0);
  // 刚拖过进度条的时刻: 在此后 ~2.5s 内不用轮询值覆盖本地值,
  // 避免回放器尚未处理完 seek 时进度条被旧值弹回。
  const lastSeekAtRef = useRef(0);
  // 打开「视频」菜单时才拉取文件列表，避免每次渲染都请求
  const refreshReplayFiles = useCallback(async () => {
    try {
      const items = await listRecordings();
      setReplayFiles(items);
      setReplayFile((cur) => cur || (items[0]?.name ?? ''));
    } catch (e: any) {
      setReplayMsg(`列表获取失败：${e?.message ?? e}`);
    }
  }, []);

  /** 下发一个回放动作；失败弹提示。 */
  const replayAction = useCallback(async (
    action: 'load' | 'play' | 'pause' | 'stop' | 'seek' | 'speed',
    extra?: { t?: number; progress?: number; speed?: number; file?: string },
  ) => {
    setReplayBusy(true);
    try {
      const cfg = { action, file: replayFile, ...extra } as Parameters<typeof sendPlayback>[0];
      if (action === 'load' || action === 'play') {
        if (!cfg.file) { window.alert('请先选择回放文件'); setReplayBusy(false); return; }
      }
      await sendPlayback(cfg);
      if (action === 'load') { setReplayState('paused'); setReplayMsg('已载入，点「播放」开始'); }
      else if (action === 'play') { setReplayState('playing'); setReplayMsg(''); }
      else if (action === 'pause') setReplayState('paused');
      else if (action === 'stop') { setReplayState('idle'); setReplayMsg(''); }
      else if (action === 'seek') setReplayMsg('已跳转');
    } catch (e: any) {
      window.alert(`回放操作失败：${e?.message ?? e}`);
    } finally {
      setReplayBusy(false);
    }
  }, [replayFile]);

  /** 拖动进度条：按比例 seek。 */
  const replaySeek = useCallback((progress: number) => {
    lastSeekAtRef.current = Date.now();
    setReplayProgress(progress);
    replayAction('seek', { progress });
  }, [replayAction]);

  /** 从后端读回实时回放状态（进程 + 回放器状态文件）。
   *  页面刷新后或正在播放时周期调用，保持 UI 与实际一致。 */
  const syncReplayState = useCallback(async () => {
    try {
      const st = await getPlaybackState();
      const stt = st.state;
      if (stt === 'playing' || stt === 'paused' || stt === 'finished') {
        setReplayState(stt);
      } else if (!st.running) {
        // 回放器没在跑且无活跃状态 ⇒ 回到 idle
        setReplayState((cur) => (cur === 'playing' || cur === 'paused' ? 'idle' : cur));
      }
      if (st.file) setReplayFile((cur) => cur || st.file);
      if (st.speed) setReplaySpeed(st.speed);
      if (typeof st.progress === 'number' && stt === 'playing'
          && Date.now() - lastSeekAtRef.current > 2500) setReplayProgress(st.progress);
      if (st.error) setReplayMsg(`回放器报错：${st.error}`);
    } catch {
      /* 后端未就绪时静默忽略 */
    }
  }, []);

  // 挂载时先同步一次（刷新后恢复状态）
  useEffect(() => { void syncReplayState(); }, [syncReplayState]);

  // 正在播放时每 2s 同步进度（关闭菜单也会同步；暂停/结束后停止轮询）
  useEffect(() => {
    if (replayState !== 'playing') return;
    const id = window.setInterval(() => { void syncReplayState(); }, 2000);
    return () => window.clearInterval(id);
  }, [replayState, syncReplayState]);

  // ---- 仿真视窗共享控制（顶栏「配置 / 仿真控制 / 建图控制」菜单 + UeView iframe 共用） ----
  const ue = useUeStore();
  const [draftHost, setDraftHost] = useState(ue.addr.host);
  const [draftHttp, setDraftHttp] = useState(String(ue.addr.httpPort));
  const [draftCtl, setDraftCtl] = useState(String(ue.addr.ctlPort));

  // 按当前标签区分顶栏菜单，避免误操作：
  //   - 自由模式 / UE 视窗：不显示建图数据/保存菜，界面由自由工作区 / UeView 自带工具栏负责
  //   - 测绘/栅格：仅保留成果文件相关的「Jetson 点云」（打开/查看成果）
  //   - 2D/3D：完整的数据菜单（图层/地图/视频/统计/保存/成果）
  const onMapTab = controlMode === 'mapping' && (activeView === '2d' || activeView === '3d');
  const onPanelTab = controlMode === 'mapping' && (activeView === 'potree' || activeView === 'raster');
  const onUeTab = controlMode === 'mapping' && activeView === 'ue';

  // 仿真视窗标签：轮询 UE 运行态
  useEffect(() => {
    if (!onUeTab) return;
    return startStatusPolling();
  }, [onUeTab]);

  // 启动就绪倒计时
  useEffect(() => {
    if (ue.timer <= 0) return;
    const t = setInterval(decrementUeTimer, 1000);
    return () => clearInterval(t);
  }, [ue.timer]);

  const saveUeAddr = () => {
    updateUeAddr({
      host: draftHost.trim() || ue.addr.host,
      httpPort: parseInt(draftHttp, 10) || ue.addr.httpPort,
      ctlPort: parseInt(draftCtl, 10) || ue.addr.ctlPort,
    });
  };

  /** 主机配置保存后：用新地址重连所有后端连接 */
  const handleHostApply = () => {
    rosManager.disconnect(true);      // 手动断开（不自动重连到旧地址）
    rosManager.connect();             // 内部按最新 hostConfig 计算地址
    wsBridge.disconnect();
    wsBridge.connect();
  };

  return (
    <div className="topbar">
      <span className="logo">◈ GeoFusion 3D</span>
      <span className={`dot ${conn}`} />
      <span style={{ fontSize: 12, color: 'var(--text-dim)' }}>{LABELS[conn]}</span>

      {/* ---- 左对齐主菜单 ----
        建图模式：配置 / 仿真控制 / 建图控制 / 自由模式
        自由模式：返回建图控制 */}
      {controlMode === 'mapping' ? (
        <div className="tb-menu-group">
          {onUeTab && (
            <Dropdown label="仿真配置">
              <div className="dd-row">
                <span className="dd-name">UE 地址</span>
                <span className="dd-name" style={{ flex: 1, textAlign: 'right', fontVariantNumeric: 'tabular-nums' }}>
                  {ue.addr.host}:{ue.addr.httpPort}
                </span>
              </div>
              <div className="ue-edit-row" style={{ margin: '8px 8px 0' }}>
                <label className="ue-edit-field"><span>IP</span><input value={draftHost} onChange={(e) => setDraftHost(e.target.value)} /></label>
                <label className="ue-edit-field"><span>HTTP</span><input value={draftHttp} onChange={(e) => setDraftHttp(e.target.value)} /></label>
                <label className="ue-edit-field"><span>控制</span><input value={draftCtl} onChange={(e) => setDraftCtl(e.target.value)} /></label>
              </div>
              <div className="dd-row">
                <button className="dd-btn" style={{ width: '100%', textAlign: 'center' }} onClick={saveUeAddr}>保存地址</button>
              </div>
              <div className="dd-row">
                <span className="dd-name" title="UE PixelStreaming 视频编码器，启动仿真时随 manifest 下发。H264：RTX NVENC 硬编 + Jetson NVDEC 硬解，唯一支持硬件全链路的选项；VP8：Jetson 无硬件解码器，仅 PC 浏览器 iframe 软解时可用。切换后需重新启动仿真生效。">像素流编码器</span>
                <div className="dd-seg">
                  <button
                    className={`dd-btn ${ue.codec === 'H264' ? 'active' : ''}`}
                    onClick={() => updateUeCodec('H264')}
                  >H264</button>
                  <button
                    className={`dd-btn ${ue.codec === 'VP8' ? 'active' : ''}`}
                    onClick={() => updateUeCodec('VP8')}
                  >VP8</button>
                </div>
              </div>
              <div className="dd-row">
                <span className="dd-name" title="UE PixelStreaming 视频帧率，启动仿真时以 -PixelStreamingFrameRate 下发给引擎。Jetson 端 rtpjitterbuffer 的开销与 RTP 包率成正比，帧率减半约可省一半包处理量。60 最流畅但 CPU 最高；30 为推荐折中；20/24 更省资源，但遥操作手感会偏顿。切换后需重新启动仿真生效。">像素流帧率</span>
                <div className="dd-seg">
                  {[60, 30, 24, 20].map((fps) => (
                    <button
                      key={fps}
                      className={`dd-btn ${ue.frameRate === fps ? 'active' : ''}`}
                      onClick={() => updateUeFrameRate(fps)}
                    >{fps}</button>
                  ))}
                </div>
              </div>
            </Dropdown>
          )}

          {onUeTab && (
            <Dropdown label="仿真控制" className="tb-dd-wide">
              <div className="dd-row">
                <span className={`ue-run ${ue.running ? 'der' : ''}`}>{ue.running ? '● 运行中' : '○ 未运行'}</span>
                <span style={{ flex: 1 }} />
              </div>
              <div className="archive-actions" style={{ margin: '8px 8px 0' }}>
                <button className="btn active" disabled={ue.busy} onClick={() => uePost('/launch')}>
                  {ue.busy ? '启动中…' : '▶ 启动仿真'}
                </button>
                <button className="btn danger" disabled={ue.busy} onClick={() => orchStop(ue.addr.host)}>停止</button>
              </div>
              {ue.msg && (
                <div className="ue-msg-in" style={{ margin: '8px' }}>
                  {ue.msg}{ue.timer > 0 && `（约 ${ue.timer}s 后画面就绪）`}
                </div>
              )}
            </Dropdown>
          )}

          <Dropdown label="建图控制" className="tb-dd-wide tb-dd-map">
            <MappingControlPanel />
          </Dropdown>

          <SystemServiceDropdown />

          <Dropdown label="性能" className="tb-dd-wide">
            <div className="dd-row" style={{ fontSize: 11, color: 'var(--text-dim)' }}>
              按电脑性能手动开关重负载功能（自动保存）
            </div>
            {PERF_ITEMS.map((it) => (
              <div className="dd-row" key={it.key}>
                <span className="dd-name" title={it.hint}>{it.label}</span>
                <label className="tg">
                  <input
                    type="checkbox"
                    checked={perf[it.key]}
                    onChange={(e) => updatePerf({ [it.key]: e.target.checked } as Partial<PerfSettings>)}
                  />
                  <span className="tg-label">{perf[it.key] ? '开' : '关'}</span>
                </label>
              </div>
            ))}
          </Dropdown>

          <button
            className="tb-menu-btn"
            title="切换到自由模式：手动/自动控制云台 + 目标识别"
            onClick={() => onControlModeChange('free')}
          >
            自由模式
          </button>
        </div>
      ) : (
        <div className="tb-menu-group">
          <button
            className="tb-menu-btn tb-menu-btn-primary"
            title="返回建图模式：云台正下视，启动/停止 Jetson 端建图程序（2run.sh）"
            onClick={() => onControlModeChange('mapping')}
          >
            返回建图控制
          </button>
        </div>
      )}

      {/* ---- 2D/3D 地图类标签：数据/图层/视频/统计菜单 ---- */}
      {onMapTab && (
        <>
          <Dropdown label="图层">
            {TOPICS.map((t) => (
              <div className="dd-row" key={t.key}>
                <span className="dd-swatch" style={{ background: colorOf(t) }} />
                <span className="dd-name" title={t.topic}>{t.label}</span>
                <span className="dd-cnt">
                  {t.kind === 'traj'
                    ? (trajPtCounts?.[t.key] ? `${trajPtCounts[t.key].toLocaleString()} 点` : '')
                    : (() => {
                        const resolved = resolveTopic(t.topic);
                        const n = (t.kind === 'ortho' ? bridgeCounts[resolved] : 0) || counts[resolved] || 0;
                        return n ? `${n} 帧` : '';
                      })()}
                </span>
                <span className="dd-toggles">
                  <label className="tg" title="接收数据">
                    <input type="checkbox" checked={!!receive[t.key]} onChange={(e) => setReceive(t.key, e.target.checked)} />
                    <span className="tg-label">收</span>
                  </label>
                  <label className="tg" title="显示/隐藏">
                    <input type="checkbox" checked={!!display[t.key]} onChange={(e) => setDisplay(t.key, e.target.checked)} />
                    <span className="tg-label">显</span>
                  </label>
                </span>
              </div>
            ))}
          </Dropdown>

          <Dropdown label="地图">
            <div className="dd-row">
              <span className="dd-name">底图</span>
              <Sel className="dd-select" value={basemapKey} onChange={(e) => onBasemapChange(e.target.value)}>
                {Object.entries(BASEMAPS).map(([k, bm]) => (
                  <option key={k} value={k}>{bm.label}</option>
                ))}
              </Sel>
            </div>
            <div className="dd-row">
              <span className="dd-name">底图显示</span>
              <label className="tg">
                <input type="checkbox" checked={showBase} onChange={(e) => setShowBase(e.target.checked)} />
                <span className="tg-label">{showBase ? '开' : '关'}</span>
              </label>
            </div>
            <div className="dd-row">
              <span className="dd-name" title={rasterOverlayLabel || undefined}>
                栅格叠加{rasterOverlayLabel ? `（${rasterOverlayLabel}）` : ''}
              </span>
              <label className="tg">
                <input
                  type="checkbox" checked={showOverlay} disabled={!rasterOverlayLabel}
                  onChange={(e) => setShowOverlay(e.target.checked)}
                />
                <span className="tg-label">{showOverlay ? '开' : '关'}</span>
              </label>
            </div>
            <div className="dd-row">
              <span className="dd-name">GPS 跟随</span>
              <label className="tg">
                <input type="checkbox" checked={followGnss} onChange={(e) => setFollowGnss(e.target.checked)} />
                <span className="tg-label">{followGnss ? '开' : '关'}</span>
              </label>
            </div>
          </Dropdown>

          {perf.video && (
          <Dropdown label="视频" className="tb-dd-wide">
            {/* 多路视频勾选面板：勾选即开（订阅+停靠栏显示），取消即关；防重复由 App 层保证 */}
            <div className="dd-row" style={{ fontSize: 11, color: 'var(--text-dim)' }}>
              勾选开启视频路（当前 {videoActiveKeys.length} 路，上限 6）
            </div>
            {IMAGE_TOPICS.map((t) => (
              <div className="dd-row" key={t.key}>
                <span className="dd-name" title={t.topic}>{t.label}</span>
                <label className="tg">
                  <input
                    type="checkbox"
                    checked={videoActiveKeys.includes(t.key)}
                    onChange={() => onToggleVideo(t.key)}
                  />
                  <span className="tg-label">{videoActiveKeys.includes(t.key) ? '开' : '关'}</span>
                </label>
              </div>
            ))}
            <div className="dd-row" style={{ borderTop: '1px solid rgba(255,255,255,.08)', marginTop: 6, paddingTop: 6, gap: 6 }}>
              <button className="dd-btn" onClick={() => onVideoPreset(['camera', 'rectified', 'dens_depth'])} title="建图 + 矫正 + 深度">建图链路</button>
              <button className="dd-btn" onClick={() => onVideoPreset(['h264_enh', 'h264_orig'])} title="增强 + 原始">H.264 对比</button>
              <button className="dd-btn" onClick={() => onVideoPreset([])}>全部关闭</button>
            </div>
            <div className="dd-row" style={{ borderTop: '1px solid rgba(255,255,255,.08)', marginTop: 6, paddingTop: 6, gap: 6 }}>
              <span className="dd-swatch" style={{ background: recording ? '#ef4444' : '#4b5563', boxShadow: recording ? '0 0 6px rgba(239,68,68,.8)' : 'none' }} />
              <span className="dd-name" style={{ flex: 'none', fontSize: 12, whiteSpace: 'nowrap' }}>{recording ? '录制中' : '未录制'}</span>
              <button
                className="dd-btn"
                style={{ marginLeft: 'auto' }}
                disabled={recordBusy}
                onClick={toggleRecord}
                title="保存 H264 裸流 + GPS 数据（.h264/.ts/.gps，回放按时间戳同步）"
              >
                {recordBusy ? '…' : (recording ? '■ 停止录制' : '● 开始录制')}
              </button>
            </div>
            {/* ---- 回放：选择录制文件 + 播放/暂停/停止/进度/倍速 ---- */}
            <div className="dd-row" style={{ borderTop: '1px solid rgba(255,255,255,.08)', marginTop: 6, paddingTop: 6, gap: 6, flexWrap: 'wrap' }}>
              <span className="dd-name" style={{ flex: 'none', fontSize: 12, whiteSpace: 'nowrap' }}>回放</span>
              {replayState === 'playing' && (
                <span
                  title="正在回放（构图画面来自录制文件）"
                  style={{ flex: 'none', fontSize: 11, whiteSpace: 'nowrap',
                           color: '#facc15', fontWeight: 600 }}
                >
                  ● 回放中
                </span>
              )}
              <Sel
                value={replayFile}
                onFocus={refreshReplayFiles}
                onChange={(e) => setReplayFile(e.target.value)}
                title="选择要回放的录制（.h264/.ts/.gps）"
                style={{ flex: 1, minWidth: 150, background: 'rgba(11,19,34,.7)', border: '1px solid rgba(255,255,255,.12)', color: '#dfe8ff', borderRadius: 4, padding: '2px 4px', fontSize: 11 }}
              >
                {replayFiles.length === 0 && <option value="">（点此加载列表）</option>}
                {replayFiles.map((f) => (
                  <option key={f.name} value={f.name}>
                    {f.name} · {f.duration_s}s · {f.size_mb}MB
                  </option>
                ))}
              </Sel>
            </div>
            <div className="dd-row" style={{ gap: 6, marginTop: 4 }}>
              <button
                className="dd-btn"
                disabled={replayBusy || !replayFile}
                onClick={() => replayAction(replayState === 'playing' ? 'pause' : 'play')}
                title="开始/暂停回放（数据经组播发回 receiver）"
              >
                {replayState === 'playing' ? '⏸ 暂停' : '▶ 播放'}
              </button>
              <button className="dd-btn" disabled={replayBusy || !replayFile} onClick={() => replayAction('stop')} title="停止并回到开头">■ 停止</button>
              <Sel
                value={replaySpeed}
                onChange={(e) => { const s = Number(e.target.value); setReplaySpeed(s); replayAction('speed', { speed: s }); }}
                title="倍速"
                style={{ background: 'rgba(11,19,34,.7)', border: '1px solid rgba(255,255,255,.12)', color: '#dfe8ff', borderRadius: 4, padding: '1px 4px', fontSize: 11 }}
              >
                {[0.5, 1, 2, 4].map((s) => <option key={s} value={s}>{s}×</option>)}
              </Sel>
            </div>
            {replayFile && (
              <div className="dd-row" style={{ gap: 6, marginTop: 4, alignItems: 'center' }}>
                <span className="dd-name" style={{ flex: 'none', fontSize: 11, color: 'var(--text-dim)', whiteSpace: 'nowrap' }}>
                  {replayState === 'playing' ? '播放中' : replayState === 'paused' ? '已暂停' : replayState === 'finished' ? '已结束' : '未开始'}
                </span>
                <input
                  type="range" min={0} max={100} step={0.1} value={replayProgress * 100}
                  onChange={(e) => setReplayProgress(Number((e.target as HTMLInputElement).value) / 100)}
                  onMouseUp={(e) => replaySeek(Number((e.target as HTMLInputElement).value) / 100)}
                  onTouchEnd={(e) => replaySeek(Number((e.target as HTMLInputElement).value) / 100)}
                  title="拖动后松手跳转（比例）"
                  style={{ flex: 1, minWidth: 120 }}
                />
              </div>
            )}
            {replayMsg && <div className="dd-row" style={{ fontSize: 11, color: 'var(--text-dim)' }}>{replayMsg}</div>}
            <div className="dd-clahe" style={{ borderTop: '1px solid rgba(255,255,255,.08)', marginTop: 6, paddingTop: 6, width: 'fit-content' }}>
              <div style={{ display: 'flex', alignItems: 'center', gap: 4, marginBottom: 4 }}>
                <label style={{ display: 'inline-flex', alignItems: 'center', gap: 4, cursor: 'pointer', fontSize: 12, color: '#dfe8ff' }}>
                  <input type="checkbox" checked={!!clahe.enhance} onChange={(e) => applyClahe({ ...clahe, enhance: e.target.checked ? 1 : 0 })} />
                  图像增强
                </label>
              </div>
              <div style={{ display: 'flex', gap: 10, alignItems: 'center', flexWrap: 'nowrap' }}>
                <label style={{ display: 'inline-flex', alignItems: 'center', gap: 3, fontSize: 11, color: '#9fb0d0', whiteSpace: 'nowrap' }}>
                  对比度
                  <input type="number" min={0.5} max={5} step={0.1} value={clahe.clahe_clip}
                    onChange={(e) => applyClahe({ ...clahe, clahe_clip: Math.max(0.5, Number(e.target.value) || 0.5) })}
                    style={{ width: 48, background: 'rgba(11,19,34,.7)', border: '1px solid rgba(255,255,255,.12)', color: '#dfe8ff', borderRadius: 4, padding: '1px 4px' }} />
                </label>
                <label style={{ display: 'inline-flex', alignItems: 'center', gap: 3, fontSize: 11, color: '#9fb0d0', whiteSpace: 'nowrap' }}>
                  网格
                  <Sel value={clahe.clahe_grid} onChange={(e) => applyClahe({ ...clahe, clahe_grid: Number(e.target.value) as 4 | 8 | 16 })}
                    style={{ width: 48, background: 'rgba(11,19,34,.7)', border: '1px solid rgba(255,255,255,.12)', color: '#dfe8ff', borderRadius: 4, padding: '1px 4px' }}>
                    <option value={4}>4</option>
                    <option value={8}>8</option>
                    <option value={16}>16</option>
                  </Sel>
                </label>
              </div>
              <div style={{ display: 'flex', alignItems: 'center', marginTop: 6 }}>
                <label style={{ display: 'inline-flex', alignItems: 'center', gap: 3, fontSize: 11, color: '#9fb0d0', whiteSpace: 'nowrap' }}>
                  色彩空间
                  <Sel value={clahe.clahe_space} onChange={(e) => applyClahe({ ...clahe, clahe_space: e.target.value as 'ycrcb' | 'lab' })}
                    style={{ width: 80, background: 'rgba(11,19,34,.7)', border: '1px solid rgba(255,255,255,.12)', color: '#dfe8ff', borderRadius: 4, padding: '1px 4px' }}>
                    <option value="ycrcb">YCrCb</option>
                    <option value="lab">Lab</option>
                  </Sel>
                </label>
              </div>
            </div>
          </Dropdown>
          )}

          <Dropdown label="统计">
            <div className="dd-stat"><span className="k">关键帧</span><span className="v">{stats.keyframes ?? '—'}</span></div>
            <div className="dd-stat"><span className="k">覆盖率</span><span className="v">{(stats.coverage ?? 0).toFixed(1)} %</span></div>
            <div className="dd-stat"><span className="k">分辨率</span><span className="v">{stats.resolution ?? '—'} m/px</span></div>
            <div className="dd-stat"><span className="k">点数</span><span className="v">{(stats.points ?? 0).toLocaleString()}</span></div>
          </Dropdown>
        </>
      )}

      {/* ---- 成果文件 / 主机配置：右对齐 ---- */}
      <div className="spacer" />

      {(onMapTab || onPanelTab) && (
        <Dropdown
          label="成果文件"
          className="tb-dd-wide"
          open={plyOpen}
          onOpenChange={() => onTogglePly()}
        >
          <PlyExportPanel
            onClose={() => onTogglePly()}
            onViewPly={onViewPly}
            onViewRaster={onViewRaster}
            mode={fileMode}
          />
        </Dropdown>
      )}

      <HostConfigDropdown onApply={handleHostApply} />
    </div>
  );
}
