// ============================================================================
// SimWorkspace —— 「仿真视窗」标签页
//   - 弹出式编排页(SceneComposerDialog)创建/编辑场景（任意组合 + 任意数量）
//   - 侧栏新增「已建场景」列表：点击场景展开显示其内载具（分组数量 + uav 明细），
//     可载入编辑器继续编辑、删除、点载具预览 3D
//   - 右侧为 UE Pixel Streaming 视窗
//   - 传感器按机型结构化配置(sensorConfigs)，随 manifest 下发给 compose_scene
// ============================================================================
import { useEffect, useState } from 'react';
import { announcePopover, onOtherPopoverOpen } from '../popover';
import UeView from './UeView';
import VehicleModelViewer from './VehicleModelViewer';
import SceneComposerDialog from './SceneComposerDialog';
import ScenePreviewDialog from './ScenePreviewDialog';
import VehicleCatalogDialog from './VehicleCatalogDialog';
import { useUeStore, updateUeAddr, orchLaunch, orchStop } from '../ueStore';
import { bus, EVENTS } from '../store';
import { getRosMasterIp } from '../hostConfig';
import { UE_MAPS, VEHICLES, expandFleet, totalVehicles, fleetAlias, defaultSceneName, findVehicle, DEFAULT_FORMATION, resolveFormation, fleetSensorConfigs, buildSensorConfig, stripTemplatePoses, normalizeOrder } from '../sim/catalog';
import {
  loadScenes, persistScenes, upsertScene, removeScene, renameScene, uid,
} from '../sim/sceneStore';
import type { VehicleDef, SimConfigSelection } from '../sim/catalog';
import type { SavedScene } from '../sim/sceneStore';

const LS_KEY = 'sim-config';

// 左侧三块面板（载具列表 / 场景组合 / 已建场景）现在是**一个开关、一次全开**，
// 不再是「开哪一块」的枚举 —— 见组件里 panels 状态的说明。

/* 与 UeHud 的三个图标同一套规格：24 网格 / 24px 1:1 / 笔画 2 / 横竖线取整数坐标。
   16 网格 + 1.5 笔画在 15~17px 下渲染，缩放比不是 1，边缘会糊成一圈灰。
   2×2 方格表示「资源/更多」：4 与 13 起、边长 7 ⇒ 中间留 2px，笔画正好落在
   整数像素上，不会糊。 */
/* 载具列表图标：24 网格描边图，渲染到 28px —— 与 .ue-hud-btn（铺满/设置/信息）
   的图标同尺寸。24 放大到 28 会让笔画变 2.33px 发虚，所以带
   vector-effect="non-scaling-stroke"：笔画恒为 2 个 CSS 像素，只有几何被放大。 */
const ICON_MENU = (
  <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor"
    strokeWidth="2" strokeLinejoin="round" vectorEffect="non-scaling-stroke">
    <rect x="4" y="4" width="7" height="7" rx="1.5" />
    <rect x="13" y="4" width="7" height="7" rx="1.5" />
    <rect x="4" y="13" width="7" height="7" rx="1.5" />
    <rect x="13" y="13" width="7" height="7" rx="1.5" />
  </svg>
);

/** 旧版自动名字以 combo- 开头（曾导致名字过长），识别后按 counts 重算为短名；手动命名则保留 */
const isAutoLongName = (n: string) => !!n && n.startsWith('combo-');
const sceneDisplayName = (name: string, counts: Record<string, number>) =>
  isAutoLongName(name) ? defaultSceneName(counts) : name;

/** 把"已保存的传感器配置"与"当前机型 catalog"重新合并。
 *
 * 为什么必须合并：localStorage 里存的是历史 sensorConfigs。如果原样复用，机型预设
 * 之后更新（例如把 X500 的 MAVLink 云台从 Chase 改到 FrontCamera）就永远不生效，
 * 表现为"前端明明勾了 MAVLink 云台，生成的 robot 里相机却没有 gimbal-id"。
 *
 * 合并规则（以机型 catalog 的 sensorTemplate 为骨架，key = DroneN/CarN）：
 *   - 同名 id 的传感器：沿用存档里的实例字段（origin/captures/enabled/label）
 *   - gimbal：**catalog 的新默认为准**；仅当存档里显式改过 mavlink 布尔值才尊重之
 *   - catalog 新增的传感器补进来；存档里 catalog 已不存在的丢弃
 */
function mergeSensorConfigs(
  saved: Record<string, any[]> | undefined,
  fleet: string[],
): Record<string, any[]> {
  if (!saved) return fleetSensorConfigs(fleet);
  const out: Record<string, any[]> = {};
  fleet.forEach((k, i) => {
    const alias = fleetAlias(i, fleet);
    const tmpl = findVehicle(k)?.sensorTemplate ?? [];
    const savedList: any[] = Array.isArray(saved[alias]) ? saved[alias] : [];
    const savedById = new Map<string, any>();
    for (const s of savedList) if (s && typeof s.id === 'string') savedById.set(s.id, s);

    out[alias] = tmpl.map((t) => {
      const base: any = buildSensorConfig([t])[0];
      const prev = savedById.get(t.id);
      if (!prev) return base;

      const merged: any = { ...base, ...prev, id: t.id, kind: t.kind, label: t.label ?? base.label };
      const defGim = (t as any).gimbal;
      const prevGim = prev.gimbal;
      if (!defGim) {
        merged.gimbal = prevGim ?? null;      // 该机型本就不配云台：保留用户可能的手动开启
      } else {
        const defMav = defGim.mavlink ?? false;
        const userChangedMavlink =
          !!prevGim && typeof prevGim.mavlink === 'boolean' && prevGim.mavlink !== defMav;
        merged.gimbal = {
          ...defGim,
          ...(prevGim ?? {}),
          // 用户没显式改过 mavlink 时，用 catalog 的新默认覆盖旧存档
          ...(userChangedMavlink ? {} : { mavlink: defMav }),
        };
      }
      return merged;
    });
  });
  return out;
}

function readSaved(): SimConfigSelection {
  try {
    const o = JSON.parse(localStorage.getItem(LS_KEY) || '{}');
    const counts = o.counts && Object.keys(o.counts).length ? o.counts : { x500: 1 };
    const fleet = expandFleet(counts, o.order);
    return {
      mapKey: o.mapKey ?? 'blocks',
      sceneKey: o.sceneKey ?? 'custom',
      sceneName: o.sceneName ?? '',
      counts,
      formation: o.formation ?? DEFAULT_FORMATION,
      sensorConfigs: mergeSensorConfigs(o.sensorConfigs, fleet),
      order: o.order,
    };
  } catch {
    const counts = { x500: 1 };
    return { mapKey: 'blocks', sceneKey: 'custom', sceneName: '', counts, formation: DEFAULT_FORMATION, sensorConfigs: fleetSensorConfigs(expandFleet(counts)) };
  }
}

export default function SimWorkspace() {
  const ue = useUeStore();
  const [sel, setSel] = useState<SimConfigSelection>(readSaved);
  const [scenes, setScenes] = useState<SavedScene[]>(loadScenes);
  const [composer, setComposer] = useState(false);
  const [catalog, setCatalog] = useState(false);
  const [expandedId, setExpandedId] = useState<string | null>(null);
  const [preview, setPreview] = useState<VehicleDef | null>(null);
  const [previewScene, setPreviewScene] = useState<SavedScene | null>(null);
  const [renameId, setRenameId] = useState<string | null>(null);
  const [nameDraft, setNameDraft] = useState('');
  const [editingSceneId, setEditingSceneId] = useState<string | null>(null);
  const [applyMsg, setApplyMsg] = useState('');
  // 已建场景列表整体展开/收起（场景多时防止它占满侧栏、把下方卡片挤出可视区）
  const [scenesOpen, setScenesOpen] = useState(true);
  // 删除场景的二次确认：点「删除」只挂起待删场景，真正落盘要再点确认，避免误删。
  const [confirmDelete, setConfirmDelete] = useState<SavedScene | null>(null);
  // 左侧三块面板收在一个图标后面：点一下**同时**展开 载具列表 / 场景组合 /
  // 已建场景，三块都是完整卡片。
  // 以前是「图标 → 三项菜单 → 某一块面板」两级操作，用户明确嫌繁琐；现在一级。
  // 图标和浮层都带 data-ue-hole —— 它们落在视频区域内，原生窗口必须在这里挖洞
  // 才显示得出来（见 ueNative.ts 顶部说明）。再点同一个图标即收起。
  const [panels, setPanels] = useState(false);

  // 左侧浮层互斥：设置 / 信息 / 云台被点开时，把这块收起来（见 popover.ts）
  useEffect(() => onOtherPopoverOpen('dock', () => setPanels(false)), []);

  // 原生叠加模式（Jetson）：弹窗遮罩带 data-ue-hole，原生视频窗口会在遮罩矩形
  // 上挖洞，于是遮罩与弹窗正常显示在视频之上（见 ueNative.ts 顶部说明）。

  const fleet = expandFleet(sel.counts, sel.order);
  const total = totalVehicles(sel.counts);
  const sceneName = sceneDisplayName(sel.sceneName, sel.counts) || defaultSceneName(sel.counts) || 'scene';

  const ensureScenes = (list: SavedScene[]) => { setScenes(list); persistScenes(list); };
  /** 确认删除：真正移除并落盘，然后关掉确认框（若删的是正在展开/重命名的场景，一并清掉其 UI 态）。 */
  const doDeleteScene = (s: SavedScene) => {
    ensureScenes(removeScene(scenes, s.id));
    if (expandedId === s.id) setExpandedId(null);
    if (renameId === s.id) setRenameId(null);
    setConfirmDelete(null);
  };

  /**
   * 保存场景到「已建场景」列表。
   * ⚠️ 关键：编辑既有场景（editingSceneId 非空）时**沿用原 id**，否则改名后会因
   * 名字不匹配而新插一条，留下「内容相同、名字不同」的副本（改名保存 = 复制）。
   */
  const persistScene = (next: SimConfigSelection, id: string): SavedScene[] => {
    const scene: SavedScene = {
      id, name: next.sceneName || defaultSceneName(next.counts),
      mapKey: next.mapKey, counts: { ...next.counts }, formation: next.formation,
      sensorConfigs: next.sensorConfigs, createdAt: Date.now(),
      // 存「与 counts 对齐过」的顺序，保证 DroneN 归属可复现
      order: normalizeOrder(next.counts, next.order),
    };
    // 编辑既有场景：直接按 id 覆盖（不经过 upsertScene —— 它是按 name 匹配的）
    let list: SavedScene[];
    if (editingSceneId && scenes.some((s) => s.id === editingSceneId)) {
      list = scenes.map((s) => (s.id === editingSceneId
        ? { ...scene, id: editingSceneId, createdAt: s.createdAt } : s));
    } else {
      list = upsertScene(scenes, scene);
    }
    // 两条分支都必须落盘（曾漏掉新建分支 ⇒「已保存」提示出现但 localStorage 里没有）
    ensureScenes(list);
    return list;
  };

  /** 保存：写入列表并持久化，但不改变当前场景、不关闭弹窗 */
  const onSaveScene = (next: SimConfigSelection) => {
    persistScene(next, editingSceneId ?? uid());
  };

  /** 应用：保存 + 作为当前场景生效，并关闭弹窗 */
  const onApplyScene = (next: SimConfigSelection) => {
    persistScene(next, editingSceneId ?? uid());
    setSel(next);
    try { localStorage.setItem(LS_KEY, JSON.stringify(next)); } catch { /* ignore */ }
    // 广播：机队顺序/counts 变了 ⇒ 其它读 localStorage 的组件（如相机变焦的飞机下拉）
    // 必须重建机队，否则会停在旧顺序上（「飞机名字与调整顺序后不一致」）。
    bus.emit(EVENTS.SIM_CONFIG_CHANGED, { order: next.order, counts: next.counts, source: 'composer-apply' });
    setEditingSceneId(null);
    setComposer(false);
  };

  const selFromScene = (s: SavedScene): SimConfigSelection => ({
    mapKey: s.mapKey, sceneKey: 'custom', sceneName: sceneDisplayName(s.name, s.counts), counts: { ...s.counts },
    formation: s.formation ?? DEFAULT_FORMATION,
    sensorConfigs: mergeSensorConfigs(s.sensorConfigs, expandFleet(s.counts, s.order)),
    order: s.order,
  });

  const loadIntoEditor = (s: SavedScene) => {
    setEditingSceneId(s.id);
    setSel(selFromScene(s));
    setComposer(true);
  };

  const applySceneFromPreview = (s: SavedScene) => { setPreviewScene(null); setSel(selFromScene(s)); };
  const editSceneFromPreview = (s: SavedScene) => { setPreviewScene(null); loadIntoEditor(s); };

  const startRename = (s: SavedScene) => { setNameDraft(sceneDisplayName(s.name, s.counts)); setRenameId(s.id); };
  const cancelRename = () => setRenameId(null);
  const commitRename = (s: SavedScene) => {
    const name = nameDraft.trim();
    if (name) ensureScenes(renameScene(scenes, s.id, name));
    setRenameId(null);
  };

  /** 构建当前机队 manifest（启动仿真用） */
  const buildManifest = () => {
    const map = UE_MAPS.find((m) => m.key === sel.mapKey)!;
    const sensorByAlias = sel.sensorConfigs ?? fleetSensorConfigs(fleet);
    const vehicles = fleet.map((k, i) => {
      const v = findVehicle(k);
      const raw = sensorByAlias[fleetAlias(i, fleet)] ?? buildSensorConfig(v.sensorTemplate);
      // 勾了「用模板」的传感器：剥掉 origin/rpy ⇒ 后端按 id 继承机型模板位姿。
      // 这样存档里残留的误填值（如 Chase 的 "0 0 0"）也不会被发出去。
      const sensors = stripTemplatePoses(raw);
      return { uav: fleetAlias(i, fleet), key: v.key, name: v.name, type: v.type, sensors };
    });
    const formation = sel.formation ?? DEFAULT_FORMATION;
    const resolved = resolveFormation(formation, fleet);
    // rosMasterIp: Web「主机配置」里的 ROS master 地址，随 manifest 下发给 orchestrator，
    // 由它写进 tmux webcomposed 生成的 yml 的 pre_window ROS_MASTER_URI。
    // 不加这个字段 orchestrator 只能退回 127.0.0.1，节点会连不上真正的 master。
    // pixelStreamingFrameRate: 由 orchestrator 转成 UE 的 -PixelStreamingFrameRate=N。
    // 帧率直接决定 RTP 包率，是 Jetson 端节省 jitterbuffer/解码开销最直接的手段。
    return { map: map.level, mapKey: map.key, sceneName: sceneName, sceneKey: sel.sceneKey, vehicles, formation, resolved, rosMasterIp: getRosMasterIp(), pixelStreamingCodec: ue.codec, pixelStreamingFrameRate: ue.frameRate };
  };

  /** ▶ 启动仿真：manifest 发 orchestrator:8070（其内部生成场景/载具并拉起 UE） */
  const startSimOrch = async () => {
    if (total === 0) { setApplyMsg('请先编排至少一台载具'); return; }
    setApplyMsg('正在编排并启动仿真…');
    const outcome = await orchLaunch(ue.addr.host, buildManifest());
    setApplyMsg(outcome);
  };

  /** 已建场景条目一键应用：只更新当前选择并保存到 localStorage，供下次启动仿真使用。 */
  const applySceneDirect = (s: SavedScene) => {
    const next = selFromScene(s);
    setSel(next);
    try {
      localStorage.setItem(LS_KEY, JSON.stringify(next));
      setApplyMsg('配置已保存；下次点击“启动仿真”时使用，不会立即改变正在运行的 UE。');
    } catch {
      setApplyMsg('当前配置已切换，但无法写入浏览器本地存储；不会立即改变正在运行的 UE。');
    }
    // 同 onApplyScene：机队顺序变了要让订阅者重建（见 EVENTS.SIM_CONFIG_CHANGED）
    bus.emit(EVENTS.SIM_CONFIG_CHANGED, { order: next.order, counts: next.counts, source: 'scene-apply' });
  };

  return (
    <div className="sim-workspace">
      <div className="sim-main">
        <UeView />

        {/* ── 左侧图标栏 ──────────────────────────────────────────────────
            原来左侧那三块面板（载具列表 / 场景组合 / 已建场景）是 flex 里 300px
            的实体侧栏，直接吃掉视频区宽度。现在收成**一个**图标，点一下把三块
            面板一次全展开（不再有中间那层三项菜单）。
            图标压在视频上（窗口现在铺满整块 .ue-view-box，不再有 43px 侧带），
            所以按钮自己带中性灰底板，并且必须靠 data-ue-hole 挖洞 ——
            挂在按钮上而不是容器上，是为了把洞压到 36×36，而不是一整条。
            挖洞必须用 "pill" 而不是空值：矩形洞的四角会露出页面底色，
            看着就是「黑方块套一个灰圆」。 */}
        <div className="sim-rail">
          <button
            data-ue-hole="pill"
            className={`sim-rail-btn${panels ? ' on' : ''}`}
            title="载具列表 / 场景组合 / 已建场景"
            onClick={() => {
              if (panels) {
                setPanels(false);
                return;
              }
              setPanels(true);
              // 打开时广播：左侧同一时刻只留一个浮层（见 popover.ts）
              announcePopover('dock');
            }}
          >{ICON_MENU}</button>
        </div>

        {/* 三块面板一次全开：从上到下 载具列表 / 场景组合 / 已建场景，
            每块都是完整卡片（有标题、有全部操作），不再是「菜单选一块」。
            整块挖一个矩形洞（data-ue-hole=""）—— 里面没有圆角切片的需求，
            一个矩形就够，洞越少越不容易闪。 */}
        {panels && (
        <div className="sim-dock" data-ue-hole="">
          {/* 提示信息放在浮层顶部而不是卡片里：三块面板都能看到
              「已请求启动」「配置已保存」这类反馈。 */}
          <div className="sim-dock-bar">
            {applyMsg ? <span className="sim-dock-msg">{applyMsg}</span> : <span />}
            <button className="sim-dock-x" onClick={() => setPanels(false)} title="收起面板">×</button>
          </div>

        <div className="sim-card">
          <div className="sim-hd-row">
            <h4>载具列表</h4>
            <span className="sim-hd-meta">{VEHICLES.length} 款机型</span>
          </div>
          <button className="btn comp-btn" onClick={() => setCatalog(true)}>机型与传感器列表</button>
        </div>

        <div className="sim-card">
          <div className="sim-hd-row">
            <h4>场景组合</h4>
            <span className={`sim-run-dot${ue.running ? ' on' : ''}`}
              title={`运行态：${ue.running ? '运行中' : '未运行'}`}>{ue.running ? '运行中' : '未运行'}</span>
          </div>
          {/* 地图 · 场景名 · 载具数 合成一行紧凑摘要 */}
          <div className="sim-brief" title={`地图 ${UE_MAPS.find((m) => m.key === sel.mapKey)?.label ?? sel.mapKey} · 场景 ${sceneName} · ${total} 台载具`}>
            <span>{UE_MAPS.find((m) => m.key === sel.mapKey)?.label ?? sel.mapKey}</span>
            <i>·</i>
            <span className="sim-brief-ellip">{sceneName}</span>
            <i>·</i>
            <span>{total} 台</span>
          </div>
          <button className="btn comp-btn" onClick={() => { setEditingSceneId(null); setComposer(true); }}>✏ 编排/新建场景</button>
          {/* 仿真控制：启动 / 停止（UE 地址等仍由菜单栏「仿真配置」负责） */}
          <div className="sim-actions">
            <button className="btn" disabled={ue.busy} onClick={startSimOrch}>{ue.busy ? '启动中…' : '▶ 启动仿真'}</button>
            <button className="btn danger" disabled={ue.busy} onClick={() => orchStop(ue.addr.host)}>停止</button>
          </div>
        </div>

        {/* 相机变焦已迁移到「云台控制」悬浮面板内（components/GimbalControlFloat.tsx），
            不再占仿真侧栏；它改的是 sim 相机 FOV，不经 ROS，走 orchestrator→hello_webcombo 转发。 */}

        {/* 已建场景列表：可整体收起（场景多时避免占满浮层，把上面的卡片挤出可视区） */}
        <div className={`sim-card sim-scenes${scenesOpen ? '' : ' collapsed'}`}>
          <h4
            className="sim-collapse-hd"
            onClick={() => setScenesOpen((v) => !v)}
            title={scenesOpen ? '收起已建场景列表' : '展开已建场景列表'}
          >
            <span className={`sim-caret${scenesOpen ? ' down' : ''}`}>▸</span>
            已建场景（{scenes.length}）
            <span className="sim-collapse-act">{scenesOpen ? '收起' : '展开'}</span>
          </h4>
          {scenesOpen && (
          <div className="sim-scenes-list">
          {scenes.length === 0 && <div className="fleet-empty">尚无场景，点击上方"编排场景"创建</div>}
          {scenes.map((s) => {
            const exp = expandedId === s.id;
            const sFleet = expandFleet(s.counts, s.order);
            const sTotal = totalVehicles(s.counts);
            // 分组按**真实机队顺序**聚合同机型（不能再用 VEHICLES 目录顺序，
            // 否则手动调过序的场景，分组顺序与下方「机队明细」chip 对不上）。
            const grouped: { v: VehicleDef; n: number }[] = [];
            for (const k of sFleet) {
              const last = grouped[grouped.length - 1];
              if (last && last.v.key === k) last.n++;
              else grouped.push({ v: findVehicle(k), n: 1 });
            }
            return (
              <div key={s.id} className={`scene-item${exp ? ' open' : ''}`}>
                <div className="scene-head" onClick={() => setPreviewScene(s)} title="点击预览该场景">
                  {renameId === s.id ? (
                    <input
                      className="scene-rename-input"
                      value={nameDraft}
                      autoFocus
                      onClick={(e) => e.stopPropagation()}
                      onChange={(e) => setNameDraft(e.target.value)}
                      onKeyDown={(e) => { if (e.key === 'Enter') commitRename(s); if (e.key === 'Escape') cancelRename(); }}
                    />
                  ) : (
                    <span className="scene-name">{sceneDisplayName(s.name, s.counts)}</span>
                  )}
                  <span className="scene-meta">{sTotal} 台 · {UE_MAPS.find((m) => m.key === s.mapKey)?.label ?? s.mapKey}</span>
                  <span className={`scene-caret ${exp ? 'down' : ''}`} onClick={(e) => { e.stopPropagation(); setExpandedId(exp ? null : s.id); }}>▸</span>
                </div>
                <div className="scene-actions-hdr" onClick={(e) => e.stopPropagation()}>
                  {renameId === s.id ? (
                    <>
                      <button className="btn small" onClick={() => commitRename(s)}>确定</button>
                      <button className="btn small" onClick={cancelRename}>取消</button>
                    </>
                  ) : (
                    <>
                      <button className="btn small" title="保存为当前配置，下次启动仿真时使用；不会立即改变运行中的 UE" onClick={() => applySceneDirect(s)}>应用配置</button>
                      <button className="btn small" title="重命名" onClick={() => startRename(s)}>重命名</button>
                      <button className="btn small danger" title="删除" onClick={() => setConfirmDelete(s)}>删除</button>
                    </>
                  )}
                </div>
                {exp && (
                  <div className="scene-body">
                    {grouped.map(({ v, n }) => (
                      <div key={v.key} className="scene-group">
                        <span className="scene-group-name">{v.label}</span>
                        <span className="scene-group-n">× {n}</span>
                        <button className="scl-prev" onClick={() => setPreview(v)}>模型</button>
                      </div>
                    ))}
                    <div className="scene-fleet-label">机队明细</div>
                    <div className="fleet-chips">
                      {sFleet.map((k, i) => {
                        const v = findVehicle(k);
                        return (
                          <span key={`${i}-${k}`} className="fleet-chip" onClick={() => setPreview(v)} title="点击查看 3D 模型">
                            <b className="fleet-alias">{fleetAlias(i, sFleet)}</b>
                            <span>{v.label}</span>
                          </span>
                        );
                      })}
                    </div>
                    <div className="scene-actions">
                      <button className="btn small" onClick={() => loadIntoEditor(s)}>编辑</button>
                      <button className="btn small" onClick={() => setSel(selFromScene(s))}>载入并应用</button>
                    </div>
                  </div>
                )}
              </div>
            );
          })}
          </div>
          )}
        </div>
        </div>
        )}

        {preview && <VehicleModelViewer vehicle={preview} onClose={() => setPreview(null)} />}
        {previewScene && (
          <ScenePreviewDialog
            scene={previewScene}
            onClose={() => setPreviewScene(null)}
            onEdit={editSceneFromPreview}
            onApply={applySceneFromPreview}
          />
        )}
        {catalog && <VehicleCatalogDialog onClose={() => setCatalog(false)} />}
        {confirmDelete && (
          <div className="cfd-mask" data-ue-hole="" onClick={() => setConfirmDelete(null)}>
            <div className="cfd-window" role="dialog" aria-modal="true" onClick={(e) => e.stopPropagation()}>
              <div className="cfd-title">删除场景</div>
              <div className="cfd-body">
                确定要删除场景「<b>{sceneDisplayName(confirmDelete.name, confirmDelete.counts)}</b>」吗？
                <div className="cfd-hint">该操作会从「已建场景」列表中移除并落盘，无法撤销。</div>
              </div>
              <div className="cfd-foot">
                <button className="btn" onClick={() => setConfirmDelete(null)} autoFocus>取消</button>
                <button className="btn danger" onClick={() => doDeleteScene(confirmDelete)}>删除</button>
              </div>
            </div>
          </div>
        )}
        {composer && (
          <SceneComposerDialog selection={sel} existing={scenes} editingId={editingSceneId} onClose={() => setComposer(false)} onSave={onSaveScene} onApply={onApplyScene} />
        )}
      </div>
    </div>
  );
}