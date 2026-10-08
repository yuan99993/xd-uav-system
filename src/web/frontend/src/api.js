// 后端 API 客户端
// 用同源相对路径，经 Vite 代理转发到 http://127.0.0.1:8000/api
// （避免浏览器系统代理拦截对 8000 端口的直接请求）
const BASE = '/api'

// 统一错误处理：后端正常错误返回 {detail: "..."}，
// 但 500 等异常可能是纯文本/HTML，直接 r.json() 会再抛
// "Unexpected token 'I'... is not valid JSON"，把真正的错误盖掉
async function parseError(r) {
  const text = await r.text()
  try {
    const data = JSON.parse(text)
    return data.detail || `请求失败 (${r.status})`
  } catch {
    return text ? `${text} (HTTP ${r.status})` : `请求失败 (HTTP ${r.status})`
  }
}

async function request(url, options) {
  const r = await fetch(url, options)
  if (!r.ok) throw new Error(await parseError(r))
  return r.json()
}

export function getHealth() {
  return request(`${BASE}/health`)
}

export function loadSampleTerrain() {
  return request(`${BASE}/terrain/load_sample`, { method: 'POST' })
}

export function uploadTerrain(file) {
  const fd = new FormData()
  fd.append('file', file)
  return request(`${BASE}/terrain/upload`, { method: 'POST', body: fd })
}

export function getTerrainInfo() {
  return request(`${BASE}/terrain/info`)
}

// bounds: {x_min, y_min, x_max, y_max}；拖动蓝色矩形后回传，返回更新后的 terrain info
export function relocateTerrain(bounds) {
  return request(`${BASE}/terrain/relocate`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(bounds),
  })
}

// start/goal: [x, y]; nofly_zones: [[[x,y], [x,y], ...], ...]
export function planPath({ start, goal, cruise_altitude, safety_margin, nofly_zones }) {
  const body = {
    start: { x: start[0], y: start[1] },
    goal: { x: goal[0], y: goal[1] },
    cruise_altitude,
    safety_margin,
    nofly_zones: (nofly_zones || []).map((pts) => ({ points: pts.map(([x, y]) => ({ x, y })) })),
  }
  return request(`${BASE}/plan`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  })
}

export function dispatchPath({ topic, segments, use_mock = true }) {
  return request(`${BASE}/dispatch`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ topic, segments, use_mock }),
  })
}

// ------------------------------------------------------------------
// 3D 地形 / 3D 规划（对齐 terrain-navigation）
// ------------------------------------------------------------------

// 山体阴影+高程着色图层（PNG）。
// 图片内容只取决于高程矩阵（拖动地形不改海拔矩阵，图片不变），
// 后端已设 Cache-Control: no-cache，重新上传后会重新校验；传入完整 bounds
// 时额外用 bounds 作 cache-bust 键，缺字段时降级为稳定 URL，避免 undefined.toFixed 抛错。
export function terrainOverlayUrl(bounds) {
  let v = 'none'
  if (bounds && bounds.x_max != null && bounds.y_max != null
      && bounds.x_min != null && bounds.y_min != null) {
    v = `${bounds.x_min.toFixed(6)},${bounds.y_min.toFixed(6)},${bounds.x_max.toFixed(6)},${bounds.y_max.toFixed(6)}`
  }
  return `${BASE}/terrain/overlay.png?v=${encodeURIComponent(v)}`
}

// 3D 视图数据：抽稀高程矩阵 + ENU 原点经纬度 + 实际尺寸（米）
export function getTerrainRaster(maxGrid = 160) {
  return request(`${BASE}/terrain/raster?max_grid=${maxGrid}`)
}

// 在线滚动地形：前端从 DEM 瓦片解码降采样后的高程网格 → 装载为当前地形
// payload: { elevation, x_min, y_min, x_max, y_max, zoom, n_tiles }
export function loadTerrainGrid(payload) {
  return request(`${BASE}/terrain/load_grid`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ source: 'online_terrarium', ...payload }),
  })
}

// 地名搜索（Nominatim 代理）：返回 [{name, lat, lng, type}]；country 为空时搜全球
export function geocode(query, limit = 5, country = 'cn') {
  const q = encodeURIComponent(query)
  const c = country ? `&country=${encodeURIComponent(country)}` : ''
  return request(`${BASE}/geocode?q=${q}&limit=${limit}${c}`).then((r) => r.results || [])
}

// 3D 规划：start/goal 为 [lng, lat]，内部转 ENU 后跑 RRT* + Dubins Airplane
export function plan3d({
  start, goal, agl_min, agl_max, turning_radius, climb_angle_deg,
  safety_margin, time_budget, step, goal_bias, loiter, nofly_zones,
}) {
  // start/goal 已是 {lng, lat, alt?, heading?} 对象，直接透传
  const body = {
    start, goal,
    agl_min, agl_max, turning_radius, climb_angle_deg,
    safety_margin, time_budget, step, goal_bias, loiter,
    nofly_zones: (nofly_zones || []).map((pts) => ({ points: pts.map(([x, y]) => ({ x, y })) })),
  }
  return request(`${BASE}/plan3d`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  })
}
