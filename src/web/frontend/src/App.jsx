import { useEffect, useState } from 'react'
import MapView from './MapView.jsx'
import ThreeDView from './ThreeDView.jsx'
import {
  getHealth,
  getTerrainInfo,
  getTerrainRaster,
  loadTerrainGrid,
  geocode,
  plan3d,
} from './api.js'

export default function App() {
  const [health, setHealth] = useState(null)
  const [terrainInfo, setTerrainInfo] = useState(null)
  const [start, setStart] = useState(null)             // [lng, lat]
  const [goal, setGoal] = useState(null)              // [lng, lat]
  const [clickMode, setClickMode] = useState('start') // 'start' | 'goal'
  const [startAlt, setStartAlt] = useState(null)       // 起点海拔（米），null=自动
  const [goalAlt, setGoalAlt] = useState(null)         // 终点海拔（米），null=自动
  const [startHeading, setStartHeading] = useState(null) // 起点航向（度），null=自动朝终点
  const [viewMode, setViewMode] = useState('2d')        // '2d' | '3d'
  const [raster, setRaster] = useState(null)
  const [path3d, setPath3d] = useState(null)
  // 3D 规划参数
  const [aglMin, setAglMin] = useState(100)
  const [aglMax, setAglMax] = useState(800)
  const [safetyMargin, setSafetyMargin] = useState(50)
  const [timeBudget, setTimeBudget] = useState(15)
  const [loiter, setLoiter] = useState('ccw')
  // 地形
  const [terrainMode, setTerrainMode] = useState('online') // 始终在线（搜索即加载）
  const [onlineStatus, setOnlineStatus] = useState(null)
  const [coverageRadius, setCoverageRadius] = useState(100) // km
  // 位置搜索
  const [locationInput, setLocationInput] = useState('')
  const [coordFormat, setCoordFormat] = useState('auto') // auto | dd | dm | dms
  const [geoResults, setGeoResults] = useState(null)
  const [geoSearching, setGeoSearching] = useState(false)
  const [gotoTarget, setGotoTarget] = useState(null)
  const [coverageCenter, setCoverageCenter] = useState(null) // [lng, lat] 覆盖圆心
  const [globalSearch, setGlobalSearch] = useState(false)
  const [baseMap, setBaseMap] = useState('gaode_satellite') // 默认高德卫星
  const [status, setStatus] = useState({ text: '就绪', type: 'info' })
  const [busy, setBusy] = useState(false)

  useEffect(() => {
    getHealth().then(setHealth).catch(() => setHealth(null))
  }, [])

  // 地形变化 → 拉取 3D 用抽稀高程
  useEffect(() => {
    if (!terrainInfo) { setRaster(null); return }
    let cancelled = false
    setRaster(null)
    getTerrainRaster(160)
      .then((value) => {
        if (!cancelled) setRaster(value)
      })
      .catch((error) => {
        if (cancelled) return
        setRaster(null)
        setStatus({ text: `三维地形生成失败：${error.message}`, type: 'error' })
      })
    return () => { cancelled = true }
  }, [terrainInfo])

  const onMapClick = (xy) => {
    if (clickMode === 'start') {
      setStart(xy)
      setStartAlt(null)
      setStartHeading(null)
      setStatus({ text: `起点: (${xy[0].toFixed(4)}, ${xy[1].toFixed(4)})`, type: 'ok' })
    } else {
      setGoal(xy)
      setGoalAlt(null)
      setStatus({ text: `终点: (${xy[0].toFixed(4)}, ${xy[1].toFixed(4)})`, type: 'ok' })
    }
  }

  // 解析单个坐标值，支持 DD / DM / DMS 三种格式，返回十进制度
  const parseSingleCoord = (s) => {
    s = s.trim()
    // 方向后缀：E/W → 经度，N/S → 纬度
    let dir = null
    const dirMatch = s.match(/([NSEW])$/i)
    if (dirMatch) {
      dir = dirMatch[1].toUpperCase()
      s = s.replace(/([NSEW])$/i, '').trim()
    }
    const neg = dir === 'W' || dir === 'S'
    if (neg && !s.startsWith('-')) s = '-' + s

    // 匹配 DMS：103°48'36" 或 103度48分36秒
    const dmsMatch = s.match(/^(-?\d+(?:\.\d+)?)\s*[°度]\s*(\d+(?:\.\d+)?)\s*['′分]\s*(\d+(?:\.\d+)?)\s*["″秒]?\s*$/)
    if (dmsMatch) {
      const d = parseFloat(dmsMatch[1])
      const m = parseFloat(dmsMatch[2])
      const sec = parseFloat(dmsMatch[3])
      const v = Math.abs(d) + m / 60 + sec / 3600
      return { value: d < 0 ? -v : v, type: 'lng-or-lat' }
    }
    // 匹配 DM：103°48.5' 或 103度48.5分
    const dmMatch = s.match(/^(-?\d+(?:\.\d+)?)\s*[°度]\s*(\d+(?:\.\d+)?)\s*['′分]?\s*$/)
    if (dmMatch) {
      const d = parseFloat(dmMatch[1])
      const m = parseFloat(dmMatch[2])
      const v = Math.abs(d) + m / 60
      return { value: d < 0 ? -v : v, type: 'lng-or-lat' }
    }
    // 匹配 DD：103.8° 或 103.8
    const ddMatch = s.match(/^(-?\d+(?:\.\d+)?)\s*[°]?\s*$/)
    if (ddMatch) {
      return { value: parseFloat(ddMatch[1]), type: 'lng-or-lat' }
    }
    return null
  }

  // 解析输入的坐标字符串，返回 { lng, lat } 或 null
  const parseLngLat = (text) => {
    text = text.trim()
    // 按逗号或空格分隔
    const parts = text.split(/[,，\s]+/).filter((p) => p.length > 0)
    if (parts.length < 2) return null

    // 把相邻的"数字+方向字母"合并（如 "103.8" "E" "36.06" "N" → ["103.8E", "36.06N"]）
    const merged = []
    for (let i = 0; i < parts.length; i++) {
      if (/^[NSEW]$/i.test(parts[i]) && merged.length > 0) {
        merged[merged.length - 1] += parts[i]
      } else {
        merged.push(parts[i])
      }
    }

    // 找出带 E/W 后缀的=经度，带 N/S 后缀的=纬度
    let lng = null, lat = null
    const unmarked = []
    for (const part of merged) {
      const hasEW = /[EW]$/i.test(part)
      const hasNS = /[NS]$/i.test(part)
      const parsed = parseSingleCoord(part)
      if (!parsed) return null
      if (hasEW) lng = parsed.value
      else if (hasNS) lat = parsed.value
      else unmarked.push(parsed.value)
    }

    // 没有后缀标识的，按值范围判断
    if (lng === null || lat === null) {
      for (const v of unmarked) {
        if (Math.abs(v) > 90 && Math.abs(v) <= 180 && lng === null) lng = v
        else if (Math.abs(v) <= 90 && lat === null) lat = v
      }
    }

    if (lng == null || lat == null) return null
    if (!(-180 <= lng && lng <= 180 && -90 <= lat && lat <= 90)) return null
    return { lng, lat }
  }

  const flyTo = (lng, lat, label) => {
    setTerrainMode('online')
    setOnlineStatus({ loading: true })
    setTerrainInfo(null)
    setRaster(null)
    setViewMode('2d')
    setCoverageCenter([lng, lat])
    setGotoTarget({ lng, lat, sideKm: coverageRadius, nonce: Date.now() })
    setGeoResults(null)
    setStart(null)
    setGoal(null)
    setStartAlt(null)
    setGoalAlt(null)
    setStartHeading(null)
    setPath3d(null)
    setStatus({
      text: `跳转至 ${label || `${lng.toFixed(5)}, ${lat.toFixed(5)}`}，加载 ${coverageRadius}km 正方形地形…`,
      type: 'info',
    })
  }

  // 点击覆盖半径按钮：以当前设定点为中心，立即重新加载对应边长的正方形地形
  const handleRadiusChange = (r) => {
    setCoverageRadius(r)
    if (!coverageCenter) return
    setTerrainInfo(null)
    setRaster(null)
    setViewMode('2d')
    setPath3d(null)
    setOnlineStatus({ loading: true })
    setGotoTarget({ lng: coverageCenter[0], lat: coverageCenter[1], sideKm: r, nonce: Date.now() })
    setStatus({ text: `重新加载 ${r}km 边长正方形地形…`, type: 'info' })
  }

  const handleLocate = async () => {
    const text = locationInput.trim()
    if (!text) return
    const ll = parseLngLat(text)
    if (ll) {
      flyTo(ll.lng, ll.lat)
      return
    }
    setGeoSearching(true)
    setGeoResults(null)
    try {
      const results = await geocode(text, 6)
      if (results.length === 0) {
        setStatus({ text: `未找到「${text}」，可用 经度,纬度`, type: 'error' })
      } else if (results.length === 1) {
        flyTo(results[0].lng, results[0].lat, results[0].name)
      } else {
        setGeoResults(results)
        setStatus({ text: `找到 ${results.length} 个候选，请点击选择`, type: 'info' })
      }
    } catch (e) {
      setStatus({ text: `搜索失败：${e.message}`, type: 'error' })
    } finally {
      setGeoSearching(false)
    }
  }

  const handleOnlineTerrain = async (payload) => {
    try {
      const info = await loadTerrainGrid(payload)
      setTerrainInfo(info)
      setPath3d(null)
      return info
    } catch (e) {
      setStatus({ text: `地形加载失败：${e.message}`, type: 'error' })
      throw e
    }
  }

  const handlePlan3d = async () => {
    if (!start || !goal) {
      setStatus({ text: '请先在地图上点击设置起点和终点', type: 'error' })
      return
    }
    if (!terrainInfo || onlineStatus?.loading || onlineStatus?.error || !raster) {
      setStatus({ text: '地形尚未就绪，请等待地形状态显示海拔范围后再规划', type: 'error' })
      return
    }
    setBusy(true)
    setStatus({ text: '3D 规划中（RRT* + Dubins）…', type: 'info' })
    try {
      const result = await plan3d({
        start: { lng: start[0], lat: start[1], alt: startAlt, heading: startHeading },
        goal: { lng: goal[0], lat: goal[1], alt: goalAlt },
        agl_min: aglMin,
        agl_max: aglMax,
        safety_margin: safetyMargin,
        time_budget: timeBudget,
        loiter,
        nofly_zones: [],
      })
      setPath3d(result)
      setViewMode('3d')
      setStatus({
        text: `完成：${result.length.toFixed(0)}m，${result.waypoints.length} 航点（${result.iterations} 迭代 / ${result.plan_time.toFixed(1)}s）`,
        type: 'ok',
      })
    } catch (err) {
      setStatus({ text: `规划失败：${err.message}`, type: 'error' })
    } finally {
      setBusy(false)
    }
  }

  const handleClear = () => {
    setStart(null)
    setGoal(null)
    setPath3d(null)
    setStatus({ text: '已清空起终点', type: 'info' })
  }

  return (
    <div className="app">
      <aside className="sidebar">
        <h2>地形路径规划</h2>
        <div className="mission-console-entry">
          <strong>固定翼任务控制台</strong>
          <span>多机任务、ROS 地形规划、冲突检查与飞行控制</span>
          <a href="/swarm.html">进入任务控制台</a>
        </div>

        {/* 底图：只保留高德街道 + 卫星 */}
        <div className="section">
          <h3>底图</h3>
          <button onClick={() => setBaseMap('gaode_street')} className={baseMap === 'gaode_street' ? 'primary' : ''}>高德街道</button>
          <button onClick={() => setBaseMap('gaode_satellite')} className={baseMap === 'gaode_satellite' ? 'primary' : ''}>高德卫星</button>
        </div>

        {/* 位置搜索 */}
        <div className="section">
          <h3>搜索地点</h3>
          <div style={{ display: 'flex', gap: 6, marginBottom: 6 }}>
            <select
              value={coordFormat}
              onChange={(e) => setCoordFormat(e.target.value)}
              style={{ width: 'auto', padding: '5px 6px', border: '1px solid #ccc', borderRadius: 4, fontSize: 12 }}
            >
              <option value="auto">自动识别</option>
              <option value="dd">十进制度</option>
              <option value="dm">度分</option>
              <option value="dms">度分秒</option>
            </select>
            <input
              type="text"
              style={{ flex: 1, padding: '5px 8px', border: '1px solid #ccc', borderRadius: 4, fontSize: 13 }}
              placeholder={
                coordFormat === 'dd' ? '103.8065, 36.0611' :
                coordFormat === 'dm' ? "103°48.39′, 36°3.67′" :
                coordFormat === 'dms' ? "103°48'23\" 36°3'40\"" :
                '地名 或 经度,纬度'
              }
              value={locationInput}
              onChange={(e) => setLocationInput(e.target.value)}
              onKeyDown={(e) => { if (e.key === 'Enter') handleLocate() }}
            />
            <button className="primary" onClick={handleLocate} disabled={geoSearching}>
              {geoSearching ? '…' : '跳转'}
            </button>
          </div>
          <div className="hint" style={{ marginBottom: 6 }}>
            {coordFormat === 'dd' && '十进制度：103.8065, 36.0611 或 103.8065°E, 36.0611°N'}
            {coordFormat === 'dm' && '度分：103°48.39′, 36°3.67′ 或 103°48.39′E 36°3.67′N'}
            {coordFormat === 'dms' && '度分秒：103°48′23″ 36°3′40″ 或 103°48′23″E 36°3′40″N'}
            {coordFormat === 'auto' && '自动识别：输入地名 或 任意格式经纬度（支持 DD/DM/DMS + E/W/N/S 后缀）'}
          </div>
          <label style={{ display: 'flex', alignItems: 'center', gap: 5, fontSize: 12 }}>
            <input type="checkbox" checked={globalSearch} onChange={(e) => setGlobalSearch(e.target.checked)} style={{ width: 'auto' }} />
            搜国外地点
          </label>
          {geoResults && (
            <div style={{ marginTop: 6, maxHeight: 160, overflowY: 'auto', border: '1px solid #eee', borderRadius: 4 }}>
              {geoResults.map((r, i) => (
                <div
                  key={i}
                  onClick={() => flyTo(r.lng, r.lat, r.name)}
                  style={{ padding: '6px 8px', fontSize: 12, cursor: 'pointer', borderBottom: i < geoResults.length - 1 ? '1px solid #f0f0f0' : 'none' }}
                  onMouseEnter={(e) => { e.currentTarget.style.background = '#f0f7ff' }}
                  onMouseLeave={(e) => { e.currentTarget.style.background = '' }}
                >
                  {r.name}
                  <span style={{ color: '#999', marginLeft: 6 }}>{r.lng.toFixed(4)},{r.lat.toFixed(4)}</span>
                </div>
              ))}
            </div>
          )}
          <div className="hint">搜索后自动加载以该点为中心、边长 {coverageRadius}km 的真实地形</div>
        </div>

        {/* 覆盖范围：边长可点击立即重新加载 */}
        <div className="section">
          <h3>地形覆盖（正方形边长）</h3>
          <div style={{ display: 'flex', flexWrap: 'wrap', gap: 6 }}>
            {[1, 5, 10, 30, 50, 100, 200].map((r) => (
              <button
                key={r}
                onClick={() => handleRadiusChange(r)}
                className={coverageRadius === r ? 'primary' : ''}
                style={{ flex: '1 1 calc(25% - 6px)', minWidth: '48px' }}
              >{r}km</button>
            ))}
          </div>
          <div className="hint">
            以设定点为中心的 {coverageRadius}km × {coverageRadius}km 正方形。点击数字立即重新加载。
          </div>
        </div>

        {/* 地形状态 */}
        {onlineStatus && (
          <div className="section" style={{ padding: '8px 12px' }}>
            <div className="status info">
              {onlineStatus.loading && '正在加载真实地形…'}
              {!onlineStatus.loading && onlineStatus.error && `失败：${onlineStatus.error}`}
              {!onlineStatus.loading && !onlineStatus.error &&
                `z${onlineStatus.zoom} · ${onlineStatus.nTiles} 瓦片 · ${onlineStatus.resolutionM}m/格`}
            </div>
            {terrainInfo && (
              <div className="hint" style={{ marginTop: 6 }}>
                海拔 {terrainInfo.elevation_min.toFixed(0)}~{terrainInfo.elevation_max.toFixed(0)}m
              </div>
            )}
          </div>
        )}

        {/* 起终点设置按钮 */}
        <div className="section">
          <h3>设置起终点</h3>
          <button onClick={() => setClickMode('start')} className={clickMode === 'start' ? 'primary' : ''} style={{ width: '48%' }}>设起点</button>
          <button onClick={() => setClickMode('goal')} className={clickMode === 'goal' ? 'primary' : ''} style={{ width: '48%' }}>设终点</button>
          <div className="hint">{clickMode === 'start' ? '点击地图设置起点（绿色）' : '点击地图设置终点（红色）'}</div>
        </div>

        {/* 起终点 3D 参数 */}
        {start && (
          <div className="section">
            <h3>起点</h3>
            <div className="hint" style={{ marginBottom: 8 }}>{start[0].toFixed(4)}, {start[1].toFixed(4)}</div>
            <label>海拔（米）— 空=自动</label>
            <input type="number" placeholder="自动" value={startAlt ?? ''}
              onChange={(e) => setStartAlt(e.target.value === '' ? null : parseFloat(e.target.value))} />
            <label>航向（度，0=北 顺）— 空=自动</label>
            <input type="number" placeholder="自动" value={startHeading ?? ''}
              onChange={(e) => setStartHeading(e.target.value === '' ? null : parseFloat(e.target.value))} />
          </div>
        )}
        {goal && (
          <div className="section">
            <h3>终点</h3>
            <div className="hint" style={{ marginBottom: 8 }}>{goal[0].toFixed(4)}, {goal[1].toFixed(4)}</div>
            <label>海拔（米）— 空=自动</label>
            <input type="number" placeholder="自动" value={goalAlt ?? ''}
              onChange={(e) => setGoalAlt(e.target.value === '' ? null : parseFloat(e.target.value))} />
          </div>
        )}

        {/* 视图切换 */}
        <div className="section">
          <h3>视图</h3>
          <button onClick={() => setViewMode('2d')} className={viewMode === '2d' ? 'primary' : ''}>2D 地图</button>
          <button
            onClick={() => setViewMode('3d')}
            className={viewMode === '3d' ? 'primary' : ''}
            disabled={!raster}
            title={raster ? '查看三维地形' : '请先等待在线地形加载完成'}
          >3D 视图</button>
        </div>

        {/* 规划参数 */}
        <div className="section">
          <h3>规划参数</h3>
          <label>离地高度下限 AGL min（米）</label>
          <input type="number" value={aglMin} onChange={(e) => setAglMin(parseFloat(e.target.value))} />
          <label>离地高度上限 AGL max（米）</label>
          <input type="number" value={aglMax} onChange={(e) => setAglMax(parseFloat(e.target.value))} />
          <label>安全余量（米）</label>
          <input type="number" value={safetyMargin} onChange={(e) => setSafetyMargin(parseFloat(e.target.value))} />
          <label>规划时间预算（秒）</label>
          <input type="number" value={timeBudget} onChange={(e) => setTimeBudget(parseFloat(e.target.value))} />
          <label>终点盘旋方向</label>
          <select value={loiter} onChange={(e) => setLoiter(e.target.value)} style={{ width: '100%', padding: '4px 6px' }}>
            <option value="ccw">逆时针</option>
            <option value="cw">顺时针</option>
            <option value="none">无</option>
          </select>
        </div>

        {/* 操作 */}
        <div className="section">
          <button
            onClick={handlePlan3d}
            className="primary"
            disabled={busy || !start || !goal || !terrainInfo || !raster || onlineStatus?.loading || Boolean(onlineStatus?.error)}
            title={!terrainInfo || !raster ? '请先等待在线地形加载完成' : ''}
            style={{ width: '100%' }}
          >
            {busy ? '规划中…' : '3D 规划（地形跟随）'}
          </button>
          <button onClick={handleClear} disabled={busy} style={{ width: '100%' }}>清空起终点</button>
          {path3d && (
            <div className="hint" style={{ marginTop: 6 }}>
              {path3d.segments.length} 段 · {path3d.length.toFixed(0)}m
            </div>
          )}
        </div>

        <div className={`status ${status.type}`}>{status.text}</div>
      </aside>

      {viewMode === '2d' ? (
        <MapView
          terrainInfo={terrainInfo}
          start={start}
          goal={goal}
          noflyZones={[]}
          path={null}
          path3d={path3d}
          onMapClick={onMapClick}
          onMapDblClick={() => {}}
          onTerrainRelocate={() => {}}
          clickMode={clickMode}
          baseMap={baseMap}
          onlineTerrain={terrainMode === 'online'}
          onOnlineTerrain={handleOnlineTerrain}
          onOnlineStatus={setOnlineStatus}
          autoFit={true}
          goto={gotoTarget}
          coverageRadius={coverageRadius}
          coverageCenter={coverageCenter}
        />
      ) : (
        <ThreeDView
          raster={raster}
          start={start}
          goal={goal}
          startAlt={startAlt}
          goalAlt={goalAlt}
          startHeading={startHeading}
          path3d={path3d}
          clickMode={clickMode}
          onPickPoint={onMapClick}
        />
      )}
    </div>
  )
}
