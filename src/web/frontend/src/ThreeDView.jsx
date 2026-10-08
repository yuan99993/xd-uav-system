import { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { terrainOverlayUrl } from './api.js'

/**
 * 3D 地形 + 路径视图（three.js）
 *
 * 坐标映射（与后端 ENU 一致）：
 *   three.x = ENU 东向 (米)   = (lng - lng0) * 米/度经度
 *   three.z = -ENU 北向 (米)  = -(lat - lat0) * 111320   （three 里北 = -z）
 *   three.y = 相对高度 (米)   = (海拔 - 地形最低海拔) * 夸张系数
 *
 * 地形：/api/terrain/raster 的抽稀高程矩阵 → PlaneGeometry 顶点位移，
 *       贴 /api/terrain/overlay.png 山体阴影图（图片顶部=最北，与网格 row0 对齐）。
 * 路径：plan3d 返回的 [lng, lat, alt] 航点 → 3D 折线 + 到地面的垂直参考线。
 */

const M_PER_DEG_LAT = 111320.0

// [lng, lat] → ENU 米制（相对 raster 原点）
function toEnu(lng, lat, origin) {
  const mPerDegLng = M_PER_DEG_LAT * Math.cos((origin.lat * Math.PI) / 180)
  return [(lng - origin.lng) * mPerDegLng, (lat - origin.lat) * M_PER_DEG_LAT]
}

export default function ThreeDView({ raster, start, goal, startAlt, goalAlt, startHeading, path3d, clickMode, onPickPoint }) {
  const containerRef = useRef(null)
  const sceneRef = useRef(null) // { renderer, scene, camera, controls, terrainData, pathGroup, markerGroup }
  const [exagg, setExagg] = useState(4.0)  // 高度夸张：让地形起伏和路径升降更明显
  const [profile, setProfile] = useState(null) // 高度剖面数据 [{d, terrain, alt}]
  const pickCbRef = useRef(onPickPoint)
  useEffect(() => { pickCbRef.current = onPickPoint }, [onPickPoint])

  // 地形 / 夸张系数变化 → 整个 3D 场景重建（几何量小，重建开销可忽略）
  useEffect(() => {
    if (!containerRef.current || !raster) return
    const container = containerRef.current
    const width = container.clientWidth
    const height = container.clientHeight
    if (width < 2 || height < 2) return

    // ---- renderer / scene / camera ----
    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.setPixelRatio(window.devicePixelRatio)
    renderer.setSize(width, height)
    container.appendChild(renderer.domElement)

    // 计算地形高差，用于相机和雾的合理布置
    let eMax = -Infinity
    for (const row of raster.elevation) for (const v of row) if (v != null && v > eMax) eMax = v
    if (!isFinite(eMax)) eMax = 0
    const elevRange = Math.max(50, eMax - (raster.elevation_min || 0))
    const terrainSize = Math.max(raster.width_m, raster.height_m)

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0xbfd7ea)
    // 雾推远：近端 = 地形尺寸 4 倍，远端 = 10 倍，避免小地形被雾遮住
    scene.fog = new THREE.Fog(0xbfd7ea, terrainSize * 4, terrainSize * 12)

    const camera = new THREE.PerspectiveCamera(50, width / height, 1, terrainSize * 30)
    // 相机位置：侧前方斜俯视，高度 = 地形尺寸 × 0.5 + 高差 × 夸张，保证看到起伏
    const homePosition = () =>
      new THREE.Vector3(
        raster.width_m * 0.35,
        terrainSize * 0.55 + elevRange * exagg * 1.5,
        raster.height_m * 0.7,
      )
    camera.position.copy(homePosition())

    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true
    controls.dampingFactor = 0.08
    controls.maxPolarAngle = Math.PI / 2.05
    // 目标点：地形中心，高度为地形高差中点 × 夸张
    const targetY = elevRange * 0.3 * exagg
    controls.target.set(0, targetY, 0)

    // ---- 灯光（与 hillshade 光照一致的西北向太阳）----
    scene.add(new THREE.HemisphereLight(0xdfeaf5, 0x506050, 0.9))
    const sun = new THREE.DirectionalLight(0xfff4e0, 1.4)
    sun.position.set(-terrainSize * 0.5, terrainSize, -terrainSize * 0.3)
    scene.add(sun)

    const grid = new THREE.GridHelper(
      Math.max(raster.width_m, raster.height_m) * 1.2, 12, 0x88a0b8, 0xc0d0dd)
    grid.position.y = -2
    scene.add(grid)

    const pathGroup = new THREE.Group()
    const markerGroup = new THREE.Group()
    scene.add(pathGroup, markerGroup)

    // ---- 地形 mesh：高程矩阵 → 顶点位移 ----
    const { rows, cols, width_m: w, height_m: h, elevation } = raster
    const elevFlat = new Float32Array(rows * cols)
    let eMin = Infinity
    for (const row of elevation) for (const v of row) if (v != null && v < eMin) eMin = v
    if (!isFinite(eMin)) eMin = 0
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        const v = elevation[r][c]
        elevFlat[r * cols + c] = (v == null ? eMin : v) - eMin
      }
    }

    const geo = new THREE.PlaneGeometry(w, h, cols - 1, rows - 1)
    geo.rotateX(-Math.PI / 2) // (x,y,0) → (x,0,-y)：平面 y+ 变为 -z（北）
    const pos = geo.attributes.position
    for (let i = 0; i < pos.count; i++) {
      pos.setY(i, elevFlat[i] * exagg)
    }
    pos.needsUpdate = true
    geo.computeVertexNormals()

    // 贴图：山体阴影 PNG。用 raster 里的地理 bounds 作 cache-bust + 正确裁剪范围
    const material = new THREE.MeshStandardMaterial({
      color: 0x9aa88f, roughness: 0.95, metalness: 0,
    })
    const overlayBounds = raster.bounds || {
      x_min: raster.origin.lng, y_min: raster.origin.lat,
      x_max: raster.origin.lng + 0.1, y_max: raster.origin.lat + 0.1,
    }
    new THREE.TextureLoader().load(terrainOverlayUrl(overlayBounds), (tex) => {
      tex.colorSpace = THREE.SRGBColorSpace
      material.map = tex
      material.color.set(0xffffff)
      material.needsUpdate = true
    }, undefined, () => {/* 失败保持纯色 */})
    const mesh = new THREE.Mesh(geo, material)
    scene.add(mesh)

    const terrainData = { raster, elevFlat, eMin }
    sceneRef.current = { renderer, scene, camera, controls, terrainData, pathGroup, markerGroup, homePosition, targetY, terrainMesh: mesh }

    // ---- 3D 点击拾取起终点：区分"拖拽旋转"与"点击"，点击时 raycast 地形表面 ----
    const raycaster = new THREE.Raycaster()
    const ndc = new THREE.Vector2()
    const downRef = { x: 0, y: 0, t: 0, btn: -1 }
    const onPointerDown = (e) => {
      downRef.x = e.clientX
      downRef.y = e.clientY
      downRef.t = Date.now()
      downRef.btn = e.button
    }
    const onPointerUp = (e) => {
      if (downRef.btn !== 0) return
      // 移动超过 6px 或按住超过 600ms → 是拖拽视角，不是选点
      if (Math.hypot(e.clientX - downRef.x, e.clientY - downRef.y) > 6) return
      if (Date.now() - downRef.t > 600) return
      const rect = renderer.domElement.getBoundingClientRect()
      ndc.x = ((e.clientX - rect.left) / rect.width) * 2 - 1
      ndc.y = -((e.clientY - rect.top) / rect.height) * 2 + 1
      raycaster.setFromCamera(ndc, camera)
      const hits = raycaster.intersectObject(mesh, false)
      if (!hits.length) return
      const p = hits[0].point
      const mPerDegLng = M_PER_DEG_LAT * Math.cos((raster.origin.lat * Math.PI) / 180)
      const lng = raster.origin.lng + p.x / mPerDegLng
      const lat = raster.origin.lat + (-p.z) / M_PER_DEG_LAT
      pickCbRef.current?.([lng, lat])
    }
    renderer.domElement.addEventListener('pointerdown', onPointerDown)
    renderer.domElement.addEventListener('pointerup', onPointerUp)

    // 尺寸自适应
    const ro = new ResizeObserver(() => {
      const cw = container.clientWidth
      const ch = container.clientHeight
      if (cw < 2 || ch < 2) return
      renderer.setSize(cw, ch)
      camera.aspect = cw / ch
      camera.updateProjectionMatrix()
    })
    ro.observe(container)

    let raf = 0
    const animate = () => {
      raf = requestAnimationFrame(animate)
      controls.update()
      renderer.render(scene, camera)
    }
    animate()

    return () => {
      cancelAnimationFrame(raf)
      ro.disconnect()
      renderer.domElement.removeEventListener('pointerdown', onPointerDown)
      renderer.domElement.removeEventListener('pointerup', onPointerUp)
      controls.dispose()
      renderer.dispose()
      geo.dispose()
      material.dispose()
      if (material.map) material.map.dispose()
      if (renderer.domElement.parentNode === container) container.removeChild(renderer.domElement)
      sceneRef.current = null
    }
  }, [raster, exagg])

  // 路径 / 起终点变化 → 只重建对应 group（场景不动）+ 计算高度剖面
  useEffect(() => {
    const s = sceneRef.current
    if (!s) return
    drawPath(s, path3d, exagg)
    drawMarkers(s, start, goal, startAlt, goalAlt, startHeading, exagg)

    // 高度剖面：沿路径采样地形海拔与航迹海拔（绝对海拔，米）
    if (path3d?.waypoints?.length > 1) {
      const { origin } = s.terrainData.raster
      const pts = []
      for (const [lng, lat, alt] of path3d.waypoints) {
        if (lng == null) continue
        const [xE, yN] = toEnu(lng, lat, origin)
        pts.push({ x: xE, z: -yN, alt })
      }
      const prof = []
      let dist = 0
      let prev = null
      for (const p of pts) {
        if (prev) dist += Math.hypot(p.x - prev.x, p.z - prev.z)
        prev = p
        const ground = sampleElev(s.terrainData, p.x, p.z)
        prof.push({ d: dist, terrain: (ground ?? 0) + (s.terrainData.eMin || 0), alt: p.alt })
      }
      setProfile(prof.length > 1 ? prof : null)
    } else {
      setProfile(null)
    }
  }, [path3d, start, goal, startAlt, goalAlt, startHeading, raster, exagg])

  const resetCamera = () => {
    const s = sceneRef.current
    if (!s) return
    s.camera.position.copy(s.homePosition())
    s.controls.target.set(0, s.targetY || 0, 0)
    s.controls.update()
  }

  if (!raster) {
    return (
      <div className="map-container" style={{ display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
        <div className="map-too-narrow-hint">3D 视图：请先加载地形</div>
      </div>
    )
  }
  return (
    <div className="map-container">
      <div
        ref={containerRef}
        style={{ width: '100%', height: '100%', cursor: clickMode === 'goal' ? 'cell' : 'crosshair' }}
      />
      <div style={{
        position: 'absolute', top: 10, left: 10, zIndex: 5,
        background: 'rgba(255,255,255,0.92)', padding: '6px 10px', borderRadius: 6,
        fontSize: 12, color: '#333', display: 'flex', gap: 6, alignItems: 'center',
      }}>
        高度夸张
        {[1.5, 2.5, 4, 6].map((v) => (
          <button key={v} onClick={() => setExagg(v)}
            className={exagg === v ? 'primary' : ''} style={{ margin: 0, padding: '2px 8px' }}>
            {v}×
          </button>
        ))}
        <button onClick={resetCamera} style={{ margin: 0, padding: '2px 8px' }}>重置视角</button>
      </div>
      <div style={{
        position: 'absolute', top: 44, left: 10, zIndex: 5,
        background: 'rgba(255,255,255,0.92)', padding: '4px 10px', borderRadius: 6,
        fontSize: 12, color: clickMode === 'goal' ? '#ff4d4f' : '#00b050', fontWeight: 600,
      }}>
        3D 点击地形{clickMode === 'goal' ? '设终点（红）' : '设起点（绿）'} · 拖拽=旋转视角
      </div>
      {path3d && (
        <div style={{
          position: 'absolute', bottom: 10, left: 10, zIndex: 5,
          background: 'rgba(255,255,255,0.92)', padding: '6px 10px', borderRadius: 6, fontSize: 12, color: '#333',
        }}>
          3D 路径：{path3d.length?.toFixed(0)}m（{path3d.waypoints?.length} 航点，
          RRT* {path3d.iterations} 次迭代 / {path3d.plan_time?.toFixed(1)}s）
        </div>
      )}
      {path3d && (
        <div style={{
          position: 'absolute', bottom: 10, right: 10, zIndex: 5,
          background: 'rgba(255,255,255,0.94)', padding: '8px 10px', borderRadius: 6,
          border: '1px solid #e0e0e0',
        }}>
          <ProfileChart profile={profile} />
        </div>
      )}
    </div>
  )
}

// ---- 高度剖面图：横轴=沿路径距离(km)，灰=地形海拔，橙=航迹海拔 ----
function ProfileChart({ profile }) {
  if (!profile || profile.length < 2) return null
  const W = 480
  const H = 150
  const padL = 52
  const padR = 14
  const padT = 16
  const padB = 24
  // 降采样到 ~300 点，SVG 尺寸可控
  const stride = Math.max(1, Math.ceil(profile.length / 300))
  const pts = profile.filter((_, i) => i % stride === 0 || i === profile.length - 1)
  const dTotal = pts[pts.length - 1].d || 1
  let aMin = Infinity
  let aMax = -Infinity
  for (const p of pts) {
    aMin = Math.min(aMin, p.terrain, p.alt)
    aMax = Math.max(aMax, p.terrain, p.alt)
  }
  if (!isFinite(aMin)) return null
  const padA = Math.max(30, (aMax - aMin) * 0.08)
  aMin -= padA
  aMax += padA
  const X = (d) => padL + (d / dTotal) * (W - padL - padR)
  const Y = (a) => H - padB - ((a - aMin) / (aMax - aMin)) * (H - padT - padB)
  const terrainPts = pts.map((p) => `${X(p.d).toFixed(1)},${Y(p.terrain).toFixed(1)}`).join(' ')
  const pathPts = pts.map((p) => `${X(p.d).toFixed(1)},${Y(p.alt).toFixed(1)}`).join(' ')
  const fmt = (v) => Math.round(v)
  return (
    <div>
      <div style={{ fontSize: 11, color: '#666', marginBottom: 2 }}>
        高度剖面（航迹 vs 地形） · 航迹海拔 {fmt(Math.min(...pts.map((p) => p.alt)))}~{fmt(Math.max(...pts.map((p) => p.alt)))}m
      </div>
      <svg width={W} height={H} style={{ display: 'block' }}>
        {/* 地形剖面（填充） */}
        <polygon
          points={`${X(pts[0].d)},${Y(aMin)} ${terrainPts} ${X(pts[pts.length - 1].d)},${Y(aMin)}`}
          fill="rgba(100,120,95,0.45)"
          stroke="#7a8a72"
          strokeWidth="1"
        />
        {/* 航迹高度（橙线） */}
        <polyline points={pathPts} fill="none" stroke="#ff6a00" strokeWidth="2.2" />
        {/* 轴 */}
        <line x1={padL} y1={H - padB} x2={W - padR} y2={H - padB} stroke="#ccc" />
        <line x1={padL} y1={padT} x2={padL} y2={H - padB} stroke="#ccc" />
        <text x={padL - 6} y={padT + 8} fontSize="10" fill="#888" textAnchor="end">{fmt(aMax)}m</text>
        <text x={padL - 6} y={H - padB} fontSize="10" fill="#888" textAnchor="end">{fmt(aMin)}m</text>
        <text x={padL} y={H - 6} fontSize="10" fill="#888">0km</text>
        <text x={W - padR} y={H - 6} fontSize="10" fill="#888" textAnchor="end">{(dTotal / 1000).toFixed(1)}km</text>
      </svg>
    </div>
  )
}

// ---- 地形高度查询（最近邻，足够可视化用）；入参为 three 坐标 (x, z) ----
function sampleElev(terrainData, x, z) {
  const { raster, elevFlat } = terrainData
  const { rows, cols, width_m: w, height_m: h } = raster
  const c = Math.round(((x + w / 2) / w) * (cols - 1))
  const r = Math.round(((z + h / 2) / h) * (rows - 1))
  if (r < 0 || r >= rows || c < 0 || c >= cols) return null
  return elevFlat[r * cols + c]
}

function drawPath(s, path3d, exagg) {
  s.pathGroup.clear()
  if (!path3d?.waypoints?.length || !path3d.waypoints[0]) return
  const { origin } = s.terrainData.raster
  const pts = []
  for (const [lng, lat, alt] of path3d.waypoints) {
    if (lng == null) continue
    const [xE, yN] = toEnu(lng, lat, origin)
    pts.push(new THREE.Vector3(xE, (alt - s.terrainData.eMin) * exagg, -yN))
  }
  if (pts.length < 2) return

  // 主路径线（橙色）
  const lineGeo = new THREE.BufferGeometry().setFromPoints(pts)
  s.pathGroup.add(new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ color: 0xff6a00 })))

  // 每隔若干点画一条到地面的垂直参考线（展示离地净空）
  const stemPts = []
  const every = Math.max(1, Math.floor(pts.length / 40))
  for (let i = 0; i < pts.length; i += every) {
    const p = pts[i]
    const ground = sampleElev(s.terrainData, p.x, p.z)
    if (ground == null) continue
    stemPts.push(new THREE.Vector3(p.x, ground * exagg, p.z), p.clone())
  }
  if (stemPts.length) {
    const stemGeo = new THREE.BufferGeometry().setFromPoints(stemPts)
    s.pathGroup.add(new THREE.LineSegments(stemGeo, new THREE.LineBasicMaterial({
      color: 0xffa940, transparent: true, opacity: 0.55,
    })))
  }
  // 首末端点球（尺寸随地形自适应）
  const terrainSize = Math.max(s.terrainData.raster.width_m, s.terrainData.raster.height_m)
  const mk = (p, color) => {
    const m = new THREE.Mesh(new THREE.SphereGeometry(terrainSize * 0.01, 16, 12),
      new THREE.MeshBasicMaterial({ color }))
    m.position.copy(p)
    s.pathGroup.add(m)
  }
  mk(pts[0], 0x00b050)
  mk(pts[pts.length - 1], 0xff4d4f)
}

function drawMarkers(s, start, goal, startAlt, goalAlt, startHeading, exagg) {
  s.markerGroup.clear()
  const { origin } = s.terrainData.raster
  const eMin = s.terrainData.eMin
  // 标记尺寸随地形大小自适应：大地形用大标记，小地形用小标记
  const terrainSize = Math.max(s.terrainData.raster.width_m, s.terrainData.raster.height_m)
  const markerR = terrainSize * 0.008   // 圆柱半径
  const markerH = terrainSize * 0.04    // 圆柱高度
  const arrowLen = terrainSize * 0.03   // 航向箭头长度

  const add = (ll, color, altOverride, headingDeg) => {
    if (!ll) return
    const [xE, yN] = toEnu(ll[0], ll[1], origin)
    const ground = sampleElev(s.terrainData, xE, -yN)
    const groundAbs = (ground ?? 0) + (eMin || 0)
    const alt = altOverride != null ? altOverride : groundAbs + 60
    const y = (alt - (eMin || 0)) * exagg

    // 圆柱标记（高度=markerH，以 y 为中心）
    const m = new THREE.Mesh(
      new THREE.CylinderGeometry(markerR, markerR, markerH, 16),
      new THREE.MeshBasicMaterial({ color }),
    )
    m.position.set(xE, y, -yN)
    s.markerGroup.add(m)

    // 顶部球（更显眼）
    const ball = new THREE.Mesh(
      new THREE.SphereGeometry(markerR * 1.8, 16, 12),
      new THREE.MeshBasicMaterial({ color }),
    )
    ball.position.set(xE, y + markerH * 0.6, -yN)
    s.markerGroup.add(ball)

    // 从地面到标记的垂直线
    const groundY = ((ground ?? 0)) * exagg
    const lineGeo = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(xE, groundY, -yN),
      new THREE.Vector3(xE, y, -yN),
    ])
    s.markerGroup.add(new THREE.Line(lineGeo,
      new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.6 })))

    // 航向箭头（起点）
    if (headingDeg != null) {
      const rad = (headingDeg * Math.PI) / 180
      const dx = Math.sin(rad) * arrowLen
      const dz = -Math.cos(rad) * arrowLen
      const arrowGeo = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(xE, y, -yN),
        new THREE.Vector3(xE + dx, y, -yN + dz),
      ])
      s.markerGroup.add(new THREE.Line(arrowGeo,
        new THREE.LineBasicMaterial({ color, linewidth: 3 })))
    }
  }
  add(start, 0x00b050, startAlt, startHeading)
  add(goal, 0xff4d4f, goalAlt, null)
}
