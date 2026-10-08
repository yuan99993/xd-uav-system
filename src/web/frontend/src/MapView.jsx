import { useEffect, useRef, useState } from 'react'
import L from 'leaflet'
import { terrainOverlayUrl } from './api.js'
import { fetchTerrainForSquare } from './demTiles.js'

/**
 * 地图视图组件
 *
 * 坐标系说明：
 *   使用 Leaflet 默认坐标系 EPSG3857（Web 墨卡托），坐标用经纬度 [lat, lng]。
 *   后端 terrain_info 返回的 bounds 中 x=经度、y=纬度，和 Leaflet 的 [lat, lng] 互换。
 *
 * 底图：
 *   - OSM 街道图（默认）
 *   - 谷歌卫星图
 *   - 谷歌街道图
 *   用户可在界面切换。
 */

// 底图源配置：name, url, attribution
// 所有源都必须设 subdomains（即使 URL 没有 {s}），否则 leaflet 会崩溃
const BASE_MAPS = {
  offline: {
    name: '离线网格',
    offline: true,  // 标记：不请求网络，纯网格线
    attribution: '离线模式',
  },
  gaode_street: {
    name: '高德街道',
    url: 'https://webrd0{s}.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
    subdomains: '1234',
    attribution: '© 高德地图',
  },
  gaode_satellite: {
    name: '高德卫星',
    url: 'https://webst0{s}.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
    subdomains: '1234',
    attribution: '© 高德地图',
  },
  arcgis_street: {
    name: 'ArcGIS 街道',
    url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}',
    subdomains: 'a',
    attribution: '© ArcGIS',
  },
  arcgis_satellite: {
    name: 'ArcGIS 卫星',
    url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    subdomains: 'a',
    attribution: '© ArcGIS',
  },
  osm: {
    name: 'OSM',
    url: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
    subdomains: 'abc',
    attribution: '© OpenStreetMap',
  },
}

// 离线网格底图：用 canvas 画经纬网格，完全不依赖网络
const OfflineGridLayer = L.GridLayer.extend({
  createTile: function (coords) {
    const tile = document.createElement('canvas')
    tile.width = 256
    tile.height = 256
    const ctx = tile.getContext('2d')
    ctx.fillStyle = '#e8f0f8'
    ctx.fillRect(0, 0, 256, 256)
    ctx.strokeStyle = '#b0c8e0'
    ctx.lineWidth = 1
    for (let i = 0; i <= 256; i += 32) {
      ctx.beginPath()
      ctx.moveTo(i, 0)
      ctx.lineTo(i, 256)
      ctx.stroke()
      ctx.beginPath()
      ctx.moveTo(0, i)
      ctx.lineTo(256, i)
      ctx.stroke()
    }
    // 标注坐标（调试用）
    ctx.fillStyle = '#8090a0'
    ctx.font = '10px monospace'
    ctx.fillText(`z${coords.z} x${coords.x} y${coords.y}`, 6, 14)
    return tile
  },
})

// 根据 baseMap key 创建底图图层（在线 tileLayer 或离线网格）
function createBaseLayer(mapKey) {
  const cfg = BASE_MAPS[mapKey] || BASE_MAPS.offline
  if (cfg.offline) {
    return new OfflineGridLayer()
  }
  return L.tileLayer(cfg.url, {
    subdomains: cfg.subdomains,
    attribution: cfg.attribution,
    maxZoom: 20,
  })
}

export default function MapView({
  terrainInfo,
  start,
  goal,
  noflyZones,
  path,
  path3d,
  onMapClick,
  onMapDblClick,
  onTerrainRelocate,
  clickMode = 'none',
  baseMap = 'offline',
  onlineTerrain = false,   // 在线地形模式（固定半径，不随视野滚动）
  onOnlineTerrain,         // async (gridPayload) => 调后端装载
  onOnlineStatus,          // ({loading, zoom?, resolutionM?, error?}) => void
  autoFit = true,          // 地形变化后是否自动框选视野（在线模式必须 false，否则循环触发）
  goto,                    // {lng, lat, sideKm?, nonce}：外部请求飞行+加载地形
  coverageRadius = 100,    // 正方形边长（km），中心=设定点
  coverageCenter = null,   // [lng, lat] 正方形中心
}) {
  const containerRef = useRef(null)
  const mapRef = useRef(null)
  const overlayRef = useRef(null)
  const overlayImgRef = useRef(null) // 山体阴影 PNG 图层
  const baseLayerRef = useRef(null)
  const clickLayerRef = useRef(null)
  const noflyLayerRef = useRef(null)
  const pathLayerRef = useRef(null)
  const path3dLayerRef = useRef(null)
  const coverageCircleRef = useRef(null) // 地形覆盖范围圆
  // 拖动矩形的临时状态 + 上一次矩形尺寸（用来判断是"平移"还是"重新加载"）
  const dragStateRef = useRef(null)
  const prevSizeRef = useRef(null)
  // 用 ref 存最新回调，避免 map 重新初始化
  const clickCbRef = useRef(onMapClick)
  const dblClickCbRef = useRef(onMapDblClick)
  const relocateCbRef = useRef(onTerrainRelocate)
  const onlineCbRef = useRef(onOnlineTerrain)
  const onlineStatusCbRef = useRef(onOnlineStatus)
  useEffect(() => { clickCbRef.current = onMapClick }, [onMapClick])
  useEffect(() => { dblClickCbRef.current = onMapDblClick }, [onMapDblClick])
  useEffect(() => { relocateCbRef.current = onTerrainRelocate }, [onTerrainRelocate])
  useEffect(() => { onlineCbRef.current = onOnlineTerrain }, [onOnlineTerrain])
  useEffect(() => { onlineStatusCbRef.current = onOnlineStatus }, [onOnlineStatus])

  // 初始化地图（只跑一次）
  useEffect(() => {
    if (!containerRef.current || mapRef.current) return
    mapRef.current = L.map(containerRef.current, {
      zoomControl: true,
    }).setView([39.9, 116.4], 14)  // 北京附近

    // 默认底图
    baseLayerRef.current = createBaseLayer(baseMap).addTo(mapRef.current)

    clickLayerRef.current = L.layerGroup().addTo(mapRef.current)
    noflyLayerRef.current = L.layerGroup().addTo(mapRef.current)
    pathLayerRef.current = L.layerGroup().addTo(mapRef.current)
    path3dLayerRef.current = L.layerGroup().addTo(mapRef.current)
    coverageCircleRef.current = L.layerGroup().addTo(mapRef.current)

    mapRef.current.on('click', (e) => {
      // e.latlng = { lat, lng }；后端 x=经度, y=纬度，所以传 [lng, lat]
      clickCbRef.current?.([e.latlng.lng, e.latlng.lat])
    })
    mapRef.current.on('dblclick', () => {
      dblClickCbRef.current?.()
    })
    // 关掉默认双击放大
    mapRef.current.doubleClickZoom.disable()
    // 强制 leaflet 重新测量容器尺寸（修复初始化时高度/宽度为 0 的问题）
    setTimeout(() => mapRef.current?.invalidateSize(), 50)
    setTimeout(() => mapRef.current?.invalidateSize(), 200)
    setTimeout(() => mapRef.current?.invalidateSize(), 500)
    setTimeout(() => mapRef.current?.invalidateSize(), 1000)
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  // 底图切换（mount 时也执行一次，必须和初始化共用 createBaseLayer）
  useEffect(() => {
    if (!mapRef.current) return
    if (baseLayerRef.current) {
      mapRef.current.removeLayer(baseLayerRef.current)
    }
    baseLayerRef.current = createBaseLayer(baseMap).addTo(mapRef.current)
  }, [baseMap])

  // 窗口大小变化时同步 leaflet
  useEffect(() => {
    const onResize = () => mapRef.current?.invalidateSize()
    window.addEventListener('resize', onResize)
    return () => window.removeEventListener('resize', onResize)
  }, [])

  // 当外部容器尺寸变化时（比如侧边栏撑开后），强制 leaflet 重新测量
  // 用 ResizeObserver 比 window.resize 更可靠，能捕捉到 DOM 变化
  useEffect(() => {
    if (!containerRef.current) return
    const ro = new ResizeObserver(() => {
      mapRef.current?.invalidateSize()
    })
    ro.observe(containerRef.current)
    return () => ro.disconnect()
  }, [])

  // 地形加载：铺山体阴影图（PNG）+ 蓝色范围矩形（可拖拽）
  useEffect(() => {
    if (!mapRef.current) return
    if (overlayImgRef.current) {
      mapRef.current.removeLayer(overlayImgRef.current)
      overlayImgRef.current = null
    }
    if (overlayRef.current) {
      mapRef.current.removeLayer(overlayRef.current)
      overlayRef.current = null
    }
    if (!terrainInfo) {
      prevSizeRef.current = null
      return
    }

    const b = terrainInfo.bounds
    // leaflet 经纬度坐标: [lat=y, lng=x]，左下=[y_min, x_min]，右上=[y_max, x_max]
    const bounds = [[b.y_min, b.x_min], [b.y_max, b.x_max]]

    // 山体阴影 + 高程着色图层（URL 带 bounds 作 cache-bust，拖动后自动换新图）
    overlayImgRef.current = L.imageOverlay(terrainOverlayUrl(b), bounds, {
      interactive: false,
      opacity: 0.9,
    }).addTo(mapRef.current)

    overlayRef.current = L.rectangle(bounds, {
      color: '#80a0c0',
      weight: 1,
      fill: false, // 范围框只描边，不挡住下面的地形图
    }).addTo(mapRef.current)

    // 只有"尺寸变化"（首次加载/重新加载）才把镜头框到矩形；
    // 拖动是纯平移（宽高不变），此时不要 fitBounds，否则镜头会跳、缩放被重置。
    // 在线滚动地形模式同样不能 fit：加载完会改视野 → 又触发 moveend → 死循环。
    const newW = b.x_max - b.x_min
    const newH = b.y_max - b.y_min
    const prev = prevSizeRef.current
    if (autoFit && (!prev || Math.abs(prev.w - newW) > 1e-12 || Math.abs(prev.h - newH) > 1e-12)) {
      mapRef.current.fitBounds(bounds)
    }
    prevSizeRef.current = { w: newW, h: newH }
  }, [terrainInfo, autoFit])

  // 在线地形：固定半径加载模式（不随视野滚动）
  // 只在 goto（搜索地点）时触发加载，平时拖/缩地图不会重载
  const flyingUntilRef = useRef(0)
  useEffect(() => {
    const map = mapRef.current
    if (!map || !onlineTerrain) return
    // 模式开启时，如当前 goto 有值，立即加载一次（否则等下次搜索）
  }, [onlineTerrain])

  // goto → 飞行 + 加载以设定点为中心、边长=sideKm 的正方形地形
  useEffect(() => {
    const map = mapRef.current
    if (!map || !goto || !goto.nonce) return
    let cancelled = false

    const sideKm = goto.sideKm ?? coverageRadius ?? 100
    const duration = 1.0
    flyingUntilRef.current = Date.now() + duration * 1000 + 200
    // 飞行到目标位置，缩放级别随边长自动选（边长越大缩得越小）
    const targetZoom = sideKm >= 200 ? 8
      : sideKm >= 100 ? 9
      : sideKm >= 50 ? 10
      : sideKm >= 30 ? 11
      : sideKm >= 10 ? 12
      : sideKm >= 5 ? 13 : 14
    map.flyTo([goto.lat, goto.lng], targetZoom, { duration })

    // 飞行动画结束后加载该位置的正方形地形
    const loadTimer = setTimeout(async () => {
      onlineStatusCbRef.current?.({ loading: true })
      try {
        const payload = await fetchTerrainForSquare(goto.lng, goto.lat, sideKm)
        if (cancelled) return
        await onlineCbRef.current?.(payload)
        if (cancelled) return
        onlineStatusCbRef.current?.({
          loading: false, zoom: payload.zoom,
          resolutionM: payload.resolution_m, nTiles: payload.n_tiles,
        })
      } catch (e) {
        if (cancelled) return
        onlineStatusCbRef.current?.({ loading: false, error: e.message })
      }
    }, duration * 1000 + 300)

    return () => {
      cancelled = true
      clearTimeout(loadTimer)
    }
  }, [goto])

  // 地形覆盖正方形：边长=coverageRadius，中心=coverageCenter（与实际加载的地形范围一致）
  useEffect(() => {
    const squareLayer = coverageCircleRef.current
    const map = mapRef.current
    if (!squareLayer || !map) return
    squareLayer.clearLayers()
    if (!coverageCenter) return
    const [lng, lat] = coverageCenter
    const latHalf = (coverageRadius / 2) / 110.574
    const lngHalf = (coverageRadius / 2) / (110.574 * Math.cos((lat * Math.PI) / 180))
    // 蓝色虚线正方形 + 中心点标记
    L.rectangle([[lat - latHalf, lng - lngHalf], [lat + latHalf, lng + lngHalf]], {
      color: '#1677ff',
      weight: 2,
      dashArray: '8 6',
      fillColor: '#1677ff',
      fillOpacity: 0.06,
    }).addTo(squareLayer)
    L.circleMarker([lat, lng], {
      radius: 5,
      color: '#1677ff',
      fillColor: '#1677ff',
      fillOpacity: 1,
    }).addTo(squareLayer)
  }, [coverageCenter, coverageRadius])

  // 「移动地形」模式：让蓝色矩形可整体拖动（纯手工实现，不依赖额外插件）
  useEffect(() => {
    const map = mapRef.current
    const rect = overlayRef.current
    if (!map || !rect) return

    const setMoveCursor = (on) => {
      const el = rect.getElement && rect.getElement()
      if (el) el.style.cursor = on ? 'move' : ''
    }
    const setActiveStyle = (on) => {
      rect.setStyle(on
        ? { color: '#1677ff', weight: 2, fillColor: '#69b1ff', fillOpacity: 0.35 }
        : { color: '#80a0c0', weight: 1, fill: false })
    }

    if (onlineTerrain || clickMode !== 'move') {
      rect.off('mousedown')
      setMoveCursor(false)
      setActiveStyle(false)
      return
    }
    setMoveCursor(true)
    setActiveStyle(true)

    const onMouseDown = (e) => {
      // 阻止冒泡：既不触发地图拖动，也不触发 map.click 误设起点/终点
      L.DomEvent.stopPropagation(e.originalEvent)
      L.DomEvent.preventDefault(e.originalEvent)
      map.dragging.disable()
      dragStateRef.current = {
        startLatLng: e.latlng,
        startBounds: rect.getBounds(),
      }
    }
    const onMouseMove = (e) => {
      const st = dragStateRef.current
      if (!st) return
      const dLat = e.latlng.lat - st.startLatLng.lat
      const dLng = e.latlng.lng - st.startLatLng.lng
      const b0 = st.startBounds
      rect.setBounds(L.latLngBounds(
        [b0.getSouth() + dLat, b0.getWest() + dLng],
        [b0.getNorth() + dLat, b0.getEast() + dLng],
      ))
    }
    const finish = () => {
      const st = dragStateRef.current
      if (!st) return
      dragStateRef.current = null
      map.dragging.enable()
      const b = rect.getBounds()
      // 松开鼠标 → 把新范围回传父组件 → 后端 relocate
      relocateCbRef.current?.({
        x_min: b.getWest(),
        y_min: b.getSouth(),
        x_max: b.getEast(),
        y_max: b.getNorth(),
      })
    }

    rect.on('mousedown', onMouseDown)
    map.on('mousemove', onMouseMove)
    map.on('mouseup', finish)
    window.addEventListener('mouseup', finish)  // 鼠标拖出地图外松开也能结束
    return () => {
      rect.off('mousedown', onMouseDown)
      map.off('mousemove', onMouseMove)
      map.off('mouseup', finish)
      window.removeEventListener('mouseup', finish)
      map.dragging.enable()
      setMoveCursor(false)
      setActiveStyle(false)
    }
  }, [clickMode, terrainInfo, onlineTerrain])

  // 起点/终点
  useEffect(() => {
    if (!clickLayerRef.current) return
    clickLayerRef.current.clearLayers()
    if (start) {
      // 起点：绿色实心圆，大半径便于看见
      L.circleMarker([start[1], start[0]], {
        radius: 8,
        color: '#00b050',
        fillColor: '#00b050',
        fillOpacity: 1,
        weight: 2,
      })
        .bindTooltip('起点', { permanent: false, direction: 'top' })
        .addTo(clickLayerRef.current)
    }
    if (goal) {
      L.circleMarker([goal[1], goal[0]], {
        radius: 8,
        color: '#ff4d4f',
        fillColor: '#ff4d4f',
        fillOpacity: 1,
        weight: 2,
      })
        .bindTooltip('终点', { permanent: false, direction: 'top' })
        .addTo(clickLayerRef.current)
    }
  }, [start, goal])

  // 禁飞区
  useEffect(() => {
    if (!noflyLayerRef.current) return
    noflyLayerRef.current.clearLayers()
    noflyZones.forEach((zone, idx) => {
      const latlngs = zone.map(([x, y]) => [y, x])
      L.polygon(latlngs, {
        color: '#ff0000',
        fillColor: '#ff8080',
        fillOpacity: 0.4,
      })
        .bindTooltip(`禁飞区 ${idx + 1}`, { permanent: false })
        .addTo(noflyLayerRef.current)
    })
  }, [noflyZones])

  // 规划路径（2D A*）
  useEffect(() => {
    if (!pathLayerRef.current) return
    pathLayerRef.current.clearLayers()
    if (!path || !path.waypoints?.length) return
    const latlngs = path.waypoints.map(([x, y, z]) => [y, x])
    L.polyline(latlngs, { color: '#1677ff', weight: 3 })
      .addTo(pathLayerRef.current)
    // 端点标记
    L.circleMarker(latlngs[0], { radius: 4, color: '#1677ff' })
      .addTo(pathLayerRef.current)
    L.circleMarker(latlngs[latlngs.length - 1], { radius: 4, color: '#1677ff' })
      .addTo(pathLayerRef.current)
  }, [path])

  // 3D 规划路径（橙色，与 3D 视图一致）
  useEffect(() => {
    if (!path3dLayerRef.current) return
    path3dLayerRef.current.clearLayers()
    if (!path3d || !path3d.waypoints?.length) return
    const latlngs = path3d.waypoints
      .filter(([x, y]) => x != null && y != null)
      .map(([x, y]) => [y, x])
    if (latlngs.length < 2) return
    L.polyline(latlngs, { color: '#ff6a00', weight: 4, opacity: 0.95 })
      .bindTooltip(`3D 路径 ${path3d.length?.toFixed(0)}m`, { sticky: true })
      .addTo(path3dLayerRef.current)
  }, [path3d])

  // 监听容器宽度，太窄时给提示
  const [tooNarrow, setTooNarrow] = useState(false)
  useEffect(() => {
    if (!containerRef.current) return
    const ro = new ResizeObserver((entries) => {
      const w = entries[0].contentRect.width
      setTooNarrow(w < 200)
    })
    ro.observe(containerRef.current)
    return () => ro.disconnect()
  }, [])

  return (
    <div ref={containerRef} className="map-container">
      {tooNarrow && (
        <div className="map-too-narrow-hint">
          地图区域过窄，请拉宽浏览器窗口
        </div>
      )}
    </div>
  )
}
