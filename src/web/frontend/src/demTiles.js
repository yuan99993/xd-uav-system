/**
 * 在线 DEM 高程瓦片工具（Mapzen Terrarium / AWS 开放数据集）
 *
 * 全球真实高程，RGB 三色编码：
 *     height(m) = r * 256 + g + b / 256 - 32768
 * 海洋也是有效值（≈0），只有极端负值（<-1000）按无数据处理。
 *
 * 工作流（"滚动地图加载"）：
 *   地图移动/缩放停止 → 取视野 bounds → 选瓦片 zoom → 拉覆盖瓦片
 *   → canvas 拼接 → 全分辨率解码高程（不能在编码颜色空间缩放！）
 *   → 块平均降采样 → 回传后端 /api/terrain/load_grid
 */

const TILE_SIZE = 256
const MIN_ZOOM = 8      // z8 单瓦片覆盖约 400~600km，兜底
const MAX_ZOOM = 14     // z14 中纬度约 7~9m/px，对固定翼规划足够
const MAX_TILE_AXIS = 5 // 单方向最多 5 张
const MAX_GRID = 350    // 回传后端网格长边上限（大范围地形需更大网格）
const INVALID_HEIGHT = -1000 // 低于此值视为无数据
const MAX_MERCATOR_LAT = 85.05112878

const tileImgCache = new Map() // "z/x/y" → Promise<HTMLImageElement>

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v))
}

// 经纬度 → XYZ 瓦片序号（Web Mercator）
export function lngLatToTile(lng, lat, z) {
  const n = 2 ** z
  const safeLat = clamp(lat, -MAX_MERCATOR_LAT, MAX_MERCATOR_LAT)
  const latR = (safeLat * Math.PI) / 180
  const x = Math.floor(((lng + 180) / 360) * n)
  const y = Math.floor(
    ((1 - Math.log(Math.tan(latR) + 1 / Math.cos(latR)) / Math.PI) / 2) * n,
  )
  return [clamp(x, 0, n - 1), clamp(y, 0, n - 1)]
}

// 瓦片 (z,x,y) 的地理边界（WGS84 经纬度）
export function tileBounds(z, x, y) {
  const n = 2 ** z
  return {
    west: (x / n) * 360 - 180,
    east: ((x + 1) / n) * 360 - 180,
    north: (Math.atan(Math.sinh(Math.PI * (1 - (2 * y) / n))) * 180) / Math.PI,
    south: (Math.atan(Math.sinh(Math.PI * (1 - (2 * (y + 1)) / n))) * 180) / Math.PI,
  }
}

// 在层级上限内选 zoom：找到"用 ≤ MAX_TILE_AXIS 张瓦片能覆盖正方形边长"的最高 zoom。
// 返回 { zoom, nTiles }，nTiles 为奇数以便以中心点对称取瓦片。
// @param {number} sideKm 正方形边长（km）
// @param {number} lat    中心纬度（影响瓦片地面尺寸）
export function chooseZoomForSquare(sideKm, lat) {
  const cosLat = Math.cos((lat * Math.PI) / 180)
  const sideM = sideKm * 1000
  for (let z = MAX_ZOOM; z >= MIN_ZOOM; z--) {
    const tileM = (360 / 2 ** z) * 111320 * cosLat
    // 覆盖 sideM 所需的最小奇数张瓦片（中心对称）
    const need = Math.ceil(sideM / tileM)
    const n = need % 2 === 0 ? need + 1 : need
    if (n <= MAX_TILE_AXIS) return { zoom: z, nTiles: n }
  }
  return { zoom: MIN_ZOOM, nTiles: MAX_TILE_AXIS }
}

// 单瓦片地面分辨率（米/像素），用于 UI 提示
function tileGroundResolutionM(z, lat) {
  return (Math.cos((lat * Math.PI) / 180) * 2 * Math.PI * 6378137) / (2 ** z * TILE_SIZE)
}

function loadTileImage(z, x, y) {
  const key = `${z}/${x}/${y}`
  if (!tileImgCache.has(key)) {
    const p = new Promise((resolve, reject) => {
      const img = new Image()
      img.onload = () => resolve(img)
      img.onerror = () => {
        tileImgCache.delete(key)
        reject(new Error(`高程瓦片 ${key} 加载失败（网络不通或区域无覆盖）`))
      }
      img.src = `/api/terrain/tile?z=${z}&x=${x}&y=${y}`
    })
    tileImgCache.set(key, p)
  }
  return tileImgCache.get(key)
}

// 全尺寸高程（像素数较多时也可接受：3×3 瓦片 = 768² ≈ 59 万点）
function decodeTerrarium(imageData) {
  const { data, width, height } = imageData
  const out = new Float32Array(width * height)
  for (let i = 0, k = 0; i < data.length; i += 4, k++) {
    const h = data[i] * 256 + data[i + 1] + data[i + 2] / 256 - 32768
    out[k] = h < INVALID_HEIGHT ? NaN : h
  }
  return { heights: out, width, height }
}

// 块平均降采样到 outRows×outCols（先解码后降采样，避免在 RGB 编码空间插值出错）
function downsample(heights, w, h, outRows, outCols) {
  const grid = new Array(outRows)
  for (let r = 0; r < outRows; r++) {
    const row = new Array(outCols)
    const r0 = Math.floor((r * h) / outRows)
    const r1 = Math.max(r0 + 1, Math.floor(((r + 1) * h) / outRows))
    for (let c = 0; c < outCols; c++) {
      const c0 = Math.floor((c * w) / outCols)
      const c1 = Math.max(c0 + 1, Math.floor(((c + 1) * w) / outCols))
      let sum = 0
      let cnt = 0
      for (let i = r0; i < r1; i++) {
        for (let j = c0; j < c1; j++) {
          const v = heights[i * w + j]
          if (!Number.isNaN(v)) {
            sum += v
            cnt++
          }
        }
      }
      row[c] = cnt > 0 ? Math.round(sum / cnt * 10) / 10 : null
    }
    grid[r] = row
  }
  return grid
}

// 正方形地形缓存：同一中心+边长秒切，不重复拉瓦片
const squareCache = new Map()
const SQUARE_CACHE_MAX = 8

const kmPerDegLat = 110.574

/**
 * 加载以 (centerLng, centerLat) 为中心、边长 sideKm 的正方形真实地形。
 * 瓦片拼接后按正方形外接矩形精确裁剪像素，网格与边界严格对齐。
 * 返回可直接 POST /api/terrain/load_grid 的载荷。
 */
export async function fetchTerrainForSquare(centerLng, centerLat, sideKm) {
  if (![centerLng, centerLat, sideKm].every(Number.isFinite) || sideKm <= 0) {
    throw new Error('地形范围参数无效')
  }
  if (Math.abs(centerLat) >= MAX_MERCATOR_LAT) {
    throw new Error(`在线地形仅支持纬度 ${(-MAX_MERCATOR_LAT).toFixed(3)}° 到 ${MAX_MERCATOR_LAT.toFixed(3)}°`)
  }
  const key = `${centerLng.toFixed(4)},${centerLat.toFixed(4)},${sideKm}`
  if (squareCache.has(key)) return squareCache.get(key)

  const { zoom, nTiles } = chooseZoomForSquare(sideKm, centerLat)
  const [cx, cy] = lngLatToTile(centerLng, centerLat, zoom)
  const half = Math.floor(nTiles / 2)
  const x0 = cx - half
  const y0 = cy - half

  // 并发拉瓦片
  const jobs = []
  for (let ty = 0; ty < nTiles; ty++) {
    for (let tx = 0; tx < nTiles; tx++) {
      jobs.push(
        loadTileImage(zoom, x0 + tx, y0 + ty).then((img) => ({ tx, ty, img })),
      )
    }
  }
  const tiles = await Promise.all(jobs)

  // 拼接（瓦片行 y0 = 最北 → canvas 顶部）
  const fullW = TILE_SIZE * nTiles
  const fullH = TILE_SIZE * nTiles
  const canvas = document.createElement('canvas')
  canvas.width = fullW
  canvas.height = fullH
  const ctx = canvas.getContext('2d')
  for (const { tx, ty, img } of tiles) {
    ctx.drawImage(img, tx * TILE_SIZE, ty * TILE_SIZE)
  }

  // 正方形目标边界（中心 ± sideKm/2），保证瓦片网格完全覆盖
  const latSpan = sideKm / kmPerDegLat
  const lngSpan = sideKm / (kmPerDegLat * Math.cos((centerLat * Math.PI) / 180))
  const minLng = centerLng - lngSpan / 2
  const maxLng = centerLng + lngSpan / 2
  const minLat = centerLat - latSpan / 2
  const maxLat = centerLat + latSpan / 2
  if (minLng < -180 || maxLng > 180 ||
      minLat <= -MAX_MERCATOR_LAT || maxLat >= MAX_MERCATOR_LAT) {
    throw new Error('所选地形范围越过在线 DEM 支持边界，请缩小范围或调整中心点')
  }

  // 地理 → 拼接画布像素坐标（裁剪到画布范围）
  const nw = tileBounds(zoom, x0, y0)
  const ne = tileBounds(zoom, x0 + nTiles - 1, y0)
  const se = tileBounds(zoom, x0, y0 + nTiles - 1)
  // 必须使用整个拼接画布的经度跨度。旧实现只使用一张瓦片的宽度，
  // 3×3/5×5 加载时会把裁剪窗口横向放大 3/5 倍并取错区域。
  const lngPerPixel = (ne.east - nw.west) / fullW
  const latPerPixel = (nw.north - se.south) / fullH
  const cx0 = Math.max(0, Math.floor((minLng - nw.west) / lngPerPixel))
  const cx1 = Math.min(fullW, Math.ceil((maxLng - nw.west) / lngPerPixel))
  const cy0 = Math.max(0, Math.floor((nw.north - maxLat) / latPerPixel))
  const cy1 = Math.min(fullH, Math.ceil((nw.north - minLat) / latPerPixel))
  const cropW = cx1 - cx0
  const cropH = cy1 - cy0
  if (cropW < 2 || cropH < 2) {
    throw new Error('在线 DEM 裁剪范围为空，请缩小范围或调整中心点')
  }

  // 裁剪 → 解码 → 降采样（先解码后降采样，不能在 RGB 编码空间插值）
  const { heights, width, height } = decodeTerrarium(
    ctx.getImageData(cx0, cy0, cropW, cropH),
  )
  const scale = Math.min(1, MAX_GRID / Math.max(cropW, cropH))
  const outCols = Math.max(2, Math.round(cropW * scale))
  const outRows = Math.max(2, Math.round(cropH * scale))
  const elevation = downsample(heights, width, height, outRows, outCols)

  const payload = {
    elevation,
    x_min: minLng,
    y_min: minLat,
    x_max: maxLng,
    y_max: maxLat,
    zoom,
    n_tiles: nTiles * nTiles,
    resolution_m: Math.round(tileGroundResolutionM(zoom, centerLat)),
  }
  if (squareCache.size >= SQUARE_CACHE_MAX) {
    squareCache.delete(squareCache.keys().next().value)
  }
  squareCache.set(key, payload)
  return payload
}
