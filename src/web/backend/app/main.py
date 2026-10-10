"""FastAPI 后端入口：Web 指挥控制中心 - 地形规划 + 路径下发。

启动：
    cd backend
    pip install -r requirements.txt
    uvicorn app.main:app --reload --port 8000

接口总览：
    GET  /api/health            健康检查
    POST /api/terrain/upload    上传 GeoTIFF 地形文件
    GET  /api/terrain/info      查询当前地形信息
    POST /api/plan              规划路径（起点、终点、可选禁飞区）
    POST /api/dispatch          把规划好的路径下发到机载 ROS
"""
from __future__ import annotations

import json
import logging
import math
import os
import urllib.error
import urllib.request
from urllib.parse import urlencode
from pathlib import Path
from typing import List, Optional

from fastapi import FastAPI, File, HTTPException, Query, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, Response
from pydantic import BaseModel, Field

from .planner import PathSegment, TerrainAStarPlanner
from .planner3d import Plan3DParams, TerrainRRTStar3D
from .ros_bridge import get_bridge
from .terrain import GDAL_OK, TerrainMap, _png_bytes, load_synthetic_terrain, load_terrain
from .terrain_mission import (
    activate_terrain,
    active_manifest,
    configure_projection_runtime,
    wgs84_wkt,
)
from .terrain_resource import CANONICAL_DAVOSDORF_DEM

import numpy as np

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
logger = logging.getLogger(__name__)

# ----- 路径常量 -----
BASE_DIR = Path(__file__).resolve().parent.parent
DATA_DIR = BASE_DIR / "data"
DATA_DIR.mkdir(exist_ok=True)
SAMPLE_TIF = CANONICAL_DAVOSDORF_DEM

# ----- 全局状态：当前加载的地形（一个时刻只有一个，简化框架） -----
_current_terrain: Optional[TerrainMap] = None
_current_terrain_name: Optional[str] = None
_current_terrain_meta: dict = {}  # 在线地形附加信息（zoom、瓦片数等）


# ----- 数据模型 -----
class Point2D(BaseModel):
    x: float = Field(..., description="东向坐标（经度或 UTM 东）")
    y: float = Field(..., description="北向坐标（纬度或 UTM 北）")


class NoFlyZone(BaseModel):
    """禁飞区：水平面上的多边形"""
    points: List[Point2D] = Field(..., description="多边形顶点（顺时针或逆时针）")


class PlanRequest(BaseModel):
    start: Point2D
    goal: Point2D
    cruise_altitude: float = Field(100.0, description="巡航高度（米，海拔）")
    safety_margin: float = Field(30.0, description="安全余量（米）")
    nofly_zones: List[NoFlyZone] = Field(default_factory=list, description="禁飞区列表")


class RelocateRequest(BaseModel):
    """拖动蓝色地形矩形后，回传的新地理范围"""
    x_min: float = Field(..., description="最西端经度")
    y_min: float = Field(..., description="最南端纬度")
    x_max: float = Field(..., description="最东端经度")
    y_max: float = Field(..., description="最北端纬度")


class LoadGridRequest(BaseModel):
    """在线 DEM 瓦片（Terrarium）在前端解码后回传的规则高程网格。

    行 0 = 最北、列 0 = 最西（与 GDAL geo_transform 约定一致），
    后端据此直接构造 TerrainMap，无需 GDAL。
    """
    elevation: List[List[Optional[float]]] = Field(..., description="rows×cols 高程矩阵（米），null=无数据")
    x_min: float = Field(..., description="网格最西端经度")
    y_min: float = Field(..., description="网格最南端纬度")
    x_max: float = Field(..., description="网格最东端经度")
    y_max: float = Field(..., description="网格最北端纬度")
    source: str = Field("online_terrarium", description="数据来源标签")
    zoom: int = Field(0, description="DEM 瓦片层级")
    n_tiles: int = Field(0, description="瓦片张数")
    vertical_datum: str = Field("MSL", pattern="^MSL$", description="正高基准")


class SegmentOut(BaseModel):
    segment_start: List[float]
    segment_tangent: List[float]
    curvature: float
    segment_length: float
    is_periodic: bool
    reached: bool


class PlanResponse(BaseModel):
    status: str
    n_segments: int
    length: float
    waypoints: List[List[float]]
    segments: List[SegmentOut]


class DispatchRequest(BaseModel):
    topic: str = Field("/terrain_path", description="机载 ROS topic 名")
    segments: List[SegmentOut]
    use_mock: bool = Field(True, description="是否用假桥接（框架阶段=True）")


class DispatchResponse(BaseModel):
    status: str
    topic: str
    n_segments: int


class LonLatAlt(BaseModel):
    lng: float = Field(..., description="经度（度）")
    lat: float = Field(..., description="纬度（度）")
    alt: Optional[float] = Field(None, description="海拔（米）；省略时自动 = 地形高程 + agl_min")
    heading: Optional[float] = Field(None, description="航向角（度，0=北，顺时针）；省略时自动朝向终点")


class Plan3DRequest(BaseModel):
    """3D 规划请求：经纬度出入口，内部转 ENU 米制。"""
    start: LonLatAlt
    goal: LonLatAlt
    agl_min: float = Field(50.0, description="离地高度下限（米）")
    agl_max: float = Field(150.0, description="离地高度上限/天花板（米）")
    turning_radius: float = Field(66.67, description="最小转弯半径（米）")
    climb_angle_deg: float = Field(10.0, description="最大爬升/俯冲角（度）")
    safety_margin: float = Field(30.0, description="地面安全余量（米）")
    time_budget: float = Field(3.0, description="RRT* 规划时间预算（秒）")
    step: float = Field(250.0, description="RRT* 单步最大扩展长度（米）")
    goal_bias: float = Field(0.10, description="朝目标采样的概率 0~1")
    loiter: str = Field("ccw", description="终点盘旋方向：ccw | cw | none")
    nofly_zones: List[NoFlyZone] = Field(default_factory=list, description="禁飞区（经纬度多边形）")


class Plan3DResponse(BaseModel):
    status: str
    length: float                # 3D 总长（米）
    iterations: int
    plan_time: float
    solution_cost: float
    waypoints: List[List[float]]  # [lng, lat, alt_msl]
    segments: List[SegmentOut]


# ----- FastAPI -----
app = FastAPI(title="Terrain Navigation Web - 指挥控制中心",
             description="基于地形的全局路径规划 + 下发机载 ROS")

# 允许前端跨域访问（Vite 默认 5173）
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# 兜底：任何未捕获异常都返回 JSON（默认是纯文本 "Internal Server Error"，
# 前端 r.json() 会报 "Unexpected token 'I'... is not valid JSON"）
@app.exception_handler(Exception)
async def unhandled_exception_handler(request, exc):
    logger.exception("未处理异常：%s %s", request.method, request.url.path)
    return JSONResponse(
        status_code=500,
        content={"detail": f"服务器内部错误：{exc}"},
    )


@app.get("/api/health")
def health():
    return {"status": "ok", "terrain_loaded": _current_terrain is not None}


@app.post("/api/terrain/upload")
async def upload_terrain(
        file: UploadFile = File(...),
        vertical_datum: str = Query(..., pattern="^MSL$")):
    """上传 GeoTIFF 文件并加载"""
    global _current_terrain, _current_terrain_name, _current_terrain_meta
    filename = Path(file.filename or "").name
    if not filename or filename != file.filename:
        raise HTTPException(400, "地形文件名无效")
    if not filename.lower().endswith((".tif", ".tiff")):
        raise HTTPException(400, "仅支持 .tif/.tiff")
    save_path = DATA_DIR / filename
    with open(save_path, "wb") as f:
        f.write(await file.read())
    configure_projection_runtime()
    _current_terrain = load_terrain(str(save_path))
    _current_terrain_name = filename
    _current_terrain_meta = {
        "source": "uploaded_geotiff",
        "original_filename": filename,
        "vertical_datum": vertical_datum,
    }
    logger.info("地形已加载：%s (%dx%d)", filename,
                _current_terrain.rows, _current_terrain.cols)
    return {"status": "ok", "filename": filename,
            "rows": _current_terrain.rows, "cols": _current_terrain.cols}


@app.post("/api/terrain/load_sample")
def load_sample():
    """加载内置示例地形 davosdorf.tif；GDAL 未装时退化为合成地形"""
    global _current_terrain, _current_terrain_name, _current_terrain_meta
    if GDAL_OK and SAMPLE_TIF.exists():
        configure_projection_runtime()
        _current_terrain = load_terrain(str(SAMPLE_TIF))
        _current_terrain_name = SAMPLE_TIF.name
        _current_terrain_meta = {
            "source": "tracked_sample_geotiff", "vertical_datum": "MSL"}
    else:
        # 没装 GDAL → 合成地形（让用户能立刻看到规划效果）
        _current_terrain = load_synthetic_terrain()
        _current_terrain_name = "synthetic_terrain(无GDAL).tif"
        _current_terrain_meta = {
            "source": "synthetic_preview_only", "vertical_datum": "MSL"}
    return {"status": "ok", "filename": _current_terrain_name,
            "rows": _current_terrain.rows, "cols": _current_terrain.cols}


@app.get("/api/terrain/info")
def terrain_info():
    if _current_terrain is None:
        raise HTTPException(404, "尚未加载地形，先 POST /api/terrain/load_sample")
    gt = _current_terrain.geo_transform
    x_min = gt[0]
    y_min = gt[3] + gt[5] * _current_terrain.rows
    x_max = gt[0] + gt[1] * _current_terrain.cols
    y_max = gt[3]
    active = None
    try:
        active = active_manifest()
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError):
        pass
    return {
        "filename": _current_terrain_name,
        "rows": _current_terrain.rows,
        "cols": _current_terrain.cols,
        "bounds": {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max},
        "elevation_min": float(np.nanmin(_current_terrain.elevation)),
        "elevation_max": float(np.nanmax(_current_terrain.elevation)),
        "terrain_meta": _current_terrain_meta,
        "active_terrain_id": active.get("terrain_id") if active else None,
    }


@app.post("/api/terrain/activate")
def activate_current_terrain():
    """Create the formal GeoTIFF package consumed by mission control and ROS."""
    if _current_terrain is None:
        raise HTTPException(400, "尚未加载地形，先加载")
    if _current_terrain_meta.get("source") == "synthetic_preview_only":
        raise HTTPException(400, "合成预览地形不能作为 ROS/Gazebo 正式地形")
    try:
        manifest = activate_terrain(
            _current_terrain, _current_terrain_name or "terrain",
            _current_terrain_meta)
    except (OSError, RuntimeError, ValueError) as error:
        raise HTTPException(400, str(error)) from error
    return {"status": "ok", "terrain": manifest}


@app.get("/api/terrain/overlay.png")
def terrain_overlay(max_size: int = 2048):
    """山体阴影 + 高程着色 PNG，叠加到底图上。

    图片范围 = /api/terrain/info 的 bounds；前端用 Leaflet ImageOverlay 贴图。
    """
    if _current_terrain is None:
        raise HTTPException(404, "尚未加载地形，先 POST /api/terrain/load_sample")
    rgba = _current_terrain.hillshade_rgba(max_size=max(64, min(max_size, 4096)))
    png = _png_bytes(rgba)
    gt = _current_terrain.geo_transform
    bounds = {
        "x_min": gt[0],
        "y_min": gt[3] + gt[5] * _current_terrain.rows,
        "x_max": gt[0] + gt[1] * _current_terrain.cols,
        "y_max": gt[3],
    }
    return Response(
        content=png,
        media_type="image/png",
        headers={
            # 让前端不用再发一次 info 就能定位图片
            "X-Bounds-West": str(bounds["x_min"]),
            "X-Bounds-East": str(bounds["x_max"]),
            "X-Bounds-South": str(bounds["y_min"]),
            "X-Bounds-North": str(bounds["y_max"]),
            "Cache-Control": "no-cache",
        },
    )


@app.get("/api/terrain/raster")
def terrain_raster(max_grid: int = 160):
    """3D 视图数据：抽稀高程矩阵 + ENU 尺寸 + ENU 原点经纬度。"""
    if _current_terrain is None:
        raise HTTPException(404, "尚未加载地形，先 POST /api/terrain/load_sample")
    return _current_terrain.raster_payload(max_grid=max(32, min(max_grid, 400)))


@app.post("/api/terrain/relocate")
def relocate(req: RelocateRequest):
    """把地形块移动/缩放到新的地理范围（网页上拖动蓝色矩形后调用）。

    海拔高度矩阵不变，只更新地理参考，等于把同一组山头搬到新位置。
    返回更新后的 terrain info（和 GET /api/terrain/info 结构一致）。
    """
    if _current_terrain is None:
        raise HTTPException(400, "尚未加载地形，先加载")
    if not (-180 <= req.x_min < req.x_max <= 180 and
            -90 <= req.y_min < req.y_max <= 90):
        raise HTTPException(400, "地形范围非法：经度需在 -180~180、纬度在 -90~90 内")
    try:
        _current_terrain.set_bounds(req.x_min, req.y_min, req.x_max, req.y_max)
    except ValueError as e:
        raise HTTPException(400, str(e))
    logger.info("地形范围已移动到：lng %.5f~%.5f, lat %.5f~%.5f",
                req.x_min, req.y_min, req.x_max, req.y_max)
    return terrain_info()


# ------------------------------------------------------------------
# 在线滚动地形：DEM 瓦片代理 + 高程网格装载
#
# 数据源 Mapzen Terrarium（AWS 开放数据集，全球覆盖，RGB 编码高程）：
#     height = r*256 + g + b/256 - 32768   （米，海平面 RGB≈(128,0,0)）
# 浏览器统一走同源 /api/terrain/tile，由后端去 S3 取，规避浏览器
# CORS 与用户网络环境差异；瓦片由前端 canvas 解码后再回传后端。
# ------------------------------------------------------------------
TERRARIUM_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"


@app.get("/api/terrain/tile")
def terrain_tile(z: int, x: int, y: int):
    """Terrarium 高程瓦片同源代理（透传 PNG，浏览器无需跨域）。"""
    if not (0 <= z <= 15):
        raise HTTPException(400, "瓦片层级 z 需在 0~15")
    n = 2 ** z
    if not (0 <= x < n and 0 <= y < n):
        raise HTTPException(400, f"瓦片坐标越界：z={z} 时 x/y 应在 [0,{n})")
    url = TERRARIUM_URL.format(z=z, x=x, y=y)

    def fetch(opener: urllib.request.OpenerDirector) -> bytes:
        req = urllib.request.Request(url, headers={"User-Agent": "terrain-nav-web/1.0"})
        with opener.open(req, timeout=12) as resp:
            return resp.read()

    # 本机直连优先（实测可达）；若直连失败且环境里配了代理，再用环境代理回退
    try:
        data = fetch(urllib.request.build_opener(urllib.request.ProxyHandler({})))
    except (urllib.error.URLError, TimeoutError, OSError) as e1:
        proxy_url = (os.environ.get("HTTPS_PROXY") or os.environ.get("https_proxy")
                     or os.environ.get("HTTP_PROXY") or os.environ.get("http_proxy"))
        if not proxy_url:
            raise HTTPException(502, f"高程瓦片下载失败（直连）：{e1}")
        try:
            data = fetch(urllib.request.build_opener(
                urllib.request.ProxyHandler({"http": proxy_url, "https": proxy_url})))
        except (urllib.error.URLError, TimeoutError, OSError) as e2:
            raise HTTPException(502, f"高程瓦片下载失败（直连/代理均失败）：{e1}；{e2}")

    if len(data) < 64 or data[:8] != b"\x89PNG\r\n\x1a\n":
        raise HTTPException(502, "高程瓦片响应不是有效 PNG")
    return Response(
        content=data,
        media_type="image/png",
        headers={"Cache-Control": "public, max-age=86400"},  # DEM 是静态数据，可缓存一天
    )


@app.post("/api/terrain/load_grid")
def load_grid(req: LoadGridRequest):
    """接收前端从在线瓦片解码出的高程网格，直接替换当前地形（无需 GDAL）。

    前端拖动/缩放地图 → 视野内瓦片 → 解码降采样 → 调本接口；
    成功后即可像示例地形一样做 2D/3D 规划。
    """
    global _current_terrain, _current_terrain_name, _current_terrain_meta
    if not req.elevation or not req.elevation[0]:
        raise HTTPException(400, "高程矩阵为空")
    if not (-180 <= req.x_min < req.x_max <= 180 and
            -90 <= req.y_min < req.y_max <= 90):
        raise HTTPException(400, "地形范围非法：经度 -180~180、纬度 -90~90，且 min<max")
    rows = len(req.elevation)
    cols = len(req.elevation[0])
    if any(len(r) != cols for r in req.elevation):
        raise HTTPException(400, "高程矩阵各行长度不一致")

    elev = np.array(req.elevation, dtype=np.float32)  # None → np.nan
    if int(np.isnan(elev).sum()) >= elev.size:
        raise HTTPException(400, "高程矩阵全部为无数据点（该区域可能无 DEM 覆盖）")

    dx = (req.x_max - req.x_min) / cols
    dy = (req.y_min - req.y_max) / rows  # 负号：行 0 在最北，纬度向南递减
    gt = (float(req.x_min), float(dx), 0.0, float(req.y_max), 0.0, float(dy))
    _current_terrain = TerrainMap(
        elevation=elev, geo_transform=gt, wkt=wgs84_wkt(),
        rows=rows, cols=cols,
    )
    _current_terrain_meta = {
        "source": req.source, "zoom": req.zoom, "n_tiles": req.n_tiles,
        "online": True, "vertical_datum": req.vertical_datum,
    }
    _current_terrain_name = (
        f"在线真实地形 z{req.zoom}（{req.n_tiles} 瓦片, "
        f"{req.x_min:.4f},{req.y_min:.4f}~{req.x_max:.4f},{req.y_max:.4f}）"
    )
    logger.info("在线地形已装载：z=%d, %d 瓦片, %dx%d, 海拔 %.0f~%.0fm",
                req.zoom, req.n_tiles, rows, cols,
                float(np.nanmin(elev)), float(np.nanmax(elev)))
    return terrain_info()


# ------------------------------------------------------------------
# 地名搜索：代理 OpenStreetMap Nominatim（免费、无需 key）
# 浏览器不直连，统一走后端：规避 CORS 与网络环境差异。
# ------------------------------------------------------------------
@app.get("/api/geocode")
def geocode(q: str, limit: int = 5, country: str = "cn"):
    """地名 → 经纬度候选列表。q 为地名（中英文均可）。"""
    q = (q or "").strip()
    if not q:
        raise HTTPException(400, "查询内容为空")
    params = {
        "q": q,
        "format": "json",
        "limit": str(max(1, min(limit, 10))),
        "addressdetails": "1",
    }
    if country:
        params["countrycodes"] = country  # 默认偏向中国结果；传 country="" 搜全球
    url = "https://nominatim.openstreetmap.org/search?" + urlencode(params)

    def fetch(opener: urllib.request.OpenerDirector) -> bytes:
        req = urllib.request.Request(url, headers={"User-Agent": "terrain-nav-web/1.0"})
        with opener.open(req, timeout=10) as resp:
            return resp.read()

    try:
        raw = fetch(urllib.request.build_opener(urllib.request.ProxyHandler({})))
    except (urllib.error.URLError, TimeoutError, OSError) as e1:
        proxy_url = (os.environ.get("HTTPS_PROXY") or os.environ.get("https_proxy")
                     or os.environ.get("HTTP_PROXY") or os.environ.get("http_proxy"))
        if not proxy_url:
            raise HTTPException(502, f"地名搜索失败：{e1}")
        try:
            raw = fetch(urllib.request.build_opener(
                urllib.request.ProxyHandler({"http": proxy_url, "https": proxy_url})))
        except (urllib.error.URLError, TimeoutError, OSError) as e2:
            raise HTTPException(502, f"地名搜索失败（直连/代理均失败）：{e1}；{e2}")

    try:
        items = json.loads(raw.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        raise HTTPException(502, "地名搜索响应解析失败")

    results = []
    for it in items[:limit]:
        try:
            results.append({
                "name": it.get("display_name", ""),
                "lat": float(it["lat"]),
                "lng": float(it["lon"]),
                "type": it.get("type", ""),
            })
        except (KeyError, ValueError, TypeError):
            continue
    return {"query": q, "results": results}


@app.post("/api/plan", response_model=PlanResponse)
def plan(req: PlanRequest):
    """A* 规划：起点→终点，避开地形 + 禁飞区"""
    if _current_terrain is None:
        raise HTTPException(400, "尚未加载地形，先加载")
    planner = TerrainAStarPlanner(
        _current_terrain,
        cruise_altitude=req.cruise_altitude,
        safety_margin=req.safety_margin,
    )
    # 把禁飞区多边形传给规划器（Point2D → (x,y) 元组）
    nofly_polygons = [
        [(p.x, p.y) for p in zone.points] for zone in req.nofly_zones
    ]
    try:
        result = planner.plan((req.start.x, req.start.y),
                              (req.goal.x, req.goal.y),
                              nofly_zones=nofly_polygons)
    except ValueError as e:
        # 起点/终点超出地形范围：返回 400 + 范围信息，提示用户点在蓝色矩形内
        gt = _current_terrain.geo_transform
        raise HTTPException(
            400,
            f"{e}。请把起点/终点设在蓝色地形矩形内 "
            f"(经度 {gt[0]:.5f}~{gt[0] + gt[1]*_current_terrain.cols:.5f}, "
            f"纬度 {gt[3] + gt[5]*_current_terrain.rows:.5f}~{gt[3]:.5f})",
        )
    if result is None:
        raise HTTPException(404, "规划失败：找不到安全路径（起点/终点可能落在山头，或被禁飞区封死，可降低巡航高度要求或换个点重试）")

    seg_out = [
        SegmentOut(
            segment_start=list(s.segment_start),
            segment_tangent=list(s.segment_tangent),
            curvature=s.curvature,
            segment_length=s.segment_length,
            is_periodic=s.is_periodic,
            reached=s.reached,
        ) for s in result.segments
    ]
    return PlanResponse(
        status="ok",
        n_segments=len(seg_out),
        length=result.length,
        waypoints=[list(w) for w in result.waypoints],
        segments=seg_out,
    )


@app.post("/api/plan3d", response_model=Plan3DResponse)
def plan3d(req: Plan3DRequest):
    """3D 规划（对齐 terrain-navigation TerrainOmplRrt）：

    1. 经纬度 → ENU 米制；起终点海拔缺省时贴地
    2. RRT* + Dubins Airplane 在地形约束下搜索可飞路径
    3. 求解后高度重排：贴地飞行 + 爬升率限制
    4. 输出 [lng, lat, alt] 航点 + PathSegment 段列表（可下发机载）
    """
    if _current_terrain is None:
        raise HTTPException(400, "尚未加载地形，先加载")
    terrain = _current_terrain

    if req.loiter not in ("ccw", "cw", "none"):
        raise HTTPException(400, "loiter 只能是 ccw / cw / none")

    # 经纬度 → ENU（规划全程在米制空间）
    sx, sy = terrain.to_enu(req.start.lng, req.start.lat)
    gx, gy = terrain.to_enu(req.goal.lng, req.goal.lat)
    x_min, y_min, x_max, y_max = terrain.enu_bounds()
    if not (x_min <= sx <= x_max and y_min <= sy <= y_max):
        raise HTTPException(400, "起点在地形范围外，请把起点设在蓝色地形矩形内")
    if not (x_min <= gx <= x_max and y_min <= gy <= y_max):
        raise HTTPException(400, "终点在地形范围外，请把终点设在蓝色地形矩形内")

    params = Plan3DParams(
        agl_min=req.agl_min,
        agl_max=req.agl_max,
        turning_radius=req.turning_radius,
        climb_angle_deg=req.climb_angle_deg,
        safety_margin=req.safety_margin,
        time_budget=req.time_budget,
        step=req.step,
        goal_bias=req.goal_bias,
        loiter=req.loiter,
        nofly_zones=[[(p.x, p.y) for p in zone.points] for zone in req.nofly_zones],
    )

    # 起点海拔：显式指定 > 地形高程 + agl_min；航向：显式指定 > 指向终点
    start_elev = terrain.at_enu(sx, sy)
    if start_elev is None:
        raise HTTPException(400, "起点采样海拔失败")
    start_z = req.start.alt if req.start.alt is not None else start_elev + req.agl_min
    if req.start.heading is not None:
        yaw = math.radians(req.start.heading)
    else:
        yaw = math.atan2(gy - sy, gx - sx)
    start_state = (sx, sy, start_z, yaw)

    planner = TerrainRRTStar3D(terrain, params)
    try:
        result = planner.solve(start_state, (gx, gy))
    except ValueError as e:
        raise HTTPException(400, str(e))
    if result is None:
        raise HTTPException(
            404,
            "规划失败：时间预算内未找到连接目标的可飞路径"
            "（可尝试增大 time_budget / step，或放宽 agl_min/safety_margin）",
        )

    seg_out = [
        SegmentOut(
            segment_start=s["segment_start"],
            segment_tangent=s["segment_tangent"],
            curvature=s["curvature"],
            segment_length=s["segment_length"],
            is_periodic=s["is_periodic"],
            reached=s["reached"],
        ) for s in result.segments
    ]
    logger.info("3D 规划完成：cost=%.0fm, length=%.0fm, iter=%d, %.2fs",
                result.solution_cost, result.length, result.iterations, result.plan_time)
    return Plan3DResponse(
        status="ok",
        length=result.length,
        iterations=result.iterations,
        plan_time=result.plan_time,
        solution_cost=result.solution_cost,
        waypoints=[[w[0], w[1], w[2]] for w in result.waypoints],
        segments=seg_out,
    )


@app.post("/api/dispatch", response_model=DispatchResponse)
def dispatch(req: DispatchRequest):
    """把路径下发到机载 ROS（框架阶段用假桥接）"""
    if not req.segments:
        raise HTTPException(400, "无路径段可下发")
    # 转回内部 PathSegment
    segs = [
        PathSegment(
            segment_start=tuple(s.segment_start),
            segment_tangent=tuple(s.segment_tangent),
            curvature=s.curvature,
            segment_length=s.segment_length,
            is_periodic=s.is_periodic,
            reached=s.reached,
        ) for s in req.segments
    ]
    bridge = get_bridge(use_mock=req.use_mock)
    result = bridge.send_path(req.topic, segs)
    return DispatchResponse(status=str(result.get("status", "unknown")),
                             topic=result.get("topic", req.topic),
                             n_segments=result.get("n_segments", len(segs)))
