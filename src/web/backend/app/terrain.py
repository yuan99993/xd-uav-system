"""地形读取模块：基于 GDAL 读取 GeoTIFF，输出网格地图 + 地理参考信息。

为什么需要这个模块：
    terrain-navigation 原项目用 C++ 的 grid_map 库存地形。
    我们 Web 后端用 Python 的 GDAL 复刻同样的"取海拔"功能：
        给一个经纬度 → 返回该点海拔高度。
    这样 RRT*/A* 规划时就能判断"飞机高度 < 山的高度 = 撞了"。

对应原项目代码：
    terrain_navigation/src/terrain_map.cpp 的 isInCollision()
"""
from __future__ import annotations

import math
import os
import struct
import zlib
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

M_PER_DEG_LAT = 111320.0  # 1 度纬度 ≈ 111320 米

try:
    from osgeo import gdal, osr
    GDAL_OK = True
except ImportError:
    gdal = None
    osr = None
    GDAL_OK = False


@dataclass
class TerrainMap:
    """加载到内存的地形网格。

    字段含义：
        elevation: 2D 海拔矩阵 (rows, cols)，单位米
        geo_transform: GDAL 仿射变换 [origin_x, dx, 0, origin_y, 0, dy]
                       用来把 (row, col) → (经度/东, 纬度/北)
        wkt: 坐标系 WKT 字符串（用来判断是经纬度还是 UTM 投影）
        rows / cols: 网格尺寸
    """
    elevation: np.ndarray
    geo_transform: Tuple[float, float, float, float, float, float]
    wkt: str
    rows: int
    cols: int

    def sample(self, x: float, y: float) -> Optional[float]:
        """按地理坐标采样海拔。

        Args:
            x: 东向坐标（经度或 UTM 东）
            y: 北向坐标（纬度或 UTM 北）

        Returns:
            该点海拔（米），若超出地图范围返回 None。
        """
        gt = self.geo_transform
        # (x, y) → (col, col) 反推：col = (x - origin_x) / dx
        col = int((x - gt[0]) / gt[1])
        row = int((y - gt[3]) / gt[5])
        if 0 <= row < self.rows and 0 <= col < self.cols:
            val = float(self.elevation[row, col])
            if np.isnan(val):
                return None
            return val
        return None

    def is_collision(self, x: float, y: float, alt: float, margin: float = 30.0) -> bool:
        """判断 (x, y, alt) 是否撞山。

        复刻 terrain_map.cpp isInCollision 的核心逻辑：
            山顶海拔 > 飞机高度 → 撞了

        Args:
            x, y: 水平位置
            alt: 飞机海拔高度（米）
            margin: 安全余量（米），飞机要高于山顶 margin 米才算安全
        """
        elev = self.sample(x, y)
        if elev is None:
            return True  # 出地图算撞（原项目同样处理）
        return (elev + margin) > alt

    def set_bounds(self, x_min: float, y_min: float,
                   x_max: float, y_max: float) -> None:
        """重新指定地形覆盖的地理范围（高度数组不变）。

        用途：用户在网页上"拖动蓝色矩形"，把这块地形平移/缩放
        到地图任意位置。海拔矩阵不变，只改地理参考 geo_transform，
        等于把同一组山头挪到新的经纬度。

        映射与 terrain_info 保持一致（info 用 gt[1]*cols / gt[5]*rows），
        保证拖到哪、下次查询的范围就是哪，不产生来回漂移：
            x_min = gt[0]            y_max = gt[3]
            x_max = gt[0] + gt[1]*cols
            y_min = gt[3] + gt[5]*rows
        """
        if x_max <= x_min or y_max <= y_min:
            raise ValueError("地形范围无效：x_max/y_max 必须大于 x_min/y_min")
        dx = (x_max - x_min) / self.cols
        dy = (y_min - y_max) / self.rows  # 负号：纬度由北向南递减
        self.geo_transform = (float(x_min), float(dx), 0.0,
                              float(y_max), 0.0, float(dy))


    # ------------------------------------------------------------------
    # ENU 局部米制坐标（对应 grid_map 的米制世界坐标）
    #
    # terrain-navigation 全程在局部米制里规划；这里以地形中心为 ENU 原点：
    #   x_east  = (lng - lng0) * 米/度经度
    #   y_north = (lat - lat0) * 米/度纬度
    # 规划器（planner3d）只用 ENU；API 边界（main.py）负责经纬度 ↔ ENU。
    # ------------------------------------------------------------------
    def _m_per_deg_lng(self, lat: float) -> float:
        return M_PER_DEG_LAT * math.cos(math.radians(lat))

    def enu_origin(self) -> Tuple[float, float]:
        """ENU 原点 = 地形中心 (lng0, lat0)。"""
        gt = self.geo_transform
        lng0 = gt[0] + gt[1] * self.cols / 2.0
        lat0 = gt[3] + gt[5] * self.rows / 2.0
        return lng0, lat0

    def to_enu(self, lng: float, lat: float) -> Tuple[float, float]:
        lng0, lat0 = self.enu_origin()
        return ((lng - lng0) * self._m_per_deg_lng(lat0),
                (lat - lat0) * M_PER_DEG_LAT)

    def to_geo(self, x_e: float, y_n: float) -> Tuple[float, float]:
        lng0, lat0 = self.enu_origin()
        return (lng0 + x_e / self._m_per_deg_lng(lat0),
                lat0 + y_n / M_PER_DEG_LAT)

    def enu_bounds(self) -> Tuple[float, float, float, float]:
        """地形 ENU 范围 (x_min, y_min, x_max, y_max)，米。"""
        gt = self.geo_transform
        half_e = abs(gt[1]) * self.cols / 2.0 * self._m_per_deg_lng(gt[3] + gt[5] * self.rows / 2.0)
        half_n = abs(gt[5]) * self.rows / 2.0 * M_PER_DEG_LAT
        return (-half_e, -half_n, half_e, half_n)

    # ------------------------------------------------------------------
    # 碰撞层（复刻 TerrainMap::isInCollision / getCollisionDepth 语义）
    #   distance_surface 层：elevation + margin，is_above=True 碰撞检查
    #   max_elevation 层：  elevation + agl_max，is_above=False 碰撞检查（天花板）
    # ------------------------------------------------------------------
    def at_enu(self, x_e: float, y_n: float) -> Optional[float]:
        """按 ENU 坐标取海拔（米），出图返回 None。"""
        lng, lat = self.to_geo(x_e, y_n)
        return self.sample(lng, lat)

    def is_in_collision(self, x_e: float, y_n: float, z: float,
                        margin: float = 30.0) -> bool:
        """地面层碰撞：出图 或 elevation + margin > z。"""
        elev = self.at_enu(x_e, y_n)
        if elev is None:
            return True  # 出地图算撞（同原项目）
        return (elev + margin) > z

    def above_ceiling(self, x_e: float, y_n: float, z: float,
                      agl_max: float) -> bool:
        """天花板层碰撞：出图 或 z > elevation + agl_max。"""
        elev = self.at_enu(x_e, y_n)
        if elev is None:
            return True
        return z > (elev + agl_max)

    def collision_depth(self, x_e: float, y_n: float, z: float,
                        margin: float = 30.0) -> float:
        """碰撞深度（对应 getCollisionDepth），未碰撞返回 0。"""
        elev = self.at_enu(x_e, y_n)
        if elev is None:
            return 0.0
        return max(0.0, (elev + margin) - z)

    # ------------------------------------------------------------------
    # 地形可视化 / 数据服务
    # ------------------------------------------------------------------
    def _downsample(self, max_size: int) -> np.ndarray:
        """把高程矩阵按整数步长抽稀到最长边 ≤ max_size（避免超大 PNG/JSON）。"""
        step = max(1, int(math.ceil(max(self.rows, self.cols) / max_size)))
        return self.elevation[::step, ::step]

    def hillshade_rgba(self, max_size: int = 2048) -> np.ndarray:
        """生成山体阴影 + 高程着色的 RGBA 图像（行 0 = 最北 = 图片顶部）。

        光照：西北方向、45° 仰角（物理法向量点乘光照方向）。
        NaN（出图/无数据）→ 透明。
        """
        elev = self._downsample(max_size)
        gt = self.geo_transform
        lat0 = gt[3] + gt[5] * self.rows / 2.0
        m_lng = self._m_per_deg_lng(lat0)
        step_lat = abs(gt[5]) * M_PER_DEG_LAT
        step_lng = abs(gt[1]) * m_lng

        filled = np.nan_to_num(elev, nan=float(np.nanmin(elev)))
        # 每米导数（东向 / 北向）
        dzdx = np.gradient(filled, axis=1) / max(step_lng, 1e-6)
        dzdy = np.gradient(filled, axis=0) / max(step_lat, 1e-6) * np.sign(gt[5])
        # 表面法向量（未归一化）
        nx, ny, nz = -dzdx, -dzdy, np.ones_like(dzdx)
        norm = np.sqrt(nx * nx + ny * ny + nz * nz)
        # 光照方向（指向光源）：方位角 315°(西北)、仰角 45°
        az, alt = math.radians(315.0), math.radians(45.0)
        lx = math.sin(az) * math.cos(alt)
        ly = math.cos(az) * math.cos(alt)
        lz = math.sin(alt)
        shade = np.clip((nx * lx + ny * ly + nz * lz) / norm, 0.0, 1.0)

        # 高程着色：绿(低) → 棕(中) → 灰白(高)
        e_min = float(np.nanmin(elev))
        e_max = float(np.nanmax(elev))
        rng = max(e_max - e_min, 1e-6)
        t = np.clip((elev - e_min) / rng, 0.0, 1.0)
        c_low = np.array([70, 110, 70], dtype=np.float32)
        c_mid = np.array([168, 140, 96], dtype=np.float32)
        c_high = np.array([236, 233, 224], dtype=np.float32)
        rgb = np.where(t[..., None] < 0.5,
                       c_low + (c_mid - c_low) * (t[..., None] / 0.5),
                       c_mid + (c_high - c_mid) * ((t[..., None] - 0.5) / 0.5))
        # 阴影调制 + 环境光
        rgb = rgb * (0.30 + 0.70 * shade)[..., None]

        alpha = np.where(np.isnan(elev), 0, 255).astype(np.uint8)
        rgba = np.empty(elev.shape + (4,), dtype=np.uint8)
        rgba[..., :3] = np.clip(rgb, 0, 255).astype(np.uint8)
        rgba[..., 3] = alpha
        return rgba

    def raster_payload(self, max_grid: int = 160) -> dict:
        """给前端 3D 视图的抽稀高程矩阵 + ENU 尺寸信息。"""
        elev = self._downsample(max_grid)
        rows, cols = elev.shape
        x_min, y_min, x_max, y_max = self.enu_bounds()
        lng0, lat0 = self.enu_origin()
        gt = self.geo_transform
        geo_w = gt[0] + gt[1] * self.cols      # 最东经度
        geo_n = gt[3]                          # 最北纬度
        geo_s = gt[3] + gt[5] * self.rows      # 最南纬度
        return {
            "origin": {"lng": lng0, "lat": lat0},
            "width_m": x_max - x_min,
            "height_m": y_max - y_min,
            "rows": int(rows),
            "cols": int(cols),
            "bounds": {"x_min": gt[0], "y_min": geo_s, "x_max": geo_w, "y_max": geo_n},
            "elevation": [
                [None if np.isnan(v) else round(float(v), 1) for v in row]
                for row in elev
            ],
        }

    def profile(self, lng1: float, lat1: float, lng2: float, lat2: float,
                n: int = 120) -> list:
        """两点之间沿直线的高程剖面 [[lng, lat, elev], ...]（elev 可能是 None）。"""
        pts = []
        for i in range(n):
            t = i / (n - 1) if n > 1 else 0.0
            lng = lng1 + (lng2 - lng1) * t
            lat = lat1 + (lat2 - lat1) * t
            pts.append([round(lng, 7), round(lat, 7), self.sample(lng, lat)])
        return pts


def _png_bytes(rgba: np.ndarray) -> bytes:
    """最小 PNG 编码器（RGBA 8bit），不依赖 PIL。"""
    h, w = rgba.shape[:2]
    raw = b"".join(b"\x00" + rgba[i].tobytes() for i in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data +
                struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) +
            chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def load_terrain(path: str) -> TerrainMap:
    """从 GeoTIFF 文件加载地形。

    用法：
        terrain = load_terrain("davosdorf.tif")
        h = terrain.sample(9.8, 46.8)  # 取该点海拔
    """
    if not GDAL_OK:
        raise RuntimeError(
            "GDAL 未安装。macOS 安装：\n"
            "  brew install gdal\n"
            "  然后 pip 安装对应版本的 GDAL 轮子：\n"
            "  pip install GDAL==$(gdal-config --version) "
            "--global-option=build_ext --global-option=\"$(gdal-config --include-dir)\""
        )
    if not os.path.exists(path):
        raise FileNotFoundError(path)

    ds = gdal.Open(path, gdal.GA_ReadOnly)
    if ds is None:
        raise RuntimeError(f"GDAL 打开失败：{path}")

    band = ds.GetRasterBand(1)
    elev = band.ReadAsArray().astype(np.float32)
    # 把 nodata 替换成 NaN，方便后续过滤
    nodata = band.GetNoDataValue()
    if nodata is not None:
        elev[elev == nodata] = np.nan

    gt = ds.GetGeoTransform()
    wkt = ds.GetProjection()
    return TerrainMap(
        elevation=elev,
        geo_transform=gt,
        wkt=wkt,
        rows=elev.shape[0],
        cols=elev.shape[1],
    )


def load_synthetic_terrain(rows: int = 200, cols: int = 200,
                           extent: float = 1000.0,
                           center_lng: float = 116.4,
                           center_lat: float = 39.9) -> TerrainMap:
    """生成合成地形（不依赖 GDAL），映射到真实经纬度。

    用途：
        框架阶段/演示场景，让用户在没有 GDAL 的情况下也能看到规划效果。
        生成一个 ~1000m × 1000m 的地形，模拟几座山。

    坐标系（真实地理坐标，方便叠在 OSM/谷歌底图上）：
        x: 经度（度）
        y: 纬度（度）
        z: 海拔，山头 ~150m，谷地 ~0m
        默认中心在北京附近 (116.4°E, 39.9°N)

    经纬度换算：
        1 度纬度 ≈ 111320 米
        1 度经度 ≈ 111320 × cos(纬度) 米
    """
    import math
    xs = np.linspace(0, extent, cols)
    ys = np.linspace(0, extent, rows)
    X, Y = np.meshgrid(xs, ys)  # shape (rows, cols)
    # 三座山（高斯凸起），位置/高度错开
    peaks = [
        (300, 250, 80, 120),    # (cx, cy, sigma, height)
        (700, 600, 100, 150),
        (500, 800, 70, 90),
    ]
    elev = np.zeros_like(X, dtype=np.float32)
    for cx, cy, sigma, h in peaks:
        elev += h * np.exp(-((X - cx) ** 2 + (Y - cy) ** 2) / (2 * sigma ** 2))
    # 加点噪声让看起来真实
    rng = np.random.default_rng(42)
    elev += rng.normal(0, 2, elev.shape).astype(np.float32)

    # 把 extent（米）换算成经纬度度数
    lat_delta = extent / 111320.0
    lng_delta = extent / (111320.0 * math.cos(math.radians(center_lat)))

    # 仿射变换：[origin_x, dx, 0, origin_y, 0, dy]
    # origin_x = 最西端经度, origin_y = 最北端纬度
    dx = lng_delta / (cols - 1)
    dy = -lat_delta / (rows - 1)  # 负号 = 从北向南递减（GDAL 默认）
    origin_x = center_lng - lng_delta / 2
    origin_y = center_lat + lat_delta / 2
    gt = (float(origin_x), float(dx), 0.0, float(origin_y), 0.0, float(dy))
    return TerrainMap(
        elevation=elev,
        geo_transform=gt,
        wkt="GEOGCS[\"WGS 84\"]",
        rows=rows,
        cols=cols,
    )
