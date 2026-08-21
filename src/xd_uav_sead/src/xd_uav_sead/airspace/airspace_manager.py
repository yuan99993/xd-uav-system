from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional

@dataclass
class ZoneDef:
    zone_id: int
    enabled: bool
    zone_type: int           # 0=NoFly
    level2d: int
    levelH: int
    minAlt: float
    maxAlt: float
    vertices: List[Tuple[float, float]]  # [(E,N), ...]  ENU meters


class AirspaceManager:
    def __init__(self):
        self.zones: Dict[int, ZoneDef] = {}
        self.revision = 0

    def clear(self):
        self.zones.clear()
        self.revision += 1

    def update_zone(self, z: ZoneDef):
        self.zones[z.zone_id] = z
        self.revision += 1

    def remove_zone(self, zone_id: int):
        self.zones.pop(zone_id, None)
        self.revision += 1

     # ---------- V1: polygon + alt 判定（先跑通） ----------
    def is_in_nofly(self, e: float, n: float, alt: float) -> bool:
        for z in self.zones.values():
            if not z.enabled:
                continue
            if z.zone_type != 0:   # 0=NoFly
                continue
            if alt < z.minAlt or alt > z.maxAlt:
                continue
            if self._point_in_poly(e, n, z.vertices):  # vertices: [(E,N)]
                return True
        return False

    @staticmethod
    def _point_in_poly(x: float, y: float, poly):
        if len(poly) < 3:
            return False
        for i in range(len(poly)):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % len(poly)]
            vx, vy = bx - ax, by - ay
            wx, wy = x - ax, y - ay
            cross = vx * wy - vy * wx
            if abs(cross) <= 1e-9:
                dot = wx * vx + wy * vy
                if -1e-9 <= dot <= vx * vx + vy * vy + 1e-9:
                    return True
        # poly: [(x,y)] = [(E,N)]
        inside = False
        j = len(poly) - 1
        for i in range(len(poly)):
            xi, yi = poly[i]
            xj, yj = poly[j]
            # standard ray casting
            intersect = ((yi > y) != (yj > y)) and \
                        (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
            if intersect:
                inside = not inside
            j = i
        return inside

    @staticmethod
    def _point_to_segment_distance(px, py, ax, ay, bx, by):
        vx, vy = bx - ax, by - ay
        denom = vx * vx + vy * vy
        if denom <= 1e-12:
            return ((px - ax) ** 2 + (py - ay) ** 2) ** 0.5
        t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / denom))
        return ((px - (ax + t * vx)) ** 2 + (py - (ay + t * vy)) ** 2) ** 0.5

    def min_horizontal_clearance(self, e: float, n: float, alt: float) -> float:
        best = float("inf")
        for z in self.zones.values():
            if (
                not z.enabled
                or z.zone_type != 0
                or alt < z.minAlt
                or alt > z.maxAlt
                or len(z.vertices) < 3
            ):
                continue
            if self._point_in_poly(e, n, z.vertices):
                return 0.0
            for i, a in enumerate(z.vertices):
                b = z.vertices[(i + 1) % len(z.vertices)]
                best = min(
                    best,
                    self._point_to_segment_distance(e, n, a[0], a[1], b[0], b[1]),
                )
        return best

    def path_min_horizontal_clearance(self, path, alt: float) -> float:
        best = float("inf")
        for point in path or []:
            if point is None or len(point) < 2:
                continue
            best = min(
                best,
                self.min_horizontal_clearance(float(point[0]), float(point[1]), alt),
            )
        return best

    # ---------- V2: Keys3D / GeoSOT3D（后续增强接口） ----------
    def build_keys3d_for_zone(self, z: ZoneDef):
        """
        TODO:
          - 按 level2d 扫 tile（bbox 剪枝 + polygon 过滤）
          - 按 levelH 扫高度层
          - 生成 key3d: "code2d|hcode"
        """
        raise NotImplementedError
    
    
    def export_zones_for_planner(self):
        out = []
        for z in self.zones.values():
            if not z.enabled: 
                continue
            if z.zone_type != 0:
                continue
            out.append({
                "zone_id": z.zone_id,
                "minAlt": z.minAlt,
                "maxAlt": z.maxAlt,
                "poly": [(float(e), float(n)) for (e, n) in z.vertices],
            })
        return out
