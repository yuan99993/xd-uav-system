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

    def clear(self):
        self.zones.clear()

    def update_zone(self, z: ZoneDef):
        self.zones[z.zone_id] = z

    def remove_zone(self, zone_id: int):
        self.zones.pop(zone_id, None)

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
