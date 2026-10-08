"""Dubins airplane 移植的数值自测：随机状态对，走完路径后终点误差应接近 0。"""
import math
import random
import sys

sys.path.insert(0, ".")
from app.dubins_airplane import dubins_airplane, walk_path, TWO_PI

random.seed(7)
RHO = 66.66667
GAMMA_MAX = 0.15

worst_xy = worst_z = worst_yaw = 0.0
n_fail = 0
n_none = 0
N = 3000
for i in range(N):
    s1 = (random.uniform(0, 2000), random.uniform(0, 2000),
          random.uniform(0, 300), random.uniform(-math.pi, math.pi))
    s2 = (random.uniform(0, 2000), random.uniform(0, 2000),
          random.uniform(0, 300), random.uniform(-math.pi, math.pi))
    path = dubins_airplane(s1, s2, RHO, GAMMA_MAX)
    if path is None:
        n_none += 1
        continue
    pts = walk_path(path, s1, step=1.0)
    end = pts[-1]
    L = path.length_xy()
    e_xy = math.hypot(end[0] - s2[0], end[1] - s2[1])
    e_z = abs(end[2] - s2[2])
    dyaw = abs((end[3] - s2[3] + math.pi) % TWO_PI - math.pi)
    worst_xy = max(worst_xy, e_xy)
    worst_z = max(worst_z, e_z)
    worst_yaw = max(worst_yaw, dyaw)
    if e_xy > max(0.02, 1e-4 * L) or e_z > 0.05 or dyaw > 1e-3:
        n_fail += 1
        if n_fail <= 5:
            print(f"FAIL s1={s1} s2={s2} word={path.word} gamma={path.gamma:.4f} "
                  f"L={L:.1f} e_xy={e_xy:.4f} e_z={e_z:.4f} e_yaw={dyaw:.5f} "
                  f"lengths={[round(l,1) for l in path.lengths]}")

print(f"tested {N} pairs, none={n_none}, fail={n_fail}")
print(f"worst xy err={worst_xy:.4f}m, z err={worst_z:.4f}m, yaw err={worst_yaw:.6f}rad")
print("PASS" if n_fail == 0 else "FAIL")
