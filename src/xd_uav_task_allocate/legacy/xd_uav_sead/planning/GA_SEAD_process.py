import random
import time
import numpy as np
from matplotlib import pyplot as plt
import dubins
import rospy
import math
import dubins

# [新增/修复] 几何与规划工具函数区域
# 请将此部分替换原文件中 class GA_SEAD 之前的所有函数


# 1. 基础几何函数
def _seg_intersect(a, b, c, d):
    # segment ab intersects cd
    def cross(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    def onseg(p, q, r):
        return min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and min(p[1], r[1]) <= q[
            1
        ] <= max(p[1], r[1])

    o1 = cross(a, b, c)
    o2 = cross(a, b, d)
    o3 = cross(c, d, a)
    o4 = cross(c, d, b)
    if (
        (o1 == 0 and onseg(a, c, b))
        or (o2 == 0 and onseg(a, d, b))
        or (o3 == 0 and onseg(c, a, d))
        or (o4 == 0 and onseg(c, b, d))
    ):
        return True
    return (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)


def _point_in_poly(x, y, poly):
    # ray casting
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i][0], poly[i][1]
        xj, yj = poly[j][0], poly[j][1]
        if ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
        ):
            inside = not inside
        j = i
    return inside


def _seg_hits_poly(p0, p1, poly):
    # endpoint inside OR intersects any edge
    if _point_in_poly(p0[0], p0[1], poly) or _point_in_poly(p1[0], p1[1], poly):
        return True
    n = len(poly)
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        if _seg_intersect(p0, p1, a, b):
            return True
    return False


# 2. 避障核心工具


def _get_expanded_vertices(poly, margins):
    """
    获取多边形外扩顶点。
    margins: 可以是一个数值，也可以是一个数值列表（生成多层外扩圈）。
    """
    if not poly:
        return []
    # 如果传入的是单个数值，转为列表
    if not isinstance(margins, list):
        margins = [margins]

    center = np.mean(poly, axis=0)
    candidates = []

    for m in margins:
        for p in poly:
            vec = np.array([p[0] - center[0], p[1] - center[1]])
            norm = np.linalg.norm(vec)
            if norm > 0:
                # 向外延伸 margin 距离
                new_p = np.array(p) + (vec / norm) * m
                candidates.append(new_p)
    return candidates


def _is_dubins_blocked(sp, gp, r_min, zones, step=5.0):
    """检测 Dubins 路径是否撞墙 (比直线检测更准)"""
    try:
        path = dubins.shortest_path(sp, gp, r_min)
        configs, _ = path.sample_many(step)
        for pt in configs:
            x, y = pt[0], pt[1]
            for z in zones:
                poly = z.get("poly", [])
                if len(poly) < 3:
                    continue
                if _point_in_poly(x, y, poly):
                    return True
        return False
    except:
        return True


def _find_best_bypass(sp, gp, r_min, zones, uav_v=None):
    """
    [补回的函数] 用于 GA 计算 Cost 时估算绕行代价
    """
    # 1. 如果直飞没问题，代价为0
    if not _is_dubins_blocked(sp, gp, r_min, zones):
        return None, 0.0

    # 2. 找出阻挡的障碍物
    blocked_zones = []
    for z in zones:
        if _is_dubins_blocked(sp, gp, r_min, [z]):
            poly = z.get("poly", [])
            center = np.mean(poly, axis=0)
            dist = np.linalg.norm(np.array(sp[:2]) - center)
            blocked_zones.append((dist, z))

    if not blocked_zones:
        return None, float("inf")

    # 3. 针对最近的障碍物，尝试多层外扩寻找估算点
    blocked_zones.sort(key=lambda x: x[0])
    target_zone = blocked_zones[0][1]

    # 使用与 planner 一致的多层参数
    margins = [r_min * 2.0 + 50.0, r_min * 4.0 + 100.0]
    candidates = _get_expanded_vertices(target_zone.get("poly", []), margins)

    best_cost = float("inf")

    # 4. 快速评估
    for cand in candidates:
        # 简单检查点是否在障碍物内
        cand_inside = False
        for z in zones:
            if _point_in_poly(cand[0], cand[1], z.get("poly", [])):
                cand_inside = True
                break
        if cand_inside:
            continue

        # 估算代价
        try:
            h = math.atan2(gp[1] - cand[1], gp[0] - cand[0])
            wp = (cand[0], cand[1], h)

            c1 = dubins.shortest_path(sp, wp, r_min).path_length()
            c2 = dubins.shortest_path(wp, gp, r_min).path_length()
            total = c1 + c2

            if total < best_cost:
                best_cost = total
        except:
            pass

    return None, best_cost


# [GA_SEAD_process.py] -> 替换 plan_path_with_avoidance

# [GA_SEAD_process.py] -> 替换 plan_path_with_avoidance


def plan_path_with_avoidance(sp, gp, Rmin, zones, uav_v, sampling_step=5.0):
    """
    【增强旗舰版 V4】迭代式多重避障规划 - 流线型防绕圈版
    解决：
    1. 彻底解决"迂回绕圈"：引入平分角航向，确保路径切向顺滑。
    2. 狭缝逃逸：自动走外圈。
    3. 远程直飞支持：配合 DPGA 的直飞逻辑，生成高质量直线段。
    """
    full_sampled_points = []
    curr_sp = sp
    waypoints = [sp]
    max_iterations = 20

    # 基础采样航向
    base_headings = np.linspace(-np.pi, np.pi, 8, endpoint=False)

    for _ in range(max_iterations):
        # 1. 检查当前点到终点是否通畅
        blocked_zones = []
        for z in zones:
            if _is_dubins_blocked(curr_sp, gp, Rmin, [z], step=sampling_step):
                poly = z.get("poly", [])
                if len(poly) < 3:
                    continue
                center = np.mean(poly, axis=0)
                dist = np.linalg.norm(np.array(curr_sp[:2]) - center)
                blocked_zones.append((dist, z))

        if not blocked_zones:
            break  # 路通了

        blocked_zones.sort(key=lambda x: x[0])
        found_valid_wp = False

        # 尝试前 3 个阻挡物
        for _, target_zone in blocked_zones[:3]:
            poly = target_zone.get("poly", [])

            # 多层外扩：增加一个超大圈用于包围绕行
            margins = [Rmin * 2.0 + 50.0, Rmin * 4.0 + 100.0, Rmin * 8.0 + 200.0]
            candidates_pos = _get_expanded_vertices(poly, margins)

            candidate_options = []

            for cand in candidates_pos:
                # [前置检查] 候选点不能在禁飞区内
                cand_inside = False
                for z_check in zones:
                    if _point_in_poly(cand[0], cand[1], z_check.get("poly", [])):
                        cand_inside = True
                        break
                if cand_inside:
                    continue

                # [狭缝过滤] 检查是否在两个禁飞区中间
                is_in_gap = False
                gap_threshold = Rmin * 4.0 + 50.0
                for z_other in zones:
                    if z_other is target_zone:
                        continue
                    poly_o = z_other.get("poly", [])
                    if len(poly_o) < 3:
                        continue
                    center_o = np.mean(poly_o, axis=0)
                    # 粗略判定：如果点离另一个障碍物也很近
                    radius_o = max(
                        [np.linalg.norm(np.array(v) - center_o) for v in poly_o]
                    )
                    if np.linalg.norm(cand - center_o) - radius_o < gap_threshold:
                        is_in_gap = True
                        break
                if is_in_gap:
                    continue

                # [关键优化] 智能航向采样
                headings_to_test = []

                # 1. 基础航向：指向终点 (Direct)
                angle_to_goal = math.atan2(gp[1] - cand[1], gp[0] - cand[0])
                headings_to_test.append(angle_to_goal)

                # 2. 基础航向：顺延起点 (Smooth)
                angle_from_start = math.atan2(
                    cand[1] - curr_sp[1], cand[0] - curr_sp[0]
                )
                headings_to_test.append(angle_from_start)

                # 3. [神器] 平分角航向 (Bisector) - 最顺滑的切线方向
                # 计算入射角和出射角的平均向量角度
                vec_in = np.array([np.cos(angle_from_start), np.sin(angle_from_start)])
                vec_out = np.array([np.cos(angle_to_goal), np.sin(angle_to_goal)])
                vec_avg = vec_in + vec_out
                if np.linalg.norm(vec_avg) > 0.1:
                    bisector_angle = math.atan2(vec_avg[1], vec_avg[0])
                    headings_to_test.append(bisector_angle)

                # 4. 补充一些固定方向以防万一
                headings_to_test.extend(list(base_headings))

                for h in headings_to_test:
                    wp = (cand[0], cand[1], h)
                    try:
                        # [防绕圈检查]
                        # 计算 Dubins 路径长度
                        dubins_len = dubins.shortest_path(
                            curr_sp, wp, Rmin
                        ).path_length()
                        # 计算直线距离
                        linear_dist = np.linalg.norm(
                            np.array(curr_sp[:2]) - np.array(cand[:2])
                        )

                        # 如果 Dubins 长度远大于直线距离（例如 > 1.5倍 + 一个圆周），说明发生了严重的绕圈
                        # 我们给这种路径加巨大的惩罚，迫使算法选别的角度
                        loop_penalty = 0
                        if dubins_len > linear_dist * 1.5 + 2 * np.pi * Rmin:
                            loop_penalty = 10000.0  # 惩罚绕圈

                        len_rem = math.hypot(gp[0] - cand[0], gp[1] - cand[1])
                        total_est_cost = dubins_len + len_rem + loop_penalty

                        candidate_options.append((total_est_cost, wp))
                    except:
                        pass

            # 按代价排序 (Cost 包含了防绕圈惩罚)
            candidate_options.sort(key=lambda x: x[0])

            best_wp = None
            for cost, wp in candidate_options:
                # 只有在这里才做昂贵的碰撞检测
                if not _is_dubins_blocked(curr_sp, wp, Rmin, zones, step=sampling_step):
                    best_wp = wp
                    break

            if best_wp:
                waypoints.append(best_wp)
                curr_sp = best_wp
                found_valid_wp = True
                break

        if not found_valid_wp:
            rospy.logwarn(
                f"[Planner] CRITICAL: Stuck! No safe bypass found. Direct Line Attempted."
            )
            break

    waypoints.append(gp)

    # 拼接路径
    for i in range(len(waypoints) - 1):
        p_start = waypoints[i]
        p_end = waypoints[i + 1]
        if np.linalg.norm(np.array(p_start[:2]) - np.array(p_end[:2])) < 0.5:
            continue
        try:
            path = dubins.shortest_path(p_start, p_end, Rmin)
            pts, _ = path.sample_many(sampling_step)
            if i > 0 and len(pts) > 0:
                full_sampled_points.extend(pts[1:])
            else:
                full_sampled_points.extend(pts)
        except Exception as e:
            full_sampled_points.append([p_end[0], p_end[1], p_end[2]])

    return full_sampled_points


class GA_SEAD(object):
    def __init__(self, targets, population_size=300):
        self.targets = targets
        " Global message (information of UAVs) "
        self.uav_id = []
        self.uav_type = []
        self.uav_velocity = []
        self.uav_Rmin = []
        self.uav_position = []
        self.depots = []
        " GA parameters "
        self.initial_population_size = population_size
        self.population_size = population_size
        self.crossover_num = 66
        self.mutation_num = 32
        self.mutation_prob = [0.25, 0.25, 0.25, 0.25]
        self.crossover_prob = [0.5, 0.5]
        self.elitism_num = 2
        self.lambda_1 = 0
        self.lambda_2 = 10
        self.lambda_3 = 6.0   # 任务负载均衡惩罚权重（5~10）
        " The precomputed matrix for optimization "
        self.uavType_for_missions = []
        self.tasks_status = [3 for _ in range(len(self.targets))]
        self.cost_matrix = []
        self.discrete_heading = [
            _ for _ in range(0, 36)
        ]  # Heading angles discretization = 10
        self.remaining_targets = []
        self.task_amount_array = []
        self.task_index_array = []
        self.target_sequence = []
        self.target_index_array = []
        self.penalty_coeff = 200

    # 在 class GA_SEAD 里新增一个工具函数（统一修正 UAV_ID）
    def _fix_and_get_uav_index(self, chromosome, j):
        """
        chromosome[3][j] = uav_id gene
        若该 uav_id 已不在线，则按任务类型挑一个在线可用 uav_id，并回写 chromosome
        返回：assign_uav（在 self.uav_id 里的 index）
        """
        uav_id_gene = int(chromosome[3][j])

        if uav_id_gene not in self.uav_id:
            task_type = int(chromosome[2][j]) if len(chromosome) > 2 else 1

            # 1) 按任务类型挑“仍在线”的候选
            cand = []
            if 1 <= task_type <= 3 and self.uavType_for_missions:
                cand = [
                    uid
                    for uid in self.uavType_for_missions[task_type - 1]
                    if uid in self.uav_id
                ]

            # 2) 兜底：随便挑一个在线 UAV
            if not cand:
                cand = list(self.uav_id)

            # 3) 回写修正，避免后续再炸
            uav_id_gene = random.choice(cand)
            chromosome[3][j] = uav_id_gene

        return self.uav_id.index(uav_id_gene)

    def fitness_evaluate_calculate(self, population):
        fitness_value = []
        uav_num = len(self.uav_id)
        chromosome_len = len(population[0][0])
        for chromosome in population:
            cost = [0 for _ in range(uav_num)]
            pre_site = [site for site in self.uav_position]
            task_sequence_time = [[] for _ in range(uav_num)]  # time
            time_list = []
            for j in range(chromosome_len):
                # assign_uav = self.uav_id.index(chromosome[3][j]) #发现这个被替换了，sun
                assign_uav = self._fix_and_get_uav_index(chromosome, j)
                assign_target = chromosome[1][j]
                assign_heading = chromosome[4][j]
                sp = pre_site[assign_uav]
                gp = self.targets[assign_target - 1] + [assign_heading * 2 * np.pi / 36]
                if sp == gp:
                    gp[-1] += 1e-5
                cost[assign_uav] += dubins.shortest_path(
                    sp, gp, self.uav_Rmin[assign_uav]
                ).path_length()
                task_sequence_time[assign_uav].append(
                    [
                        assign_target,
                        chromosome[2][j],
                        cost[assign_uav] / self.uav_velocity[assign_uav],
                    ]
                )
                pre_site[assign_uav] = self.targets[assign_target - 1] + [
                    assign_heading * 2 * np.pi / 36
                ]
            for j in range(uav_num):
                cost[j] += dubins.shortest_path(
                    pre_site[j], self.depots[j], self.uav_Rmin[j]
                ).path_length()
            for sequence in task_sequence_time:
                time_list.extend(sequence)
            time_list.sort()
            " The punishment of the task precedence constraints "
            # penalty, j = 0, 0
            # for task_num in self.tasks_status:
            #     if task_num >= 2:
            #         for k in range(1, task_num):
            #             penalty += max(0, time_list[j + k - 1][2] - time_list[j + k][2])
            #     j += task_num
            # ====== 阶段顺序时间约束惩罚（核心修改）======
            penalty = 0

            # 建立：target_id -> {stage: finish_time}
            target_stage_time = {}

            for entry in time_list:
                tgt = entry[0]      # 目标编号
                stage = entry[1]    # 阶段 1/2/3
                t_finish = entry[2]

                if tgt not in target_stage_time:
                    target_stage_time[tgt] = {}
                target_stage_time[tgt][stage] = t_finish

            # 对每个目标检查 阶段1 ≤ 阶段2 ≤ 阶段3
            for tgt, stages in target_stage_time.items():
                if 1 in stages and 2 in stages:
                    penalty += max(0, stages[1] - stages[2]) * self.penalty_coeff   # 侦察必须先完成
                if 2 in stages and 3 in stages:
                    penalty += max(0, stages[2] - stages[3]) * self.penalty_coeff   # 打击必须先完成

            # fitness_value.extend(
            #     [
            #         1
            #         / (
            #             np.max(np.divide(cost, self.uav_velocity))
            #             + self.lambda_1 * np.sum(cost)
            #             + self.lambda_2 * penalty
            #         )
            #     ]
            # )
            mission_time = np.max(np.divide(cost, self.uav_velocity))
            total_distance = np.sum(cost)

            # ===== 新增：任务负载均衡惩罚 =====
            task_counts = [len(seq) for seq in task_sequence_time]
            load_std = np.std(task_counts) if len(task_counts) > 1 else 0
            # =================================

            fitness_value.extend(
                [
                    1
                    / (
                        mission_time
                        + self.lambda_1 * total_distance
                        + self.lambda_2 * penalty
                        + self.lambda_3 * load_std
                    )
                ]
            )

        roulette_wheel = np.array(fitness_value) / np.sum(fitness_value)
        return fitness_value, roulette_wheel

    def fitness_evaluate(self, population):
        fitness_value = []
        uav_num = len(self.uav_id)
        for chromosome in population:
            cost = [0 for _ in range(uav_num)]
            task_sequence_time = [[] for _ in range(uav_num)]  # time
            time_list = []
            pre_site, pre_heading = [0 for _ in range(uav_num)], [
                0 for _ in range(uav_num)
            ]
            for j in range(len(chromosome[0])):
                # assign_uav = self.uav_id.index(chromosome[3][j]) #这儿也被替换了，sun
                assign_uav = self._fix_and_get_uav_index(chromosome, j)
                assign_target = chromosome[1][j]
                assign_heading = chromosome[4][j]
                cost[assign_uav] += self.cost_matrix[assign_uav][pre_site[assign_uav]][
                    pre_heading[assign_uav]
                ][assign_target][assign_heading]
                task_sequence_time[assign_uav].append(
                    [
                        assign_target,
                        chromosome[2][j],
                        cost[assign_uav] / self.uav_velocity[assign_uav],
                    ]
                )
                pre_site[assign_uav], pre_heading[assign_uav] = (
                    assign_target,
                    assign_heading,
                )
            for j in range(uav_num):
                cost[j] += self.cost_matrix[j][pre_site[j]][pre_heading[j]][0][0]
            for sequence in task_sequence_time:
                time_list.extend(sequence)
            time_list.sort()
            " The punishment of the task precedence constraints "
            # penalty, j = 0, 0
            # for task_num in self.tasks_status:
            #     if task_num >= 2:
            #         for k in range(1, task_num):
            #             penalty += max(0, time_list[j + k - 1][2] - time_list[j + k][2])
            #     j += task_num
            # ====== 阶段顺序时间约束惩罚（核心修改）======
            penalty = 0

            # 建立：target_id -> {stage: finish_time}
            target_stage_time = {}

            for entry in time_list:
                tgt = entry[0]      # 目标编号
                stage = entry[1]    # 阶段 1/2/3
                t_finish = entry[2]

                if tgt not in target_stage_time:
                    target_stage_time[tgt] = {}
                target_stage_time[tgt][stage] = t_finish

            # 对每个目标检查 阶段1 ≤ 阶段2 ≤ 阶段3
            for tgt, stages in target_stage_time.items():
                if 1 in stages and 2 in stages:
                    penalty += max(0, stages[1] - stages[2]) * self.penalty_coeff   # 侦察必须先完成
                if 2 in stages and 3 in stages:
                    penalty += max(0, stages[2] - stages[3]) * self.penalty_coeff   # 打击必须先完成

            # fitness_value.extend(
            #     [
            #         1
            #         / (
            #             np.max(np.divide(cost, self.uav_velocity))
            #             + self.lambda_1 * np.sum(cost)
            #             + self.lambda_2 * penalty
            #         )
            #     ]
            # )
            mission_time = np.max(np.divide(cost, self.uav_velocity))
            total_distance = np.sum(cost)

            # ===== 新增：任务负载均衡惩罚 =====
            task_counts = [len(seq) for seq in task_sequence_time]
            load_std = np.std(task_counts) if len(task_counts) > 1 else 0
            # =================================

            fitness_value.extend(
                [
                    1
                    / (
                        mission_time
                        + self.lambda_1 * total_distance
                        + self.lambda_2 * penalty
                        + self.lambda_3 * load_std
                    )
                ]
            )

        roulette_wheel = np.array(fitness_value) / np.sum(fitness_value)
        return fitness_value, roulette_wheel

    def chromosome_objectives_evaluate(self, chromosome):
        uav_num = len(self.uav_id)
        cost = [0 for _ in range(uav_num)]
        task_sequence_time = [[] for _ in range(uav_num)]  # time
        time_list = []
        pre_site, pre_heading = [0 for _ in range(uav_num)], [0 for _ in range(uav_num)]
        for j in range(len(chromosome[0])):
            # assign_uav = self.uav_id.index(chromosome[3][j]) #这儿也被替换了，sun
            assign_uav = self._fix_and_get_uav_index(chromosome, j)
            assign_target = chromosome[1][j]
            assign_heading = chromosome[4][j]
            cost[assign_uav] += self.cost_matrix[assign_uav][pre_site[assign_uav]][
                pre_heading[assign_uav]
            ][assign_target][assign_heading]
            task_sequence_time[assign_uav].append(
                [
                    assign_target,
                    chromosome[2][j],
                    cost[assign_uav] / self.uav_velocity[assign_uav],
                ]
            )
            pre_site[assign_uav], pre_heading[assign_uav] = (
                assign_target,
                assign_heading,
            )
        for j in range(uav_num):
            cost[j] += self.cost_matrix[j][pre_site[j]][pre_heading[j]][0][0]
        for sequence in task_sequence_time:
            time_list.extend(sequence)
        time_list.sort()
        " The punishment of the task precedence constraints "
        # penalty, j = 0, 0
        # for task_num in self.tasks_status:
        #     if task_num >= 2:
        #         for k in range(1, task_num):
        #             penalty += max(0, time_list[j + k - 1][2] - time_list[j + k][2])
        #     j += task_num

        # ====== 阶段顺序时间约束惩罚（核心修改）======
        penalty = 0

        # 建立：target_id -> {stage: finish_time}
        target_stage_time = {}

        for entry in time_list:
            tgt = entry[0]      # 目标编号
            stage = entry[1]    # 阶段 1/2/3
            t_finish = entry[2]

            if tgt not in target_stage_time:
                target_stage_time[tgt] = {}
            target_stage_time[tgt][stage] = t_finish

        # 对每个目标检查 阶段1 ≤ 阶段2 ≤ 阶段3
        for tgt, stages in target_stage_time.items():
            if 1 in stages and 2 in stages:
                penalty += max(0, stages[1] - stages[2]) * self.penalty_coeff   # 侦察必须先完成
            if 2 in stages and 3 in stages:
                penalty += max(0, stages[2] - stages[3]) * self.penalty_coeff   # 打击必须先完成

        # mission_time = np.max(np.divide(cost, self.uav_velocity))
        # total_distance = np.sum(cost)
        # fittness = 1 / (
        #     mission_time + self.lambda_1 * total_distance + self.lambda_2 * penalty
        # )
        # return fittness, mission_time, total_distance, penalty
        mission_time = np.max(np.divide(cost, self.uav_velocity))
        total_distance = np.sum(cost)

        # ===== 新增：任务负载均衡惩罚 =====
        task_counts = [len(seq) for seq in task_sequence_time]
        load_std = np.std(task_counts) if len(task_counts) > 1 else 0
        # =================================

        fittness = 1 / (
            mission_time
            + self.lambda_1 * total_distance
            + self.lambda_2 * penalty
            + self.lambda_3 * load_std
        )

        return fittness, mission_time, total_distance, penalty


    def generate_population(self):
        def generate_chromosome():
            chromosome = np.zeros((5, sum(self.tasks_status)), dtype=int)
            for i in range(chromosome.shape[1]):
                chromosome[0][i] = i + 1  # order
                chromosome[1][i] = random.choice(
                    [
                        n
                        for n in range(1, len(self.targets) + 1)  # target id
                        if np.count_nonzero(chromosome[1] == n)
                        < self.tasks_status[n - 1]
                    ]
                )
            " Convert to target-bundle chromsome "
            target_based_gene = self.order2target_bundle(chromosome)
            for i in range(len(target_based_gene)):
                target_based_gene[i][2] = mission_type_list[i]  # mission type
                target_based_gene[i][3] = random.choice(
                    self.uavType_for_missions[target_based_gene[i][2] - 1]
                )  # uav id
                target_based_gene[i][4] = random.choice(
                    self.discrete_heading
                )  # heading angle
            return self.target_bundle2order(target_based_gene)  # back to order-based

        mission_type_list = []
        for tasks in self.tasks_status:
            mission_type_list.extend([n + 1 for n in range(3 - tasks, 3)])
        return [generate_chromosome() for _ in range(self.population_size)]

    @staticmethod
    def selection(roulette_wheel, num):
        "roulette wheeol method"
        return np.random.choice(
            np.arange(len(roulette_wheel)), size=num, replace=False, p=roulette_wheel
        )

    @staticmethod
    def order2target_bundle(chromosome):
        "Convert the order-based to target-bundle chromoome"
        zipped_gene = [
            list(g)
            for g in zip(
                chromosome[0],
                chromosome[1],
                chromosome[2],
                chromosome[3],
                chromosome[4],
            )
        ]
        return sorted(sorted(zipped_gene, key=lambda u: u[2]), key=lambda u: u[1])

    @staticmethod
    def order2task_bundle(chromosome):
        "Convert the order-based to task-bundle chromoome"
        zipped_gene = [
            list(g)
            for g in zip(
                chromosome[0],
                chromosome[1],
                chromosome[2],
                chromosome[3],
                chromosome[4],
            )
        ]
        return sorted(zipped_gene, key=lambda u: u[2])

    @staticmethod
    def target_bundle2order(chromosome):
        "Convert the target-bundle to order-based chromoome"
        order_based_gene = np.array(sorted(chromosome, key=lambda u: u[0]))
        return [[g[i] for g in order_based_gene] for i in range(5)]

    def crossover_operator(self, wheel, population):
        def two_point_crossover(parent_1, parent_2):
            target_based_gene = []
            " Convert to target-bundle chromsome "
            for parent in [parent_1, parent_2]:
                target_based_gene.append(self.order2target_bundle(parent))
            " Choose cut sites "
            cut_point_1, cut_point_2 = sorted(random.sample(range(len(parent_1[0])), 2))
            cut_len = cut_point_2 - cut_point_1
            " Echange the genes "
            (
                target_based_gene[0][cut_point_1:cut_point_2],
                target_based_gene[1][cut_point_1:cut_point_2],
            ) = [
                target_based_gene[0][cut_point_1 + i][:3]
                + target_based_gene[1][cut_point_1 + i][3:]
                for i in range(cut_len)
            ], [
                target_based_gene[1][cut_point_1 + i][:3]
                + target_based_gene[0][cut_point_1 + i][3:]
                for i in range(cut_len)
            ]
            " Convert back to order-based chromosome "
            child_1 = self.target_bundle2order(target_based_gene[0])
            child_2 = self.target_bundle2order(target_based_gene[1])
            return [child_1, child_2]

        def target_bundle_crossover(parent_1, parent_2):
            target_based_gene = []
            " Convert to target-bundle chromsome "
            for parent in [parent_1, parent_2]:
                target_based_gene.append(self.order2target_bundle(parent))
            " Select targets to exchange "
            targets_exchanged = random.sample(
                self.remaining_targets, random.randint(1, len(self.remaining_targets))
            )
            " Echange the genes "
            for i in range(len(target_based_gene[0])):
                if target_based_gene[0][i][1] in targets_exchanged:
                    target_based_gene[0][i], target_based_gene[1][i] = (
                        target_based_gene[0][i][:3] + target_based_gene[1][i][3:],
                        target_based_gene[1][i][:3] + target_based_gene[0][i][3:],
                    )
            " Convert back to order-based chromsome "
            child_1 = self.target_bundle2order(target_based_gene[0])
            child_2 = self.target_bundle2order(target_based_gene[1])
            return [child_1, child_2]

        children = []
        for k in range(0, self.crossover_num, 2):
            p_1, p_2 = self.selection(wheel, 2)
            children.extend(
                np.random.choice(
                    [two_point_crossover, target_bundle_crossover],
                    p=self.crossover_prob,
                )(population[p_1], population[p_2])
            )
        return children

    def mutation_operator(self, wheel, population):
        def point_agent_mutation(chromosome):
            "Choose a point to mutate"
            mut_point = np.random.randint(0, len(chromosome[0]))
            new_gene = []
            for i in range(len(chromosome)):  # copy chromosome
                new_gene.append(chromosome[i][:])
            " Mutate the assigned UAV "
            new_gene[3][mut_point] = random.choice(
                [i for i in self.uavType_for_missions[new_gene[2][mut_point] - 1]]
            )
            return new_gene

        def point_heading_mutation(chromosome):
            "Choose a point to mutate"
            mut_point = np.random.randint(0, len(chromosome[0]))
            new_gene = []
            for i in range(len(chromosome)):  # copy chromosome
                new_gene.append(chromosome[i][:])
            " Mutate the assigned heading "
            new_gene[4][mut_point] = random.choice(
                [i for i in self.discrete_heading if i != chromosome[4][mut_point]]
            )
            return new_gene

        def target_bundle_mutation(chromosome):
            "Convert to target-bundle chromsome"
            target_based_gene = self.order2target_bundle(chromosome)
            " Shuffle the gene which is the same remaing tasks on the targets "
            for task_type in self.target_sequence:
                random.shuffle(task_type)
            shuffle_sequence = (
                self.target_sequence[0]
                + self.target_sequence[1]
                + self.target_sequence[2]
            )
            " Mutate the genes "
            mutate_target_based = [[] for _ in range(len(target_based_gene))]
            j = 0
            for sequence in shuffle_sequence:
                mutate_target_based[j : j + self.tasks_status[sequence]] = [
                    [
                        b[:1]
                        for b in target_based_gene[j : j + self.tasks_status[sequence]]
                    ][i]
                    + [
                        a[1:]
                        for a in target_based_gene[
                            self.target_index_array[sequence] : self.target_index_array[
                                sequence + 1
                            ]
                        ]
                    ][i]
                    for i in range(self.tasks_status[sequence])
                ]
                j += self.tasks_status[sequence]
            " Convert back to order-based chromsome "
            return self.target_bundle2order(mutate_target_based)

        def task_bundle_mutation(chromosome):
            "Convert to task-bundle chromsome"
            task_based_gene = self.order2task_bundle(chromosome)
            " Choose a task to mutate "
            mut_task = np.random.randint(0, 3)
            " Shuffle the genes "
            task_sequence = list(range(self.task_amount_array[mut_task]))
            random.shuffle(task_sequence)
            " Copy the origrnal chromosome "
            mutate_task_based = []
            for i in range(len(task_based_gene)):
                mutate_task_based.append(task_based_gene[i][:])
            " Mutate the genes "
            for i, sequence in enumerate(task_sequence):
                mutate_task_based[self.task_index_array[mut_task] + i][3:] = (
                    task_based_gene[self.task_index_array[mut_task] + sequence][3:]
                )
            " Convert back to order-based chromsome "
            order_based_gene = np.array(sorted(mutate_task_based, key=lambda u: u[0]))
            return [[g[i] for g in order_based_gene] for i in range(5)]

        mutation_operators = [
            point_agent_mutation,
            point_heading_mutation,
            target_bundle_mutation,
            task_bundle_mutation,
        ]
        return [
            np.random.choice(mutation_operators, p=self.mutation_prob)(
                population[self.selection(wheel, 1)[0]]
            )
            for _ in range(self.mutation_num)
        ]

    def elitism_operator(self, fitness, population):
        fitness_ranking = sorted(
            range(len(fitness)), key=lambda u: fitness[u], reverse=True
        )[: self.elitism_num]
        return [population[_] for _ in fitness_ranking]

    def information_setting(self, information, population):
        lost_agent, build_graph = False, False
        terminated_tasks, new_target = sorted(
            information[8], key=lambda u: u[1]
        ), sorted(information[9])

        # 在 information_setting() 开头读取 zones
        zones = (
            information[10]
            if (len(information) > 10 and information[10] is not None)
            else []
        )
        # ====== 关键日志：确认 GA 侧真的收到 zones ======
        z0 = zones[0] if zones else None
        z0_poly = z0.get("poly", []) if isinstance(z0, dict) else []
        print(
            f"[DBG][GA] zones_in={len(zones)} z0_verts={len(z0_poly)} "
            f"z0_first={(z0_poly[0] if z0_poly else None)}",
            flush=True,
        )

        NOFLY_PENALTY = 1e6  # 你也可以改成 1e5~1e7，取决于距离量级

        clear_task, new_task = [], []
        print(terminated_tasks)
        " Terminated tasks check"
        if terminated_tasks:
            for task in terminated_tasks:
                if self.tasks_status[task[0] - 1] == 3 - task[1] + 1:
                    self.tasks_status[task[0] - 1] -= 1
                    clear_task.append(task)
        " New targets check "
        if new_target:
            new_target.sort()
            for target in new_target:
                if target not in self.targets:
                    self.targets.append(target)
                    self.tasks_status.append(3)
                    new_task.append(self.targets.index(target) + 1)
            build_graph = True
        " UAVs check "
        if not set(self.uav_id) == set(information[0]):
            build_graph = True
            lost_agent = True
        " Clear the information "
        self.uav_id = information[0]
        self.uav_type = information[1]
        self.uav_velocity = information[2]
        self.uav_Rmin = information[3]
        self.uav_position = information[4]
        self.depots = information[5]
        self.uavType_for_missions = [[] for _ in range(3)]
        " Classify capable UAVs to the missions => [Surveillance(1): [C, V](1, 3), Combat(2): [C, A, V](1, 2, 3), Munition(3): [A](2)] "
        for i, agent in enumerate(self.uav_type):
            if agent == 1:  # surveillance
                self.uavType_for_missions[0].append(self.uav_id[i])
                # self.uavType_for_missions[2].append(self.uav_id[i])
            elif agent == 2:  # attack, combat
                # self.uavType_for_missions[0].append(self.uav_id[i])
                self.uavType_for_missions[1].append(self.uav_id[i])
                # self.uavType_for_missions[2].append(self.uav_id[i])
            elif agent == 3:  # munition
                self.uavType_for_missions[2].append(self.uav_id[i])
        " <<<<<<<<<<<<<<<<<<<< Cost Graph >>>>>>>>>>>>>>>>>>>> "
        if build_graph:
            self.cost_matrix = [
                [
                    [
                        [
                            [0 for a in range(len(self.discrete_heading))]
                            for b in range(len(self.targets) + 1)
                        ]
                        for c in range(len(self.discrete_heading))
                    ]
                    for d in range(len(self.targets) + 1)
                ]
                for u in range(len(self.uav_id))
            ]

            # ====== ✅ 就插在这里：blocked_tt 预计算（在 for a... 之前） ======
            blocked_tt = [
                [False] * (len(self.targets) + 1) for _ in range(len(self.targets) + 1)
            ]
            if zones:
                for i in range(1, len(self.targets) + 1):
                    p0 = (self.targets[i - 1][0], self.targets[i - 1][1])
                    for j in range(1, len(self.targets) + 1):
                        if i == j:
                            continue
                        p1 = (self.targets[j - 1][0], self.targets[j - 1][1])
                        hit = False
                        for z in zones:
                            poly = z.get("poly", [])
                            if len(poly) >= 3 and _seg_hits_poly(p0, p1, poly):
                                hit = True
                                break
                        blocked_tt[i][j] = hit

            for a in range(1, len(self.targets) + 1):
                for b in self.discrete_heading:
                    for c in range(1, len(self.targets) + 1):
                        for d in self.discrete_heading:
                            source_point = self.targets[a - 1] + [b * 10 * np.pi / 180]
                            end_point = self.targets[c - 1] + [d * 10 * np.pi / 180]
                            if source_point == end_point:
                                end_point[-1] += 1e-5
                            for u in range(len(self.uav_id)):
                                # 给 target→target 的 cost 加惩罚
                                # ... 在 calculating cost matrix 的循环里 ...
                                # 1. 计算直飞代价
                                distance = dubins.shortest_path(
                                    source_point, end_point, self.uav_Rmin[u]
                                ).path_length()

                                # 2. 检测阻挡
                                is_blocked = False
                                if zones:
                                    if blocked_tt[a][c]:
                                        is_blocked = True
                                    else:
                                        is_blocked = _is_dubins_blocked(
                                            source_point,
                                            end_point,
                                            self.uav_Rmin[u],
                                            zones,
                                        )

                                # 3. [核心修改] 如果被挡，尝试寻找绕行代价
                                if is_blocked:
                                    # print(f"[GA][BLOCK] edge T{a}->{c} (uav_idx={u}) blocked, try bypass", flush=True)
                                    # 尝试找个绕行点
                                    _, bypass_cost = _find_best_bypass(
                                        source_point, end_point, self.uav_Rmin[u], zones
                                    )

                                    if bypass_cost < float("inf"):
                                        # 找到了！用绕行代价替换惩罚
                                        # 稍微加一点点惩罚系数(如1.1倍)，倾向于选不绕路的直飞
                                        distance = bypass_cost * 1.05
                                    else:
                                        # 实在找不到路（死路），才给天价惩罚
                                        print(
                                            f"[GA][PENALTY] edge T{a}->{c} no bypass, +{NOFLY_PENALTY}",
                                            flush=True,
                                        )
                                        distance += NOFLY_PENALTY

                                self.cost_matrix[u][a][b][c][d] = distance

        " Update the real time information in graph "
        " Cost => UAVs to targets "
        for a in range(1, len(self.targets) + 1):
            for b in self.discrete_heading:
                point = self.targets[a - 1] + [b * 10 * np.pi / 180]
                for u in range(len(self.uav_id)):

                    # 给 UAV_pos→target 与 target→depot 加惩罚（动态点不在 blocked_tt 里）
                    distance = dubins.shortest_path(
                        self.uav_position[u], point, self.uav_Rmin[u]
                    ).path_length()
                    is_blocked = False
                    if zones:
                        # 使用更精确的 Dubins 检测
                        is_blocked = _is_dubins_blocked(
                            self.uav_position[u], point, self.uav_Rmin[u], zones
                        )

                    if is_blocked:
                        # 尝试寻找绕行
                        _, bypass_cost = _find_best_bypass(
                            self.uav_position[u], point, self.uav_Rmin[u], zones
                        )
                        if bypass_cost < float("inf"):
                            distance = bypass_cost * 1.05
                        else:
                            distance += NOFLY_PENALTY
                    self.cost_matrix[u][0][0][a][b] = distance

                    distance = dubins.shortest_path(
                        point, self.depots[u], self.uav_Rmin[u]
                    ).path_length()
                    if zones:
                        p0 = (point[0], point[1])
                        p1 = (self.depots[u][0], self.depots[u][1])
                        for z in zones:
                            poly = z.get("poly", [])
                            if len(poly) >= 3 and _seg_hits_poly(p0, p1, poly):
                                distance += NOFLY_PENALTY
                                break
                    self.cost_matrix[u][a][b][0][0] = distance

        " Cost => UAVs to the base "
        for u in range(len(self.uav_id)):
            # UAV_pos→depot加惩罚
            distance = dubins.shortest_path(
                self.uav_position[u], self.depots[u], self.uav_Rmin[u]
            ).path_length()
            if zones:
                p0 = (self.uav_position[u][0], self.uav_position[u][1])
                p1 = (self.depots[u][0], self.depots[u][1])
                for z in zones:
                    poly = z.get("poly", [])
                    if len(poly) >= 3 and _seg_hits_poly(p0, p1, poly):
                        distance += NOFLY_PENALTY
                        break
            self.cost_matrix[u][0][0][0][0] = distance

        " GA parameters "
        self.population_size = round(self.initial_population_size / len(self.uav_id))
        self.crossover_num = round((self.population_size - self.elitism_num) * 0.67)
        self.mutation_num = self.population_size - self.crossover_num - self.elitism_num
        self.lambda_1 = 1 / (sum(self.uav_velocity))

        " Precomputed matrix "
        self.remaining_targets = [
            target_id
            for target_id in range(1, len(self.targets) + 1)
            if not self.tasks_status[target_id - 1] == 0
        ]
        self.task_amount_array = [
            np.count_nonzero(np.array(self.tasks_status) >= 3 - t) for t in range(3)
        ]
        self.task_index_array = [
            0,
            self.task_amount_array[0],
            self.task_amount_array[0] + self.task_amount_array[1],
        ]
        self.target_sequence = [
            [
                index
                for (index, value) in enumerate(self.tasks_status)
                if value == task_num
            ]
            for task_num in range(1, 4)
        ]
        self.target_index_array = [0]
        for k, times in enumerate(self.tasks_status):
            self.target_index_array.append(self.target_index_array[k] + times)

        " Revise the population "
        if population:
            "Clear the empty chromsomes"
            information[7] = [elite for elite in information[7] if elite]
            " <<< Task completed revision >>> "
            for elite in information[7]:
                for task in clear_task:
                    for site in range(len(elite[0])):
                        if elite[1][site] == task[0] and elite[2][site] == task[1]:
                            for row in elite:
                                row.pop(site)
                            elite[0] = [
                                sequence for sequence in range(1, len(elite[0]) + 1)
                            ]
                            break
            " <<< UAV set changed revision >>> "
            if lost_agent:
                for elite in information[7]:
                    for index, task_type in enumerate(elite[2]):
                        if elite[3][index] not in self.uav_id:
                            elite[3][index] = random.choice(
                                self.uavType_for_missions[task_type - 1]
                            )
            " <<< Targets mewly found revision >>> "
            if new_target:
                for elite in information[7]:
                    for target in new_task:
                        insert_index = sorted(np.random.choice(len(elite[0]) + 1, 3))
                        insert_index = [insert_index[i] + i for i in range(3)]
                        task_type = 1
                        for point in insert_index:
                            elite[1].insert(point, target)
                            elite[2].insert(point, task_type)
                            elite[3].insert(
                                point,
                                random.choice(self.uavType_for_missions[task_type - 1]),
                            )
                            elite[4].insert(point, random.choice(self.discrete_heading))
                            task_type += 1
                    elite[0] = [sequence for sequence in range(1, len(elite[1]) + 1)]
            " Regenerate the population "
            if new_target or clear_task or lost_agent:
                try:
                    population = self.generate_population()
                except IndexError:  # no solution
                    return [], 1e-5, None
            " Elite chromsomes incorporation "
            population.extend(
                [
                    elite
                    for elite in information[7]
                    if len(elite[0]) == sum(self.tasks_status)
                ]
            )
        return population

    def run_GA(self, iteration, uav_message, population=None):
        a = []
        population = self.information_setting(uav_message, population)
        residual_tasks = sum(self.tasks_status)
        if residual_tasks == 0:
            empty = True
        else:
            self.crossover_prob = [0, 1] if residual_tasks <= 1 else [0.5, 0.5]
            empty = False
        if not empty:
            if not population:
                population = self.generate_population()
                iteration -= 1
            fitness, wheel = self.fitness_evaluate(population)
            a.append(1 / max(fitness))
            for iterate in range(iteration):
                new_population = []
                new_population.extend(self.elitism_operator(fitness, population))
                new_population.extend(self.crossover_operator(wheel, population))
                new_population.extend(self.mutation_operator(wheel, population))
                fitness, wheel = self.fitness_evaluate(new_population)
                population = new_population
                a.append(1 / max(fitness))
            return population[fitness.index(max(fitness))], max(fitness), population, a
        else:
            return [[] for _ in range(5)], 0, [], 0

    def run_GA_time_period_version(
        self, time_interval, uav_message, population=None, update=True
    ):
        iteration = 0
        start_time = time.time()
        if update:
            population = self.information_setting(uav_message, population)
            rospy.logwarn("[SEAD] GA update triggered (reallocation)")
        residual_tasks = sum(self.tasks_status)
        if residual_tasks == 0:
            empty = True
        else:
            self.crossover_prob = [0, 1] if residual_tasks <= 1 else [0.5, 0.5]
            # Can't do the two-points crossover operation since the remaining taks less than two
            empty = False
        if not empty:
            if not population:
                try:
                    population = self.generate_population()
                except IndexError:  # no solution
                    return [], 1e-5, None
            fitness, wheel = self.fitness_evaluate(population)
            while time.time() - start_time <= time_interval:
                iteration += 1
                new_population = []
                new_population.extend(self.elitism_operator(fitness, population))
                new_population.extend(self.crossover_operator(wheel, population))
                new_population.extend(self.mutation_operator(wheel, population))
                fitness, wheel = self.fitness_evaluate(new_population)
                population = new_population
                # if iteration % 50 == 0:
                # print(f"[GA] Iteration: {iteration}, Best Fitness: {max(fitness):.4f}") #sun
                # ===============

            # === 新增日志 ===
            # print(f"[GA] Optimization Finished. Best Cost: {1/max(fitness):.2f}") #sun
            # ===============
            return population[fitness.index(max(fitness))], max(fitness), population
        else:
            residual_fitness, _, _, _ = self.chromosome_objectives_evaluate(
                [[] for _ in range(5)]
            )
            return [[] for _ in range(5)], residual_fitness, []

    def run_RS(self, iteration, uav_message, population=None):
        a = []
        population = self.generate_population()
        fitness, _ = self.fitness_evaluate(population)
        a.append(1 / max(fitness))
        iteration -= 1
        for iterate in range(iteration):
            new_population = self.generate_population()
            new_fitness, _ = self.fitness_evaluate(new_population)
            for i, chromosome in enumerate(new_population):
                if new_fitness[i] > min(fitness):
                    fitness[fitness.index(min(fitness))] = new_fitness[i]
                    population[i] = chromosome
            a.append(1 / max(fitness))
        return population[fitness.index(max(fitness))], max(fitness), population, a

    def plot_result(self, best_solution, curve=None):
        def dubins_plot(state_list, c, time_):
            distance = 0
            route_ = [[] for _ in range(2)]
            arrow_ = []
            for a in range(1, len(state_list) - 1):
                state_list[a][2] *= np.pi / 180
            for a in range(len(state_list) - 1):
                sp = state_list[a]
                gp = (
                    state_list[a + 1]
                    if state_list[a] != state_list[a + 1]
                    else [
                        state_list[a + 1][0],
                        state_list[a + 1][1],
                        state_list[a + 1][2] - 1e-5,
                    ]
                )
                dubins_path = dubins.shortest_path(sp, gp, self.uav_Rmin[c])
                path, _ = dubins_path.sample_many(0.1)
                route_[0].extend([b[0] for b in path])
                route_[1].extend([b[1] for b in path])
                distance += dubins_path.path_length()
                try:
                    time_[a].append(distance / self.uav_velocity[c])
                except IndexError:
                    pass
            arrow_.extend(
                [
                    [
                        route_[0][arr],
                        route_[1][arr],
                        route_[0][arr + 100],
                        route_[1][arr + 100],
                    ]
                    for arr in range(0, len(route_[0]), 15000)
                ]
            )
            print(state_list)
            return distance, route_, arrow_

        print(f"best gene:{best_solution}")
        uav_num = len(self.uav_id)
        dist = np.zeros(uav_num)
        task_sequence_state = [[] for _ in range(uav_num)]
        task_route = [[] for _ in range(uav_num)]
        route_state = [[] for _ in range(uav_num)]
        arrow_state = [[] for _ in range(uav_num)]
        for j in range(len(best_solution[0])):
            assign_uav = self.uav_id.index(best_solution[3][j])
            assign_target = best_solution[1][j]
            assign_heading = best_solution[4][j] * 10
            task_sequence_state[assign_uav].append(
                [
                    self.targets[assign_target - 1][0],
                    self.targets[assign_target - 1][1],
                    assign_heading,
                ]
            )
            task_route[assign_uav].extend([[assign_target, best_solution[2][j]]])
        for j in range(uav_num):
            task_sequence_state[j] = (
                [self.uav_position[j]] + task_sequence_state[j] + [self.depots[j]]
            )
            dist[j], route_state[j], arrow_state[j] = dubins_plot(
                task_sequence_state[j], j, task_route[j]
            )
        for route in task_route:
            print(f"best route:{route}")
        # arrange to target-based to check time sequence constraints
        time_list = []
        for time_sequence in task_route:
            time_list.extend(time_sequence)
        time_list.sort()
        print(time_list)
        penalty, j = 0, 0
        for task_num in self.tasks_status:
            if task_num >= 2:
                for k in range(1, task_num):
                    penalty += max(0, time_list[j + k - 1][2] - time_list[j + k][2])
            j += task_num
        print(f"mission time: {max(np.divide(dist, self.uav_velocity))} (sec)")
        print(f"total distance: {sum(dist)} (m)")
        print(
            max(np.divide(dist, self.uav_velocity))
            + self.lambda_1 * sum(dist)
            + self.lambda_2 * penalty
        )
        print(f"penalty: {penalty}")
        print(1 / self.fitness_evaluate_calculate([best_solution])[0][0])
        print(1 / self.fitness_evaluate([best_solution])[0][0])
        color_style = [
            "tab:blue",
            "tab:green",
            "tab:orange",
            "#DC143C",
            "#808080",
            "#030764",
            "#06C2AC",
            "#008080",
            "#DAA520",
            "#580F41",
            "#7BC8F6",
            "#C875C4",
        ]
        font = {"family": "Times New Roman", "weight": "normal", "size": 8}
        font0 = {"family": "Times New Roman", "weight": "normal", "size": 10}
        font1 = {
            "family": "Times New Roman",
            "weight": "normal",
            "color": "m",
            "size": 8,
        }
        font2 = {
            "family": "Times New Roman",
            "weight": "normal",
            "color": "r",
            "size": 8,
        }
        if curve:
            plt.subplot(122)
            plt.plot([b for b in range(1, len(curve) + 1)], curve, "-")
            print(len(curve))
            plt.grid()
            # plt.ylabel('E ( $\mathregular{J_i}$ / $\mathregular{J_1}$ )', fontsize=12)
            plt.subplot(121)
        else:
            fig, ax = plt.subplots()
            labels = ax.get_xticklabels() + ax.get_yticklabels()
            [label.set_fontname("Times New Roman") for label in labels]
        for i in range(uav_num):
            plt.plot(
                route_state[i][0],
                route_state[i][1],
                "-",
                linewidth=0.8,
                color=color_style[i],
                label=f"UAV {self.uav_id[i]}",
            )
            plt.text(
                self.uav_position[i][0] - 100,
                self.uav_position[i][1] - 200,
                f"UAV {self.uav_id[i]}",
                font,
            )
            plt.axis("equal")
            for arrow in arrow_state[i]:
                plt.arrow(
                    arrow[0],
                    arrow[1],
                    arrow[2] - arrow[0],
                    arrow[3] - arrow[1],
                    width=16,
                    color=color_style[i],
                )
        plt.plot(
            [x[0] for x in self.uav_position],
            [x[1] for x in self.uav_position],
            "k^",
            markerfacecolor="none",
            markersize=8,
        )
        # i = 1
        # for pos in self.uav_position:
        #     plt.plot(pos[0], pos[1], '^', color=color_style[i-1], markerfacecolor='none', markersize=8, label=f'UAV {i}')
        #     i += 1
        plt.plot(
            [b[0] for b in self.targets],
            [b[1] for b in self.targets],
            "ms",
            label="Target position",
            markerfacecolor="none",
            markersize=6,
        )
        plt.plot(
            self.depots[0][0],
            self.depots[0][1],
            "r*",
            markerfacecolor="none",
            markersize=10,
            label="Base",
        )
        for t in self.targets:
            plt.text(t[0] + 100, t[1] + 100, f"Target {self.targets.index(t)+1}", font1)
        plt.text(self.depots[0][0] - 100, self.depots[0][1] - 200, "Base", font2)
        plt.legend(loc="upper right", prop=font)
        plt.xlabel("East, m", font0)
        plt.ylabel("North, m", font0)
        # plt.grid()
        plt.show()


if __name__ == "__main__":
    targets_sites = [
        [3100, 2200],
        [500, 3700],
        [2300, 2500],
        [2000, 3900],
        [4450, 3600],
        [4630, 4780],
        [1400, 4500],
        [3300, 3415],
        [1640, 1700],
        [4230, 1700],
        [500, 2200],
        [3000, 4500],
        [5000, 2810],
    ]
    uavs = [
        [i for i in range(1, 12)],
        [1, 2, 3, 1, 3, 2, 1, 2, 3, 1, 2],
        [70, 80, 90, 60, 100, 80, 75, 90, 85, 70, 65],
        [200, 250, 300, 180, 300, 260, 225, 295, 250, 200, 170],
        [
            [1000, 300, -np.pi],
            [1500, 700, np.pi / 2],
            [3000, 0, np.pi / 3],
            [1800, 400, -20 * np.pi / 180],
            [2200, 280, 45 * np.pi / 180],
            [4740, 300, 140 * np.pi / 180],
            [4000, 100, 70 * np.pi / 180],
            [3500, 450, -75 * np.pi / 180],
            [5000, 900, -115 * np.pi / 180],
            [2780, 500, -55 * np.pi / 180],
            [4000, 600, 85 * np.pi / 180],
        ],
        [[0, 0, -np.pi / 2] for _ in range(11)],
    ]
    ga = GA_SEAD(targets_sites[:4], 100)
    uavs = [[row[j] for j in range(3)] for row in uavs] + [[], [], [], []]

    start = time.time()
    solution, fitness_, ga_population, convergence = ga.run_GA(100, uavs)
    print(f"Process time: {time.time() - start} (sec)")
    print(f"Cost value = {1/fitness_}")
    print(f"Fitness value = {fitness_}")

    ga.plot_result(solution, convergence)
