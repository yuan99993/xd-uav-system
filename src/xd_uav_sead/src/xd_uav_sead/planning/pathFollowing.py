import numpy as np
from numpy import sin, cos, sqrt, pi, arctan2
import scipy.linalg as la
from xd_uav_sead.comms.communication_info import *
import time
import math


def PlusMinusPi(theta):
    if theta > pi:
        return theta - 2 * pi
    elif theta < -pi:
        return theta + 2 * pi
    else:
        return theta


class CraigReynolds_Path_Following(object):
    def __init__(self, method, recedingHorizon, path, path_window=3, Kp=1, Kd=5):
        self.method = method
        self.recedingHorizon = recedingHorizon  # (sec)
        self.pathWindow = path_window
        " PID parameters "
        self.Kp = Kp
        self.Kd = Kd
        " Path "
        self.path = path
        # 下面这几个是新增的，固定翼航点稀疏化用的
        self.fw_path_index = 0  # 固定翼路径进度索引
        self.last_pub_point = None
        self.min_pub_dist = 15.0  # 固定翼推荐 10~25 m
        self.min_pub_time = 0.5  # s，防抖
        self.last_pub_time = time.time()
        self.Rmin = 30  # 固定翼最小转弯半径，可选参数

    def get_fixed_wing_waypoint(
        self,
        uav_x,
        uav_y,
        min_dist=10.0,
        default_z=None,
        update=False,
        lookahead_dist=None,
        completion_dist=None,
    ):
        """Return a forward look-ahead point on the sampled fixed-wing path.

        The old implementation treated ``1.5 * Rmin`` as a waypoint acceptance
        radius.  Since a Dubins path is sampled every few metres, that skipped
        tens of curve samples at once and cut the corner which encoded the turn
        radius.  Track the nearest *forward* sample instead, then walk a bounded
        arc length along the path to obtain a stable look-ahead target.
        """
        if not self.path:
            return None

        if default_z is None:
            default_z = 0.0
        rmin = max(float(getattr(self, "Rmin", min_dist)), float(min_dist))
        if lookahead_dist is None:
            lookahead_dist = max(float(min_dist), min(0.5 * rmin, 40.0))
        if completion_dist is None:
            completion_dist = max(float(min_dist), min(0.25 * rmin, 20.0))

        points = []
        source_indices = []
        for index, point in enumerate(self.path):
            try:
                points.append([float(point[0]), float(point[1])])
                source_indices.append(index)
            except (TypeError, ValueError, IndexError):
                continue
        if not points:
            return None

        start = 0 if update else max(0, int(self.fw_path_index))
        valid_start = 0
        while valid_start < len(source_indices) and source_indices[valid_start] < start:
            valid_start += 1
        if valid_start >= len(points):
            valid_start = len(points) - 1

        uav_pos = np.array([float(uav_x), float(uav_y)], dtype=float)
        # Outbound and return legs can be geometrically close.  Searching the
        # complete remainder may jump directly onto the return leg, especially
        # when a newly loaded route starts and ends at home.  Project only in
        # a bounded forward arc and advance that window with fw_path_index.
        search_arc = max(60.0, 2.0 * rmin, 4.0 * float(lookahead_dist))
        search_end = valid_start
        accumulated = 0.0
        while search_end < len(points) - 1 and accumulated < search_arc:
            accumulated += float(
                np.linalg.norm(
                    np.asarray(points[search_end + 1])
                    - np.asarray(points[search_end])
                )
            )
            search_end += 1
        distances = [
            np.linalg.norm(np.asarray(point) - uav_pos)
            for point in points[valid_start : search_end + 1]
        ]
        nearest = valid_start + int(np.argmin(distances))
        self.fw_path_index = source_indices[nearest]

        final_xy = np.asarray(points[-1])
        if nearest == len(points) - 1 and np.linalg.norm(final_xy - uav_pos) <= completion_dist:
            return None

        travelled = 0.0
        target = nearest
        while target < len(points) - 1 and travelled < float(lookahead_dist):
            travelled += float(
                np.linalg.norm(np.asarray(points[target + 1]) - np.asarray(points[target]))
            )
            target += 1

        point = self.path[source_indices[target]]
        return [
            float(point[0]),
            float(point[1]),
            float(point[2]) if len(point) >= 3 else default_z,
        ]

    # 这段是原始代码
    def get_desirePoint_withWindow(self, v, x, y, theta, start_index):
        futurePoint = np.array(
            [
                x + v * cos(theta) * self.recedingHorizon,
                y + v * sin(theta) * self.recedingHorizon,
            ]
        )
        end_index = (
            start_index + self.pathWindow
            if start_index + self.pathWindow < len(self.path)
            else len(self.path) - 1
        )
        d_optimal, desirePoint = 1e5, 0
        for i in range(start_index, end_index):
            a = np.array([self.path[i][0], self.path[i][1]])
            b = np.array([self.path[i + 1][0], self.path[i + 1][1]])

            va = futurePoint - a
            vb = b - a
            projection = np.dot(va, vb) / np.dot(vb, vb) * vb
            normalPoint = a + projection

            if not max(self.path[i][0], self.path[i + 1][0]) >= normalPoint[0] >= min(
                self.path[i][0], self.path[i + 1][0]
            ) or not max(self.path[i][1], self.path[i + 1][1]) >= normalPoint[1] >= min(
                self.path[i][1], self.path[i + 1][1]
            ):
                normalPoint = b
            d = np.linalg.norm(va - (normalPoint - a))
            if d < d_optimal:
                d_optimal = d
                desirePoint = normalPoint
                direct_projection = np.dot(vb, desirePoint - np.array([x, y]))
                index = i
        return desirePoint, index, d_optimal, direct_projection

    # def get_desirePoint_withWindow(self, v, x, y, theta, start_index):

    #     # ===== 安全初始化 =====
    #     index = start_index
    #     direct_projection = 0.0
    #     d_optimal = 1e5
    #     desirePoint = np.array([x, y])

    #     # 路径末尾保护
    #     if start_index >= len(self.path) - 1:
    #         last = self.path[-1]
    #         return np.array([last[0], last[1]]), len(self.path)-1, 0.0, 0.0

    #     futurePoint = np.array([
    #         x + v * cos(theta) * self.recedingHorizon,
    #         y + v * sin(theta) * self.recedingHorizon
    #     ])

    #     end_index = min(start_index + self.pathWindow, len(self.path) - 1)

    #     for i in range(start_index, end_index):
    #         a = np.array(self.path[i][:2])
    #         b = np.array(self.path[i+1][:2])

    #         vb = b - a
    #         if np.dot(vb, vb) < 1e-6:
    #             continue

    #         va = futurePoint - a
    #         projection = np.dot(va, vb) / np.dot(vb, vb) * vb
    #         normalPoint = a + projection

    #         if not (
    #             min(a[0], b[0]) <= normalPoint[0] <= max(a[0], b[0]) and
    #             min(a[1], b[1]) <= normalPoint[1] <= max(a[1], b[1])
    #         ):
    #             normalPoint = b

    #         d = np.linalg.norm(futurePoint - normalPoint)

    #         if d < d_optimal:
    #             d_optimal = d
    #             desirePoint = normalPoint
    #             direct_projection = np.dot(vb, desirePoint - np.array([x, y]))
    #             index = i

    #     return desirePoint, index, d_optimal, direct_projection

    def get_desirePoint(self, v, x, y, theta):
        futurePoint = np.array(
            [
                x + v * cos(theta) * self.recedingHorizon,
                y + v * sin(theta) * self.recedingHorizon,
            ]
        )
        d_optimal = 1e5
        for i in range(len(self.path) - 1):
            a = np.array([self.path[i][0], self.path[i][1]])
            b = np.array([self.path[i + 1][0], self.path[i + 1][1]])

            va = futurePoint - a
            vb = b - a
            projection = np.dot(va, vb) / np.dot(vb, vb) * vb
            normalPoint = a + projection

            if not max(self.path[i][0], self.path[i + 1][0]) >= normalPoint[0] >= min(
                self.path[i][0], self.path[i + 1][0]
            ) or not max(self.path[i][1], self.path[i + 1][1]) >= normalPoint[1] >= min(
                self.path[i][1], self.path[i + 1][1]
            ):
                normalPoint = b
            d = np.linalg.norm(va - (normalPoint - a))
            if d < d_optimal:
                d_optimal = d
                desirePoint = normalPoint
                index = i
        return desirePoint, index, d_optimal

    def get_desireVelocity(self, v, x, y, vx, vy):
        v_unitVector = np.array(vx, vy) / np.linalg.norm([vx, vy])
        futurePoint = np.add([x, y], v * v_unitVector * self.recedingHorizon)
        d_optimal = 1e5
        for i in range(len(self.path) - 1):
            a = np.array([self.path[i][0], self.path[i][1]])
            b = np.array([self.path[i + 1][0], self.path[i + 1][1]])

            va = futurePoint - a
            vb = b - a
            projection = np.dot(va, vb) / np.dot(vb, vb) * vb
            normalPoint = a + projection

            if not max(self.path[i][0], self.path[i + 1][0]) > normalPoint[0] > min(
                self.path[i][0], self.path[i + 1][0]
            ) or not max(self.path[i][1], self.path[i + 1][1]) > normalPoint[1] > min(
                self.path[i][1], self.path[i + 1][1]
            ):
                normalPoint = b
            d = np.linalg.norm(va - (normalPoint - a))
            if d < d_optimal:
                d_optimal = d
                desirePoint = normalPoint
                index = i

        desire_v = np.substract(futurePoint, [x, y]) + np.substract(desirePoint, [x, y])
        return desire_v, index, d_optimal

    def bang_bang_control(
        self, v, Rmin, currentPosition, currentHeading, desirePoint, c=0
    ):
        desireHeading = arctan2(
            desirePoint[1] - currentPosition[1], desirePoint[0] - currentPosition[0]
        )
        relativeAngle = desireHeading - currentHeading  # Unit: [+- pi
        error_of_heading = (
            relativeAngle
            if 2 * abs(relativeAngle) <= 2 * pi
            else -(relativeAngle / abs(relativeAngle)) * (2 * pi - abs(relativeAngle))
        )
        if error_of_heading > c:
            u = 1
        elif error_of_heading < -c:
            u = -1
        else:
            u = 0
        return u * v * Rmin**-1, error_of_heading

    def PID_control(
        self, v, Rmin, currentPosition, currentHeading, desirePoint, pre_error=None
    ):
        desireHeading = arctan2(
            desirePoint[1] - currentPosition[1], desirePoint[0] - currentPosition[0]
        )
        relativeAngle = desireHeading - currentHeading  # Unit: [+- pi]
        error_of_heading = (
            relativeAngle
            if abs(relativeAngle) <= pi
            else -(relativeAngle / abs(relativeAngle)) * (2 * pi - abs(relativeAngle))
        )
        if not pre_error:
            "P control"
            u = self.Kp * error_of_heading
        else:
            "PD control"
            u = self.Kp * error_of_heading + self.Kd * (error_of_heading - pre_error)

        omega_max = v * Rmin**-1
        if u > omega_max:
            u = omega_max
        elif u < -omega_max:
            u = -omega_max
        return u, error_of_heading

    def LQR_control(
        self, position, heading, desirePoint, Q, R, Rmin, e, pe, pth_e, tv, v, dt=0.1
    ):
        """
        Return utheta, us, pe, pth_e
            referance: atsushisakai.github.io/PythonRobotics/
        """
        desireHeading = arctan2(
            desirePoint[1] - position[1], desirePoint[0] - position[0]
        )
        relativeAngle = desireHeading - heading  # Unit: [+- pi]
        th_e = (
            relativeAngle
            if 2 * abs(relativeAngle) <= 2 * pi
            else -(relativeAngle / abs(relativeAngle)) * (2 * pi - abs(relativeAngle))
        )

        # A = [1.0, dt , 0.0, 0.0, 0.0
        #      0.0, 0.0, v  , 0.0, 0.0
        #      0.0, 0.0, 1.0, dt , 0.0
        #      0.0, 0.0, 0.0, 0.0, 0.0
        #      0.0, 0.0, 0.0, 0.0, 1.0]
        A = np.zeros((5, 5))
        A[0, 0] = 1.0
        A[0, 1] = dt
        A[1, 2] = v
        A[2, 2] = 1.0
        A[2, 3] = dt
        A[4, 4] = 1.0

        # B = [0.0, 0.0
        #      0.0, 0.0
        #      0.0, 0.0
        #      v/R, 0.0
        #      0.0, dt ]
        B = np.zeros((5, 2))
        B[3, 0] = v / Rmin
        B[4, 1] = dt

        K, _, _ = dlqr(A, B, Q, R)

        # state vector
        # x = [e, dot_e, th_e, dot_th_e, delta_v]
        # e: lateral distance to the path
        # dot_e: derivative of e
        # th_e: angle difference to the path
        # dot_th_e: derivative of th_e
        # delta_v: difference between current speed and target speed
        x = np.zeros((5, 1))
        x[0, 0] = e
        x[1, 0] = (e - pe) / dt
        x[2, 0] = th_e
        x[3, 0] = (th_e - pth_e) / dt
        x[4, 0] = v - tv

        # input vector
        # u = [delta, accel]
        # delta: steering angle
        # accel: acceleration
        ustar = -K @ x

        # steering input
        delta = ustar[0, 0]

        # accel input
        accel = ustar[1, 0]

        return delta, accel, e, th_e


def solve_dare(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    x = Q
    x_next = Q
    max_iter = 150
    eps = 0.01

    for i in range(max_iter):
        x_next = A.T @ x @ A - A.T @ x @ B @ la.inv(R + B.T @ x @ B) @ B.T @ x @ A + Q
        if (abs(x_next - x)).max() < eps:
            break
        x = x_next
    return x_next


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_dare(A, B, Q, R)

    # compute the LQR gain
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

    eig_result = la.eig(A - B @ K)

    return K, X, eig_result[0]
