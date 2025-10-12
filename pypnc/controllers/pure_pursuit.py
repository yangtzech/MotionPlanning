"""Pure Pursuit controller (compatibility wrapper).

Adapted from Control/Pure_Pursuit.py to use pypnc models and utils.
"""

import math
from typing import List, Optional, Tuple

import numpy as np

# compatibility: avoid importing models/utils until needed


class Config:
    Kp = 0.3
    Ld = 2.6
    kf = 0.1
    dt = 0.1
    dist_stop = 0.7
    dc = 0.0
    RF = 3.3
    RB = 0.8
    W = 2.4
    WD = 0.7 * W
    WB = 2.5
    TR = 0.44
    TW = 0.7
    MAX_STEER = 0.30
    MAX_ACCELERATION = 5.0


class Node:
    def __init__(self, x: float, y: float, yaw: float, v: float, direct: float):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct

    def update(self, a: float, delta: float, direct: float):
        self.x += self.v * math.cos(self.yaw) * Config.dt
        self.y += self.v * math.sin(self.yaw) * Config.dt
        self.yaw += self.v / Config.WB * math.tan(delta) * Config.dt
        self.direct = direct
        self.v += self.direct * a * Config.dt

    @staticmethod
    def limit_input(delta: float) -> float:
        if delta > 1.2 * Config.MAX_STEER:
            return 1.2 * Config.MAX_STEER

        if delta < -1.2 * Config.MAX_STEER:
            return -1.2 * Config.MAX_STEER

        return delta


class PATH:
    def __init__(self, cx: List[float], cy: List[float]):
        self.cx = cx
        self.cy = cy
        self.ind_end = len(self.cx) - 1
        self.index_old: Optional[int] = None

    def target_index(self, node: Node) -> Tuple[int, float]:
        if self.index_old is None:
            self.calc_nearest_ind(node)

        Lf = Config.kf * node.v + Config.Ld

        start_ind = self.index_old if self.index_old is not None else 0
        for ind in range(start_ind, self.ind_end + 1):
            if self.calc_distance(node, ind) > Lf:
                self.index_old = ind
                return ind, Lf

        self.index_old = self.ind_end

        return self.ind_end, Lf

    def calc_nearest_ind(self, node: Node) -> None:
        dx = [node.x - x for x in self.cx]
        dy = [node.y - y for y in self.cy]
        ind = np.argmin(np.hypot(dx, dy))
        self.index_old = int(ind)

    def calc_distance(self, node: Node, ind: int) -> float:
        return math.hypot(node.x - self.cx[ind], node.y - self.cy[ind])


def pure_pursuit(node: Node, ref_path: PATH, index_old: int):
    ind, Lf = ref_path.target_index(node)
    ind = max(ind, index_old)

    tx = ref_path.cx[ind]
    ty = ref_path.cy[ind]

    alpha = math.atan2(ty - node.y, tx - node.x) - node.yaw

    delta = math.atan2(
        2.0 * Config.WB * math.sin(alpha),
        Lf,
    )

    return delta, ind


def pid_control(target_v: float, v: float, dist: float, direct: float) -> float:
    a = 0.3 * (target_v - direct * v)

    if dist < 10.0:
        if v > 3.0:
            a = -2.5
        elif v < -2.0:
            a = -1.0

    return a


def generate_path(s: List[Tuple[float, float, float]]):
    max_c = math.tan(Config.MAX_STEER) / Config.WB

    path_x, path_y, yaw, direct = [], [], [], []
    x_rec, y_rec, yaw_rec, direct_rec = [], [], [], []
    direct_flag = 1.0

    import CurvesGenerator.reeds_shepp as rs  # keep original path generator

    for i in range(len(s) - 1):
        s_x, s_y, s_yaw = s[i][0], s[i][1], np.deg2rad(s[i][2])
        g_x, g_y, g_yaw = s[i + 1][0], s[i + 1][1], np.deg2rad(s[i + 1][2])

        path_i = rs.calc_optimal_path(s_x, s_y, s_yaw, g_x, g_y, g_yaw, max_c)

        ix = path_i.x
        iy = path_i.y
        iyaw = path_i.yaw
        idirect = path_i.directions

        for j in range(len(ix)):
            if idirect[j] == direct_flag:
                x_rec.append(ix[j])
                y_rec.append(iy[j])
                yaw_rec.append(iyaw[j])
                direct_rec.append(idirect[j])
            else:
                if len(x_rec) == 0 or direct_rec[0] != direct_flag:
                    direct_flag = idirect[j]
                    continue

                path_x.append(x_rec)
                path_y.append(y_rec)
                yaw.append(yaw_rec)
                direct.append(direct_rec)
                # preserve last point as start of next segment, invert direction
                x_rec = [x_rec[-1]]
                y_rec = [y_rec[-1]]
                yaw_rec = [yaw_rec[-1]]
                direct_rec = [-direct_rec[-1]]

    path_x.append(x_rec)
    path_y.append(y_rec)
    yaw.append(yaw_rec)
    direct.append(direct_rec)

    x_all, y_all = [], []

    for ix, iy in zip(path_x, path_y):
        x_all += ix
        y_all += iy

    return path_x, path_y, yaw, direct, x_all, y_all
