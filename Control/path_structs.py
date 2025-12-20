import math

import numpy as np
from utils import angle_normalize, pi_2_pi


class Node:
    def __init__(self, x, y, yaw, v, direct, config):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct
        self.config = config

        self.ed = 0.0
        self.e_phi = 0.0

    def update(self, a, delta, direct, ed, e_phi):
        config = self.config
        # delta = np.clip(delta, -self.config.MAX_STEER, self.config.MAX_STEER)

        self.x += self.v * math.cos(self.yaw) * config.dt
        self.y += self.v * math.sin(self.yaw) * config.dt
        self.yaw += self.v / config.WB * math.tan(delta) * config.dt
        self.yaw = angle_normalize(self.yaw, math.pi)
        self.direct = direct
        self.v += self.direct * a * config.dt

        self.ed = ed
        self.e_phi = e_phi


class Nodes:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []
        self.direct = []

    def add(self, t, node):
        self.x.append(node.x)
        self.y.append(node.y)
        self.yaw.append(node.yaw)
        self.v.append(node.v)
        self.t.append(t)
        self.direct.append(node.direct)


class PATH:
    def __init__(self, cx, cy, cyaw, ccurv, cdirect, config, cv=None):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ccurv = ccurv
        self.cdirect = cdirect
        self.ind_end = len(self.cx) - 1
        self.index_old = None
        self.config = config
        if cv is None:
            self.cv = [config.MAX_SPEED * d for d in cdirect]  # 默认全程最大速度
        else:
            self.cv = cv

    def look_ahead_index(self, node):
        if self.index_old is None:
            self.index_old = self.calc_nearest_ind(node)
        config = self.config
        Lf = config.kf * node.v + config.Ld
        for ind in range(self.index_old, self.ind_end + 1):
            if self.calc_distance(node, ind) > Lf:
                self.index_old = ind
                return ind, Lf
        self.index_old = self.ind_end
        return self.ind_end, Lf

    def calc_nearest_ind(self, node):
        dx = [node.x - x for x in self.cx]
        dy = [node.y - y for y in self.cy]
        ind = np.argmin(np.hypot(dx, dy))
        return ind

    def calc_distance(self, node, ind):
        return math.hypot(node.x - self.cx[ind], node.y - self.cy[ind])

    def cal_ed_e_phi(self, node, target_ind):
        tx = self.cx[target_ind]
        ty = self.cy[target_ind]
        tyaw = self.cyaw[target_ind]

        # 计算目标点的法向量，指向目标点的左侧
        target_normal_vec = np.array(
            [math.cos(tyaw + math.pi / 2.0), math.sin(tyaw + math.pi / 2.0)]
        )
        # 计算节点到目标点的向量
        node_to_target_vec = np.array([node.x - tx, node.y - ty])
        # 计算横向误差，左正右负
        ed = np.dot(node_to_target_vec, target_normal_vec)
        # 计算航向误差，左正右负
        e_phi = angle_normalize(node.yaw - tyaw, math.pi)
        return ed, e_phi
