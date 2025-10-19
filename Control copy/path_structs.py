import math

import numpy as np


class Node:
    def __init__(self, x, y, yaw, v, direct, config):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct
        self.config = config

    def update(self, a, delta, direct):
        config = self.config
        delta = np.clip(delta, -self.config.MAX_STEER, self.config.MAX_STEER)

        self.x += self.v * math.cos(self.yaw) * config.dt
        self.y += self.v * math.sin(self.yaw) * config.dt
        self.yaw += self.v / config.WB * math.tan(delta) * config.dt
        self.direct = direct
        self.v += self.direct * a * config.dt


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
    def __init__(self, cx, cy, config, cv=None):
        self.cx = cx
        self.cy = cy
        self.ind_end = len(self.cx) - 1
        self.index_old = None
        self.config = config
        if cv is None:
            self.cv = [config.MAX_SPEED] * len(cx)  # 默认全程最大速度
        else:
            self.cv = cv

    def target_index(self, node):
        if self.index_old is None:
            self.calc_nearest_ind(node)
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
        self.index_old = ind

    def calc_distance(self, node, ind):
        return math.hypot(node.x - self.cx[ind], node.y - self.cy[ind])
