"""
路径生成工具（供测试与仿真使用）
原 `generate_path` 从 `Pure_Pursuit.py` 提取到此处，供多个控制器复用。
"""

import math
import os
import sys

import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../MotionPlanning/")

import CurvesGenerator.reeds_shepp as rs


def generate_path(s, max_steer=0.30, wheelbase=2.5):
    """
    divide paths into some sections, in each section, the direction is the same.
    :param s: list of (x, y, yaw_deg)
    :param max_steer: maximum steer (rad)
    :param wheelbase: wheel base (m)
    :return: path_x, path_y, yaw, direct, x_all, y_all
    """

    max_c = math.tan(max_steer) / wheelbase  # max curvature

    path_x, path_y, yaw, direct, rc = [], [], [], [], []
    x_rec, y_rec, yaw_rec, direct_rec, rc_rec = [], [], [], [], []
    direct_flag = 1.0

    for i in range(len(s) - 1):
        s_x, s_y, s_yaw = s[i][0], s[i][1], np.deg2rad(s[i][2])
        g_x, g_y, g_yaw = s[i + 1][0], s[i + 1][1], np.deg2rad(s[i + 1][2])

        path_i = rs.calc_optimal_path(s_x, s_y, s_yaw, g_x, g_y, g_yaw, max_c)

        ix = path_i.x
        iy = path_i.y
        iyaw = path_i.yaw
        idirect = path_i.directions
        irc, rds = rs.calc_curvature(ix, iy, iyaw, idirect)

        for j in range(len(ix)):
            if idirect[j] == direct_flag:
                x_rec.append(ix[j])
                y_rec.append(iy[j])
                yaw_rec.append(iyaw[j])
                direct_rec.append(idirect[j])
                rc_rec.append(irc[j])
            elif len(x_rec) == 0:
                direct_flag = idirect[j]

                x_rec.append(ix[j])
                y_rec.append(iy[j])
                yaw_rec.append(iyaw[j])
                direct_rec.append(idirect[j])
                rc_rec.append(irc[j])
            else:
                # 第一次遇到方向变化，保存当前段路径，重新开始记录
                path_x.append(x_rec)
                path_y.append(y_rec)
                yaw.append(yaw_rec)
                direct.append(direct_rec)
                rc.append(rc_rec)

                direct_flag = idirect[j]

                x_rec, y_rec, yaw_rec, direct_rec, rc_rec = (
                    [x_rec[-1], ix[j]],
                    [y_rec[-1], iy[j]],
                    [yaw_rec[-1], iyaw[j]],
                    [direct_flag, direct_flag],
                    [rc_rec[-1], irc[j]],
                )

    path_x.append(x_rec)
    path_y.append(y_rec)
    yaw.append(yaw_rec)
    direct.append(direct_rec)
    rc.append(rc_rec)

    x_all, y_all = [], []
    for ix, iy in zip(path_x, path_y):
        x_all += ix
        y_all += iy

    return path_x, path_y, yaw, rc, direct, x_all, y_all
