import math
from typing import List, Tuple

import numpy as np


def pi_2_pi(theta: float) -> float:
    """Normalize angle to [-pi, pi]."""
    if theta > math.pi:
        return theta - 2.0 * math.pi

    if theta < -math.pi:
        return theta + 2.0 * math.pi

    return theta


def calc_curvature_point(s1: Tuple[float, float], s2: Tuple[float, float], s3: Tuple[float, float]) -> float:
    ta = math.hypot(s2[0] - s1[0], s2[1] - s1[1])
    tb = math.hypot(s3[0] - s2[0], s3[1] - s2[1])
    M = np.array([[1, -ta, ta ** 2],
                  [1, 0, 0],
                  [1, tb, tb ** 2]])
    X = np.array([[s1[0]], [s2[0]], [s3[0]]])
    Y = np.array([[s1[1]], [s2[1]], [s3[1]]])
    A = np.linalg.solve(M, X)
    B = np.linalg.solve(M, Y)
    k = 2 * (A[2][0] * B[1][0] - A[1][0] * B[2][0]) / (A[1][0] ** 2 + B[1][0] ** 2) ** (3 / 2)

    return k


def calc_curvature(x: List[float], y: List[float]) -> List[float]:
    K = [0.0]
    x, y = map(np.asarray, (x, y))
    ta = (np.diff(x[0:-1]) ** 2 + np.diff(y[0:-1]) ** 2) ** 0.5
    tb = (np.diff(x[1:len(x)]) ** 2 + np.diff(y[1:len(y)]) ** 2) ** 0.5
    for i in range(len(ta) - 2):
        M = np.array([[1, -ta[i], ta[i] ** 2],
                      [1, 0, 0],
                      [1, tb[i], tb[i] ** 2]])
        X = np.array([[x[i]], [x[i + 1]], [x[i + 2]]])
        Y = np.array([[y[i]], [y[i + 1]], [y[i + 2]]])
        A = np.linalg.solve(M, X)
        B = np.linalg.solve(M, Y)
        k = 2 * (A[2][0] * B[1][0] - A[1][0] * B[2][0]) / \
            (A[1][0] ** 2 + B[1][0] ** 2) ** (3 / 2)
        K.append(k)
    K.append(0.0)
    K.append(0.0)
    K.append(0.0)

    return K
