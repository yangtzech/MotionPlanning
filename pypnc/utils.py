import math
from typing import List, Sequence, Tuple

import numpy as np


def pi_2_pi(theta: float) -> float:
    """Normalize angle to [-pi, pi]."""
    if theta > math.pi:
        return theta - 2.0 * math.pi

    if theta < -math.pi:
        return theta + 2.0 * math.pi

    return theta


def calc_curvature_point(
    s1: Tuple[float, float],
    s2: Tuple[float, float],
    s3: Tuple[float, float],
) -> float:
    # distances along polyline segments
    ta = math.hypot(s2[0] - s1[0], s2[1] - s1[1])
    tb = math.hypot(s3[0] - s2[0], s3[1] - s2[1])
    M = np.array(
        [
            [1, -ta, ta**2],
            [1, 0, 0],
            [1, tb, tb**2],
        ]
    )
    X = np.array([[s1[0]], [s2[0]], [s3[0]]])
    Y = np.array([[s1[1]], [s2[1]], [s3[1]]])
    A = np.linalg.solve(M, X)
    B = np.linalg.solve(M, Y)
    denom = (A[1][0] ** 2 + B[1][0] ** 2) ** (3 / 2)
    if denom == 0:
        return 0.0
    k = 2 * (A[2][0] * B[1][0] - A[1][0] * B[2][0]) / denom

    return float(k)


def calc_curvature(x: Sequence[float], y: Sequence[float]) -> List[float]:
    """Estimate curvature along discrete points.

    Accepts list-like inputs and returns a list of curvature values.
    Internally converts inputs to numpy arrays.
    """
    K: List[float] = [0.0]
    x_arr, y_arr = map(np.asarray, (x, y))
    # step distances
    ta = (np.diff(x_arr[0:-1]) ** 2 + np.diff(y_arr[0:-1]) ** 2) ** 0.5
    tb = (
        np.diff(x_arr[1 : len(x_arr)]) ** 2 + np.diff(y_arr[1 : len(y_arr)]) ** 2
    ) ** 0.5
    # iterate where a full 3-point fit is possible
    for i in range(max(0, len(ta) - 2)):
        M = np.array(
            [
                [1, -ta[i], ta[i] ** 2],
                [1, 0, 0],
                [1, tb[i], tb[i] ** 2],
            ]
        )
        X = np.array([[x_arr[i]], [x_arr[i + 1]], [x_arr[i + 2]]])
        Y = np.array([[y_arr[i]], [y_arr[i + 1]], [y_arr[i + 2]]])
        A = np.linalg.solve(M, X)
        B = np.linalg.solve(M, Y)
        denom = (A[1][0] ** 2 + B[1][0] ** 2) ** (3 / 2)
        if denom == 0:
            K.append(0.0)
        else:
            k = 2 * (A[2][0] * B[1][0] - A[1][0] * B[2][0]) / denom
            K.append(float(k))
    K.append(0.0)
    K.append(0.0)
    K.append(0.0)

    return K
