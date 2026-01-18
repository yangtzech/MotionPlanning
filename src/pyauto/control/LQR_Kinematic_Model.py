"""
Stanley
author: huiming zhou
"""

import numpy as np

from ..common.logger import log_controller_warning
from ..common.utils import process_wheel_angle
from .config_control import Config
from .controller_base import ControlCommand, ControllerBase
from .path_structs import PATH, Node


class LQRKinematicController(ControllerBase):
    def __init__(self, config: Config):
        super().__init__(config)
        self.config = config

    def ComputeControlCommand(self, node: Node, reference: PATH) -> ControlCommand:
        ref_path = reference

        target_ind = ref_path.calc_nearest_ind(node)
        ed, e_phi = ref_path.cal_ed_e_phi(node, target_ind)

        ts = self.config.dt
        ed_old = node.ed
        e_phi_old = node.e_phi

        matrix_ad_, matrix_bd_ = self.UpdateMatrix(node)

        matrix_r_ = np.diag(self.config.lqr.r)
        matrix_q_ = np.diag(self.config.lqr.q)

        matrix_k_ = self.SolveLQRProblem(
            matrix_ad_,
            matrix_bd_,
            matrix_q_,
            matrix_r_,
            self.config.lqr.eps,
            self.config.lqr.max_iteration,
        )

        matrix_state_ = np.zeros((self.config.lqr.state_size, 1))
        matrix_state_[0][0] = ed
        matrix_state_[1][0] = (ed - ed_old) / ts
        matrix_state_[2][0] = e_phi
        matrix_state_[3][0] = (e_phi - e_phi_old) / ts

        steering_angle_feedback = np.atan2(-matrix_k_.dot(matrix_state_)[0][0], 1.0)
        steering_angle_forward = self.ComputeFeedForward(ref_path.ccurv[target_ind])

        # 后处理
        delta = process_wheel_angle(
            steering_angle_feedback + steering_angle_forward,
            -self.config.MAX_STEER,
            self.config.MAX_STEER,
        )

        return ControlCommand(
            steer=delta,
            target_ind=target_ind,
            lat_error=ed,
            yaw_error=e_phi,
        )

    def UpdateMatrix(self, vehicle_state):
        """
        update the matrix of the linear quadratic regulator.
        :param vehicle_state: vehicle state
        :return: matrix_ad_, matrix_bd_
        """

        ts = self.config.dt

        wheelbase = self.config.WB

        v = vehicle_state.v

        state_size = self.config.lqr.state_size

        matrix_ad_ = np.zeros((state_size, state_size))

        matrix_ad_[0, 0] = 1.0
        matrix_ad_[0, 1] = ts
        matrix_ad_[1, 2] = v
        matrix_ad_[2, 2] = 1.0
        matrix_ad_[2, 3] = ts

        matrix_bd_ = np.zeros((state_size, 1))
        matrix_bd_[3, 0] = v / wheelbase

        return matrix_ad_, matrix_bd_

    def SolveLQRProblem(self, A, B, Q, R, tolerance, max_num_iteration):
        """
        iteratively calculating feedback matrix K
        :param A: matrix_a_
        :param B: matrix_b_
        :param Q: matrix_q_
        :param R: matrix_r_
        :param tolerance: lqr_eps
        :param max_num_iteration: max_iteration
        :return: feedback matrix K
        """

        assert (
            np.size(A, 0) == np.size(A, 1)
            and np.size(B, 0) == np.size(A, 0)
            and np.size(Q, 0) == np.size(Q, 1)
            and np.size(Q, 0) == np.size(A, 1)
            and np.size(R, 0) == np.size(R, 1)
            and np.size(R, 0) == np.size(B, 1)
        ), "LQR solver: one or more matrices have incompatible dimensions."

        M = np.zeros((np.size(Q, 0), np.size(R, 1)))

        AT = A.T
        BT = B.T
        MT = M.T

        P = Q
        num_iteration = 0
        diff = np.inf

        while num_iteration < max_num_iteration and diff > tolerance:
            num_iteration += 1
            P_next = AT @ P @ A - (AT @ P @ B + M) @ np.linalg.pinv(R + BT @ P @ B) @ (BT @ P @ A + MT) + Q

            # check the difference between P and P_next
            diff = (abs(P_next - P)).max()
            P = P_next

        if num_iteration >= max_num_iteration:
            log_controller_warning(f"LQR solver cannot converge to a solution, last consecutive result diff is: {diff}")

        K = np.linalg.inv(BT @ P @ B + R) @ (BT @ P @ A + MT)

        return K

    def ComputeFeedForward(self, ref_curvature):
        """
        compute feedforward steering angle
        :param ref_curvature: reference curvature
        :return: feedforward steering angle
        """
        return np.atan2(self.config.WB * ref_curvature, 1.0)
