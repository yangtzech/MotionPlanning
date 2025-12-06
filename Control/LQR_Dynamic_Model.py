"""
Stanley
author: huiming zhou
"""

import numpy as np
from config_control import Config
from controller_base import ControlCommand, ControllerBase
from path_structs import PATH, Node
from utils import process_wheel_angle


class LQRDynamicController(ControllerBase):
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
        steering_angle_forward = self.ComputeFeedForward(
            node, ref_path.ccurv[target_ind], matrix_k_
        )
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

    def UpdateMatrix(self, node: Node):
        """
        calc A and b matrices of linearized, discrete system.
        :return: A, b
        """

        config = self.config

        ts_ = config.dt
        v = node.v
        mass_ = config.m_f + config.m_r

        c_f = config.c_f
        c_r = config.c_r
        l_f = config.l_f
        l_r = config.l_r
        Iz = config.Iz

        state_size = config.lqr.state_size

        matrix_a_ = np.zeros((state_size, state_size))  # continuous A matrix

        if node.direct == -1:
            """
            A matrix (Gear Reverse)
            [0.0, 0.0, 1.0 * v 0.0;
             0.0, -(c_f + c_r) / m / v, (c_f + c_r) / m,
             (l_r * c_r - l_f * c_f) / m / v;
             0.0, 0.0, 0.0, 1.0;
             0.0, (lr * cr - lf * cf) / i_z / v, (l_f * c_f - l_r * c_r) / i_z,
             -1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z / v;]
            """

            matrix_a_[0][1] = 0.0
            matrix_a_[0][2] = 1.0 * v
        else:
            """
            A matrix (Gear Drive)
            [0.0, 1.0, 0.0, 0.0;
             0.0, -(c_f + c_r) / m / v, (c_f + c_r) / m,
             (l_r * c_r - l_f * c_f) / m / v;
             0.0, 0.0, 0.0, 1.0;
             0.0, (lr * cr - lf * cf) / i_z / v, (l_f * c_f - l_r * c_r) / i_z,
             -1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z / v;]
            """

            matrix_a_[0][1] = 1.0
            matrix_a_[0][2] = 0.0

        matrix_a_[1][1] = -1.0 * (+c_r) / mass_ / v
        matrix_a_[1][2] = (c_f + c_r) / mass_
        matrix_a_[1][3] = (l_r * c_r - l_f * c_f) / mass_ / v
        matrix_a_[2][3] = 1.0
        matrix_a_[3][1] = (l_r * c_r - l_f * c_f) / Iz / v
        matrix_a_[3][2] = (l_f * c_f - l_r * c_r) / Iz
        matrix_a_[3][3] = -1.0 * (l_f**2 * c_f + l_r**2 * c_r) / Iz / v

        # Tustin's method (bilinear transform)
        matrix_i = np.eye(state_size)  # identical matrix
        matrix_ad_ = np.linalg.pinv(matrix_i - ts_ * 0.5 * matrix_a_) @ (
            matrix_i + ts_ * 0.5 * matrix_a_
        )  # discrete A matrix

        # b = [0.0, c_f / m, 0.0, l_f * c_f / I_z].T
        matrix_b_ = np.zeros((state_size, 1))  # continuous b matrix
        matrix_b_[1][0] = c_f / mass_
        matrix_b_[3][0] = l_f * c_f / Iz
        matrix_bd_ = matrix_b_ * ts_  # discrete b matrix

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
            P_next = (
                AT @ P @ A
                - (AT @ P @ B + M) @ np.linalg.pinv(R + BT @ P @ B) @ (BT @ P @ A + MT)
                + Q
            )

            # check the difference between P and P_next
            diff = (abs(P_next - P)).max()
            P = P_next

        if num_iteration >= max_num_iteration:
            print(
                "LQR solver cannot converge to a solution",
                "last consecutive result diff is: ",
                diff,
            )

        K = np.linalg.inv(BT @ P @ B + R) @ (BT @ P @ A + MT)

        return K

    def ComputeFeedForward(self, node: Node, ref_curvature, matrix_k_):
        """
        calc feedforward control term to decrease the steady error.
        :param node: node
        :param ref_curvature: curvature of the target point in ref trajectory
        :param matrix_k_: feedback matrix K
        :return: feedforward term
        """
        config = self.config
        # 计算前馈项
        mass_ = config.m_f + config.m_r
        wheelbase_ = config.WB
        l_f_ = config.l_f
        l_r_ = config.l_r
        c_f_ = config.c_f
        c_r_ = config.c_r

        kv = (
            l_r_ * mass_ / 2.0 / c_f_ / wheelbase_
            - l_f_ * mass_ / 2.0 / c_r_ / wheelbase_
        )

        v = node.v

        if node.direct == -1:
            steer_angle_feedforward = np.atan2(wheelbase_ * ref_curvature, 1.0)
        else:
            steer_angle_feedforward = (
                wheelbase_ * ref_curvature
                + kv * v * v * ref_curvature
                - matrix_k_[0][2]
                * (
                    l_r_ * ref_curvature
                    - l_f_ * mass_ * v * v * ref_curvature / 2.0 / c_r_ / wheelbase_
                )
            )

        return steer_angle_feedforward
