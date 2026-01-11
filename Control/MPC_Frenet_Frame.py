"""
Stanley
author: huiming zhou
"""

import copy
import math

import cvxpy
import numpy as np
from config_control import Config
from controller_base import ControlCommand, ControllerBase
from path_structs import PATH, Node
from utils import process_wheel_angle


class MPC_Frenet_FrameController(ControllerBase):
    def __init__(self, config: Config):
        super().__init__(config)
        self.config = config
        self.a_opt = None
        self.delta_opt = None

    def ComputeControlCommand(self, node: Node, reference: PATH) -> ControlCommand:
        ref_path = reference

        target_ind = ref_path.calc_nearest_ind(node)
        ed, e_phi = ref_path.cal_ed_e_phi(node, target_ind)

        z_ref = self.calc_ref_trajectory_in_T_step(node, ref_path, target_ind)

        z0 = [ed, 0.0, e_phi, 0.0, node.v]

        self.a_opt, self.delta_opt, x_opt, y_opt, yaw_opt, v_opt = (
            self.linear_mpc_control(
                z_ref,
                z0,
                node,
                self.a_opt,
                self.delta_opt,
                direct=node.direct,
                ref_path=ref_path,
            )
        )
        print("a_opt:", self.a_opt)
        print("delta_opt:", self.delta_opt)

        if self.delta_opt is not None:
            delta_exc, a_exc = self.delta_opt[0], self.a_opt[0]

        return ControlCommand(
            steer=process_wheel_angle(
                delta_exc, -self.config.MAX_STEER, self.config.MAX_STEER
            ),
            target_ind=target_ind,
            lat_error=ed,
            yaw_error=e_phi,
            acceleration=a_exc,
        )

    def calc_ref_trajectory(self, node: Node, ref_path: PATH, index: int):
        config = self.config

        z_ref = np.zeros((config.mpc.NZ, config.mpc.T + 1))

        z_ref[0, 0] = ref_path.cx[index]
        z_ref[1, 0] = ref_path.cy[index]
        z_ref[2, 0] = ref_path.cv[index]
        z_ref[3, 0] = ref_path.cyaw[index]

        dist_move = 0.0
        for i in range(1, config.mpc.T + 1):
            dist_move += abs(node.v) * config.dt
            # TODO：根据dist_move和轨迹点上的距离信息插值
            index = min(
                index + int(round(dist_move / config.mpc.d_dist)), ref_path.ind_end
            )

            z_ref[0, i] = ref_path.cx[index]
            z_ref[1, i] = ref_path.cy[index]
            z_ref[2, i] = ref_path.cv[index]
            z_ref[3, i] = ref_path.cyaw[index]

        # 展开航向角，避免在 ±pi 处跳变导致大误差
        yaw_unwrapped = np.unwrap(np.r_[node.yaw, z_ref[3, :]])
        z_ref[3, :] = yaw_unwrapped[1:]

        return z_ref

    def calc_ref_trajectory_in_T_step(self, node: Node, ref_path: PATH, index: int):
        config = self.config
        z_ref = np.zeros((config.mpc_frenet.NZ, config.mpc.T + 1))

        z_ref[4, 0] = ref_path.cv[index]
        dist_move = 0.0

        for i in range(1, config.mpc.T + 1):
            dist_move += abs(node.v) * config.dt
            ind_move = int(round(dist_move / config.mpc.d_dist))
            index = min(index + ind_move, ref_path.ind_end)

            z_ref[4, i] = ref_path.cv[index]

        return z_ref

    def linear_mpc_control(
        self, z_ref, z0, node: Node, a_old, delta_old, direct, ref_path: PATH
    ):
        config = self.config
        if a_old is None or delta_old is None:
            a_old = [0.0] * config.mpc.T
            delta_old = [0.0] * config.mpc.T

        x, y, yaw, v = None, None, None, None

        for k in range(config.mpc.max_iteration):
            print("a_old:", a_old)
            print("delta_old:", delta_old)

            z_bar = self.predict_states_in_T_step(
                node, z0, a_old, delta_old, z_ref, direct, ref_path
            )
            a_rec, delta_rec = a_old[:], delta_old[:]
            a_old, delta_old, x, y, yaw, v = self.solve_linear_mpc(
                z_ref, z_bar, z0, delta_old
            )

            du_a_max = max([abs(ia - iao) for ia, iao in zip(a_old, a_rec)])
            du_d_max = max([abs(ide - ido) for ide, ido in zip(delta_old, delta_rec)])

            if max(du_a_max, du_d_max) < config.mpc.du_res:
                break

        return a_old, delta_old, x, y, yaw, v

    def predict_states_in_T_step(
        self, node: Node, z0, a, delta, z_ref, direct, ref_path: PATH
    ):
        """
        given the current state, using the acceleration and delta strategy of last time,
        predict the states of vehicle in T steps.
        :param z0: initial state
        :param a: acceleration strategy of last time
        :param delta: delta strategy of last time
        :param z_ref: reference trajectory
        :return: predict states in T steps (z_bar, used for calc linear motion model)

        """
        P = self.config.mpc_frenet
        z_bar = z_ref * 0.0
        for i in range(P.NZ):
            z_bar[i, 0] = z0[i]
        # 使用自定义浅拷贝，避免重建并丢失额外状态
        node0 = copy.copy(node)
        # 预测使用传入的行驶方向参数，确保一致
        node0.direct = direct

        for ai, di, i in zip(a, delta, range(1, P.T + 1)):
            print("original node copy:", node0.x, node0.y, node0.yaw, node0.v)
            print("apply ai, di, direct:", ai, di, direct)
            # 使用副本节点进行状态预测，避免修改原始 node
            node0.update(ai, di, direct, 0.0, 0.0)
            print("Predicted node copy:", node0.x, node0.y, node0.yaw, node0.v)
            target_ind = ref_path.calc_nearest_ind(node0)
            ed, e_phi = ref_path.cal_ed_e_phi(node0, target_ind)
            z_bar[0, i] = ed
            z_bar[1, i] = 0.0
            z_bar[2, i] = e_phi
            z_bar[3, i] = 0.0
            z_bar[4, i] = node0.v

        return z_bar

    def solve_linear_mpc(self, z_ref, z_bar, z0, d_bar):
        """
        solve the quadratic optimization problem using cvxpy, solver: OSQP
        :param z_ref: reference trajectory (desired trajectory: [x, y, v, yaw])
        :param z_bar: predicted states in T steps
        :param z0: initial state
        :param d_bar: delta_bar
        :return: optimal acceleration and steering strategy
        """

        print("z0:", z0)
        print("z_ref:", z_ref)
        print("z_bar:", z_bar)
        config = self.config

        z = cvxpy.Variable((config.mpc_frenet.NZ, config.mpc_frenet.T + 1))
        u = cvxpy.Variable((config.mpc_frenet.NU, config.mpc_frenet.T))

        cost = 0.0
        constrains = []

        for t in range(config.mpc_frenet.T):
            cost += cvxpy.quad_form(u[:, t], config.mpc_frenet.R)
            cost += cvxpy.quad_form(z_ref[:, t] - z[:, t], config.mpc_frenet.Q)

            A, B = self.calc_linear_discrete_model(z_bar[4, t])

            constrains += [z[:, t + 1] == A @ z[:, t] + B @ u[:, t]]

            if t < config.mpc_frenet.T - 1:
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], config.mpc_frenet.Rd)
                constrains += [
                    cvxpy.abs(u[1, t + 1] - u[1, t])
                    <= config.MAX_STEER_CHANGE * config.dt
                ]

        cost += cvxpy.quad_form(
            z_ref[:, config.mpc_frenet.T] - z[:, config.mpc_frenet.T],
            config.mpc_frenet.Qf,
        )
        constrains += [z[:, 0] == z0]
        constrains += [z[4, :] <= config.MAX_SPEED]
        constrains += [z[4, :] >= config.MIN_SPEED]
        constrains += [cvxpy.abs(u[0, :]) <= config.MAX_ACCELERATION]
        constrains += [cvxpy.abs(u[1, :]) <= config.MAX_STEER]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constrains)
        prob.solve(solver=cvxpy.OSQP)

        a, delta, x, y, yaw, v = None, None, None, None, None, None

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            a = u.value[0, :]
            delta = u.value[1, :]
        else:
            print("solve states:", prob.status)
            print("Cannot solve linear mpc!")

        return a, delta, x, y, yaw, v

    def calc_linear_discrete_model(self, v):
        config = self.config
        A = np.array(
            [
                [1.0, config.dt, 0.0, 0.0, 0.0],
                [0.0, 0.0, v, 0.0, 0.0],
                [0.0, 0.0, 1.0, config.dt, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1.0],
            ]
        )

        # 输入向量 u = [a, delta]
        # 期望的映射：
        #   - 速度 v 的更新由加速度 a 影响：v_{t+1} = v_t + dt * a_t
        #   - 航向误差角速度 e_phi_dot 由转角 delta 影响：e_phi_dot = (v / WB) * delta
        # 原实现中将两者的作用顺序反了，导致"加速度"控制作用在航向角速度、"转角"控制作用在速度，
        # 优化器会给出饱和的转角而车辆速度几乎不变。
        B = np.array(
            [
                [0.0, 0.0],  # ed 无直接受控项
                [0.0, 0.0],  # ed_dot 无直接受控项
                [0.0, 0.0],  # e_phi 无直接受控项
                [0.0, v / config.WB],  # e_phi_dot 受 delta 影响
                [config.dt, 0.0],  # v 受 a 影响
            ]
        )

        return A, B
