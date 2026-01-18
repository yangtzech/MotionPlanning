"""
Stanley
author: huiming zhou
"""

import math

import cvxpy
import numpy as np

from ..common.logger import log_controller_debug, log_controller_error
from ..common.utils import process_wheel_angle
from .config_control import Config
from .controller_base import ControlCommand, ControllerBase
from .path_structs import PATH, Node


class MPC_XY_FrameController(ControllerBase):
    def __init__(self, config: Config):
        super().__init__(config)
        self.config = config
        self.a_opt = None
        self.delta_opt = None

    def ComputeControlCommand(self, node: Node, reference: PATH) -> ControlCommand:
        ref_path = reference

        target_ind = ref_path.calc_nearest_ind(node)
        ed, e_phi = ref_path.cal_ed_e_phi(node, target_ind)

        z_ref = self.calc_ref_trajectory(node, ref_path, target_ind)

        z0 = [node.x, node.y, node.v, node.yaw]

        self.a_opt, self.delta_opt, x_opt, y_opt, yaw_opt, v_opt = self.linear_mpc_control(
            z_ref, z0, self.a_opt, self.delta_opt, direct=node.direct
        )

        if self.delta_opt is not None:
            delta_exc, a_exc = self.delta_opt[0], self.a_opt[0]

        return ControlCommand(
            steer=process_wheel_angle(delta_exc, -self.config.MAX_STEER, self.config.MAX_STEER),
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
            index = min(index + int(round(dist_move / config.mpc.d_dist)), ref_path.ind_end)

            z_ref[0, i] = ref_path.cx[index]
            z_ref[1, i] = ref_path.cy[index]
            z_ref[2, i] = ref_path.cv[index]
            z_ref[3, i] = ref_path.cyaw[index]

        # 展开航向角，避免在 ±pi 处跳变导致大误差
        yaw_unwrapped = np.unwrap(np.r_[node.yaw, z_ref[3, :]])
        z_ref[3, :] = yaw_unwrapped[1:]

        return z_ref

    def linear_mpc_control(self, z_ref, z0, a_old, delta_old, direct):
        config = self.config
        if a_old is None or delta_old is None:
            a_old = [0.0] * config.mpc.T
            delta_old = [0.0] * config.mpc.T

        x, y, yaw, v = None, None, None, None

        for k in range(config.max_iteration):
            log_controller_debug(f"a_old: {a_old}")
            log_controller_debug(f"delta_old: {delta_old}")

            z_bar = self.predict_states_in_T_step(z0, a_old, delta_old, z_ref, direct)
            a_rec, delta_rec = a_old[:], delta_old[:]
            a_old, delta_old, x, y, yaw, v = self.solve_linear_mpc(z_ref, z_bar, z0, delta_old)

            du_a_max = max([abs(ia - iao) for ia, iao in zip(a_old, a_rec)])
            du_d_max = max([abs(ide - ido) for ide, ido in zip(delta_old, delta_rec)])

            if max(du_a_max, du_d_max) < config.mpc.du_res:
                break

        return a_old, delta_old, x, y, yaw, v

    def predict_states_in_T_step(self, z0, a, delta, z_ref, direct):
        """
        given the current state, using the acceleration and delta strategy of last time,
        predict the states of vehicle in T steps.
        :param z0: initial state
        :param a: acceleration strategy of last time
        :param delta: delta strategy of last time
        :param z_ref: reference trajectory
        :return: predict states in T steps (z_bar, used for calc linear motion model)
        """
        config = self.config
        z_bar = z_ref * 0.0

        for i in range(config.mpc.NZ):
            z_bar[i, 0] = z0[i]

        node = Node(x=z0[0], y=z0[1], v=z0[2], yaw=z0[3], direct=direct, config=config)

        for ai, di, i in zip(a, delta, range(1, config.mpc.T + 1)):
            node.update(ai, di, direct, 0.0, 0.0)
            z_bar[0, i] = node.x
            z_bar[1, i] = node.y
            z_bar[2, i] = node.v
            z_bar[3, i] = node.yaw

        # 展开预测航向，使线性化点与参考航向同一支系，避免在 cost/线性化中出现 ±pi 跳变
        yaw_unwrapped = np.unwrap(np.r_[z0[3], z_bar[3, :]])
        z_bar[3, :] = yaw_unwrapped[1:]

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

        log_controller_debug(f"z0: {z0}")
        log_controller_debug(f"z_ref: {z_ref}")
        log_controller_debug(f"z_bar: {z_bar}")
        config = self.config

        z = cvxpy.Variable((config.mpc.NZ, config.mpc.T + 1))
        u = cvxpy.Variable((config.mpc.NU, config.mpc.T))

        cost = 0.0
        constrains = []

        for t in range(config.mpc.T):
            cost += cvxpy.quad_form(u[:, t], config.mpc.R)
            cost += cvxpy.quad_form(z_ref[:, t] - z[:, t], config.mpc.Q)

            A, B, C = self.calc_linear_discrete_model(z_bar[2, t], z_bar[3, t], d_bar[t])

            constrains += [z[:, t + 1] == A @ z[:, t] + B @ u[:, t] + C]

            if t < config.mpc.T - 1:
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], config.mpc.Rd)
                constrains += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= config.MAX_STEER_CHANGE * config.dt]

        cost += cvxpy.quad_form(z_ref[:, config.mpc.T] - z[:, config.mpc.T], config.mpc.Qf)

        constrains += [z[:, 0] == z0]
        constrains += [z[2, :] <= config.MAX_SPEED]
        constrains += [z[2, :] >= config.MIN_SPEED]
        constrains += [cvxpy.abs(u[0, :]) <= config.MAX_ACCELERATION]
        constrains += [cvxpy.abs(u[1, :]) <= config.MAX_STEER]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constrains)
        prob.solve(solver=cvxpy.OSQP)

        a, delta, x, y, yaw, v = None, None, None, None, None, None

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            x = z.value[0, :]
            y = z.value[1, :]
            v = z.value[2, :]
            yaw = z.value[3, :]
            a = u.value[0, :]
            delta = u.value[1, :]
        else:
            log_controller_error("Cannot solve linear mpc!")

        return a, delta, x, y, yaw, v

    def calc_linear_discrete_model(self, v, phi, delta):
        """
        calc linear and discrete time dynamic model.
        :param v: speed: v_bar
        :param phi: angle of vehicle: phi_bar
        :param delta: steering angle: delta_bar
        :return: A, B, C
        """
        config = self.config

        A = np.array(
            [
                [1.0, 0.0, config.dt * math.cos(phi), -config.dt * v * math.sin(phi)],
                [0.0, 1.0, config.dt * math.sin(phi), config.dt * v * math.cos(phi)],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, config.dt * math.tan(delta) / config.WB, 1.0],
            ]
        )

        B = np.array(
            [
                [0.0, 0.0],
                [0.0, 0.0],
                [config.dt, 0.0],
                [0.0, config.dt * v / (config.WB * math.cos(delta) ** 2)],
            ]
        )

        C = np.array(
            [
                config.dt * v * math.sin(phi) * phi,
                -config.dt * v * math.cos(phi) * phi,
                0.0,
                -config.dt * v * delta / (config.WB * math.cos(delta) ** 2),
            ]
        )

        return A, B, C
