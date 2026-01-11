import numpy as np


class BaseConfig:
    # system config
    Ld = 2.6  # look ahead distance
    kf = 0.1  # look forward gain
    dt = 0.1  # T step
    dist_stop = 0.7  # stop distance: stop permitted when dist to goal < dist_stop
    # speed_stop = 0.5 / 3.6  # stop permitted when speed < speed_stop
    dc = 0.0

    # vehicle config
    RF = 3.3  # [m] distance from rear to vehicle front end of vehicle
    RB = 0.8  # [m] distance from rear to vehicle back end of vehicle
    W = 2.4  # [m] width of vehicle
    WD = 0.7 * W  # [m] distance between left-right wheels
    WB = 2.5  # [m] Wheel base
    TR = 0.44  # [m] Tyre radius
    TW = 0.7  # [m] Tyre width
    MAX_STEER = np.deg2rad(45.0)  # [rad] max steering angle
    MAX_STEER_CHANGE = np.deg2rad(30.0)  # [rad/s] max steering speed
    MAX_ACCELERATION = 1.0  # [m/s2] maximum acceleration
    MAX_SPEED = 55.0 / 3.6  # [m/s] maximum speed
    MIN_SPEED = -20.0 / 3.6  # [m/s] minimum speed

    target_speed = 10.0 / 3.6  # target speed

    # Vehicle config
    wheelbase = 2.33  # wheel base: front to rear axle [m]
    wheeldist = 1.85  # wheel dist: left to right wheel [m]
    v_w = 2.33  # vehicle width [m]
    r_b = 0.80  # rear to back [m]
    r_f = 3.15  # rear to front [m]
    t_r = 0.40  # tire radius [m]
    t_w = 0.30  # tire width [m]

    c_f = 155494.663  # [N / rad]
    c_r = 155494.663  # [N / rad]
    m_f = 570  # [kg]
    m_r = 570  # [kg]
    l_f = 1.165  # [m]
    l_r = 1.165  # [m]
    Iz = 1436.24  # [kg m2]

    # Controller Config
    ts = 0.10  # [s]
    max_iteration = 150
    eps = 0.01
    time_max = 500.0  # max simulation time

    matrix_q = [1.0, 0.0, 1.0, 0.0]
    matrix_r = [1.0]

    state_size = 4

    # max_acceleration = 5.0  # [m / s^2]
    # max_steer_angle = np.deg2rad(40)  # [rad]
    # max_speed = 30 / 3.6  # [km/h]


class PIDConfig:
    # PID config
    Kp = 0.3  # proportional gain


class PurePursuitConfig:
    Ld = 2.6
    kf = 0.1
    # ...PurePursuit专用参数...


class StanleyConfig:
    k = 0.5
    # ...Stanley专用参数...


class RearWheelFeedbackConfig:
    K_theta = 1.0
    K_e = 0.5


class LQRConfig:
    state_size = 4
    q = [0.5, 0.0, 1.0, 0.0]
    r = [1.0]
    eps = 0.01  # convergence threshold for LQR
    max_iteration = 150  # maximum number of iterations for LQR


class MPCConfig:
    NZ = 4  # state vector: z = [x, y, v, phi]
    NU = 2  # input vector: u = [acceleration, steer]
    T = 6  # finite time horizon length

    # MPC config
    Q = np.diag([1.0, 1.0, 1.0, 1.0])  # penalty for states
    Qf = np.diag([1.0, 1.0, 1.0, 1.0])  # penalty for end state
    R = np.diag([0.01, 0.1])  # penalty for inputs
    Rd = np.diag([0.01, 0.1])  # penalty for change of inputs

    max_iteration = 10  # max iteration

    N_IND = 10  # search index number
    d_dist = 0.2  # dist step
    du_res = 0.1  # threshold for stopping iteration


class MPC_Frenet_FrameConfig(MPCConfig):
    NZ = 5  # state vector: z = [e, e_dot, theta_e, theta_e_dot, v]
    Q = np.diag([1.0, 1.0, 1.0, 1.0, 1.0])  # penalty for states
    Qf = np.diag([1.0, 1.0, 1.0, 1.0, 1.0])  # penalty for end state


class Config(BaseConfig):
    pid = PIDConfig()
    pure_pursuit = PurePursuitConfig()
    stanley = StanleyConfig()
    rear_wheel_feedback = RearWheelFeedbackConfig()
    lqr = LQRConfig()
    mpc = MPCConfig()
    mpc_frenet = MPC_Frenet_FrameConfig()
