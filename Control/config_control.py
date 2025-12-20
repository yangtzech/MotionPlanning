import numpy as np


class BaseConfig:
    # system config
    Ld = 2.6  # look ahead distance
    kf = 0.1  # look forward gain
    dt = 0.1  # T step
    dist_stop = 0.7  # stop distance
    dc = 0.0

    # vehicle config
    RF = 3.3  # [m] distance from rear to vehicle front end of vehicle
    RB = 0.8  # [m] distance from rear to vehicle back end of vehicle
    W = 2.4  # [m] width of vehicle
    WD = 0.7 * W  # [m] distance between left-right wheels
    WB = 2.5  # [m] Wheel base
    TR = 0.44  # [m] Tyre radius
    TW = 0.7  # [m] Tyre width
    MAX_STEER = np.deg2rad(20)
    MAX_ACCELERATION = 5.0
    MAX_SPEED = 30.0 / 3.6  # [m/s]

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

    dist_stop = 1.5  # stop permitted when dist to goal < dist_stop
    speed_stop = 0.5 / 3.6  # stop permitted when speed < speed_stop
    time_max = 500.0  # max simulation time
    iter_max = 5  # max iteration
    target_speed = 10.0 / 3.6  # target speed
    N_IND = 10  # search index number
    dt = 0.2  # time step
    d_dist = 0.3  # dist step
    du_res = 0.1  # threshold for stopping iteration

    # vehicle config
    RF = 3.3  # [m] distance from rear to vehicle front end of vehicle
    RB = 0.8  # [m] distance from rear to vehicle back end of vehicle
    W = 2.4  # [m] width of vehicle
    WD = 0.7 * W  # [m] distance between left-right wheels
    WB = 2.5  # [m] Wheel base
    TR = 0.44  # [m] Tyre radius
    TW = 0.7  # [m] Tyre width

    steer_max = np.deg2rad(45.0)  # max steering angle [rad]
    steer_change_max = np.deg2rad(30.0)  # maximum steering speed [rad/s]
    speed_max = 55.0 / 3.6  # maximum speed [m/s]
    speed_min = -20.0 / 3.6  # minimum speed [m/s]
    acceleration_max = 1.0  # maximum acceleration [m/s2]


class Config(BaseConfig):
    pid = PIDConfig()
    pure_pursuit = PurePursuitConfig()
    stanley = StanleyConfig()
    rear_wheel_feedback = RearWheelFeedbackConfig()
    lqr = LQRConfig()
    mpc = MPCConfig()
