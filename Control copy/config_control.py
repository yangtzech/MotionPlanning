import numpy as np

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

max_acceleration = 5.0  # [m / s^2]
max_steer_angle = np.deg2rad(40)  # [rad]
max_speed = 30 / 3.6  # [km/h]


class Config:
    # PID config
    Kp = 0.3  # proportional gain

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
    MAX_STEER = 0.30
    MAX_ACCELERATION = 5.0
    MAX_SPEED = 30.0 / 3.6  # [m/s]
