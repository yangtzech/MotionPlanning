# 基于模型预测的速度与转向控制


这是一个利用模型预测控制（MPC）进行路径跟踪的仿真示例。MPC 控制器基于线性化车辆模型同时调整速度与转向，求解采用优化建模工具 cvxpy。




## MPC 建模

状态向量：

$$
z = [x, y, v, \phi]
$$

其中，$x$ 为车辆在 $x$ 方向的位置，$y$ 为 $y$ 方向的位置，$v$ 为速度，$\phi$ 为偏航角。

输入向量：

$$
u = [a, \delta]
$$

其中，$a$ 为加速度，$\delta$ 为转向角。

MPC 控制器最小化如下用于路径跟踪的代价函数：

$$
\min \ Q_f(z_{T,ref}-z_T)^2 + Q\Sigma(z_{t,ref}-z_t)^2 + R\Sigma u_t^2 + R_d\Sigma(u_{t+1}-u_t)^2
$$

其中 $z_{ref}$ 来自目标路径与目标速度。约束条件包括：

- 线性化后的车辆模型：

	$$z_{t+1} = A z_t + B u + C$$

- 最大转向角速度：

	$$|u_{t+1}-u_t| < du_{max}$$

- 最大转向角：

	$$|u_t| < u_{max}$$

- 初始状态：

	$$z_0 = z_{0,ob}$$

- 速度上下限：

	$$v_{min} < v_t < v_{max}$$

- 输入上下限：

	$$u_{min} < u_t < u_{max}$$

对应实现可参见：

PythonRobotics/model_predictive_speed_and_steer_control.py at f51a73f47cb922a12659f8ce2d544c347a2a8156 · AtsushiSakai/PythonRobotics <https://github.com/AtsushiSakai/PythonRobotics/blob/f51a73f47cb922a12659f8ce2d544c347a2a8156/PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py#L247-L301>

## 车辆模型线性化

连续时间车辆模型：

$$
\dot{x} = v\cos(\phi), \quad
\dot{y} = v\sin(\phi), \quad
\dot{v} = a, \quad
\dot{\phi} = \frac{v \tan(\delta)}{L}
$$

其状态方程可写作：

$$
\dot{z} = \frac{\partial}{\partial z} z = f(z, u) = A' z + B' u
$$

其中：

$$
A' =
\begin{bmatrix}
\frac{\partial }{\partial x} v\cos(\phi) & \frac{\partial }{\partial y} v\cos(\phi) & \frac{\partial }{\partial v} v\cos(\phi) & \frac{\partial }{\partial \phi} v\cos(\phi)\\
\frac{\partial }{\partial x} v\sin(\phi) & \frac{\partial }{\partial y} v\sin(\phi) & \frac{\partial }{\partial v} v\sin(\phi) & \frac{\partial }{\partial \phi} v\sin(\phi)\\
\frac{\partial }{\partial x} a & \frac{\partial }{\partial y} a & \frac{\partial }{\partial v} a & \frac{\partial }{\partial \phi} a\\
\frac{\partial }{\partial x} \frac{v \tan(\delta)}{L} & \frac{\partial }{\partial y} \frac{v \tan(\delta)}{L} & \frac{\partial }{\partial v} \frac{v \tan(\delta)}{L} & \frac{\partial }{\partial \phi} \frac{v \tan(\delta)}{L}
\end{bmatrix}
 =
\begin{bmatrix}
0 & 0 & \cos(\bar{\phi}) & -\bar{v} \sin(\bar{\phi})\\
0 & 0 & \sin(\bar{\phi}) & \bar{v} \cos(\bar{\phi})\\
0 & 0 & 0 & 0\\
0 & 0 & \tfrac{\tan(\bar{\delta})}{L} & 0
\end{bmatrix}
$$

$$
B' =
\begin{bmatrix}
\frac{\partial }{\partial a} v\cos(\phi) & \frac{\partial }{\partial \delta} v\cos(\phi)\\
\frac{\partial }{\partial a} v\sin(\phi) & \frac{\partial }{\partial \delta} v\sin(\phi)\\
\frac{\partial }{\partial a} a & \frac{\partial }{\partial \delta} a\\
\frac{\partial }{\partial a} \frac{v \tan(\delta)}{L} & \frac{\partial }{\partial \delta} \frac{v \tan(\delta)}{L}
\end{bmatrix}
 =
\begin{bmatrix}
0 & 0\\
0 & 0\\
1 & 0\\
0 & \tfrac{\bar{v}}{L \cos^2(\bar{\delta})}
\end{bmatrix}
$$

使用前向欧拉离散化（步长为 $dt$）得到离散模型：

$$
z_{k+1} = z_k + f(z_k, u_k) dt
$$

在平衡点 $\bar{z}, \bar{u}$ 处做一阶泰勒展开：

$$
z_{k+1} = z_k + (f(\bar{z}, \bar{u}) + A' z_k + B' u_k - A' \bar{z} - B' \bar{u}) dt
$$

即：

$$
z_{k+1} = (I + dt A') z_k + (dt B') u_k + (f(\bar{z}, \bar{u}) - A' \bar{z} - B' \bar{u}) dt
$$

可写为标准形式：

$$
z_{k+1} = A z_k + B u_k + C
$$

其中

$$
A = I + dt A' =
\begin{bmatrix}
1 & 0 & \cos(\bar{\phi}) dt & -\bar{v} \sin(\bar{\phi}) dt\\
0 & 1 & \sin(\bar{\phi}) dt & \bar{v} \cos(\bar{\phi}) dt\\
0 & 0 & 1 & 0\\
0 & 0 & \tfrac{\tan(\bar{\delta})}{L} dt & 1
\end{bmatrix}
$$

$$
B = dt B' =
\begin{bmatrix}
0 & 0\\
0 & 0\\
dt & 0\\
0 & \tfrac{\bar{v}}{L \cos^2(\bar{\delta})} dt
\end{bmatrix}
$$

$$
C = (f(\bar{z}, \bar{u}) - A' \bar{z} - B' \bar{u}) dt
 =
\begin{bmatrix}
\bar{v} \sin(\bar{\phi}) \bar{\phi} dt\\
-\bar{v} \cos(\bar{\phi}) \bar{\phi} dt\\
0\\
-\tfrac{\bar{v} \bar{\delta}}{L \cos^2(\bar{\delta})} dt
\end{bmatrix}
$$


## 参考资料

- Vehicle Dynamics and Control | Rajesh Rajamani | Springer <http://www.springer.com/us/book/9781461414322>
- MPC Book - MPC Lab @ UC-Berkeley <https://sites.google.com/berkeley.edu/mpc-lab/mpc-course-material>

- 参考：Welcome to CVXPY 1.0 — CVXPY 1.0.6 documentation <http://www.cvxpy.org/>

- Model predictive speed and steering control <https://github.com/AtsushiSakai/PythonRobotics/blob/master/docs/modules/6_path_tracking/model_predictive_speed_and_steering_control/model_predictive_speed_and_steering_control_main.rst>