import casadi as ca
import numpy as np
from os import system
import os
from matplotlib import pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

import time
from sklearn.neighbors import KDTree
import yaml
import math


class high_lvl_mpc:
    """
    MPC with high-level decision variables
    """

    def __init__(
        self,
        T,
        dt,
        nearest_point_conut=3,
        so_path="./nmpc.so",
        use_drag_coefficient=False,
    ):
        """
        Nonlinear MPC for quadrotor control
        """
        self.so_path = so_path

        # Time constant
        self._T = T
        self._dt = dt
        self._N = int(self._T / self._dt)
        self.nearest_point_conut = nearest_point_conut
        # Gravity
        self._gz = 9.81
        #
        # state dimension (px, py, pz, yaw          # quadrotor position
        #                  vx, vy, vz,
        #                  ax, ay, az)           # quadrotor linear velocity
        self._s_dim = 10
        # action dimensions (ax_cmd, ay_cmd, az_cmd, yaw_cmd)
        self._u_dim = 4
        self._obstacle_dim = 3
        self._weights_dim = self._s_dim * 2 + self._u_dim + 1
        self._initDynamics(use_drag_coefficient)

    def _initDynamics(self, use_drag_coefficient=False):
        # # # # # # # # # # # # # # # # # # #
        # ---------- Input states -----------
        # # # # # # # # # # # # # # # # # # #

        px, py, pz = ca.SX.sym("px"), ca.SX.sym("py"), ca.SX.sym("pz")
        yaw = ca.SX.sym("yaw")
        #
        vx, vy, vz = ca.SX.sym("vx"), ca.SX.sym("vy"), ca.SX.sym("vz")
        ax, ay, az = ca.SX.sym("ax"), ca.SX.sym("ay"), ca.SX.sym("az")
        # -- conctenated vector
        self._x = ca.vertcat(px, py, pz, yaw, vx, vy, vz, ax, ay, az)

        # # # # # # # # # # # # # # # # # # #
        # --------- Control Command ------------
        # # # # # # # # # # # # # # # # # # #

        ax_cmd, ay_cmd, az_cmd = (
            ca.SX.sym("ax_cmd"),
            ca.SX.sym("ay_cmd"),
            ca.SX.sym("az_cmd"),
        )
        yaw_dot = ca.SX.sym("yaw_dot")
        # -- conctenated vector
        self._u = ca.vertcat(ax_cmd, ay_cmd, az_cmd, yaw_dot)
        P = ca.SX.sym(
            "P",
            self._s_dim
            + self._s_dim * self._N
            + self.nearest_point_conut * self._obstacle_dim * self._N
            + self._s_dim
            + self._u_dim * 2
            + self._weights_dim
            + 1,
        )
        # init pose + reference waypoint* N + nearest obstacle point* N+ target state + weight+drone_radius

        # # # # # # # # # # # # # # # # # # #
        # --------- System Dynamics ---------
        # # # # # # # # # # # # # # # # # # #
        gain_start_index = -self._u_dim * 2 - self._weights_dim - 1
        tau_start_index = gain_start_index + self._u_dim
        gain = P[gain_start_index: gain_start_index + self._u_dim]
        tau = P[tau_start_index: tau_start_index + self._u_dim]
        if use_drag_coefficient:  # Greatly increases solution time
            rotmat = self.acc2rotmat(ca.vertcat(ax, ay, az + self._gz), yaw)
            air_drag = (
                rotmat
                * ca.diag([0.033, 0.033, 0.033])
                * rotmat.T
                * ca.vertcat(vx, vy, vz)
            )

        else:
            air_drag = ca.vertcat(0, 0, 0)
        x_dot = ca.vertcat(
            vx,
            vy,
            vz,
            yaw_dot,
            ax - air_drag[0],
            ay - air_drag[1],
            az - air_drag[2],
            # (gain[0] * ax_cmd - ax) * tau[0],
            # (gain[1] * ay_cmd - ay) * tau[1],
            # (gain[2] * az_cmd - self._gz - az) * tau[2],
            # The gain parameter significantly affects the operational efficiency.
            # We recommend incorporating it directly into the code during the build process for deployment
            (ax_cmd - ax) * tau[0],
            (ay_cmd - ay) * tau[1],
            (az_cmd - self._gz - az) * tau[2],
        )
        self.f = ca.Function(
            "f",
            [self._x, self._u, tau, gain],
            [x_dot],
            ["x", "u", "tau", "gain"],
            ["ode"],
        )
        # # Fold
        F = self.sys_dynamics(self._dt)
        X = ca.SX.sym("X", self._s_dim, self._N + 1)
        U = ca.SX.sym("U", self._u_dim, self._N)
        self.init_pose = P[0: self._s_dim]

        self.ref_traj = P[self._s_dim: self._s_dim + self._s_dim * self._N]
        obstacle_start_index = self._s_dim + self._s_dim * self._N
        obstacle_size = self.nearest_point_conut * self._obstacle_dim * self._N
        self.obstacles = P[obstacle_start_index: obstacle_start_index + obstacle_size]
        tartget_start_index = obstacle_start_index + obstacle_size
        self.target = P[tartget_start_index: tartget_start_index + self._s_dim]
        weights = P[-self._weights_dim - 1: -1]
        self._Q_goal = ca.diag(weights[0: self._s_dim])
        self._Q_pen = ca.diag(weights[self._s_dim: self._s_dim + self._s_dim])
        self._Q_u = ca.diag(
            weights[self._s_dim * 2: self._s_dim * 2 + self._u_dim])
        self._Q_colide = ca.diag(weights[-4:-1])
        self.avoidance_lambda = weights[-1]
        self.drone_ridius = P[-1]
        # # # # # # # # # # # # # # # # # # # #
        # # ---- Non-linear Optimization -----
        # # # # # # # # # # # # # # # # # # # #
        self.nlp_w = []  # nlp variables
        self.mpc_obj = 0  # objective
        self.nlp_g = []  # constraint functions
        fMap = F.map(self._N, "openmp")  # parallel
        X_next = fMap(X[:, : self._N], U, tau, gain)
        self.nlp_w += [X[:, 0]]
        # # starting point.
        self.nlp_g += [X[:, 0] - P[0: self._s_dim]]
        step_weight = 1.0
        for k in range(self._N):
            #
            self.nlp_w += [U[:, k]]
            vi = X[4:7, k + 1]
            # cost for tracking the goal position
            cost_goal_k, cost_gap_k, cost_collide_k = 0, 0, 0
            if k >= self._N - 1:  # The goal postion.
                delta_s_k = X[:, k + 1] - self.target
                cost_goal_k = delta_s_k.T @ self._Q_goal @ delta_s_k
            else:
                ref_index = self._s_dim * k
                x_target = self.ref_traj[ref_index: ref_index + self._s_dim]
                cos_yaw = ca.cos(x_target[3])
                sin_yaw = ca.sin(-x_target[3])
                rot = ca.SX.eye(self._s_dim)
                rot[0, 0] = cos_yaw
                rot[0, 1] = -sin_yaw
                rot[1, 0] = sin_yaw
                rot[1, 1] = cos_yaw

                rot[4, 4] = cos_yaw
                rot[4, 5] = -sin_yaw
                rot[5, 4] = sin_yaw
                rot[5, 5] = cos_yaw
                obstacle_index = self.nearest_point_conut * self._obstacle_dim * k
                for d_index in range(
                    0, self.nearest_point_conut * self._obstacle_dim, self._obstacle_dim
                ):
                    p_obstacle = self.obstacles[
                        obstacle_index
                        + d_index: obstacle_index
                        + d_index
                        + self._obstacle_dim
                    ]
                    vec2obstacle = p_obstacle - X[0:3, k + 1]
                    viToObstacle = ca.dot(
                        vi, vec2obstacle / ca.norm_2(vec2obstacle))
                    viToObstacle = ca.norm_2(viToObstacle)
                    dist = ca.norm_2(vec2obstacle) - self.drone_ridius
                    cost_collide_k += (
                        self.avoidance_lambda * self.softplus(
                            dist * -32) * viToObstacle
                    )

                delta_p_k = X[:, k + 1] - x_target
                cost_gap_k = (
                    rot @ delta_p_k).T @ self._Q_pen @ (rot @ delta_p_k)
            delta_u_k = U[:, k] - [0, 0, self._gz, 0.0]
            cost_u_k = delta_u_k.T @ self._Q_u @ delta_u_k

            self.mpc_obj = (
                self.mpc_obj + cost_goal_k + cost_u_k + cost_gap_k + cost_collide_k
            )

            # New NLP variable for state at end of interval
            self.nlp_w += [X[:, k + 1]]
            # Add equality constraint
            self.nlp_g += [X_next[:, k] - X[:, k + 1]]
            step_weight *= 1.0
        # nlp objective
        self.nlp_dict = {
            "f": self.mpc_obj,
            "x": ca.vertcat(*self.nlp_w),
            "p": P,
            "g": ca.vertcat(*self.nlp_g),
        }

        # # # # # # # # # # # # # # # # # # #
        # -- ipopt
        # # # # # # # # # # # # # # # # # # #
        self.ipopt_options = {
            "verbose": False,
            "ipopt.tol": 1e-4,
            "ipopt.acceptable_tol": 1e-8,
            "ipopt.max_iter": 10,
            "ipopt.warm_start_init_point": "yes",
            "ipopt.print_level": 0,
            "print_time": False,
        }
        self.solver = ca.nlpsol(
            "solver", "ipopt", self.nlp_dict, self.ipopt_options)

    def sigmoid(self, x):
        return 1 / (1 + ca.exp(-x))

    def relu(self, x):
        return ca.fmax(0, x)

    def softplus(self, x):
        return ca.log(1 + ca.exp(x))

    def acc2rotmat(self, acc, yaw):
        proj_xb_des = ca.veccat(ca.cos(yaw), ca.sin(yaw), 0)
        zb_des = acc / ca.norm_2(acc)
        yb_des = ca.cross(zb_des, proj_xb_des) / ca.norm_2(
            ca.cross(zb_des, proj_xb_des)
        )
        xb_des = ca.cross(yb_des, zb_des)
        rotmat = ca.SX.zeros(3, 3)
        rotmat[:, 0] = xb_des
        rotmat[:, 1] = yb_des
        rotmat[:, 2] = zb_des
        return rotmat

    def GenenateSoDescription(self):
        base_path = os.path.dirname(self.so_path)
        base_path = os.path.join(base_path, "description")
        if not os.path.exists(base_path):
            os.makedirs(base_path)

        folder_name = self.so_path.split("/")[-1].split(".")[0]
        folder_path = os.path.join(base_path, folder_name)
        os.makedirs(folder_path, exist_ok=True)
        description_yaml_pth = os.path.join(folder_path, "description.yaml")
        with open(description_yaml_pth, "w") as f:
            description = {
                "date": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                "s_dim": self._s_dim,
                "u_dim": self._u_dim,
                "obstacle_dim": self._obstacle_dim,
                "weights_dim": self._weights_dim,
                "T": self._T,
                "dt": self._dt,
                "nearest_point_conut": self.nearest_point_conut,
            }
            yaml.dump(description, f)
        os.system(f"cp {__file__} {folder_path}")

    def GenerateC(self):
        so_path_folder = os.path.dirname(self.so_path)
        if not os.path.exists(so_path_folder):
            os.makedirs(so_path_folder)
        self.solver = ca.nlpsol(
            "solver", "ipopt", self.nlp_dict, self.ipopt_options)
        #    jit compile for speed up
        print("Generating shared c file........")
        cname = self.solver.generate_dependencies("nmpc_v0.c")
        print("Generating shared library........")
        system("gcc -fPIC -shared -O3 " + cname + " -o " + self.so_path)  # -O3
        # reload compiled mpc
        self.solver = ca.nlpsol(
            "solver", "ipopt", self.so_path, self.ipopt_options)
        system("rm nmpc_v0.c")
        # Generate description file
        print("Generating description file........")
        self.GenenateSoDescription()
        print("Done")

    def solve(
        self, ref_states, gain, tau, weights, drone_radius, nlp_w0, lbw, ubw, lbg, ubg
    ):
        # # # # # # # # # # # # # # # #
        # -------- solve NLP ---------
        # # # # # # # # # # # # # # # #
        #
        self.sol = self.solver(
            x0=nlp_w0,
            lbx=lbw,
            ubx=ubw,
            p=ref_states + gain + tau + weights + [drone_radius],
            lbg=lbg,
            ubg=ubg,
        )
        #
        sol_x0 = self.sol["x"].full()
        opt_u = sol_x0[self._s_dim: self._s_dim + self._u_dim]

        nlp_w0 = sol_x0
        #
        x0_array = np.reshape(
            sol_x0[: -self._s_dim], newshape=(-1, self._s_dim + self._u_dim)
        )

        # return optimal action, and a sequence of predicted optimal trajectory.
        return opt_u, x0_array, nlp_w0

    def sys_dynamics(self, dt):
        M = 4  # refinement
        DT = dt / M
        X0 = ca.SX.sym("X", self._s_dim)
        U = ca.SX.sym("U", self._u_dim)
        tau = ca.SX.sym("tau", self._u_dim)
        gain = ca.SX.sym("gain", self._u_dim)
        # #
        X = X0
        for _ in range(M):
            # --------- RK4------------
            k1 = DT * self.f(X, U, tau, gain)
            k2 = DT * self.f(X + 0.5 * k1, U, tau, gain)
            k3 = DT * self.f(X + 0.5 * k2, U, tau, gain)
            k4 = DT * self.f(X + k3, U, tau, gain)
            #
            X = X + (k1 + 2 * k2 + 2 * k3 + k4) / 6
        # Fold
        F = ca.Function("F", [X0, U, tau, gain], [X])
        return F


def read_params_from_yaml(yaml_path):
    import yaml

    weights = []
    tau = []
    gain = []
    mpc_T = 0
    mpc_dt = 0
    with open(yaml_path, "r") as f:
        files = yaml.load(f, Loader=yaml.FullLoader)
        weights.append(files["goal_p_x"])
        weights.append(files["goal_p_y"])
        weights.append(files["goal_p_z"])
        weights.append(files["goal_yaw"])
        weights.append(files["goal_v_x"])
        weights.append(files["goal_v_y"])
        weights.append(files["goal_v_z"])
        weights.append(files["goal_a_x"])
        weights.append(files["goal_a_y"])
        weights.append(files["goal_a_z"])

        weights.append(files["path_p_x"])
        weights.append(files["path_p_y"])
        weights.append(files["path_p_z"])
        weights.append(files["path_yaw"])
        weights.append(files["path_v_x"])
        weights.append(files["path_v_y"])
        weights.append(files["path_v_z"])
        weights.append(files["path_a_x"])
        weights.append(files["path_a_y"])
        weights.append(files["path_a_z"])

        weights.append(files["u_a_x"])
        weights.append(files["u_a_y"])
        weights.append(files["u_a_z"])
        weights.append(files["u_yaw_dot"])

        weights.append(files["collide_lambda"])

        tau.append(files["tau_a_x"])
        tau.append(files["tau_a_y"])
        tau.append(files["tau_a_z"])
        tau.append(files["tau_yaw_dot"])

        gain.append(files["gain_a_x"])
        gain.append(files["gain_a_y"])
        gain.append(files["gain_a_z"])
        gain.append(files["gain_yaw_dot"])

        mpc_T = files["mpc_T"]
        mpc_dt = files["mpc_dt"]
        use_drag_coefficient = int(files["use_drag_coefficient"])
        use_drag_coefficient = True if use_drag_coefficient == 1 else False
        use_nearest_point_count = files["nearest_point_num"]
        max_iteer_num = files["mpc_max_iter"]
        drone_radius = files["drone_radius"]
    return (
        weights,
        tau,
        gain,
        mpc_T,
        mpc_dt,
        use_drag_coefficient,
        use_nearest_point_count,
        max_iteer_num,
        drone_radius,
    )


if __name__ == "__main__":
    (
        weights,
        tau,
        gain,
        mpc_T,
        mpc_dt,
        use_drag_coefficient,
        use_nearest_point_count,
        max_iter_num,
        drone_radius,
    ) = read_params_from_yaml(os.path.join(os.path.dirname(__file__), "../config", "mpc_parameters.yaml"))
    mpc = high_lvl_mpc(
        mpc_T,
        mpc_dt,
        use_nearest_point_count,
        os.path.join(os.path.dirname(__file__), "../so", "mpc_obstacle_v2.so"),
        use_drag_coefficient,
    )
    p_init = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    obstacles = []
    for obs_z in np.linspace(0, 3, 10):
        for theta in np.linspace(0, 2 * 3.14, 10):
            obs_x = 0.1 * math.cos(theta) + 1.0
            obs_y = 0.1 * math.sin(theta)
            obs_pt = np.array([obs_x, obs_y, obs_z])
            obstacles.append(obs_pt)
    obstacles = np.array(obstacles)
    obstacles_tree = KDTree(obstacles)
    p_goal = [5.0, 0.1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    aMaxZ = 20.0
    aMaxXY = 10.0
    aMaxYawDot = 10.0
    quadU0 = [0.0, 0.0, 9.81, 0.0]
    quadS0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    uMin = [-aMaxXY, -aMaxXY, -aMaxZ, -aMaxYawDot]
    uMax = [aMaxXY, aMaxXY, aMaxZ, aMaxYawDot]
    xMin = [-ca.inf for _ in range(10)]
    xMax = [ca.inf for _ in range(10)]
    gMin = [0 for _ in range(10)]
    gMax = [0 for _ in range(10)]
    nlpw0 = quadS0.copy()
    mLbw = xMin.copy()
    mUbw = xMax.copy()
    mLbg = gMin.copy()
    mUbg = gMax.copy()
    dp = (np.array(p_goal) - np.array(p_init)) / mpc._N
    nearest_obacle_indices = []
    ref_traj = []
    obstacles_list = []
    for i in range(mpc._N):
        pi = np.array(p_init) + i * dp
        ref_traj += pi.tolist()
        nearest_dist, nearest_obacle_indice_i = obstacles_tree.query(
            [pi[0:3]], k=use_nearest_point_count
        )
        for obstacle_index in range(use_nearest_point_count):
            nearest_obacle = obstacles[nearest_obacle_indice_i[0]
                                       [obstacle_index]]
            obstacles_list += nearest_obacle.tolist()
            nearest_obacle_indices.append(
                nearest_obacle_indice_i[0][obstacle_index])
        nlpw0 += quadU0 + quadS0
        mLbw += uMin
        mLbw += xMin
        mUbw += uMax
        mUbw += xMax
        mLbg += gMin
        mUbg += gMax
    mpc_p = p_init + ref_traj + obstacles_list + p_goal
    # warm up
    for i in range(100):
        mpc.solve(
            mpc_p, tau, gain, weights, drone_radius, nlpw0, mLbw, mUbw, mLbg, mUbg
        )
    time_cost = 0.0

    for i in range(max_iter_num):
        start = time.time()
        opt_u, x0_array, nlpw0 = mpc.solve(
            mpc_p, tau, gain, weights, drone_radius, nlpw0, mLbw, mUbw, mLbg, mUbg
        )
        ref_traj = []
        obstacles_list = []
        need_replan = False
        for i in range(mpc._N):
            pi = x0_array[i, 0:10]
            ref_traj += pi.tolist()
            nearest_dist, nearest_obacle_indice_i = obstacles_tree.query(
                [pi[0:3]], k=use_nearest_point_count
            )
            for obstacle_index in range(use_nearest_point_count):
                nearest_obacle = obstacles[nearest_obacle_indice_i[0]
                                           [obstacle_index]]
                obstacles_list += nearest_obacle.tolist()
                nearest_obacle_indices.append(
                    nearest_obacle_indice_i[0][obstacle_index]
                )
            if nearest_obacle_indices[i] != nearest_obacle_indice_i[0][0]:
                need_replan = True
                nearest_obacle_indices[i] = nearest_obacle_indice_i[0][0]
        mpc_p = p_init + ref_traj + obstacles_list + p_goal
        end = time.time()
        time_cost += end - start
        if not need_replan:
            break

    print(f"Time: {time_cost}")

    traj = x0_array[:, :3]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2])
    for obstacle in obstacles:
        ax.scatter(obstacle[0], obstacle[1],
                   obstacle[2], c="b", marker="o", s=5)
    ax.scatter(p_goal[0], p_goal[1], p_goal[2], c="r", marker="o", s=5)
    ax.scatter(0, 0, 1, c="g", marker="*", s=5)
    img_save_path = os.path.join(os.path.dirname(__file__), "../", "test")
    if not os.path.exists(img_save_path):
        os.mkdir(img_save_path)
    plt.savefig(os.path.join(img_save_path, "mpc.png"))
    plt.show()
    mpc.GenerateC()
