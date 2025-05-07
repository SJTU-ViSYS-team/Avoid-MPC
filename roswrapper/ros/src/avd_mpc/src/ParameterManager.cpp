#include "ParameterManager.h"
#include <XmlRpcValue.h>

ParameterManager::ParameterManager() : isInit(false) {
}
void ParameterManager::init(ros::NodeHandle &nodeHandle) {
    SetupPerceptionParam(nodeHandle);
    SetupConParam(nodeHandle);
    SetupTaskParam(nodeHandle);
    isInit = true;
}
void ParameterManager::SetupPerceptionParam(ros::NodeHandle &nodeHandle) {
    nodeHandle.getParam("visualize", perception.visualize);
    XmlRpc::XmlRpcValue matTbc;
    ROS_INFO("Read Tbc");
    if (!nodeHandle.getParam("T_b_c", matTbc)) {
        ROS_ERROR("[MPC] Tbc is not an array.");
        return;
    }
    if (matTbc.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("[MPC] Tbc is not an array.");
        return;
    }
    int tbcRows = matTbc.size();
    int tbcCols = matTbc[0].size();
    if (tbcRows != 4 || tbcCols != 4) {
        ROS_ERROR("[MPC] Tbc size is not 4x4.");
        return;
    }
    perception.Tbc = Eigen::Matrix4d::Zero();
    for (int i = 0; i < tbcRows; i++) {
        for (int j = 0; j < tbcCols; j++) {
            if (matTbc[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                perception.Tbc(i, j) =
                    static_cast<double>(static_cast<int>(matTbc[i][j]));
            } else if (matTbc[i][j].getType() ==
                       XmlRpc::XmlRpcValue::TypeDouble) {
                perception.Tbc(i, j) = static_cast<double>(matTbc[i][j]);
            } else {
                ROS_ERROR("[MPC] 'T_b_c' contains non-numeric elements.");
                return;
            }
        }
    }
    nodeHandle.getParam("fx", perception.fx);
    nodeHandle.getParam("fy", perception.fy);
    nodeHandle.getParam("cx", perception.cx);
    nodeHandle.getParam("cy", perception.cy);
    nodeHandle.getParam("pixel2meter", perception.pixel2Meter);
    nodeHandle.getParam("depth_max", perception.depthMax);
    nodeHandle.getParam("depth_min", perception.depthMin);
    nodeHandle.getParam("resize_scale", perception.resizeScale);
    nodeHandle.getParam("keyframe_th_dist", perception.keyframeDistanceTh);
    nodeHandle.getParam("keyframe_th_count", perception.keyframeCountTh);
    nodeHandle.getParam("max_frame_count", perception.maxFrameCount);
}
void ParameterManager::SetupConParam(ros::NodeHandle &nodeHandle) {
    nodeHandle.getParam("mpc_dt", con.dt);
    nodeHandle.getParam("mpc_T", con.T);
    nodeHandle.getParam("con_dt", con.conDt);
    nodeHandle.getParam("mpc_max_iter", con.maxIter);
    nodeHandle.getParam("nearest_point_num", con.nearestPointSum);
    const std::vector<std::string> weightsName = {
        "goal_p_x", "goal_p_y", "goal_p_z", "goal_yaw",  "goal_v_x",
        "goal_v_y", "goal_v_z", "goal_a_x", "goal_a_y",  "goal_a_z",
        "path_p_x", "path_p_y", "path_p_z", "path_yaw",  "path_v_x",
        "path_v_y", "path_v_z", "path_a_x", "path_a_y",  "path_a_z",
        "u_a_x",    "u_a_y",    "u_a_z",    "u_yaw_dot", "collide_lambda"};
    std::vector<double> weights;
    for (const std::string &name : weightsName) {
        double weight;
        nodeHandle.getParam(name, weight);
        con.weights.push_back(weight);
    }
    const std::vector<std::string> tausName = {"tau_a_x", "tau_a_y", "tau_a_z",
                                               "tau_yaw_dot"};
    for (const std::string &name : tausName) {
        double tau;
        nodeHandle.getParam(name, tau);
        con.taus.push_back(tau);
    }
    const std::vector<std::string> gainsName = {"gain_a_x", "gain_a_y",
                                                "gain_a_z", "gain_yaw_dot"};
    for (const std::string &name : gainsName) {
        double gain;
        nodeHandle.getParam(name, gain);
        con.gains.push_back(gain);
    }
    nodeHandle.getParam("speed", con.speed);
    nodeHandle.getParam("mpc_so", con.soPath);
    nodeHandle.getParam("drone_radius", con.droneRadius);
    nodeHandle.getParam("safety_distance", con.safetyDistance);
    nodeHandle.getParam("a_min_z", con.aMinZ);
    nodeHandle.getParam("a_max_z", con.aMaxZ);
    nodeHandle.getParam("a_max_xy", con.aMaxXy);
    nodeHandle.getParam("a_max_yaw_dot", con.aMaxYawDot);

    nodeHandle.getParam("decay", con.decay);

    nodeHandle.getParam("use_odom_est", con.useOdomEst);
    nodeHandle.getParam("only_trust_vel", con.onlyTrustVel);

    nodeHandle.getParam("slow_down_kp", con.slowDownKp);
    nodeHandle.getParam("slow_down_kd", con.slowDownKd);
    ROS_INFO("\033[95m[MPC]\033[0m Read con config file at %s",
             con.soPath.c_str());
}
void ParameterManager::SetupTaskParam(ros::NodeHandle &nodeHandle) {
    nodeHandle.getParam("task", task.task);
    nodeHandle.getParam("speed", task.speed);
    nodeHandle.getParam("height", task.height);
    nodeHandle.getParam("goal_x", task.farestPoint);
}