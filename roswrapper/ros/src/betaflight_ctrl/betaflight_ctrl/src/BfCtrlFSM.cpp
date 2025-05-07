#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif
#include "BfCtrlFSM.h"
#include <uav_utils/converters.h>
using namespace std;
using namespace uav_utils;

BfCtrlFSM::BfCtrlFSM(GeometricController &controller_)
    : controller(controller_) /*, thrust_curve(thrust_curve_)*/
{
    state = INIT;
    hover_pose.setZero();
}

void BfCtrlFSM::process(const ros::TimerEvent &event) {
    ros::Time now_time = ros::Time::now();
    Controller_Output_t u;
    Desired_State_t des(odom_data);
    // STEP1: state machine runs
    switch (state) {
    case INIT: {
        if (!is_init_success(now_time)) {
            return; // do nothing
        } else if (Param::get().no_odom) {
            ROS_INFO("\033[32m[bfctrl]\033[0m (No Odom Mode) INIT(L0) --> "
                     "AUTO_HOVER(L2)");
            state = AUTO_HOVER;
        } else {
            hover_des = Desired_State_t(odom_data);
            hover_des.p.z() += Param::get().takeoff_land.height;
            ROS_INFO("\033[32m[bfctrl]\033[0m INIT(L0) --> "
                     "AUTO_TAKEOFF(L1)");
            set_start_pose_for_takeoff_land(odom_data);
            state = AUTO_TAKEOFF;
        }
        break;
    }
    case AUTO_TAKEOFF: {
        des = get_takeoff_land_des(Param::get().takeoff_land.speed);
        if (abs(hover_des.p.z() - odom_data.p.z()) <
            0.1) // Try to jump to AUTO_HOVER
        {
            if (odom_data.v.norm() > 3.0) {
                ROS_ERROR("\033[32m[bfctrl]\033[0m Reject AUTO_HOVER(L2). "
                          "Odom_Vel=%fm/s, which "
                          "seems that the locolization module goes wrong!",
                          odom_data.v.norm());
                return;
            }
            state = AUTO_HOVER;
            set_hov_with_odom();
            ROS_INFO(
                "\033[32m[bfctrl]\033[0m AUTO_TAKEOFF(L1) --> AUTO_HOVER(L2)");
        } else if (cmd_is_received(now_time)) {
            state = CMD_CTRL;
            des = get_cmd_des();
            ROS_INFO(
                "\033[32m[bfctrl]\033[0m AUTO_TAKEOFF(L1) --> CMD_CTRL(L3)");
            break;
        }
        break;
    }
    case AUTO_HOVER: {
        if (cmd_is_received(now_time)) {
            state = CMD_CTRL;
            des = get_cmd_des();
            ROS_INFO("\033[32m[bfctrl]\033[0m AUTO_HOVER(L2) --> CMD_CTRL(L3)");
            break;
        } else if (check_takeoff_cmd() && allow_takeoff(now_time)) {
            state = CMD_TAKEOFF;
            set_start_pose_for_takeoff_land(odom_data);
            takeoff_land.toggle_takeoff_land_time = now_time;
            ROS_INFO("\033[32m[bfctrl]\033[0m AUTO_HOVER(L1) --> "
                     "CMD_TAKEOFF");
        } else if (check_land_cmd()) {
            state = AUTO_LAND;
            set_start_pose_for_takeoff_land(odom_data);
            ROS_INFO("\033[32m[bfctrl]\033[0m AUTO_HOVER(L2) --> AUTO_LAND");
        } else if (slow_down_is_received(now_time)) {
            slow_down.xAcc = slow_down_data.x_acc;
            slow_down.yAcc = slow_down_data.y_acc;
            slow_down.height = odom_data.p.z();
            slow_down.yaw = get_yaw_from_quaternion(odom_data.q);
            state = SLOW_DOWN;
            ROS_INFO("\033[32m[bfctrl]\033[0m AUTO_HOVER(L2) --> "
                     "SLOW_DOWN");
        }
        des = get_hover_des();
        break;
    }

    case CMD_CTRL: {
        if (!cmd_is_received(now_time) || check_land_cmd()) {
            state = AUTO_HOVER;
            set_hov_with_odom();
            des = get_hover_des();
            ROS_INFO(
                "\033[32m[bfctrl]\033[0m From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
        } else if (slow_down_is_received(now_time)) {
            slow_down.xAcc = slow_down_data.x_acc;
            slow_down.yAcc = slow_down_data.y_acc;
            slow_down.height = odom_data.p.z();
            slow_down.yaw = get_yaw_from_quaternion(odom_data.q);
            state = SLOW_DOWN;
            ROS_INFO("\033[32m[bfctrl]\033[0m CMD_CTRL(L3) --> "
                     "SLOW_DOWN");
            set_hov_with_odom();
            des = get_hover_des();
        } else {
            des = get_cmd_des();
        }
        break;
    }

    case CMD_TAKEOFF: {
        if (odom_data.p(2) >=
            takeoff_land_data.takeoff_height) // reach the desired height
        {
            state = AUTO_HOVER;
            set_hov_with_odom();
            ROS_INFO("\033[32m[bfctrl]\033[0m CMD_TAKEOFF --> AUTO_HOVER(L2)");

            takeoff_land.delay_trigger.first = true;
            takeoff_land.delay_trigger.second =
                now_time + ros::Duration(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
        } else {
            des = get_takeoff_land_des(Param::get().takeoff_land.speed);
        }

        break;
    }

    case AUTO_LAND: {
        if (odom_data.p(2) <= 0.1){
            state = AUTO_HOVER;
            set_hov_with_odom();
            des = get_hover_des();
            ROS_INFO(
                "\033[32m[bfctrl]\033[0m From AUTO_LAND to AUTO_HOVER(L2)!");
        } else if (!get_landed()) {
            des = get_takeoff_land_des(-Param::get().takeoff_land.speed);
        }
        break;
    }

    case SLOW_DOWN: {
        if (fabs(odom_data.v.x()) < 0.5 && fabs(odom_data.v.y()) < 0.5) {
            state = AUTO_HOVER;
            set_hov_with_odom();
            des = get_hover_des();
            ROS_INFO(
                "\033[32m[bfctrl]\033[0m From SLOW_DOWN to AUTO_HOVER(L2)!");
        } else {
            des = get_slow_down_des();
        }
    }
    default:
        break;
    }

    // // STEP2: estimate thrust model
    if (Param::get().thr_map.update && !isOnGround()) {
        controller.estimateThrustModel(imu_data.a, vrf_hud_Data.cur_thrust_);
    }
    double hover_percentage = controller.GetHoverThrustPercentage();

    // STEP3: solve and update new control commands
    if (state == CMD_CTRL) {
        u = controller.GeometryController(des, odom_data, cmd_data.mode);
    } else if (!Param::get().no_odom) {
        u = controller.GeometryController(
            des, odom_data, quadrotor_msgs::Command::POSITION_MODE);
    }
    // STEP4: publish control commands to mavros
    if (state == CMD_CTRL || !Param::get().no_odom) {
        publish_ctrl(u, now_time);
    }
    publish_des(des, u, now_time);
    publish_statue(now_time, hover_percentage);

}

bool BfCtrlFSM::isOnGround() {
    if (Param::get().no_odom) {
        return vrf_hud_Data.cur_thrust_ <
               Param::get().thr_map.hover_percentage / 2;
    }
    return (vrf_hud_Data.cur_thrust_ <
                Param::get().thr_map.hover_percentage / 2 &&
            odom_data.v.norm() < 0.1);
}


Desired_State_t BfCtrlFSM::get_hover_des() {
    Desired_State_t des;
    des.p = hover_pose.head<3>();
    des.v = Eigen::Vector3d::Zero();
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = hover_pose(3);
    des.yaw_rate = 0.0;

    return des;
}

Desired_State_t BfCtrlFSM::get_cmd_des() {
    Desired_State_t des;
    des.p = cmd_data.p;
    des.v = cmd_data.v;
    des.a = cmd_data.a;
    des.j = cmd_data.j;
    des.w = cmd_data.w;
    des.q = cmd_data.q;
    des.yaw = cmd_data.yaw;
    des.yaw_rate = cmd_data.yaw_rate;
    des.thrust = cmd_data.thrust;
    return des;
}

Desired_State_t BfCtrlFSM::get_takeoff_land_des(const double speed) {
    ros::Time now = ros::Time::now();
    double delta_t = (now - takeoff_land.toggle_takeoff_land_time)
                         .toSec(); // speed > 0 means takeoff

    Desired_State_t des;
    des.p = takeoff_land.start_pose.head<3>() +
            Eigen::Vector3d(0, 0, speed * delta_t);
    des.v = Eigen::Vector3d(0, 0, speed);
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = takeoff_land.start_pose(3);
    des.yaw_rate = 0.0;

    return des;
}

Desired_State_t BfCtrlFSM::get_auto_takeoff_des(const double speed_max,
                                                const double height_max) {
    ros::Time now = ros::Time::now();
    double delta_t = (now - takeoff_land.toggle_takeoff_land_time)
                         .toSec(); // speed > 0 means takeoff
    double t_coef = 4 * speed_max / height_max;
    double height = height_max / (1 + std::exp(-t_coef * (delta_t - 5)));
    double speed = t_coef * (height_max - height) * height / height_max;
    Desired_State_t des;
    des.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0, 0, height);
    des.v = Eigen::Vector3d(0, 0, speed);
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = takeoff_land.start_pose(3);
    des.yaw_rate = 0.0;

    return des;
}
void BfCtrlFSM::set_hov_with_odom() {
    hover_pose.head<3>() = odom_data.p;
    hover_pose(3) = get_yaw_from_quaternion(odom_data.q);

    last_set_hover_pose_time = ros::Time::now();
}

Desired_State_t BfCtrlFSM::get_slow_down_des() {
    Desired_State_t des;
    double dt = 1. / Param::get().ctrl_freq_max;
    Eigen::Vector3d curVec = odom_data.v;
    Eigen::Vector3d curPos = odom_data.p;
    double x_acc = fabs(slow_down_data.x_acc * curVec.x()) / curVec.x();
    double y_acc = fabs(slow_down_data.y_acc * curVec.y()) / curVec.y();
    Eigen::Vector3d dVec(-x_acc * dt, -y_acc * dt, 0);
    if (-dVec.x() > curVec.x()) {
        dVec.x() = -curVec.x();
    } else if (-dVec.y() > curVec.y()) {
        dVec.y() = -curVec.y();
    }
    des.v = curVec + dVec;
    des.p = curPos + des.v * dt + 0.5 * dVec * dt;
    des.p.z() = slow_down.height;
    des.v.x() = 0;
    des.v.y() = 0;
    des.v.z() = 0;
    des.yaw = slow_down.yaw;
    des.yaw_rate = 0;
    return des;
}
void BfCtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom) {
    takeoff_land.start_pose.head<3>() = odom_data.p;
    takeoff_land.start_pose(3) = get_yaw_from_quaternion(odom_data.q);

    takeoff_land.toggle_takeoff_land_time = ros::Time::now();
}

bool BfCtrlFSM::cmd_is_received(const ros::Time &now_time) {
    return (now_time - cmd_data.rcv_stamp).toSec() <
           Param::get().msg_timeout.cmd;
}

bool BfCtrlFSM::odom_is_received(const ros::Time &now_time) {
    return (now_time - odom_data.rcv_stamp).toSec() <
           Param::get().msg_timeout.odom;
}

bool BfCtrlFSM::imu_is_received(const ros::Time &now_time) {
    return (now_time - imu_data.rcv_stamp).toSec() <
           Param::get().msg_timeout.imu;
}

bool BfCtrlFSM::vfr_is_received(const ros::Time &now_time) {
    return (now_time - vrf_hud_Data.rcv_stamp).toSec() <
           Param::get().msg_timeout.vfr;
}
bool BfCtrlFSM::slow_down_is_received(const ros::Time &now_time) {
    return (now_time - slow_down_data.rcv_stamp).toSec() <
           Param::get().msg_timeout.slow_down;
}
bool BfCtrlFSM::recv_new_odom() {
    if (odom_data.recv_new_msg) {
        odom_data.recv_new_msg = false;
        return true;
    }

    return false;
}

void BfCtrlFSM::publish_ctrl(const Controller_Output_t &u,
                             const ros::Time &stamp) {
    mavros_msgs::AttitudeTarget msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = std::string("world");
    if (Param::get().use_bodyrate_ctrl || is_angular_cmd_mode()) {
        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    } else {
        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    }
    Eigen::Quaterniond odom2imu = imu_data.q * odom_data.q.inverse();
    Eigen::Quaterniond quatImu = odom2imu * u.q;
    msg.orientation.x = quatImu.x();
    msg.orientation.y = quatImu.y();
    msg.orientation.z = quatImu.z();
    msg.orientation.w = quatImu.w();

    msg.body_rate.x = u.bodyrates.x();
    msg.body_rate.y = u.bodyrates.y();
    msg.body_rate.z = u.bodyrates.z();

    msg.thrust = u.thrust;

    ctrl_FCU_pub.publish(msg);
}

void BfCtrlFSM::publish_des(const Desired_State_t &des,
                            const Controller_Output_t &u,
                            const ros::Time &stamp) {
    nav_msgs::Odometry desMsg;
    desMsg.header.stamp = stamp;
    desMsg.header.frame_id = "world";
    desMsg.pose.pose.position.x = des.p.x();
    desMsg.pose.pose.position.y = des.p.y();
    desMsg.pose.pose.position.z = des.p.z();
    desMsg.twist.twist.linear.x = des.v.x();
    desMsg.twist.twist.linear.x = des.v.y();
    desMsg.twist.twist.linear.x = des.v.z();
    desMsg.pose.pose.orientation.w = u.q.w();
    desMsg.pose.pose.orientation.x = u.q.x();
    desMsg.pose.pose.orientation.y = u.q.y();
    desMsg.pose.pose.orientation.z = u.q.z();
    desMsg.twist.twist.angular.x = u.bodyrates.x();
    desMsg.twist.twist.angular.y = u.bodyrates.y();
    desMsg.twist.twist.angular.z = u.bodyrates.z();
    des_pub.publish(desMsg);
}
bool BfCtrlFSM::is_init_success(ros::Time &now_time) {
    if (!Param::get().no_odom && !odom_is_received(now_time)) {
        ROS_ERROR("\033[32m[bfctrl]\033[0m Reject AUTO_TAKEOFF(L1). No odom!");
        return false;
    } else {
        return true;
    }
}
bool BfCtrlFSM::allow_takeoff(ros::Time &now_time) {
    if (cmd_is_received(now_time)) {
        ROS_ERROR("\033[32m[bfctrl]\033[0m Reject CMD_TAKEOFF. You are sending "
                  "commands before toggling into CMD_TAKEOFF, which "
                  "is not allowed. Stop sending commands now!");
        return false;
    }  else if (Param::get().no_odom) {
        ROS_ERROR("\033[32m[bfctrl]\033[0m Reject CMD_TAKEOFF. No odom mode "
                  "not support auto "
                  "takeoff!");
        return false;
    } else {
        return true;
    }
}

void BfCtrlFSM::publish_statue(ros::Time &now_time, double hoverPercentage) {
    quadrotor_msgs::BfctrlStatue msg;
    msg.header.frame_id = "world";
    msg.header.stamp = now_time;
    msg.hover_percentage = hoverPercentage;
    msg.status = GetBfCTRLStatus();
    statue_pub.publish(msg);
}
int BfCtrlFSM::GetBfCTRLStatus() {
    switch (state) {
    case INIT:
        return quadrotor_msgs::BfctrlStatue::BFCTRL_STATUS_INIT;
    case AUTO_TAKEOFF:
        return quadrotor_msgs::BfctrlStatue::BFCTRL_STATUS_MANUAL;
    case AUTO_HOVER:
        return quadrotor_msgs::BfctrlStatue::BFCTRL_STATUS_WAITINGCMD;
    case CMD_CTRL:
        return quadrotor_msgs::BfctrlStatue::BFCTRL_STATUS_CMD;
    case CMD_TAKEOFF:
        return quadrotor_msgs::BfctrlStatue::BFCTRL_STATUS_TAKEOFF;
    case AUTO_LAND:
        return quadrotor_msgs::BfctrlStatue::BFCTRL_STATUS_LAND;
    default:
        return 255;
    }
    return 255;
}
