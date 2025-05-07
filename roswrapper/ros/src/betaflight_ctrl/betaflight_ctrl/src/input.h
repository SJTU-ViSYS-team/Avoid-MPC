#ifndef __INPUT_H
#define __INPUT_H

#include <Eigen/Dense>
#include <ros/ros.h>

#include "BfCtrlParam.h"
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include <quadrotor_msgs/Command.h>
#include <quadrotor_msgs/SlowDown.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <queue>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <uav_utils/utils.h>

constexpr unsigned int ROLL_CHANNAL = 1;
constexpr unsigned int PITCH_CHANNAL = 0;
constexpr unsigned int YAW_CHANNAL = 3;
constexpr unsigned int THROTTLE_CHANNAL = 2;

class Odom_Data_t {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Vector3d w;

    Eigen::Quaterniond homeQ;
    Eigen::Vector3d homeT;       // local->global
    Eigen::Quaterniond homeQInv; // global->local
    Eigen::Matrix3d homeRInv;    // global->local
    ros::Publisher pubLocalOdom;
    nav_msgs::Odometry localOdomMsg;

    ros::Publisher pubHome;
    tf::TransformBroadcaster tfBroadcaster;
    bool recived;
    double yaw;
    nav_msgs::Odometry msg;
    ros::Time rcv_stamp;
    bool recv_new_msg;

    Odom_Data_t();
    void feed(nav_msgs::OdometryConstPtr pMsg);

private:
    void Global2Local();
    void SendLocalOdom();
};

class Imu_Data_t {
public:
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
    Eigen::Vector3d a;

    sensor_msgs::Imu msg;
    ros::Time rcv_stamp;

    Imu_Data_t();
    void feed(sensor_msgs::ImuConstPtr pMsg);
};

class State_Data_t {
public:
    mavros_msgs::State current_state;
    mavros_msgs::State state_before_offboard;

    State_Data_t();
    void feed(mavros_msgs::StateConstPtr pMsg);
};

class Command_Data_t {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    uint8_t mode;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    Eigen::Vector3d w;
    Eigen::Quaterniond q;
    double thrust;
    double yaw;
    double yaw_rate;

    quadrotor_msgs::Command msg;
    ros::Time rcv_stamp;

    Command_Data_t();
    void feed(quadrotor_msgs::CommandConstPtr pMsg);
};

class Battery_Data_t {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double volt{0.0};
    double percentage{0.0};

    sensor_msgs::BatteryState msg;
    ros::Time rcv_stamp;

    Battery_Data_t();
    void feed(sensor_msgs::BatteryStateConstPtr pMsg);
};

class Takeoff_Land_Data_t {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool triggered{false};
    uint8_t takeoff_land_cmd; // see TakeoffLand.msg for its defination
    double takeoff_height;
    quadrotor_msgs::TakeoffLand msg;
    ros::Time rcv_stamp;

    Takeoff_Land_Data_t();
    void feed(quadrotor_msgs::TakeoffLandConstPtr pMsg);
};

class vfr_hud_Data_t {
public:
    vfr_hud_Data_t();
    void feed(mavros_msgs::VFR_HUDConstPtr pMsg);
    std::queue<std::pair<ros::Time, double>> timed_thrust_;
    double cur_thrust_;
    ros::Time rcv_stamp;
};

class Slow_Down_Data_t {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    uint8_t slow_down_cmd; // see TakeoffLand.msg for its defination
    double x_acc;
    double y_acc;
    quadrotor_msgs::SlowDown msg;
    ros::Time rcv_stamp;

    Slow_Down_Data_t();
    void feed(quadrotor_msgs::SlowDownConstPtr pMsg);
};
#endif