#ifndef __BFCTRLFSM_H
#define __BFCTRLFSM_H

#include <ros/assert.h>
#include <ros/ros.h>

#include "controller.h"
#include "input.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/BfctrlStatue.h>
struct AutoTakeoffLand_t {
    bool landed{true};
    ros::Time toggle_takeoff_land_time;
    std::pair<bool, ros::Time> delay_trigger{
        std::pair<bool, ros::Time>(false, ros::Time(0))};
    Eigen::Vector4d start_pose;

    static constexpr double MOTORS_SPEEDUP_TIME =
        3.0; // motors idle running for 3 seconds before takeoff
    static constexpr double DELAY_TRIGGER_TIME =
        2.0; // Time to be delayed when reach at target height
};
struct Slown_Down_t {
    double xAcc;
    double yAcc;
    double height;
    double yaw;
};
class BfCtrlFSM {
public:
    State_Data_t state_data;
    Odom_Data_t odom_data;
    Imu_Data_t imu_data;
    Command_Data_t cmd_data;
    Battery_Data_t bat_data;
    Takeoff_Land_Data_t takeoff_land_data;
    Slow_Down_Data_t slow_down_data;
    vfr_hud_Data_t vrf_hud_Data;
    Desired_State_t hover_des;
    GeometricController &controller;

    ros::Publisher ctrl_FCU_pub;
    ros::Publisher des_pub;
    ros::Publisher statue_pub;
    ros::ServiceClient set_FCU_mode_srv;
    ros::ServiceClient arming_client_srv;

    Eigen::Vector4d hover_pose;
    ros::Time last_set_hover_pose_time;

    enum State_t {
        INIT = 0,
        AUTO_TAKEOFF, // bfctrl is deactived. FCU is controled by the remote
                      // controller only
        AUTO_HOVER,   // bfctrl is actived, it will keep the drone hover from
                      // odom measurments while waiting for commands from
                      // PositionCommand topic.
        CMD_CTRL,     // bfctrl is actived, and controling the drone.
        CMD_TAKEOFF,
        AUTO_LAND,
        SLOW_DOWN
    };

    BfCtrlFSM(GeometricController &);
    void process(const ros::TimerEvent &event);
    bool cmd_is_received(const ros::Time &now_time);
    bool odom_is_received(const ros::Time &now_time);
    bool imu_is_received(const ros::Time &now_time);
    bool vfr_is_received(const ros::Time &now_time);
    bool slow_down_is_received(const ros::Time &now_time);
    bool recv_new_odom();
    State_t get_state() {
        return state;
    }
    bool get_landed() {
        return takeoff_land.landed;
    }
    bool isOnGround();

private:
    State_t state; // Should only be changed in BfCtrlFSM::process() function!
    AutoTakeoffLand_t takeoff_land;
    Slown_Down_t slow_down;
    // ---- control related ----
    Desired_State_t get_hover_des();
    Desired_State_t get_cmd_des();
    Desired_State_t get_slow_down_des();
    // ---- auto takeoff/land ----
    void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);
    Desired_State_t get_takeoff_land_des(const double speed);
    Desired_State_t get_auto_takeoff_des(const double speed_max,
                                         const double height_max);
    // ---- tools ----
    void set_hov_with_odom();

    void publish_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
    void publish_des(const Desired_State_t &des, const Controller_Output_t &u,
                     const ros::Time &stamp);
    void publish_statue(ros::Time &now_time, double hoverPercentage);
    bool is_init_success(ros::Time &now_time);
    bool allow_takeoff(ros::Time &now_time);
    inline bool check_takeoff_cmd() {
        return Param::get().takeoff_land.enable &&
               takeoff_land_data.triggered &&
               takeoff_land_data.takeoff_land_cmd ==
                   quadrotor_msgs::TakeoffLand::TAKEOFF;
    }
    inline bool check_land_cmd() {
        return !Param::get().no_odom && takeoff_land_data.triggered &&
               takeoff_land_data.takeoff_land_cmd ==
                   quadrotor_msgs::TakeoffLand::LAND;
    }
    inline bool is_angular_cmd_mode() {
        return (state == CMD_CTRL) &&
               (cmd_data.mode == quadrotor_msgs::Command::ANGULAR_MODE);
    }
    int GetBfCTRLStatus();
};

#endif