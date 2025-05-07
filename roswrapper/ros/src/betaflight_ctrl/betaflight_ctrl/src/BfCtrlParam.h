#ifndef __BFCTRLPARAM_H
#define __BFCTRLPARAM_H

#include <ros/ros.h>

class Parameter_t {
public:
    static Parameter_t &get() {
        static Parameter_t instance;
        return instance;
    }

    struct RotorDrag {
        double x, y, z;
        double k_thrust_horz;
    };

    struct MsgTimeout {
        double odom;
        double rc;
        double cmd;
        double imu;
        double bat;
        double vfr;
        double slow_down;
    };

    struct ThrustMapping {
        bool update;
        bool print_val;
        double K1;
        double K2;
        double K3;
        bool accurate_thrust_model;
        double hover_percentage;
    };

    struct RCReverse {
        bool roll;
        bool pitch;
        bool yaw;
        bool throttle;
    };

    struct AutoTakeoffLand {
        bool enable;
        bool enable_auto_arm;
        bool no_RC;
        double height;
        double speed;
    };
    struct GeoController {
        double drag_dx;
        double drag_dy;
        double drag_dz;
        double Kpos_x_;
        double Kpos_y_;
        double Kpos_z_;
        double Kvel_x_;
        double Kvel_y_;
        double Kvel_z_;
        double attctrl_tau_;
        double max_fb_acc_;
    };
    RotorDrag rt_drag;
    MsgTimeout msg_timeout;
    RCReverse rc_reverse;
    ThrustMapping thr_map;
    AutoTakeoffLand takeoff_land;
    GeoController geometry_controller;
    int pose_solver;
    double mass;
    double gra;
    double max_angle;
    double ctrl_freq_max;
    double max_manual_vel;
    double max_manual_vel_z;
    double low_voltage;
    bool use_bodyrate_ctrl;
    bool velocity_yaw;
    int gear_channal;

    bool use_global_odom;
    bool no_odom;
    void config_from_ros_handle(const ros::NodeHandle &nh);

private:
    Parameter_t();
    template <typename TName, typename TVal>
    void read_essential_param(const ros::NodeHandle &nh, const TName &name,
                              TVal &val) {
        if (nh.getParam(name, val)) {
            // pass
        } else {
            ROS_ERROR_STREAM("Read param: " << name << " failed.");
            ROS_BREAK();
        }
    };
    void ReadGeoControllerParam(const ros::NodeHandle &nh);
};
using Param = Parameter_t;
#endif