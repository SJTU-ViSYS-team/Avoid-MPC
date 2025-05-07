#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif
#include "BfCtrlParam.h"

Parameter_t::Parameter_t() {
}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh) {

    read_essential_param(nh, "rotor_drag/x", rt_drag.x);
    read_essential_param(nh, "rotor_drag/y", rt_drag.y);
    read_essential_param(nh, "rotor_drag/z", rt_drag.z);
    read_essential_param(nh, "rotor_drag/k_thrust_horz", rt_drag.k_thrust_horz);

    read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
    read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
    read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
    read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
    read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);
    read_essential_param(nh, "msg_timeout/vfr", msg_timeout.vfr);
    read_essential_param(nh, "msg_timeout/slow_down", msg_timeout.slow_down);

    read_essential_param(nh, "pose_solver", pose_solver);
    read_essential_param(nh, "mass", mass);
    read_essential_param(nh, "gra", gra);
    read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
    read_essential_param(nh, "use_bodyrate_ctrl", use_bodyrate_ctrl);
    read_essential_param(nh, "max_manual_vel", max_manual_vel);
    read_essential_param(nh, "max_manual_vel_z", max_manual_vel_z);
    read_essential_param(nh, "max_angle", max_angle);
    read_essential_param(nh, "low_voltage", low_voltage);

    read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
    read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
    read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
    read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

    read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);

    read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
    read_essential_param(nh, "auto_takeoff_land/takeoff_height",
                         takeoff_land.height);
    read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed",
                         takeoff_land.speed);
    read_essential_param(nh, "thrust_model/update", thr_map.update);
    read_essential_param(nh, "thrust_model/print_value", thr_map.print_val);
    read_essential_param(nh, "thrust_model/K1", thr_map.K1);
    read_essential_param(nh, "thrust_model/K2", thr_map.K2);
    read_essential_param(nh, "thrust_model/K3", thr_map.K3);
    read_essential_param(nh, "thrust_model/accurate_thrust_model",
                         thr_map.accurate_thrust_model);
    read_essential_param(nh, "thrust_model/hover_percentage",
                         thr_map.hover_percentage);
    read_essential_param(nh, "geo_controller/velocity_yaw", velocity_yaw);

    read_essential_param(nh, "rc_channal/gear_channal", gear_channal);

    read_essential_param(nh, "use_global_odom", use_global_odom);
    read_essential_param(nh, "no_odom", no_odom);
    ReadGeoControllerParam(nh);
    max_angle /= (180.0 / M_PI);

    if (takeoff_land.enable_auto_arm && !takeoff_land.enable) {
        takeoff_land.enable_auto_arm = false;
        ROS_ERROR("\"enable_auto_arm\" is only allowd with "
                  "\"auto_takeoff_land\" enabled.");
    }
    if (takeoff_land.no_RC &&
        (!takeoff_land.enable_auto_arm || !takeoff_land.enable)) {
        takeoff_land.no_RC = false;
        ROS_ERROR("\"no_RC\" is only allowd with both \"auto_takeoff_land\" "
                  "and \"enable_auto_arm\" enabled.");
    }

    if (thr_map.print_val) {
        ROS_WARN(
            "You should disable \"print_value\" if you are in regular usage.");
    }
};
void Parameter_t::ReadGeoControllerParam(const ros::NodeHandle &nh) {
    read_essential_param(nh, "geo_controller/drag_dx",
                         geometry_controller.drag_dx);
    read_essential_param(nh, "geo_controller/drag_dy",
                         geometry_controller.drag_dy);
    read_essential_param(nh, "geo_controller/drag_dz",
                         geometry_controller.drag_dz);
    read_essential_param(nh, "geo_controller/Kpos_x_",
                         geometry_controller.Kpos_x_);
    read_essential_param(nh, "geo_controller/Kpos_y_",
                         geometry_controller.Kpos_y_);
    read_essential_param(nh, "geo_controller/Kpos_z_",
                         geometry_controller.Kpos_z_);
    read_essential_param(nh, "geo_controller/Kvel_x_",
                         geometry_controller.Kvel_x_);
    read_essential_param(nh, "geo_controller/Kvel_y_",
                         geometry_controller.Kvel_y_);
    read_essential_param(nh, "geo_controller/Kvel_z_",
                         geometry_controller.Kvel_z_);
    read_essential_param(nh, "geo_controller/attctrl_tau_",
                         geometry_controller.attctrl_tau_);
    read_essential_param(nh, "geo_controller/max_fb_acc_",
                         geometry_controller.max_fb_acc_);
}