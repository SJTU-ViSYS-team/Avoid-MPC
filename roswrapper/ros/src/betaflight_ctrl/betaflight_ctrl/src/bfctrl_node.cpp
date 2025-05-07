#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif
#include "BfCtrlFSM.h"
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Bool.h>
void mySigintHandler(int sig) {
    ROS_INFO("\033[32m[bfctrl]\033[0m exit...");
    ros::shutdown();
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "bfctrl");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    Param::get().config_from_ros_handle(nh);

    GeometricController controller;
    BfCtrlFSM fsm(controller);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10,
        boost::bind(&State_Data_t::feed, &fsm.state_data, _1));

    ros::Publisher local_pub =
        nh.advertise<nav_msgs::Odometry>("local_odom", 100);
    fsm.odom_data.pubLocalOdom = local_pub;
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "odom", 100, boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub = nh.subscribe<quadrotor_msgs::Command>(
        "cmd", 100, boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(
        "/mavros/imu/data", 100,
        boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1), ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());
    ros::Publisher home_pub =
        nh.advertise<nav_msgs::Odometry>("home_pose", 100);
    fsm.odom_data.pubHome = home_pub;
    ros::Publisher statue_pub =
        nh.advertise<quadrotor_msgs::BfctrlStatue>("statue", 100);

    ros::Subscriber bat_sub = nh.subscribe<sensor_msgs::BatteryState>(
        "/mavros/battery", 100,
        boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1),
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber takeoff_land_sub =
        nh.subscribe<quadrotor_msgs::TakeoffLand>(
            "takeoff_land", 100,
            boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
            ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>(
        "/mavros/vfr_hud/", 100,
        boost::bind(&vfr_hud_Data_t::feed, &fsm.vrf_hud_Data, _1),
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    ros::Subscriber slow_down_sub = nh.subscribe<quadrotor_msgs::SlowDown>(
        "slow_down", 100,
        boost::bind(&Slow_Down_Data_t::feed, &fsm.slow_down_data, _1),
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>(
        "/mavros/setpoint_raw/attitude", 10);
    fsm.des_pub = nh.advertise<nav_msgs::Odometry>("des", 10);
    fsm.statue_pub = statue_pub;
    if (Param::get().no_odom) {
        ROS_WARN("\033[32m[bfctrl]\033[0m No Odom Mode, be careful!");
    } else {
        ROS_INFO("\033[32m[bfctrl]\033[0m Waiting for odom");
        while (!fsm.odom_is_received(ros::Time::now())) {
            ros::spinOnce();
        }
        ROS_INFO("\033[32m[bfctrl]\033[0m Odom received");
    }
    ros::Duration(1.0).sleep();
    ros::Timer processTimer =
        nh.createTimer(ros::Duration(0.02), &BfCtrlFSM::process,
                       &fsm); // Define timer for constant loop rate
    ros::spin();

    return 0;
}
