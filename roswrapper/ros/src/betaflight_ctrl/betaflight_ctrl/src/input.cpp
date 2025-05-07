#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif
#include "input.h"
#include <std_msgs/Bool.h>

Odom_Data_t::Odom_Data_t() {
    rcv_stamp = ros::Time(0);
    q.setIdentity();
    recv_new_msg = false;
    recived = false;
};

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg) {
    ros::Time now = ros::Time::now();

    msg = *pMsg;
    rcv_stamp = now;
    recv_new_msg = true;
    uav_utils::extract_odometry(pMsg, p, v, q, w);

    if (!recived) {
        recived = true;
        homeT = p;
        double sinr_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
        double cosr_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
        double yaw = std::atan2(sinr_cosp, cosr_cosp);
        homeQ =
            Eigen::Quaterniond(std::cos(yaw / 2.), 0, 0, std::sin(yaw / 2.));
        homeQInv = homeQ.inverse();
        homeRInv = homeQInv.toRotationMatrix();
    }
// #define VEL_IN_BODY
#ifdef VEL_IN_BODY /* Set to 1 if the velocity in odom topic is relative to    \
                      current body frame, not to world frame.*/
    Eigen::Quaternion<double> wRb_q(
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Matrix3d wRb = wRb_q.matrix();
    v = wRb * v;

    static int count = 0;
    if (count++ % 500 == 0)
        ROS_WARN("VEL_IN_BODY!!!");
#endif
    static int count = 0;
    if (count++ % 100 == 0) {
        nav_msgs::Odometry homePosMsg;
        homePosMsg.header.frame_id = "world";
        homePosMsg.header.stamp = ros::Time::now();
        homePosMsg.pose.pose.orientation.w = homeQInv.w();
        homePosMsg.pose.pose.orientation.x = homeQInv.x();
        homePosMsg.pose.pose.orientation.y = homeQInv.y();
        homePosMsg.pose.pose.orientation.z = homeQInv.z();

        homePosMsg.pose.pose.position.x = homeT.x();
        homePosMsg.pose.pose.position.y = homeT.y();
        homePosMsg.pose.pose.position.z = homeT.z();
        pubHome.publish(homePosMsg);
    }
    if (!Param::get().use_global_odom) {
        Global2Local();
        localOdomMsg.header = pMsg->header;
        localOdomMsg.header.frame_id = "world";
        SendLocalOdom();
    }
    // check the frequency
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    if ((now - last_clear_count_time).toSec() > 1.0) {
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count++;
}
void Odom_Data_t::SendLocalOdom() {
    localOdomMsg.pose.pose.position.x = p.x();
    localOdomMsg.pose.pose.position.y = p.y();
    localOdomMsg.pose.pose.position.z = p.z();
    localOdomMsg.pose.pose.orientation.w = q.w();
    localOdomMsg.pose.pose.orientation.x = q.x();
    localOdomMsg.pose.pose.orientation.y = q.y();
    localOdomMsg.pose.pose.orientation.z = q.z();
    localOdomMsg.twist.twist.angular.x = w.x();
    localOdomMsg.twist.twist.angular.y = w.y();
    localOdomMsg.twist.twist.angular.z = w.z();
    localOdomMsg.twist.twist.linear.x = v.x();
    localOdomMsg.twist.twist.linear.y = v.y();
    localOdomMsg.twist.twist.linear.z = v.z();
    pubLocalOdom.publish(localOdomMsg);
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(p.x(), p.y(), p.z()));
    tf::Quaternion q_tf(q.x(), q.y(), q.z(), q.w());
    transform.setRotation(q_tf);
    tfBroadcaster.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "body"));
}
void Odom_Data_t::Global2Local() {
    p = homeRInv * (p - homeT);
    q = homeQInv * q;
    v = homeRInv * v;
    w = homeRInv * w;
}
Imu_Data_t::Imu_Data_t() {
    rcv_stamp = ros::Time(0);
}

void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg) {
    ros::Time now = ros::Time::now();

    msg = *pMsg;
    rcv_stamp = now;

    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;

    // check the frequency
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    if ((now - last_clear_count_time).toSec() > 1.0) {
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count++;
}

State_Data_t::State_Data_t() {
}

void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg) {

    current_state = *pMsg;
}

Command_Data_t::Command_Data_t() {
    rcv_stamp = ros::Time(0);
}

void Command_Data_t::feed(quadrotor_msgs::CommandConstPtr pMsg) {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;

    j(0) = msg.jerk.x;
    j(1) = msg.jerk.y;
    j(2) = msg.jerk.z;

    w(0) = msg.angularVel.x;
    w(1) = msg.angularVel.y;
    w(2) = msg.angularVel.z;

    q.w() = msg.quat.w;
    q.x() = msg.quat.x;
    q.y() = msg.quat.y;
    q.z() = msg.quat.z;

    thrust = msg.thrust;

    mode = pMsg->mode;
    // std::cout << "j1=" << j.transpose() << std::endl;

    yaw = uav_utils::normalize_angle(msg.yaw);
    yaw_rate = msg.yaw_dot;
}

Battery_Data_t::Battery_Data_t() {
    rcv_stamp = ros::Time(0);
}

void Battery_Data_t::feed(sensor_msgs::BatteryStateConstPtr pMsg) {

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    double voltage = 0;
    for (size_t i = 0; i < pMsg->cell_voltage.size(); ++i) {
        voltage += pMsg->cell_voltage[i];
    }
    // volt = 0.8 * volt + 0.2 * voltage; // Naive LPF, cell_voltage has a
    // higher frequency
    volt = pMsg->voltage;
    // volt = 0.8 * volt + 0.2 * pMsg->voltage; // Naive LPF
    percentage = pMsg->percentage;

    static ros::Time last_print_t = ros::Time(0);
    if (percentage > 0.05) {
        if ((rcv_stamp - last_print_t).toSec() > 10) {
            ROS_INFO("\033[32m[bfctrl]\033[0m Voltage=%.3f, percentage=%.3f",
                     volt, percentage);
            last_print_t = rcv_stamp;
        }
    } else {
        if ((rcv_stamp - last_print_t).toSec() > 1) {
            last_print_t = rcv_stamp;
        }
    }
}

Takeoff_Land_Data_t::Takeoff_Land_Data_t() {
    rcv_stamp = ros::Time(0);
}

void Takeoff_Land_Data_t::feed(quadrotor_msgs::TakeoffLandConstPtr pMsg) {

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    triggered = true;
    takeoff_land_cmd = pMsg->takeoff_land_cmd;
    takeoff_height = pMsg->takeoff_height;
}

vfr_hud_Data_t::vfr_hud_Data_t() {
}
void vfr_hud_Data_t::feed(mavros_msgs::VFR_HUDConstPtr pMsg) {
    rcv_stamp = ros::Time::now();
    cur_thrust_ = pMsg->throttle;
    timed_thrust_.push(
        std::pair<ros::Time, double>(rcv_stamp, double(cur_thrust_)));
    while (timed_thrust_.size() > 100) {
        timed_thrust_.pop();
    }
}

Slow_Down_Data_t::Slow_Down_Data_t() {
    rcv_stamp = ros::Time(0);
}

void Slow_Down_Data_t::feed(quadrotor_msgs::SlowDownConstPtr pMsg) {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    x_acc = pMsg->x_acc;
    y_acc = pMsg->y_acc;
}