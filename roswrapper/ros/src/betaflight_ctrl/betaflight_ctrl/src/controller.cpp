#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif
#include "controller.h"

using namespace std;
GeometricController::GeometricController() {
    Kpos_ << Param::get().geometry_controller.Kpos_x_,
        Param::get().geometry_controller.Kpos_y_,
        Param::get().geometry_controller.Kpos_z_;
    Kvel_ << Param::get().geometry_controller.Kvel_x_,
        Param::get().geometry_controller.Kvel_y_,
        Param::get().geometry_controller.Kvel_z_;
    D_ << Param::get().geometry_controller.drag_dx,
        Param::get().geometry_controller.drag_dy,
        Param::get().geometry_controller.drag_dz;
    resetThrustMapping();
}
double GeometricController::fromQuaternion2yaw(const Eigen::Quaterniond &q) {
    double yaw =
        atan2(2 * (q.x() * q.y() + q.w() * q.z()),
              q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return yaw;
}
Controller_Output_t
GeometricController::GeometryController(Desired_State_t &des,
                                        const Odom_Data_t &odom, uint8_t mode) {
    Controller_Output_t u;
    Eigen::Vector3d desired_acc = Eigen::Vector3d::Zero();
    if (mode == quadrotor_msgs::Command::ACCELERATION_MODE) {
        desired_acc = des.a;
    } else if (mode == quadrotor_msgs::Command::POSITION_MODE) {
        desired_acc = controlPosition(des.p, des.v, des.a, des.yaw, odom);
    }
    if (mode != quadrotor_msgs::Command::ANGULAR_MODE) {
        if (mode == quadrotor_msgs::Command::QUAT_MODE) {
            u.q = des.q;
            u.thrust = GetThrust(des.thrust);
        } else {
            u.q = acc2quaternion(desired_acc, des.yaw);
            const Eigen::Matrix3d rotmat = odom.q.toRotationMatrix();
            const Eigen::Vector3d zboby = rotmat.col(2);
            u.thrust = GetThrust(desired_acc.dot(zboby));
        }
        if (Param::get().use_bodyrate_ctrl) {
            u.bodyrates =
                geometric_attcontroller(u.q, odom.q); // Calculate BodyRate
        }
    } else { // mode == quadrotor_msgs::Command::ANGULAR_MODE
        u.bodyrates = des.w;
        u.thrust = GetThrust(des.thrust);
    }
    return u;
}
double GeometricController::GetThrust(double accbz) {
    double thrust = accbz / thr2acc_;
    return thrust;
}
Eigen::Vector3d GeometricController::controlPosition(
    const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
    const Eigen::Vector3d &target_acc, double &target_yaw,
    const Odom_Data_t &odom) {
    /// Compute BodyRate commands using differential flatness
    /// Controller based on Faessler 2017
    const Eigen::Vector3d a_ref = target_acc;
    if (Param::get().velocity_yaw) {
        target_yaw = getVelocityYaw(odom.v);
    }

    const Eigen::Quaterniond q_ref = acc2quaternion(
        a_ref + Eigen::Vector3d(0, 0, Param::get().gra), target_yaw);
    const Eigen::Matrix3d R_ref = q_ref.toRotationMatrix();

    const Eigen::Vector3d pos_error = target_pos - odom.p;
    const Eigen::Vector3d vel_error = target_vel - odom.v;

    // Position Controller
    const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);

    // Rotor Drag compensation
    const Eigen::Vector3d a_rd =
        R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel; // Rotor drag

    // Reference acceleration
    const Eigen::Vector3d a_des =
        a_fb + a_ref - a_rd + Eigen::Vector3d(0, 0, Param::get().gra);

    return a_des;
}

Eigen::Quaterniond
GeometricController::acc2quaternion(const Eigen::Vector3d &vector_acc,
                                    const double &yaw) {
    Eigen::Quaterniond quat;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d rotmat;

    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;
    zb_des = vector_acc / vector_acc.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
    xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1),
        xb_des(2), yb_des(2), zb_des(2);

    quat = Eigen::Quaterniond(rotmat);
    quat.normalize();
    return quat;
}

Eigen::Vector3d
GeometricController::poscontroller(const Eigen::Vector3d &pos_error,
                                   const Eigen::Vector3d &vel_error) {
    Eigen::Vector3d a_fb =
        Kpos_.asDiagonal() * pos_error +
        Kvel_.asDiagonal() * vel_error; // feedforward term for trajectory error

    if (a_fb.norm() > Param::get().geometry_controller.max_fb_acc_)
        a_fb = (Param::get().geometry_controller.max_fb_acc_ / a_fb.norm()) *
               a_fb; // Clip acceleration if reference is too large

    return a_fb;
}
Eigen::Vector3d GeometricController::geometric_attcontroller(
    const Eigen::Quaterniond &ref_att, const Eigen::Quaterniond &curr_att) {
    // Geometric attitude controller
    // Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris
    // McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)."
    // 49th IEEE conference on decision and control (CDC). IEEE, 2010. The
    // original paper inputs moment commands, but for offboard control, angular
    // rate commands are sent

    Eigen::Vector3d ratecmd;
    Eigen::Matrix3d rotmat;   // Rotation matrix of current attitude
    Eigen::Matrix3d rotmat_d; // Rotation matrix of desired attitude
    Eigen::Vector3d error_att;
    rotmat = curr_att.toRotationMatrix();
    rotmat_d = ref_att.toRotationMatrix();
    error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat -
                                     rotmat.transpose() * rotmat_d);
    ratecmd = (2.0 / Param::get().geometry_controller.attctrl_tau_) * error_att;
    return ratecmd;
}
Eigen::Vector3d
GeometricController::attcontroller(const Eigen::Quaterniond &ref_att,
                                   const Eigen::Vector3d &ref_acc,
                                   const Eigen::Quaterniond &curr_att) {
    // Geometric attitude controller
    // Attitude error is defined as in Brescianini, Dario, Markus Hehn, and
    // Raffaello D'Andrea. Nonlinear quadrocopter attitude control: Technical
    // report. ETH Zurich, 2013.

    Eigen::Vector3d ratecmd;

    const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
    const Eigen::Quaterniond q_inv = curr_att.inverse();
    const Eigen::Quaterniond qe = q_inv * ref_att;
    ratecmd(0) = (2.0 / Param::get().geometry_controller.attctrl_tau_) *
                 std::copysign(1.0, qe.w()) * qe.x();
    ratecmd(1) = (2.0 / Param::get().geometry_controller.attctrl_tau_) *
                 std::copysign(1.0, qe.w()) * qe.y();
    ratecmd(2) = (2.0 / Param::get().geometry_controller.attctrl_tau_) *
                 std::copysign(1.0, qe.w()) * qe.z();
    return ratecmd;
}
/*
  compute throttle percentage
*/
double GeometricController::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc) {
    double throttle_percentage(0.0);

    /* compute throttle, thr2acc has been estimated before */
    throttle_percentage = des_acc(2) / thr2acc_;

    return throttle_percentage;
}

bool GeometricController::estimateThrustModel(const Eigen::Vector3d &est_a,
                                              double thr) {
    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************/
    /* Model: est_a(2) = thr2acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    if (Param::get().thr_map.print_val) {
        ROS_INFO("Hover percentage: %f, cur a = %f, cur thrust = %f.",
                 Param::get().gra / thr2acc_, est_a(2), thr);
    }
    P_ = (1 - K * thr) * P_ / rho2_;
    return false;
}

void GeometricController::resetThrustMapping(void) {
    thr2acc_ = Param::get().gra / Param::get().thr_map.hover_percentage;
    P_ = 1e6;
}

double GeometricController::GetHoverThrustPercentage() {
    return Param::get().gra / thr2acc_;
}