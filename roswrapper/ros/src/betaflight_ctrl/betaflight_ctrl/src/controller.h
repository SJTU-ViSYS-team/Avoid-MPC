/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <queue>

#include "input.h"
#include <Eigen/Dense>

struct Desired_State_t {
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    Eigen::Vector3d w;
    Eigen::Quaterniond q;
    double yaw;
    double yaw_rate;
    double thrust;
    Desired_State_t(){};

    Desired_State_t(Odom_Data_t &odom)
        : p(odom.p), v(Eigen::Vector3d::Zero()), a(Eigen::Vector3d::Zero()),
          j(Eigen::Vector3d::Zero()), q(odom.q),
          yaw(uav_utils::get_yaw_from_quaternion(odom.q)), yaw_rate(0){};
};

struct Controller_Output_t {

    // Orientation of the body frame with respect to the world frame
    Eigen::Quaterniond q;

    // Body rates in body frame
    Eigen::Vector3d bodyrates; // [rad/s]

    // Collective mass normalized thrust
    double thrust;

    // Eigen::Vector3d des_v_real;
};

class GeometricController {
public:
    GeometricController();
    Controller_Output_t GeometryController(Desired_State_t &des,
                                           const Odom_Data_t &odom,
                                           uint8_t mode);
    double GetThrust(double accbz);
    bool estimateThrustModel(const Eigen::Vector3d &est_v, double cur_thrust);
    void resetThrustMapping(void);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double GetHoverThrustPercentage();

private:
    std::queue<std::pair<ros::Time, double>> timed_thrust_;
    static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;
    // GeoController
    Eigen::Vector3d Kpos_, Kvel_, D_;
    // Thrust-accel mapping params
    const double rho2_ = 0.998; // do not change
    double thr2acc_;
    double P_;
    double fromQuaternion2yaw(const Eigen::Quaterniond &q);
    double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
    inline double getVelocityYaw(const Eigen::Vector3d velocity) {
        return atan2(velocity(1), velocity(0));
    };
    inline Eigen::Vector3d matrix_hat_inv(const Eigen::Matrix3d &m) {
        Eigen::Vector3d v;
        // TODO: Sanity checks if m is skew symmetric
        v << m(7), m(2), m(3);
        return v;
    }
    Eigen::Quaterniond acc2quaternion(const Eigen::Vector3d &vector_acc,
                                      const double &yaw);
    Eigen::Vector3d poscontroller(const Eigen::Vector3d &pos_error,
                                  const Eigen::Vector3d &vel_error);
    Eigen::Vector3d controlPosition(const Eigen::Vector3d &target_pos,
                                    const Eigen::Vector3d &target_vel,
                                    const Eigen::Vector3d &target_acc,
                                    double &target_yaw,
                                    const Odom_Data_t &odom);

    Eigen::Vector3d geometric_attcontroller(const Eigen::Quaterniond &ref_att,
                                            const Eigen::Quaterniond &curr_att);
    Eigen::Vector3d attcontroller(const Eigen::Quaterniond &ref_att,
                                  const Eigen::Vector3d &ref_acc,
                                  const Eigen::Quaterniond &curr_att);
};

#endif
