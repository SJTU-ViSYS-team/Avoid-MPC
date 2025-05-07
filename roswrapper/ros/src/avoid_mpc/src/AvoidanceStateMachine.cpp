#include "AvoidanceStateMachine.h"
#include "ParameterManager.h"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
AvoidanceStateMachine::AvoidanceStateMachine(ros::NodeHandle &nodeHandle)
    : mNodeHandle(nodeHandle), mVecStateQuad(10), mStateProcess(INIT),
      mbIsReceiveOdom(false), mCogFilter(10, 0.8) {
    SetupMPC();
    InitCircleState();
    SetupROS();
    ROS_INFO("\033[95m[MPC]\033[0m Init,waiting Odom.");
}

void AvoidanceStateMachine::InitCircleState() {
    Eigen::Vector3d initPos(0, 0, mHeight);
    Eigen::Vector3d goalPos(3, 0, mHeight);
    Eigen::Vector3d dPos = (goalPos - initPos) / mMpcN;
    for (int i = 0; i < mMpcN; i++) {
        Eigen::Vector3d posi = initPos + i * dPos;
        mRefPath.push_back({posi.x(), posi.y(), posi.z(), 0, 0, 0, 0, 0, 0, 0});
    }
    mStateGlobalGoal = {0., 0., mHeight, 0., 0., 0., 0., 0., 0., 0.};
}
void AvoidanceStateMachine::GetInitPath() {
    GetCurStateQuad(ros::Time::now().toSec() + mParamDecay);
    double goalx = mStateGlobalGoal[0];
    double goaly = mStateGlobalGoal[1];
    double goalz = mStateGlobalGoal[2];
    if (mStrTask == "forward") {
        goalx = mSpeed * mMpcT + mPos.x();
        goalx = fmin(goalx, Param::GetTaskParam().farestPoint);
        goaly = 0;
        goalz = mHeight;
    } else if (mStrTask == "global_goal") {
        Eigen::Vector3d goalPos(mStateGlobalGoal[0], mStateGlobalGoal[1],
                                mStateGlobalGoal[2]);
        Eigen::Vector3d lastGoalPos(mRefPath.back()[0], mRefPath.back()[1],
                                    mRefPath.back()[2]);
        Eigen::Vector3d dPos = goalPos - lastGoalPos;
        dPos = dPos.normalized() * std::min(dPos.norm(), mSpeed * mMpcDt);
        goalPos = lastGoalPos + dPos;
        goalx = goalPos[0];
        goaly = goalPos[1];
        goalz = goalPos[2];
    }
    for (int i = 0; i < mMpcN - 1; i++) {
        mRefPath[i] = {
            mRefPath[i + 1][0], mRefPath[i + 1][1], goalz,
            mRefPath[i + 1][3], mRefPath[i + 1][4], mRefPath[i + 1][5],
            mRefPath[i + 1][6], mRefPath[i + 1][7], mRefPath[i + 1][8],
            mRefPath[i + 1][9]};
    }
    mRefPath[mMpcN - 1] = {goalx, goaly, goalz, 0, mSpeed, 0, 0, 0, 0, 0};
}
void AvoidanceStateMachine::SetupMPC() {
    mMpcDt = Param::GetConParam().dt;
    mConDt = Param::GetConParam().conDt;
    mMpcT = Param::GetConParam().T;
    mStrTask = Param::GetTaskParam().task;
    mParamDecay = Param::GetConParam().decay;
    mMpcN = mMpcT / mMpcDt;
    mHighLvlMpc =
        ObstacleAvoidanceMPC(mMpcT, mMpcDt, Param::GetConParam().soPath);
    mHighLvlMpc.SetupWeights(Param::GetConParam().weights);
    mHighLvlMpc.SetupTau(Param::GetConParam().taus);
    mHighLvlMpc.SetupGains(Param::GetConParam().gains);
    mHighLvlMpc.SetDroneAccelLimits(
        Param::GetConParam().aMinZ, Param::GetConParam().aMaxZ,
        Param::GetConParam().aMaxXy, Param::GetConParam().aMaxYawDot);
    mHighLvlMpc.SetDroneRadius(Param::GetConParam().droneRadius);
    mParamSafteyDistance = Param::GetConParam().safetyDistance;
    mSpeed = Param::GetConParam().speed;
    mParamMPCMaxIter = Param::GetConParam().maxIter;
    mParamNearestPointSum = Param::GetConParam().nearestPointSum;
    mHeight = Param::GetTaskParam().height;

    mParamIsUseOdomEstimate = Param::GetConParam().useOdomEst;
    mParamOnlyTrustVel = Param::GetConParam().onlyTrustVel;
    if (mParamOnlyTrustVel) {
        ROS_INFO("\033[95m[MPC]\033[0m Only trust vel.");
        mHeight = 0.0;
    }
    mParamSlowDownKp = Param::GetConParam().slowDownKp;
    mParamSlowDownKd = Param::GetConParam().slowDownKd;
}
void AvoidanceStateMachine::SetupROS() {
    mPubCmd =
        mNodeHandle.advertise<quadrotor_msgs::Command>("/bfctrl/cmd", 100);
    mPubTakeoffLand = mNodeHandle.advertise<quadrotor_msgs::TakeoffLand>(
        "/bfctrl/takeoff_land", 200);
    mPubTrajVis = mNodeHandle.advertise<nav_msgs::Path>("Path", 100);
    mPubObstacle =
        mNodeHandle.advertise<visualization_msgs::Marker>("obstacle", 100);
    if (Param::GetperceptionParam().visualize) {
        mPubPtCloud =
            mNodeHandle.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
    }
    mSubOdom = mNodeHandle.subscribe(
        "/bfctrl/local_odom", 50, &AvoidanceStateMachine::OdomCallback, this);
    mSubIMU = mNodeHandle.subscribe("/mavros/imu/data", 50,
                                    &AvoidanceStateMachine::IMUCallback, this);
    mSubGlobalGoal = mNodeHandle.subscribe(
        "global_goal", 50, &AvoidanceStateMachine::GlobalGoalCallback, this);
    mSubQuadStatue = mNodeHandle.subscribe(
        "/bfctrl/statue", 50, &AvoidanceStateMachine::QuadStatueCallback, this);
    mSubDepth = mNodeHandle.subscribe(
        "/depth", 5, &AvoidanceStateMachine::DepthCallback, this);

    mTimer = mNodeHandle.createTimer(ros::Duration(mConDt),
                                     &AvoidanceStateMachine::Step, this);
}
double AvoidanceStateMachine::GetYawFromPuat(double qw, double qx, double qy,
                                             double qz) {
    double yaw = atan2(2 * (qw * qz + qx * qy),
                       1 - 2 * (qy * qy + qz * qz)); // yaw
    return yaw;
}
void AvoidanceStateMachine::OdomCallback(const nav_msgs::OdometryPtr &msg) {
    mbIsReceiveOdom = true;
    mTimePos = ros::Time::now().toSec();
    if (!mParamOnlyTrustVel) {
        mPos = Eigen::Vector3d(msg->pose.pose.position.x,
                               msg->pose.pose.position.y,
                               msg->pose.pose.position.z);
        mQuat = Eigen::Quaterniond(
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    } else {
        mPos = Eigen::Vector3d::Zero();
    }
    mVel = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                           msg->twist.twist.linear.z);
}

void AvoidanceStateMachine::IMUCallback(const sensor_msgs::ImuConstPtr &msg) {
    Eigen::Vector3d accb(msg->linear_acceleration.x, msg->linear_acceleration.y,
                         msg->linear_acceleration.z);
    Eigen::Vector3d accbFiltered = mCogFilter.filter(accb);
    if (mParamIsUseOdomEstimate) {
        double curTime = ros::Time::now().toSec();
        double dt = curTime - mTimePos;
        mPos += mVel * dt + 0.5 * mAcc * dt * dt;
        mVel += mAcc * dt;
        mTimePos = curTime;
    }
    if (mParamOnlyTrustVel) {
        mQuat = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                                   msg->orientation.y, msg->orientation.z);
    }
    mAcc =
        mQuat.toRotationMatrix() * accbFiltered - Eigen::Vector3d(0, 0, 9.81);
}
void AvoidanceStateMachine::DepthCallback(
    const sensor_msgs::ImageConstPtr &msg) {
    double dt = ros::Time::now().toSec() - mTimePos;
    Eigen::Vector3d pos = mPos;
    if (mParamIsUseOdomEstimate) {
        pos = mPos + mVel * dt + 0.5 * mAcc * dt * dt;
    }
    Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();
    Twb.block<3, 3>(0, 0) = mQuat.toRotationMatrix();
    Twb.block<3, 1>(0, 3) = pos;
    mKeyFrameMap.AddVertex(Twb, msg);
}
void AvoidanceStateMachine::GlobalGoalCallback(
    const geometry_msgs::PointStampedConstPtr &msg) {
    mStateGlobalGoal[0] = msg->point.x;
    mStateGlobalGoal[1] = msg->point.y;
    mStateGlobalGoal[2] = msg->point.z;
    mStateGlobalGoal[3] = 0.;
}
void AvoidanceStateMachine::QuadStatueCallback(
    const quadrotor_msgs::BfctrlStatueConstPtr &msg) {
    mStatueQuad = msg->status;
}
bool AvoidanceStateMachine::BfctrlNotReciveTakeoffMsg(double curTime) {
    return curTime - mTakeoffLandTime > 5.0;
}
bool AvoidanceStateMachine::BfctrlNotReciveLandMsg(double curTime) {
    return curTime - mTakeoffLandTime > 5.0 && mPos.z() > 0.3;
}
void AvoidanceStateMachine::GetCurStateQuad(double curTime) {
    double dt = curTime - mTimePos;
    Eigen::Vector3d pos = mPos;
    Eigen::Vector3d vel = mVel;
    Eigen::Vector3d acc = mAcc;
    if (mParamIsUseOdomEstimate) {
        pos = mPos + mVel * dt + 0.5 * mAcc * dt * dt;
        vel = mVel + mAcc * dt;
    }
    mVecStateQuad[0] = pos.x();
    mVecStateQuad[1] = pos.y();
    mVecStateQuad[2] = pos.z();
    mVecStateQuad[3] =
        GetYawFromPuat(mQuat.w(), mQuat.x(), mQuat.y(), mQuat.z());
    mVecStateQuad[4] = vel.x();
    mVecStateQuad[5] = vel.y();
    mVecStateQuad[6] = vel.z();
    mVecStateQuad[7] = mAcc.x();
    mVecStateQuad[8] = mAcc.y();
    mVecStateQuad[9] = mAcc.z();
}
bool AvoidanceStateMachine::ProcessWaypoints(ObstacleList &obstacles) {
    obstacles.clear();
    bool needReplan = false;
    obstacles.resize(mMpcN);
    Eigen::Matrix4d Twb;
    Twb << mQuat.toRotationMatrix(), mPos, 0, 0, 0, 1;
    for (int i = 0; i < mMpcN; i++) {
        Eigen::Vector3d pi(mRefPath[i][0], mRefPath[i][1], mRefPath[i][2]);
        std::vector<double> distances;
        std::vector<Eigen::Vector3d> closest_pts;
        mKeyFrameMap.QueryNearest(pi, mParamNearestPointSum, closest_pts,
                                  distances);
        for (int obstacle_index = 0; obstacle_index < mParamNearestPointSum;
             obstacle_index++) {
            if (obstacle_index < closest_pts.size()) {
                obstacles[i].emplace_back(
                    static_cast<double>(closest_pts[obstacle_index].x()),
                    static_cast<double>(closest_pts[obstacle_index].y()),
                    static_cast<double>(closest_pts[obstacle_index].z()));
            } else {
                ROS_WARN("\033[95m[MPC]\033[0m No enough points in the map.");
                obstacles[i].emplace_back(10000, 10000, 10000);
            }
        }
        if (distances.size() == 0 ||
            std::sqrt(distances[0]) <= mParamSafteyDistance) {
            needReplan = true;
        }
    }

    return needReplan;
}
std::vector<double>
AvoidanceStateMachine::GetRefStates(const ObstacleList &obstacles) {
    std::vector<double> vecRefStates = mVecStateQuad;
    for (int i = 0; i < mMpcN; i++) {
        vecRefStates.insert(vecRefStates.end(), mRefPath[i].begin(),
                            mRefPath[i].end());
    }
    for (int i = 0; i < mMpcN; i++) {
        for (const Eigen::Vector3d &obstacle : obstacles[i]) {
            vecRefStates.push_back(obstacle.x());
            vecRefStates.push_back(obstacle.y());
            vecRefStates.push_back(obstacle.z());
        }
    }
    std::vector<double> vecTarget = mRefPath.back();
    double targetdX = mSpeed * mMpcT - std::max(0., vecTarget[0] - mPos.x());
    targetdX = std::max(0., targetdX);
    vecTarget[0] += targetdX;
    vecTarget[1] = 0.;
    vecRefStates.insert(vecRefStates.end(), vecTarget.begin(), vecTarget.end());
    return vecRefStates;
}

bool AvoidanceStateMachine::PlanWapionts() {
    bool isSafety = true;
    for (int ptIndex = 0; ptIndex < 1; ptIndex++) {
        Eigen::Vector3d p1(mRefPath[ptIndex][0], mRefPath[ptIndex][1],
                           mRefPath[ptIndex][2]);
        double nearestPts = mKeyFrameMap.GetNearestDistance(p1);
        if (nearestPts > mParamSafteyDistance) {
            continue;
        }
        std::vector<Eigen::Vector3d> edgePts;
        std::vector<double> distances;
        mKeyFrameMap.QueryNearest(p1, 1, edgePts, distances, true);
        if (edgePts.empty()) {
            isSafety = false;
            continue;
        }
        mRefPath[ptIndex][0] = edgePts[0].x();
        mRefPath[ptIndex][1] = edgePts[0].y();
        mRefPath[ptIndex][2] = edgePts[0].z();
        isSafety = true;
    }
    return isSafety;
}

void AvoidanceStateMachine::Step(const ros::TimerEvent &event) {
    double curTime = ros::Time::now().toSec();
    switch (mStateProcess) {
    case INIT: {
        if (mbIsReceiveOdom) {
            ROS_INFO("\033[95m[MPC]\033[0m Odom received, waiting trigger.");
            mStateProcess = WAIT;
        }
        break;
    }
    case WAIT: {
        if (mStatueQuad ==
                quadrotor_msgs::BfctrlStatue::BFCTRL_STATUS_WAITINGCMD ||
            mStatueQuad == quadrotor_msgs::BfctrlStatue::BFCTRL_STATUS_CMD) {
            ROS_INFO("\033[95m[MPC]\033[0m Triggered, takeoff.");
            mStateProcess = TAKEOFF;
            mTakeoffLandTime = -1;
            mTakeoffLandZ = mHeight;
        }
        break;
    }
    case TAKEOFF: {
        if (mPos.z() < 0.6 * mHeight) {
            if (BfctrlNotReciveTakeoffMsg(curTime)) {
                mTakeoffLandTime = curTime;
                mTakeoffLandZ = mPos.z();
                quadrotor_msgs::TakeoffLand takeoffMsg;
                takeoffMsg.takeoff_height = mHeight - mPos.z();
                takeoffMsg.takeoff_land_cmd =
                    quadrotor_msgs::TakeoffLand::TAKEOFF;
                mPubTakeoffLand.publish(takeoffMsg);
            }
        } else {
            ROS_INFO("\033[95m[MPC]\033[0m Reach target height, start task.");
            mStateProcess = TASK;
            ros::Duration(1.0).sleep();
        }
        break;
    }
    case TASK: {
        std::vector<double> u;
        std::vector<std::vector<double>> x0Array;
        GetInitPath();
        bool isSafety = true;
        double decay = mParamDecay;
        for (int iter = 0; iter < mParamMPCMaxIter; iter++) {
            double start = ros::Time::now().toSec();
            GetCurStateQuad(start + decay);
            isSafety = PlanWapionts();
            bool needReplan = ProcessWaypoints(mVecObstacles);
            if (!needReplan && iter > 0 && isSafety) {
                break;
            }
            std::vector<double> vecRefStates = GetRefStates(mVecObstacles);
            mHighLvlMpc.Solve(vecRefStates, u, x0Array, iter == 0);
            for (int i = 0; i < mMpcN; i++) {
                mRefPath[i].clear();
                mRefPath[i].insert(mRefPath[i].end(), x0Array[i].begin(),
                                   x0Array[i].begin() + 10);
            }
            decay = ros::Time::now().toSec() - start;
        }
        if (isSafety) {
            PubCmd(u);
        } else {
            ROS_INFO("\033[95m[MPC]\033[0m Slow down.");
            PubSlowDownCmd();
        }
        if (Param::GetperceptionParam().visualize) {
            PathVisualization(x0Array);
        }
        break;
    }
    case LAND: {
        if (BfctrlNotReciveLandMsg(curTime)) {
            quadrotor_msgs::TakeoffLand msg;
            msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::LAND;
            mPubTakeoffLand.publish(msg);
        }
    }
    }
    if (Param::GetperceptionParam().visualize) {
        PtCloudVisualization();
        ObstacleVisualization(mVecObstacles);
    }
}
void AvoidanceStateMachine::PubCmd(std::vector<double> &u) {
    quadrotor_msgs::Command cmdMsg;
    cmdMsg.header.stamp = ros::Time::now();
    cmdMsg.mode = quadrotor_msgs::Command::ACCELERATION_MODE;
    cmdMsg.acceleration.x = u[0];
    cmdMsg.acceleration.y = u[1];
    cmdMsg.acceleration.z = u[2];
    cmdMsg.yaw = 0;
    mPubCmd.publish(cmdMsg);
}
void AvoidanceStateMachine::PubSlowDownCmd() {
    Eigen::Vector3d accSlow = -mVel * mParamSlowDownKp -
                              mAcc * mParamSlowDownKd +
                              Eigen::Vector3d(0, 0, 9.8);
    double ax = std::max(-Param::GetConParam().aMaxXy,
                         std::min(Param::GetConParam().aMaxXy, accSlow.x()));
    double ay = std::max(-Param::GetConParam().aMaxXy,
                         std::min(Param::GetConParam().aMaxXy, accSlow.y()));
    double az = std::max(-Param::GetConParam().aMaxZ,
                         std::min(Param::GetConParam().aMaxZ, accSlow.z()));
    quadrotor_msgs::Command cmdMsg;
    cmdMsg.header.stamp = ros::Time::now();
    cmdMsg.mode = quadrotor_msgs::Command::ACCELERATION_MODE;
    cmdMsg.acceleration.x = ax;
    cmdMsg.acceleration.y = ay;
    cmdMsg.acceleration.z = az;
    cmdMsg.yaw = 0;
    mPubCmd.publish(cmdMsg);
}
void AvoidanceStateMachine::PathVisualization(
    const std::vector<std::vector<double>> &x0Array) {
    if (x0Array.empty()) {
        return;
    }
    nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    for (const std::vector<double> &point : x0Array) {
        geometry_msgs::PoseStamped curPose;
        curPose.pose.position.x = point[0];
        curPose.pose.position.y = point[1];
        curPose.pose.position.z = point[2];
        curPose.pose.orientation =
            GetQuatFromAcc(point[7], point[8], point[9], point[3]);
        msg.poses.push_back(curPose);
    }
    mPubTrajVis.publish(msg);
}

void AvoidanceStateMachine::PtCloudVisualization() {
    sensor_msgs::PointCloud2 msg;
    pcl::PointCloud<pcl::PointXYZRGB> ptCloud = mKeyFrameMap.GetPtCloud();
    if (ptCloud.empty()) {
        return;
    }
    pcl::toROSMsg(ptCloud, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    mPubPtCloud.publish(msg);
}

void AvoidanceStateMachine::ObstacleVisualization(
    const ObstacleList &obstacles) {
    if (obstacles.empty()) {
        return;
    }
    sensor_msgs::PointCloud2 msg;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "obstacle";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    int index = 0;
    for (const std::list<Eigen::Vector3d> &obstacle : obstacles) {
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        color.r = 1.0 - index / obstacles.size();
        color.g = index / obstacles.size();
        color.b = 0.5;
        index++;
        for (const Eigen::Vector3d &point : obstacle) {
            geometry_msgs::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            marker.points.push_back(p);
            marker.colors.push_back(color);
        }
    }
    mPubObstacle.publish(marker);
}
geometry_msgs::Quaternion AvoidanceStateMachine::GetQuatFromAcc(double accx,
                                                                double accy,
                                                                double accz,
                                                                double yaw) {
    Eigen::Vector3d vecAcc(accx, accy, accz + 9.8);
    Eigen::Quaterniond quat;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d rotmat;
    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;
    zb_des = vecAcc / vecAcc.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
    xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1),
        xb_des(2), yb_des(2), zb_des(2);
    quat = Eigen::Quaterniond(rotmat);
    geometry_msgs::Quaternion q;
    q.w = quat.w();
    q.x = quat.x();
    q.y = quat.y();
    q.z = quat.z();
    return q;
}