#ifndef AVOIDANCE_STATE_MACHINE_H
#define AVOIDANCE_STATE_MACHINE_H
#include "COGFilter.h"
#include "FrameKDMap.h"
#include "HighLvlMpc.h"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/BfctrlStatue.h>
#include <quadrotor_msgs/Command.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
class AvoidanceStateMachine {
private:
    enum ProcessState {
        INIT = 0u,
        WAIT = 1u,
        TAKEOFF = 2u,
        TASK = 3u,
        LAND = 4u
    };
    using ObstacleList = std::vector<std::list<Eigen::Vector3d>>;

public:
    AvoidanceStateMachine(ros::NodeHandle &nodeHandle);

private:
    inline void InitCircleState();
    inline void GetInitPath();
    void SetupMPC();
    void SetupROS();
    double GetYawFromPuat(double qw, double qx, double qy, double qz);
    void OdomCallback(const nav_msgs::OdometryPtr &msg);
    void IMUCallback(const sensor_msgs::ImuConstPtr &msg);
    void DepthCallback(const sensor_msgs::ImageConstPtr &msg);
    void GlobalGoalCallback(const geometry_msgs::PointStampedConstPtr &msg);
    void QuadStatueCallback(const quadrotor_msgs::BfctrlStatueConstPtr &msg);

    bool BfctrlNotReciveTakeoffMsg(double curTime);
    bool BfctrlNotReciveLandMsg(double curTime);

    void GetCurStateQuad(double curTime);

    void Step(const ros::TimerEvent &event);
    void PubCmd(std::vector<double> &u);
    void PubSlowDownCmd();
    bool ProcessWaypoints(ObstacleList &obstacles);
    bool PlanWapionts();
    std::vector<double> GetRefStates(const ObstacleList &obstacles);

    void PtCloudVisualization();
    void ObstacleVisualization(const ObstacleList &obstacles);
    void PathVisualization(const std::vector<std::vector<double>> &x0Array);
    geometry_msgs::Quaternion GetQuatFromAcc(double accx, double accy,
                                             double accz, double yaw);

private:
    ros::NodeHandle &mNodeHandle;
    double mMpcDt;
    double mConDt;
    double mMpcT;
    int mMpcN;
    double mParamDecay;
    int mParamPointCount;
    std::string mStrTask;
    bool mbIsReceiveOdom;
    double mTakeoffLandTime;
    double mTakeoffLandZ;

    int mParamMPCMaxIter;

    int mParamNearestPointSum;

    bool mParamIsUseOdomEstimate;
    bool mParamOnlyTrustVel;
    double mParamSlowDownKp;
    double mParamSlowDownKd;

    double mParamSafteyDistance;

    double mSpeed;
    double mHeight;
    uint8_t mStatueQuad;
    ProcessState mStateProcess;

    ObstacleAvoidanceMPC mHighLvlMpc;

    FrameKDMap mKeyFrameMap;

    COGFilter mCogFilter;

    ros::Publisher mPubCmd;
    ros::Publisher mPubTakeoffLand;
    ros::Publisher mPubTrajVis;
    ros::Publisher mPubPtCloud;
    ros::Publisher mPubObstacle;

    ros::Subscriber mSubOdom;
    ros::Subscriber mSubDepth;
    ros::Subscriber mSubIMU;
    ros::Subscriber mSubGlobalGoal;
    ros::Subscriber mSubQuadStatue;
    ros::Timer mTimer;

    std::vector<double> mVecStateQuad;
    std::vector<double> mStateGlobalGoal;

    std::vector<std::vector<double>> mRefPath;
    ObstacleList mVecObstacles;
    double mTimePos;
    Eigen::Quaterniond mQuat;
    Eigen::Vector3d mPos;
    Eigen::Vector3d mVel;
    Eigen::Vector3d mAcc;
};
#endif