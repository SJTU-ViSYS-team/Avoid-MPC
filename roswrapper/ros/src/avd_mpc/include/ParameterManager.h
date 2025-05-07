#ifndef PARAMETERMANAGER_H
#define PARAMETERMANAGER_H
#include <Eigen/Core>
#include <ros/ros.h>
class ParameterManager {
public:
    static ParameterManager &GetInstance() {
        static ParameterManager instance;
        return instance;
    }
    struct PerceptionParam {
        bool visualize;
        double fx;
        double fy;
        double cx;
        double cy;
        double pixel2Meter;
        double depthMax;
        double depthMin;
        double resizeScale;
        double resizeCols;
        double resizeRows;
        Eigen::Matrix4d Tbc;
        double keyframeDistanceTh;
        int keyframeCountTh;
        int maxFrameCount;
    };
    struct ConParam {
        double dt;
        double T;
        double conDt;
        int maxIter;
        double speed;
        int nearestPointSum;
        std::vector<double> weights;
        std::vector<double> gains;
        std::vector<double> taus;
        std::string soPath;
        double droneRadius;

        double safetyDistance;

        double aMinZ;
        double aMaxZ;
        double aMaxXy;
        double aMaxYawDot;

        double decay;

        bool useOdomEst;
        bool onlyTrustVel;

        double slowDownKp;
        double slowDownKd;
    };
    struct TaskParam {
        std::string task;
        double speed;
        double height;
        double farestPoint;
    };
    void init(ros::NodeHandle &nodeHandle);

    static const PerceptionParam &GetperceptionParam() {
        if (!GetInstance().isInit) {
            ROS_ERROR("ParameterManager is not initialized.");
        }
        return GetInstance().perception;
    }
    static const ConParam &GetConParam() {
        if (!GetInstance().isInit) {
            ROS_ERROR("ParameterManager is not initialized.");
        }
        return GetInstance().con;
    }
    static const TaskParam &GetTaskParam() {
        if (!GetInstance().isInit) {
            ROS_ERROR("ParameterManager is not initialized.");
        }
        return GetInstance().task;
    }

private:
    bool isInit;
    PerceptionParam perception;
    ConParam con;
    TaskParam task;

private:
    ParameterManager();
    ParameterManager(const ParameterManager &) = delete;
    ParameterManager &operator=(const ParameterManager &) = delete;

    void SetupPerceptionParam(ros::NodeHandle &nodeHandle);
    void SetupConParam(ros::NodeHandle &nodeHandle);
    void SetupTaskParam(ros::NodeHandle &nodeHandle);
};
using Param = ParameterManager;
#endif