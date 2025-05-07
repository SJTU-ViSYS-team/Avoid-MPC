#ifndef FRAME_KD_MAP_H
#define FRAME_KD_MAP_H
#include "kd_tree_two.h"
#include <fstream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <thread>
class FrameKDMap {
private:
    using PtCloudKdPtr = std::shared_ptr<KDTreeTwo<double>>;
    struct Frame {
        PtCloudKdPtr pointCloud;
        PtCloudKdPtr edgeCloud;
        Eigen::Matrix4d Twc;
        Frame() {
        }
        Frame(PtCloudKdPtr _ptCloud, PtCloudKdPtr _edgePtCloud,
              Eigen::Matrix4d &_Twc)
            : pointCloud(_ptCloud), edgeCloud(_edgePtCloud), Twc(_Twc) {
        }
        Frame(const Frame &keyFrame)
            : pointCloud(keyFrame.pointCloud), Twc(keyFrame.Twc),
              edgeCloud(keyFrame.edgeCloud) {
        }
        Frame &operator=(const Frame &keyFrame) {
            pointCloud = keyFrame.pointCloud;
            edgeCloud = keyFrame.edgeCloud;
            Twc = keyFrame.Twc;
            return *this;
        }
    };
    struct PtDists {
        Eigen::Vector3d pt;
        double dist;
        bool isInKeyFrame;
        PtDists() {
        }
        PtDists(const Eigen::Vector3d &_pt, double _dist, bool _isInKeyFrame)
            : pt(_pt), dist(_dist), isInKeyFrame(_isInKeyFrame) {
        }
        PtDists(const PtDists &ptDists)
            : pt(ptDists.pt), dist(ptDists.dist),
              isInKeyFrame(ptDists.isInKeyFrame) {
        }
        PtDists &operator=(const PtDists &ptDists) {
            pt = ptDists.pt;
            dist = ptDists.dist;
            isInKeyFrame = ptDists.isInKeyFrame;
            return *this;
        }
        bool operator<(const PtDists &other) const {
            return dist < other.dist;
        }
    };
    using Map = std::list<Frame>;
    using FramePtr = std::shared_ptr<Frame>;

public:
    FrameKDMap();
    void AddVertex(const Eigen::Matrix4d &mat4Twb,
                   const sensor_msgs::ImageConstPtr &depth);
    void QueryNearest(const Eigen::Vector3d &point, int nearestPointCount,
                      std::vector<Eigen::Vector3d> &out,
                      std::vector<double> &distances, bool queryEdge = false);
    double GetNearestDistance(const Eigen::Vector3d &point);
    pcl::PointCloud<pcl::PointXYZRGB> GetPtCloud();

private:
    void SetCurPtCloud(PtCloudKdPtr &newKdTree, PtCloudKdPtr &newEdgeKdTree);
    void ProcessDepth(const sensor_msgs::ImageConstPtr &depth,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &edgeCloud,
                      const Eigen::Matrix4d &mat4Twb);
    void BuildEdgeCloud(cv::Mat &invDepthImg,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &edgeCloud);
    Eigen::Vector4d UV2Camera(int u, int v, double depth);
    bool PtIsInFrame(const Eigen::Vector3d &ptw, const Eigen::Matrix4d &Twc);

    void RemoveOldVertex();
    void QueryNearestWithCurFrame(const Eigen::Vector3d &point,
                                  int nearestPointCount,
                                  std::vector<Eigen::Vector3d> &nearestPoints,
                                  std::vector<double> &distances,
                                  bool queryEdge = false);
    void QueryNearestThreadWorker(int start, int end, int nearestPointCount,
                                  const Eigen::Vector3d &point,
                                  std::mutex &pointsDistMutex,
                                  std::vector<PtDists> &pointDist,
                                  bool queryEdge = false);
    void GetNearestDistanceThreadWorker(int start, int end,
                                        const Eigen::Vector3d &point,
                                        std::mutex &minDistanceMutex,
                                        double &nearestDistance);
    void UpdateQueryVector();
    void InsertKeyFrame();
    void ReplaceKdTree(Frame &replacedFrame, PtCloudKdPtr &newKdTree);
    void KeyframeThreadWorker();
    template <typename T>
    void GetInvDepthImg(const cv::Mat &depthImg, cv::Mat &invDepthImg);
    bool DroneBehindPts(const Eigen::Matrix4d &Twc, const Frame &frame);
    void ErodeImageWithDepth(const cv::Mat &depthImg, cv::Mat &result);
private:
    Map mKeyFrameMap;

    double mParamDepthScale;
    int mParamWidth;
    int mParamHeight;
    double mParamPixel2Meter;
    double mParamDepthMax;
    double mParamDepthMin;
    double mParamFx;
    double mParamFy;
    double mParamCx;
    double mParamCy;
    double mParamSafeDist;
    Eigen::Matrix4d mParamTbc;

    double mParamKeyframeDistanceTh;
    int mParamKeyframeCountTh;
    int mParamMaxFrameCount;

    std::vector<FramePtr> mVecQueryVector;
    Frame mCurFrame;

    bool mbNeedProcessPtCloud;

    std::thread mThreadKeyframe;
    std::mutex mMtxKdTree;
    std::mutex mMtxCurPose;
};
#endif