#include "FrameKDMap.h"
#include "ParameterManager.h"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <thread>
FrameKDMap::FrameKDMap() {
    mParamFx = Param::GetperceptionParam().fx;
    mParamFy = Param::GetperceptionParam().fy;
    mParamCx = Param::GetperceptionParam().cx;
    mParamCy = Param::GetperceptionParam().cy;
    mParamPixel2Meter = Param::GetperceptionParam().pixel2Meter;
    mParamDepthMax = Param::GetperceptionParam().depthMax;
    mParamDepthMin = Param::GetperceptionParam().depthMin;
    mParamDepthScale = Param::GetperceptionParam().resizeScale;
    mParamTbc = Param::GetperceptionParam().Tbc;

    mParamKeyframeDistanceTh = Param::GetperceptionParam().keyframeDistanceTh;
    mParamKeyframeCountTh = Param::GetperceptionParam().keyframeCountTh;
    mParamMaxFrameCount = Param::GetperceptionParam().maxFrameCount;

    mParamFx = mParamFx / mParamDepthScale;
    mParamFy = mParamFy / mParamDepthScale;
    mParamCx = mParamCx / mParamDepthScale;
    mParamCy = mParamCy / mParamDepthScale;

    mParamSafeDist = Param::GetConParam().safetyDistance;

    mbNeedProcessPtCloud = false;
    if (mParamMaxFrameCount > 0 && !Param::GetConParam().onlyTrustVel) {
        mThreadKeyframe = std::thread(&FrameKDMap::KeyframeThreadWorker, this);
        mThreadKeyframe.detach();
    }
}
void FrameKDMap::AddVertex(const Eigen::Matrix4d &mat4Twb,
                           const sensor_msgs::ImageConstPtr &depth) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr edgeCloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    ProcessDepth(depth, cloud, edgeCloud, mat4Twb);
    if (cloud->empty()) {
        return;
    }
    PtCloudKdPtr kdtree = std::make_shared<KDTreeTwo<double>>();
    kdtree->InitializeNew(cloud);
    PtCloudKdPtr edgeKdtree = std::make_shared<KDTreeTwo<double>>();
    edgeKdtree->InitializeNew(edgeCloud);
    std::lock_guard<std::mutex> lock(mMtxKdTree);
    SetCurPtCloud(kdtree, edgeKdtree);
    mCurFrame.Twc = mat4Twb * mParamTbc;
    mbNeedProcessPtCloud = true;
}
void FrameKDMap::SetCurPtCloud(PtCloudKdPtr &newKdTree,
                               PtCloudKdPtr &newEdgeKdTree) {
    mCurFrame.pointCloud = newKdTree;
    mCurFrame.edgeCloud = newEdgeKdTree;
    UpdateQueryVector();
}
void FrameKDMap::RemoveOldVertex() {
    PtCloudKdPtr firstKdTree = mKeyFrameMap.front().pointCloud;
    firstKdTree->Clear();
    mKeyFrameMap.pop_front();
}
void FrameKDMap::UpdateQueryVector() {
    mVecQueryVector.clear();
    mVecQueryVector.push_back(std::make_shared<Frame>(mCurFrame));
    if (!mKeyFrameMap.empty()) {
        Map::iterator it = mKeyFrameMap.begin();
        for (int i = 0; i < mKeyFrameMap.size() - 1; i++) {
            mVecQueryVector.push_back(std::make_shared<Frame>(*it));
            it++;
        }
    }
}
template <typename T>
void FrameKDMap::GetInvDepthImg(const cv::Mat &depthImg, cv::Mat &invDepthImg) {

    for (int i = 0; i < depthImg.rows; i++) {
        for (int j = 0; j < depthImg.cols; j++) {
            float depth =
                static_cast<float>(depthImg.at<T>(i, j)) * mParamPixel2Meter;
            if (depth < mParamDepthMin || depth > mParamDepthMax) {
                invDepthImg.at<float>(i, j) = 0.;
            } else {
                invDepthImg.at<float>(i, j) = 1. / depth;
            }
        }
    }
}
void FrameKDMap::ProcessDepth(const sensor_msgs::ImageConstPtr &depth,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr &edgeCloud,
                              const Eigen::Matrix4d &mat4Twb) {
    cv::Mat depthImg = cv_bridge::toCvCopy(depth)->image;
    cv::Mat invDepthImg = cv::Mat::zeros(depthImg.size(), CV_32FC1);
    if (depthImg.type() == CV_16UC1) {
        GetInvDepthImg<uint16_t>(depthImg, invDepthImg);
    } else if (depthImg.type() == CV_32FC1) {
        GetInvDepthImg<float>(depthImg, invDepthImg);
    } else {
        ROS_ERROR("\033[95m[MPC]\033[0m Error: depth image type not supported");
        return;
    }
    int rows = invDepthImg.rows;
    int cols = invDepthImg.cols;
    mParamWidth = cols / mParamDepthScale;
    mParamHeight = rows / mParamDepthScale;
    cv::Size newSize(mParamWidth, mParamHeight);
    cv::resize(invDepthImg, invDepthImg, newSize, cv::INTER_MAX);
    for (int row = 0; row < invDepthImg.rows; row++) {
        for (int col = 0; col < invDepthImg.cols; col++) {
            double invDepth = invDepthImg.at<float>(row, col);
            if (invDepth < 1e-2) {
                continue;
            }
            double depth = 1 / invDepth;
            if (depth > mParamDepthMin && depth < mParamDepthMax) {
                Eigen::Vector4d pointCam = UV2Camera(col, row, depth);
                Eigen::Vector3d point =
                    (mat4Twb * mParamTbc * pointCam).topRows(3);
                (mat4Twb * mParamTbc * pointCam).topRows(3);
                cloud->emplace_back(point.x(), point.y(), point.z());
            }
        }
    }
    if (cloud->empty()) {
        return;
    }
    BuildEdgeCloud(invDepthImg, edgeCloud);
}
Eigen::Vector4d FrameKDMap::UV2Camera(int u, int v, double depth) {
    Eigen::Vector4d point;
    point[0] = (u - mParamCx) * depth / mParamFx;
    point[1] = (v - mParamCy) * depth / mParamFy;
    point[2] = depth;
    point[3] = 1;
    return point;
}
void FrameKDMap::ErodeImageWithDepth(const cv::Mat &invDepthImg,
                                     cv::Mat &result) {
    int width = invDepthImg.cols;
    int height = invDepthImg.rows;
    for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {
            // uchar pixel = invDepthImg.at<uchar>(row, col);
            float depth = 1.0f/invDepthImg.at<float>(row, col);          
            if (depth < mParamDepthMin || depth > mParamDepthMax) {
                continue;
            }
            uchar pixel = uchar(depth * 255 / (mParamDepthMax - mParamDepthMin));
            result.at<uchar>(row, col) = pixel;
            int kernelSizeW = mParamFx * mParamSafeDist / depth;
            int kernelSizeH = mParamFy * mParamSafeDist / depth;
            for (int i = -kernelSizeH; i <= kernelSizeH; i++) {
                int newRow = row + i;
                if (newRow < 0 || newRow >= height) {
                    continue;
                }
                uchar *rowPtr = result.ptr<uchar>(newRow);
                for (int j = -kernelSizeW; j <= kernelSizeW; j++) {
                    int newCol = col + j;
                    if (newCol >= 0 && newCol < width) {
                        if (rowPtr[newCol] == 0) {
                            rowPtr[newCol] =
                                uchar(depth * 255 /
                                      (mParamDepthMax - mParamDepthMin));
                        }
                        rowPtr[newCol] = std::min(rowPtr[newCol], pixel);
                    }
                }
            }
        }
    }
}

void FrameKDMap::BuildEdgeCloud(
    cv::Mat &invDepthImg, pcl::PointCloud<pcl::PointXYZ>::Ptr &edgeCloud) {
    // Step 1: inflate the depth image
    cv::Mat kernel = cv::Mat::ones(3, 3, CV_8UC1);
    cv::Mat inflatedDepthImg = cv::Mat::zeros(invDepthImg.size(), CV_8UC1);
    for (int row = 0; row < invDepthImg.rows; row++) {
        for (int col = 0; col < invDepthImg.cols; col++) {
            float invDepth = invDepthImg.at<float>(row, col);
            if (invDepth > 1e-2) {
                char depth = uchar(1 / invDepth /
                                   (mParamDepthMax - mParamDepthMin) *
                                   200.0f);
                inflatedDepthImg.at<uchar>(row, col) = uchar(depth);
            }
            else {
                inflatedDepthImg.at<uchar>(row, col) = 255;
            }
        }
    }
    cv::erode(inflatedDepthImg, inflatedDepthImg, kernel);
    cv::Mat cannyImage;
    cv::Canny(inflatedDepthImg, cannyImage, 0.1, 0.3);
    // Step 3: build edge cloud
    for (int row = 0; row < cannyImage.rows; row++) {
        for (int col = 0; col < cannyImage.cols; col++) {
            if (cannyImage.at<uchar>(row, col) > 0) {
                double depth = float(inflatedDepthImg.at<uchar>(row, col));
                depth = depth * (mParamDepthMax - mParamDepthMin) / 200.0;
                if (depth > mParamDepthMax || depth < mParamDepthMin) {
                    continue;
                }
                Eigen::Vector4d pointCam = UV2Camera(col, row, depth);
                Eigen::Vector3d point =
                    (mCurFrame.Twc * mParamTbc * pointCam).topRows(3);
                edgeCloud->emplace_back(point.x(), point.y(), point.z());
            }
        }
    }
}
bool FrameKDMap::PtIsInFrame(const Eigen::Vector3d &ptw,
                             const Eigen::Matrix4d &Twc) {
    Eigen::Vector4d ptCam =
        Twc.inverse() * Eigen::Vector4d(ptw.x(), ptw.y(), ptw.z(), 1);
    double x = ptCam.x();
    double y = ptCam.y();
    double z = ptCam.z();
    if (z > mParamDepthMax || z < 0) {
        return false;
    }
    double u = mParamFx * x / z + mParamCx;
    double v = mParamFy * y / z + mParamCy;
    if (u < 0 || u >= mParamWidth || v < 0 || v >= mParamHeight) {
        return false;
    }
    return true;
}

bool FrameKDMap::DroneBehindPts(const Eigen::Matrix4d &Twc,
                                const Frame &frame) {
    Eigen::Matrix4d Twb = Twc * mParamTbc.inverse();
    Eigen::Vector3d twb = Twb.topRightCorner(3, 1);
    Eigen::Matrix3d Rwb = Twb.topLeftCorner(3, 3);
    Eigen::Matrix3d Rbw = Rwb.transpose();
    int ptsCount =
        std::min(int(frame.pointCloud->GetPointCloud().pts.size()), 10);
    frame.pointCloud->SearchForNearest(twb.x(), twb.y(), twb.z(), ptsCount);
    const std::vector<pcl::PointXYZ> &nearestPts =
        frame.pointCloud->closest_pts;
    for (const auto &pt : nearestPts) {
        Eigen::Vector3d ptw(pt.x, pt.y, pt.z);
        Eigen::Vector3d ptb = Rbw * (ptw - twb);
        if (ptb.x() <= mParamDepthMin) {
            return false;
        }
    }
    return true;
}

void FrameKDMap::QueryNearestWithCurFrame(
    const Eigen::Vector3d &point, int nearestPointCount,
    std::vector<Eigen::Vector3d> &nearestPoints, std::vector<double> &distances,
    bool queryEdge) {
    FrameKDMap::PtCloudKdPtr ptCloud;
    if (!queryEdge) {
        ptCloud = mCurFrame.pointCloud;
    } else {
        ptCloud = mCurFrame.edgeCloud;
    }
    ptCloud->SearchForNearest(point.x(), point.y(), point.z(),
                              nearestPointCount);
    const std::vector<double> &distancesCur = ptCloud->squared_distances;
    const std::vector<pcl::PointXYZ> &pointsCur = ptCloud->closest_pts;
    nearestPoints.clear();
    distances.clear();
    for (int i = 0; i < distancesCur.size(); ++i) {
        nearestPoints.push_back(
            Eigen::Vector3d(pointsCur[i].x, pointsCur[i].y, pointsCur[i].z));
        distances.push_back(distancesCur[i]);
    }
}
void FrameKDMap::QueryNearestThreadWorker(int start, int end,
                                          int nearestPointCount,
                                          const Eigen::Vector3d &point,
                                          std::mutex &pointsDistMutex,
                                          std::vector<PtDists> &pointDist,
                                          bool queryEdge) {
    std::vector<PtDists> localPointsDist;
    for (int i = start; i < end; ++i) {
        if (i >= mVecQueryVector.size() || mVecQueryVector[i] == nullptr) {
            continue;
        }
        auto &ptrFramei = mVecQueryVector[i];
        FrameKDMap::PtCloudKdPtr ptCloud;
        int framePointCount = 0;
        if (!queryEdge) {
            framePointCount = ptrFramei->pointCloud->GetPointCloud().pts.size();
            ptCloud = ptrFramei->pointCloud;
        } else {
            framePointCount = ptrFramei->edgeCloud->GetPointCloud().pts.size();
            ptCloud = ptrFramei->edgeCloud;
        }
        ptrFramei->pointCloud->GetPointCloud().pts.size();
        int queryPointCount = std::min(nearestPointCount, framePointCount);

        ptCloud->SearchForNearest(point.x(), point.y(), point.z(),
                                  queryPointCount);
        const std::vector<double> &distancesi = ptCloud->squared_distances;
        const std::vector<pcl::PointXYZ> &nearestPts = ptCloud->closest_pts;
        if (distancesi.size() != nearestPts.size()) {
            ROS_ERROR("\033[95m[MPC]\033[0m Error: distances size not equal to "
                      "points size");
            continue;
        }
        for (int j = 0; j < distancesi.size(); ++j) {
            Eigen::Vector3d pt(nearestPts[j].x, nearestPts[j].y,
                               nearestPts[j].z);
            localPointsDist.emplace_back(pt, distancesi[j], true);
        }
    }
    if (localPointsDist.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(pointsDistMutex);
    pointDist.insert(pointDist.end(), localPointsDist.begin(),
                     localPointsDist.end());
}
void FrameKDMap::QueryNearest(const Eigen::Vector3d &point,
                              int nearestPointCount,
                              std::vector<Eigen::Vector3d> &nearestPoints,
                              std::vector<double> &distances, bool queryEdge) {
    std::vector<PtDists> pointsDist;
    std::mutex pointsDistMutex;
    mMtxKdTree.lock();
    if (!queryEdge && (mCurFrame.pointCloud.get() != nullptr) ||
        queryEdge && (mCurFrame.edgeCloud.get() != nullptr)) {
        int firstFramePointCount = 0;
        if (!queryEdge) {
            firstFramePointCount =
                mCurFrame.pointCloud->GetPointCloud().pts.size();
        } else {
            firstFramePointCount =
                mCurFrame.edgeCloud->GetPointCloud().pts.size();
        }
        if (firstFramePointCount >= nearestPointCount &&
            PtIsInFrame(point, mCurFrame.Twc)) {
            QueryNearestWithCurFrame(point, nearestPointCount, nearestPoints,
                                     distances, queryEdge);
            mMtxKdTree.unlock();
            return;
        }
    }
    int numThreads = std::thread::hardware_concurrency();
    int numVertices = mVecQueryVector.size();
    numThreads = std::min(numThreads, numVertices);
    int chunkSize = (numVertices + numThreads) / numThreads;
    std::vector<std::thread> threads;
    for (int i = 0; i < numThreads; ++i) {
        int start = i * chunkSize;
        int end = std::min(start + chunkSize, numVertices);
        if (start < end) {
            threads.emplace_back(&FrameKDMap::QueryNearestThreadWorker, this,
                                 start, end, nearestPointCount, point,
                                 std::ref(pointsDistMutex),
                                 std::ref(pointsDist), queryEdge);
        }
    }
    for (auto &t : threads) {
        t.join();
    }
    mMtxKdTree.unlock();
    nearestPoints.clear();
    distances.clear();
    if (pointsDist.empty()) {
        return;
    }
    std::sort(pointsDist.begin(), pointsDist.end());
    for (int i = 0; i < nearestPointCount && i < pointsDist.size(); ++i) {
        nearestPoints.push_back(pointsDist[i].pt);
        distances.push_back(pointsDist[i].dist);
    }
}

void FrameKDMap::GetNearestDistanceThreadWorker(int start, int end,
                                                const Eigen::Vector3d &point,
                                                std::mutex &minDistanceMutex,
                                                double &nearestDistance) {
    double localNearestDistance = std::numeric_limits<double>::max();
    for (int i = start; i < end; ++i) {
        if (i >= mVecQueryVector.size() || mVecQueryVector[i] == nullptr ||
            mVecQueryVector[i]->pointCloud->GetPointCloud().pts.empty()) {
            continue;
        }
        auto &prtFramei = mVecQueryVector[i];
        prtFramei->pointCloud->SearchForNearest(point.x(), point.y(), point.z(),
                                                1);
        std::vector<double> distances =
            prtFramei->pointCloud->squared_distances;
        if (!distances.empty()) {
            localNearestDistance = std::min(localNearestDistance, distances[0]);
        }
    }
    std::lock_guard<std::mutex> lock(minDistanceMutex);
    nearestDistance = std::min(nearestDistance, localNearestDistance);
}
double FrameKDMap::GetNearestDistance(const Eigen::Vector3d &point) {
    double nearestDistance = std::numeric_limits<double>::max();
    std::mutex minDistanceMutex;
    if (mVecQueryVector.empty()) {
        return nearestDistance;
    }
    mMtxKdTree.lock();
    int numThreads = std::thread::hardware_concurrency();
    int numVertices = mVecQueryVector.size();
    numThreads = std::min(numThreads, numVertices);
    int chunkSize = (numVertices + numThreads) / numThreads;
    std::vector<std::thread> threads;
    for (int i = 0; i < numThreads; ++i) {
        int start = i * chunkSize;
        int end = std::min(start + chunkSize, numVertices);
        if (start < end) {
            threads.emplace_back(
                &FrameKDMap::GetNearestDistanceThreadWorker, this, start, end,
                point, std::ref(minDistanceMutex), std::ref(nearestDistance));
        }
    }

    for (auto &t : threads) {
        t.join();
    }
    mMtxKdTree.unlock();
    return std::sqrt(nearestDistance);
}
void FrameKDMap::InsertKeyFrame() {
    mKeyFrameMap.emplace_back(mCurFrame.pointCloud, mCurFrame.edgeCloud,
                              mCurFrame.Twc);
    UpdateQueryVector();
}
void FrameKDMap::ReplaceKdTree(Frame &replacedFrame, PtCloudKdPtr &newKdTree) {
    replacedFrame.pointCloud->Clear();
    replacedFrame.pointCloud = newKdTree;
}
void FrameKDMap::KeyframeThreadWorker() {
    while (ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        if (!mbNeedProcessPtCloud) {
            continue;
        }
        std::lock_guard<std::mutex> lock(mMtxKdTree);
        mbNeedProcessPtCloud = false;
        if (mKeyFrameMap.empty()) {
            InsertKeyFrame();
            continue;
        }
        while (mKeyFrameMap.size() > 0) {
            Frame &oldestFrame = mKeyFrameMap.front();
            if (mKeyFrameMap.size() > mParamMaxFrameCount ||
                !DroneBehindPts(mCurFrame.Twc, oldestFrame)) {
                RemoveOldVertex();
                UpdateQueryVector();
            } else {
                break;
            }
        }
        if (mKeyFrameMap.empty()) {
            continue;
        }
        PtCloudKdPtr lastPtCloudPtr = mKeyFrameMap.back().pointCloud;
        const std::vector<pcl::PointXYZ> &lastPts =
            lastPtCloudPtr->GetPointCloud().pts;
        std::vector<int> outlierPtsIndex;
        for (int index = 0; index < lastPts.size(); index++) {
            const pcl::PointXYZ &pt = lastPts[index];
            mCurFrame.pointCloud->SearchForNearest(pt.x, pt.y, pt.z, 1);
            if (!mCurFrame.pointCloud->squared_distances.empty()) {
                double nearestDist =
                    std::sqrt(mCurFrame.pointCloud->squared_distances[0]);
                if (nearestDist > mParamKeyframeDistanceTh) {
                    outlierPtsIndex.push_back(index);
                }
            }
        }
        if (outlierPtsIndex.size() < mParamKeyframeCountTh) {
            continue;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        for (int outlierIndex : outlierPtsIndex) {
            newCloud->push_back(lastPts[outlierIndex]);
        }
        lastPtCloudPtr->InitializeNew(newCloud);
        InsertKeyFrame();
    }
}

pcl::PointCloud<pcl::PointXYZRGB> FrameKDMap::GetPtCloud() {
    std::lock_guard<std::mutex> lock(mMtxKdTree);
    pcl::PointCloud<pcl::PointXYZRGB> ptCloud;
    for (int i = 0; i < mVecQueryVector.size(); i++) {
        auto &keyFrame = mVecQueryVector[i];
        if (keyFrame == nullptr) {
            continue;
        }
        const auto &pts = keyFrame->pointCloud.get()->GetPointCloud().pts;
        std::vector<int> colors = {255, 255, 255};
        if (i != 0) {
            colors = keyFrame->pointCloud.get()->GetColors();
        }
        for (const auto &pt : pts) {
            pcl::PointXYZRGB ptRGB;
            ptRGB.x = pt.x;
            ptRGB.y = pt.y;
            ptRGB.z = pt.z;
            ptRGB.r = colors[0];
            ptRGB.g = colors[1];
            ptRGB.b = colors[2];
            ptCloud.push_back(ptRGB);
        }
    }
    return ptCloud;
}
