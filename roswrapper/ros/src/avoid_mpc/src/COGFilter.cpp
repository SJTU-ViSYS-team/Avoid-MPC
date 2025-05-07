#include "COGFilter.h"
Eigen::Vector3d COGFilter::filter(const Eigen::Vector3d &accelData) {
    mAccelDataQueue.push_back(accelData);

    if (mAccelDataQueue.size() > mWindowSize) {
        mAccelDataQueue.pop_front();
    }

    Eigen::Vector3d weightedAccel = Eigen::Vector3d::Zero();
    float totalWeight = 0.0f;

    int idx = 0;
    for (auto it = mAccelDataQueue.rbegin(); it != mAccelDataQueue.rend();
         ++it, ++idx) {
        float weight = std::pow(mWeightDecay, idx);
        weightedAccel += *it * weight;
        totalWeight += weight;
    }

    weightedAccel /= totalWeight;

    return weightedAccel;
}