#ifndef COG_FILTER_H
#define COG_FILTER_H
#include <Eigen/Core>
#include <cmath>
#include <deque>
#include <iostream>
#include <vector>
class COGFilter {
public:
    COGFilter(int windowSize, float weightDecay)
        : mWindowSize(windowSize), mWeightDecay(weightDecay) {
    }

    Eigen::Vector3d filter(const Eigen::Vector3d &accelData);
    void Reset() {
        mAccelDataQueue.clear();
    }

private:
    int mWindowSize;
    float mWeightDecay;

    std::deque<Eigen::Vector3d> mAccelDataQueue;
};
#endif