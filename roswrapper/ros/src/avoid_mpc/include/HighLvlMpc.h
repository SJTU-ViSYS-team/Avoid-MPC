#ifndef HIGH_LVL_MPC_H
#define HIGH_LVL_MPC_H
#include <casadi/casadi.hpp>
class ObstacleAvoidanceMPC {
public:
    ObstacleAvoidanceMPC();
    ObstacleAvoidanceMPC(double T, double dt, std::string soPath);
    void Solve(const std::vector<double> &vecRefStates, std::vector<double> &u,
               std::vector<std::vector<double>> &x0Array, bool faster = false);
    void SetupWeights(const std::vector<double> &weights);
    void SetupTau(const std::vector<double> &tau);
    void SetupGains(const std::vector<double> &gains);
    void SetDroneRadius(const double droneRadius);
    void SetDroneAccelLimits(const double aMinZ, const double aMaxZ,
                             const double aMaxXy, const double aMaxYawDot);

private:
    double mT;
    double mDt;
    int mN;
    int mDimX;
    int mDimU;
    double mDroneRadius;
    std::vector<double> mTau;
    std::vector<double> mGains;
    std::vector<double> mWeights;
    std::vector<double> mNlpW0;
    std::vector<double> mLbw;
    std::vector<double> mUbw;
    std::vector<double> mLbg;
    std::vector<double> mUbg;
    casadi::Function mSolver;
    casadi::Function mSolverFaster;
};
#endif