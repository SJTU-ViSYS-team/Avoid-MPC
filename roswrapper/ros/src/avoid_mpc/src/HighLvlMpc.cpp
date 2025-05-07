#include "HighLvlMpc.h"

ObstacleAvoidanceMPC::ObstacleAvoidanceMPC() {
}
ObstacleAvoidanceMPC::ObstacleAvoidanceMPC(double T, double dt,
                                           std::string soPath) {
    mT = T;
    mDt = dt;
    mN = T / dt;
    mDimX = 10;
    mDimU = 4;

    double aMinZ = 1.;
    double aMaxZ = 20.;
    double aMaxXy = 10.;
    double aMaxYawDot = 10.;
    casadi::Dict ipopt_options;
    ipopt_options["verbose"] = false;
    ipopt_options["ipopt.tol"] = 1.e-4;
    ipopt_options["ipopt.max_iter"] = 10;
    ipopt_options["ipopt.warm_start_init_point"] = "yes";
    ipopt_options["ipopt.print_level"] = 0;
    ipopt_options["print_time"] = false;

    std::vector<double> quadS0 = {0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> quadU0 = {0.0, 0.0, 0.0, 0.0};
    std::vector<double> uMin = {-aMaxXy, -aMaxXy, aMinZ, -aMaxYawDot};
    std::vector<double> uMax = {aMaxXy, aMaxXy, aMaxZ, aMaxYawDot};
    std::vector<double> xMin = std::vector<double>(mDimX, -casadi::inf);
    std::vector<double> xMax = std::vector<double>(mDimX, casadi::inf);
    std::vector<double> gMin = std::vector<double>(mDimX, 0);
    std::vector<double> gMax = std::vector<double>(mDimX, 0);

    mNlpW0 = quadS0;
    mLbw = xMin;
    mUbw = xMax;
    mLbg = gMin;
    mUbg = gMax;
    for (int i = 0; i < mN; i++) {
        mNlpW0.insert(mNlpW0.end(), quadU0.begin(), quadU0.end());
        mNlpW0.insert(mNlpW0.end(), quadS0.begin(), quadS0.end());
        mLbw.insert(mLbw.end(), uMin.begin(), uMin.end());
        mLbw.insert(mLbw.end(), xMin.begin(), xMin.end());
        mUbw.insert(mUbw.end(), uMax.begin(), uMax.end());
        mUbw.insert(mUbw.end(), xMax.begin(), xMax.end());
        mLbg.insert(mLbg.end(), gMin.begin(), gMin.end());
        mUbg.insert(mUbg.end(), gMax.begin(), gMax.end());
    }
    mSolver = casadi::nlpsol("solve", "ipopt", soPath, ipopt_options);
    ipopt_options["ipopt.max_iter"] = 10;
    mSolverFaster = casadi::nlpsol("solve", "ipopt", soPath, ipopt_options);
    mWeights = {100, 100, 100, 300, 1,  1,  1,  0., 0., 0., 0.0, 10, 10,
                30,  0,   1,   1,   0., 0., 0., 1., 1., 1., 1.,  1.};
    mTau = {0.01, 0.01, 0.01, 0};
    mGains = {1, 1, 1, 1};
}
void ObstacleAvoidanceMPC::SetupWeights(const std::vector<double> &weights) {
    mWeights = weights;
}
void ObstacleAvoidanceMPC::SetupTau(const std::vector<double> &tau) {
    mTau = tau;
}
void ObstacleAvoidanceMPC::SetDroneRadius(const double droneRadius) {
    mDroneRadius = droneRadius;
}
void ObstacleAvoidanceMPC::SetupGains(const std::vector<double> &gains) {
    mGains = gains;
}
void ObstacleAvoidanceMPC::SetDroneAccelLimits(const double aMinZ,
                                               const double aMaxZ,
                                               const double aMaxXy,
                                               const double aMaxYawDot) {
    std::vector<double> uMin = {-aMaxXy, -aMaxXy, aMinZ, -aMaxYawDot};
    std::vector<double> uMax = {aMaxXy, aMaxXy, aMaxZ, aMaxYawDot};
    std::vector<double> xMin = std::vector<double>(mDimX, -casadi::inf);
    std::vector<double> xMax = std::vector<double>(mDimX, casadi::inf);
    std::vector<double> gMin = std::vector<double>(mDimX, 0);
    std::vector<double> gMax = std::vector<double>(mDimX, 0);
    mLbw = xMin;
    mUbw = xMax;
    mLbg = gMin;
    mUbg = gMax;
    for (int i = 0; i < mN; i++) {
        mLbw.insert(mLbw.end(), uMin.begin(), uMin.end());
        mLbw.insert(mLbw.end(), xMin.begin(), xMin.end());
        mUbw.insert(mUbw.end(), uMax.begin(), uMax.end());
        mUbw.insert(mUbw.end(), xMax.begin(), xMax.end());
        mLbg.insert(mLbg.end(), gMin.begin(), gMin.end());
        mUbg.insert(mUbg.end(), gMax.begin(), gMax.end());
    }
}
void ObstacleAvoidanceMPC::Solve(const std::vector<double> &vecRefStates,
                                 std::vector<double> &u,
                                 std::vector<std::vector<double>> &x0Array,
                                 bool faster) {
    std::vector<double> refStatesCopy = vecRefStates;
    for (double gain : mGains) {
        refStatesCopy.push_back(gain);
    }
    for (double tau : mTau) {
        refStatesCopy.push_back(tau);
    }
    for (double weight : mWeights) {
        refStatesCopy.push_back(weight);
    }
    refStatesCopy.push_back(mDroneRadius);
    casadi::DM refStates = refStatesCopy;
    casadi::DMDict args;
    args["x0"] = mNlpW0;
    args["lbx"] = mLbw;
    args["ubx"] = mUbw;
    args["p"] = refStates;
    args["lbg"] = mLbg;
    args["ubg"] = mUbg;
    casadi::DMDict sol;
    if (faster) {
        sol = mSolverFaster(args);
    } else {
        sol = mSolver(args);
    }
    std::vector<double> sol_x0 = sol["x"].get_elements();

    u.clear();
    u.resize(mDimU);
    for (int i = 0; i < mDimU; i++) {
        u[i] = sol_x0[i + mDimX];
    }
    mNlpW0 = sol_x0;
    x0Array.clear();
    for (int i = 0; i < sol_x0.size() - mDimX; i++) {
        if (i % (mDimX + mDimU) == 0) {
            x0Array.push_back(std::vector<double>());
        }
        x0Array.back().push_back(sol_x0[i]);
    }
}
