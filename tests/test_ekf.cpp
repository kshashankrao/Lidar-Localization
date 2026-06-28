#include <gtest/gtest.h>
#include "ekf/ekf.h"
#include <cmath>

TEST(EKFTest, Initialization) {
    EKF ekf;
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(9);
    x0(0) = 1.0; x0(1) = 2.0; x0(2) = 3.0;
    ekf.init(x0);
    
    Eigen::VectorXd state = ekf.getState();
    EXPECT_DOUBLE_EQ(state(0), 1.0);
    EXPECT_DOUBLE_EQ(state(1), 2.0);
    EXPECT_DOUBLE_EQ(state(2), 3.0);
}

TEST(EKFTest, PredictIMUAdvancesPosition) {
    EKF ekf;
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(9);
    ekf.init(x0);
    
    // Test predict advances position based on IMU body velocities
    // dt = 0.1, vf = 10.0, vl=0, vu=0, wf=0, wl=0, wu=0
    ekf.predictIMU(0.1, 0.01, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    Eigen::VectorXd state = ekf.getState();
    
    // px = px + vf * dt = 0 + 10 * 0.1 = 1.0
    EXPECT_DOUBLE_EQ(state(0), 1.0);
}

TEST(EKFTest, PredictIMUYawWrap) {
    EKF ekf;
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(9);
    x0(8) = M_PI - 0.05; // Current yaw near +pi
    ekf.init(x0);
    
    // Rotate +0.1 rad in yaw
    ekf.predictIMU(0.1, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0); // wu = 1.0 rad/s
    
    Eigen::VectorXd state = ekf.getState();
    // The yaw should wrap from ~pi to ~-pi
    EXPECT_GE(state(8), -M_PI);
    EXPECT_LE(state(8), 0.0);
}
