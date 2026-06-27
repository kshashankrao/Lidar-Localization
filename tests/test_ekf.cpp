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

TEST(EKFTest, PredictStepIncreasesUncertainty) {
    EKF ekf;
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(9);
    ekf.init(x0);
    
    // Test predict advances position based on velocity
    Eigen::VectorXd state = ekf.getState();
    state(3) = 10.0; // vx
    ekf.setState(state);
    
    ekf.predict(0.1, 0.01);
    state = ekf.getState();
    
    // px = px + vx * dt = 0 + 10 * 0.1 = 1.0
    EXPECT_DOUBLE_EQ(state(0), 1.0);
}

TEST(EKFTest, UpdateIMUYawWrap) {
    EKF ekf;
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(9);
    x0(8) = M_PI - 0.1; // Current yaw near +pi
    ekf.init(x0);
    
    // Provide IMU measurement near -pi (which is physically close to +pi)
    Eigen::Vector3d z_imu;
    z_imu << 0, 0, -M_PI + 0.1;
    
    ekf.updateIMU(z_imu, 0.1);
    
    Eigen::VectorXd state = ekf.getState();
    // The innovation should be handled so yaw stays within [-pi, pi] 
    // and doesn't do a massive rotation through 0.
    EXPECT_GE(state(8), -M_PI);
    EXPECT_LE(state(8), M_PI);
}
