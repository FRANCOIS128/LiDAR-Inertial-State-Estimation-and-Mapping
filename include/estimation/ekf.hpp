#pragma once

#include "common/types.hpp"
#include "imu/imu_propagator.hpp"

#include <Eigen/Dense>

namespace lio {

struct EkfConfig {
    // world z is up, so gravity is -z
    Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
    double accel_noise = 0.08;
    double gyro_noise = 0.01;
    double accel_bias_noise = 1e-4;
    double gyro_bias_noise = 1e-5;
    double lidar_pos_noise = 0.05;
    double lidar_rot_noise = 0.02;
};

// Loosely coupled: IMU for predict, lidar pose for update.
// Error state 15x1: dp, dv, dtheta, dba, dbg.
class PoseEkf {
public:
    explicit PoseEkf(EkfConfig config = {});

    void set_state(const NavState& state);
    const NavState& state() const { return propagator_.state(); }

    Pose pose() const;

    void predict(const ImuMeasurement& imu, double dt);
    void update_pose(const Pose& lidar_pose);

    const Eigen::Matrix<double, 15, 15>& covariance() const { return P_; }

private:
    using Mat15 = Eigen::Matrix<double, 15, 15>;
    using Vec15 = Eigen::Matrix<double, 15, 1>;

    void inject_error(const Vec15& dx);

    EkfConfig config_;
    ImuPropagator propagator_;
    Mat15 P_ = Mat15::Identity();
};

}  // namespace lio
