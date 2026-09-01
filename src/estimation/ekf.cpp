#include "estimation/ekf.hpp"

#include "geometry/transform.hpp"

namespace lio {

PoseEkf::PoseEkf(EkfConfig config)
    : config_(config), propagator_(config.gravity) {
    // Initialize covariance with lower uncertainty on sensor biases.
    P_ = Mat15::Identity();
    P_.block<3, 3>(0, 0) *= 0.1;
    P_.block<3, 3>(3, 3) *= 0.1;
    P_.block<3, 3>(6, 6) *= 0.05;
    P_.block<3, 3>(9, 9) *= 0.01;
    P_.block<3, 3>(12, 12) *= 0.01;
}

void PoseEkf::set_state(const NavState& state) {
    propagator_.set_state(state);
}

Pose PoseEkf::pose() const {
    Pose p;
    p.position = propagator_.state().position;
    p.orientation = propagator_.state().orientation;
    return p;
}

void PoseEkf::predict(const ImuMeasurement& imu, double dt) {
    if (dt <= 0.0) {
        return;
    }

    const NavState s = propagator_.state();
    const Eigen::Matrix3d R = quaternion_to_rotation(s.orientation);
    const Eigen::Vector3d acc = imu.acceleration - s.accel_bias;

    Mat15 F = Mat15::Identity();
    // Simplified discrete error-state transition model.
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;          // dp / dv
    F.block<3, 3>(3, 6) = -R * skew(acc) * dt;                       // dv / dtheta
    F.block<3, 3>(3, 9) = -R * dt;                                   // dv / dba
    F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;         // dtheta / dbg

    Mat15 Q = Mat15::Zero();
    const double dt2 = dt * dt;
    Q.block<3, 3>(3, 3) =
        Eigen::Matrix3d::Identity() * (config_.accel_noise * config_.accel_noise) * dt2;
    Q.block<3, 3>(6, 6) =
        Eigen::Matrix3d::Identity() * (config_.gyro_noise * config_.gyro_noise) * dt;
    Q.block<3, 3>(9, 9) =
        Eigen::Matrix3d::Identity() * (config_.accel_bias_noise * config_.accel_bias_noise) * dt;
    Q.block<3, 3>(12, 12) =
        Eigen::Matrix3d::Identity() * (config_.gyro_bias_noise * config_.gyro_bias_noise) * dt;

    propagator_.propagate(imu, dt);
    P_ = F * P_ * F.transpose() + Q;  // P_k = F P F^T + Q
}

void PoseEkf::update_pose(const Pose& lidar_pose) {
    const NavState s = propagator_.state();
    const Eigen::Matrix3d R = quaternion_to_rotation(s.orientation);
    const Eigen::Matrix3d R_meas = quaternion_to_rotation(lidar_pose.orientation);

    Eigen::Matrix<double, 6, 1> residual;
    residual.head<3>() = lidar_pose.position - s.position;
    // rotation error as 3-vector: Log(R^T * R_meas)
    residual.tail<3>() = log_so3(R.transpose() * R_meas);

    Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // measure p
    H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();  // measure rotation error

    Eigen::Matrix<double, 6, 6> Rm = Eigen::Matrix<double, 6, 6>::Zero();
    Rm.block<3, 3>(0, 0) =
        Eigen::Matrix3d::Identity() * (config_.lidar_pos_noise * config_.lidar_pos_noise);
    Rm.block<3, 3>(3, 3) =
        Eigen::Matrix3d::Identity() * (config_.lidar_rot_noise * config_.lidar_rot_noise);

    const Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + Rm;
    const Eigen::Matrix<double, 15, 6> K = P_ * H.transpose() * S.inverse();
    const Vec15 dx = K * residual;

    // Joseph form. More stable than only (I-KH)P.
    const Eigen::Matrix<double, 15, 15> IKH = Mat15::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * Rm * K.transpose();

    inject_error(dx);
}

void PoseEkf::inject_error(const Vec15& dx) {
    // Put error dx back to normal state.
    NavState s = propagator_.state();
    s.position += dx.segment<3>(0);
    s.velocity += dx.segment<3>(3);
    s.orientation =
        (s.orientation * rotation_to_quaternion(exp_so3(dx.segment<3>(6)))).normalized();
    s.accel_bias += dx.segment<3>(9);
    s.gyro_bias += dx.segment<3>(12);
    propagator_.set_state(s);
}

}  // namespace lio
