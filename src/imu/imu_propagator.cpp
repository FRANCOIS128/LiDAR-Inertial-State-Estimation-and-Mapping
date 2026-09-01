#include "imu/imu_propagator.hpp"

#include "geometry/transform.hpp"

namespace lio {

ImuPropagator::ImuPropagator(Eigen::Vector3d gravity) : gravity_(std::move(gravity)) {}

void ImuPropagator::set_state(const NavState& state) {
    state_ = state;
    state_.orientation.normalize();
}

void ImuPropagator::propagate(const ImuMeasurement& imu, double dt) {
    if (dt <= 0.0) {
        return;
    }

    const Eigen::Matrix3d R = quaternion_to_rotation(state_.orientation);
    const Eigen::Vector3d omega = imu.angular_velocity - state_.gyro_bias;
    const Eigen::Vector3d acc_body = imu.acceleration - state_.accel_bias;
    // IMU gives specific force, so a_world = R*(a-ba) + g.
    // When not moving, a_meas is about (0,0,9.81), a_world is 0.
    const Eigen::Vector3d acc_world = R * acc_body + gravity_;

    // R_{k+1} = R_k * Exp(omega * dt)
    state_.orientation =
        (state_.orientation * rotation_to_quaternion(exp_so3(omega * dt))).normalized();
    state_.position = state_.position + state_.velocity * dt + 0.5 * acc_world * dt * dt;
    state_.velocity = state_.velocity + acc_world * dt;
}

}  // namespace lio
