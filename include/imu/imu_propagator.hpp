#pragma once

#include "common/types.hpp"

namespace lio {

class ImuPropagator {
public:
    explicit ImuPropagator(Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81));

    void set_state(const NavState& state);
    const NavState& state() const { return state_; }

    // One IMU step for R, v, p. dt is seconds.
    void propagate(const ImuMeasurement& imu, double dt);

private:
    NavState state_;
    Eigen::Vector3d gravity_;
};

}  // namespace lio
