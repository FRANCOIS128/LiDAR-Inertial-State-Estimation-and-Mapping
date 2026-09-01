#include "geometry/transform.hpp"
#include "imu/imu_propagator.hpp"

#include <cmath>
#include <iostream>

namespace {

int g_failed = 0;

void check(bool ok, const char* name) {
    if (!ok) {
        std::cerr << "FAIL: " << name << "\n";
        g_failed++;
    } else {
        std::cout << "ok   " << name << "\n";
    }
}

}  // namespace

int main() {
    const Eigen::Vector3d gravity(0.0, 0.0, -9.81);

    // IMU not moving: accel should read +g on body z.
    {
        lio::ImuPropagator imu(gravity);
        lio::NavState s;
        imu.set_state(s);

        lio::ImuMeasurement m;
        m.acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
        m.angular_velocity = Eigen::Vector3d::Zero();

        const double dt = 0.01;
        for (int i = 0; i < 200; ++i) {
            imu.propagate(m, dt);
        }

        check(imu.state().position.norm() < 1e-3, "zero motion keeps position");
        check(imu.state().velocity.norm() < 1e-3, "zero motion keeps velocity");
        const double yaw = lio::log_so3(lio::quaternion_to_rotation(imu.state().orientation)).norm();
        check(yaw < 1e-6, "zero motion keeps orientation");
    }

    // Constant 1 m/s^2 along x in world, no rotate.
    {
        lio::ImuPropagator imu(gravity);
        lio::NavState s;
        imu.set_state(s);

        lio::ImuMeasurement m;
        m.acceleration = Eigen::Vector3d(1.0, 0.0, 9.81);
        m.angular_velocity = Eigen::Vector3d::Zero();

        const double dt = 0.01;
        const double T = 1.0;
        for (int i = 0; i < static_cast<int>(T / dt); ++i) {
            imu.propagate(m, dt);
        }

        check(std::abs(imu.state().velocity.x() - 1.0) < 0.02, "vx ≈ a t");
        check(std::abs(imu.state().position.x() - 0.5) < 0.03, "px ≈ 0.5 a t^2");
        check(std::abs(imu.state().velocity.z()) < 0.02, "no vertical velocity");
    }

    if (g_failed > 0) {
        return 1;
    }
    std::cout << "test_imu passed\n";
    return 0;
}
