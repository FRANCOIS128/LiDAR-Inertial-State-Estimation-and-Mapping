#include "estimation/ekf.hpp"
#include "geometry/transform.hpp"

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
    lio::PoseEkf ekf;
    lio::NavState s;
    s.position = Eigen::Vector3d(0.0, 0.0, 0.0);
    ekf.set_state(s);

    lio::Pose meas;
    meas.position = Eigen::Vector3d(1.0, -0.5, 0.2);
    meas.orientation = Eigen::Quaterniond::Identity();

    for (int i = 0; i < 8; ++i) {
        ekf.update_pose(meas);
    }

    const Eigen::Vector3d p = ekf.pose().position;
    std::cout << "after updates p = " << p.transpose() << "\n";
    check((p - meas.position).norm() < 0.05, "EKF pose update moves toward measurement");

    // Rest IMU, state should still be finite.
    lio::ImuMeasurement imu;
    imu.acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
    imu.angular_velocity = Eigen::Vector3d::Zero();
    ekf.predict(imu, 0.01);
    check(std::isfinite(ekf.pose().position.x()), "predict keeps finite state");

    if (g_failed > 0) {
        return 1;
    }
    std::cout << "test_ekf passed\n";
    return 0;
}
