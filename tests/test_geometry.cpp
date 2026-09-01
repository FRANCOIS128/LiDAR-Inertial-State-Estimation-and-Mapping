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
    using lio::Pose;

    const Eigen::Matrix3d I = lio::euler_to_rotation(0.0, 0.0, 0.0);
    check((I - Eigen::Matrix3d::Identity()).norm() < 1e-9, "identity euler");

    const Eigen::Matrix3d Rz = lio::euler_to_rotation(0.0, 0.0, M_PI / 2.0);
    const Eigen::Vector3d x_axis = Rz * Eigen::Vector3d(1.0, 0.0, 0.0);
    // yaw 90 deg, x should go to y
    check((x_axis - Eigen::Vector3d(0.0, 1.0, 0.0)).norm() < 1e-9, "90 deg yaw maps x to y");

    Pose pose;
    pose.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    pose.orientation = lio::rotation_to_quaternion(Eigen::Matrix3d::Identity());
    const Eigen::Vector3d p(0.5, -1.0, 4.0);
    const Eigen::Vector3d pw = lio::transform_point(pose, p);
    check((pw - Eigen::Vector3d(1.5, 1.0, 7.0)).norm() < 1e-9, "p_world = R p + t");

    const Pose inv = lio::inverse_pose(pose);
    const Eigen::Vector3d back = lio::transform_point(inv, pw);
    check((back - p).norm() < 1e-9, "inverse transform");

    Pose a;
    a.position = Eigen::Vector3d(1.0, 0.0, 0.0);
    Pose b;
    b.position = Eigen::Vector3d(0.0, 2.0, 0.0);
    const Pose ab = lio::compose_poses(a, b);
    check((ab.position - Eigen::Vector3d(1.0, 2.0, 0.0)).norm() < 1e-9, "compose translations");

    const Eigen::Matrix3d R = lio::exp_so3(Eigen::Vector3d(0.0, 0.0, 0.3));
    const Eigen::Vector3d w = lio::log_so3(R);
    check(std::abs(w.z() - 0.3) < 1e-6, "so3 log/exp");

    if (g_failed > 0) {
        std::cerr << g_failed << " geometry checks failed\n";
        return 1;
    }
    std::cout << "test_geometry passed\n";
    return 0;
}
