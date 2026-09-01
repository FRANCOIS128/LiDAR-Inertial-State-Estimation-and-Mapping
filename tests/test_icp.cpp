#include "geometry/transform.hpp"
#include "registration/icp.hpp"

#include <cmath>
#include <iostream>
#include <random>

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

lio::PointCloudPtr make_box_cloud() {
    // Random points. Regular grid can make nearest neighbour pick wrong one.
    lio::PointCloudPtr cloud(new lio::PointCloud);
    std::mt19937 rng(3);
    std::uniform_real_distribution<float> ux(0.0f, 2.0f);
    std::uniform_real_distribution<float> uy(0.0f, 1.5f);
    std::uniform_real_distribution<float> uz(0.0f, 1.0f);
    for (int i = 0; i < 900; ++i) {
        lio::PointT p;
        p.x = ux(rng);
        p.y = uy(rng);
        p.z = uz(rng);
        cloud->push_back(p);
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

}  // namespace

int main() {
    const lio::PointCloudPtr target = make_box_cloud();

    lio::Pose true_pose;
    true_pose.position = Eigen::Vector3d(0.25, -0.18, 0.12);
    true_pose.orientation =
        lio::rotation_to_quaternion(lio::euler_to_rotation(0.08, -0.05, 0.15));

    // source = T^{-1} * target, so ICP should get T back
    const lio::PointCloudPtr source = lio::transform_cloud(target, lio::inverse_pose(true_pose));

    lio::IcpConfig cfg;
    cfg.max_iterations = 40;
    cfg.max_correspondence_distance = 1.5;
    cfg.convergence_threshold = 1e-7;

    const lio::IcpResult result = lio::align_icp(source, target, cfg);
    const lio::Pose est = lio::matrix_to_pose(result.transform);

    const double trans_err = (est.position - true_pose.position).norm();
    const Eigen::Matrix3d R_err =
        lio::quaternion_to_rotation(true_pose.orientation).transpose() *
        lio::quaternion_to_rotation(est.orientation);
    const double rot_err_deg = lio::rotation_angle_deg(R_err);

    std::cout << "ICP translation error: " << trans_err << " m\n";
    std::cout << "ICP rotation error:    " << rot_err_deg << " deg\n";
    std::cout << "mean alignment error:  " << result.mean_error << "\n";
    std::cout << "iterations:            " << result.iterations << "\n";
    std::cout << "converged:             " << (result.converged ? "yes" : "no") << "\n";

    check(result.correspondences > 50, "enough correspondences");
    check(result.converged, "ICP reports convergence");
    check(trans_err < 0.03, "translation recovered");
    check(rot_err_deg < 1.5, "rotation recovered");

    lio::PointCloudPtr empty(new lio::PointCloud);
    const lio::IcpResult empty_result = lio::align_icp(empty, target, cfg);
    check(!empty_result.converged, "empty source does not converge");

    lio::IcpConfig tight = cfg;
    tight.max_iterations = 1;
    tight.convergence_threshold = 1e-12;
    const lio::IcpResult one_iter = lio::align_icp(source, target, tight);
    check(!one_iter.converged, "max iterations is not treated as convergence");

    if (g_failed > 0) {
        return 1;
    }
    std::cout << "test_icp passed\n";
    return 0;
}
