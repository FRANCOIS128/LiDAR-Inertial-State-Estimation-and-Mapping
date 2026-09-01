#include "odometry/lidar_odometry.hpp"

#include "geometry/transform.hpp"

#include <chrono>
#include <iostream>

namespace lio {

LidarOdometry::LidarOdometry(IcpConfig icp_config) : icp_config_(icp_config) {}

void LidarOdometry::set_initial_pose(const Pose& pose) {
    current_pose_ = pose;
}

Pose LidarOdometry::process_frame(const LidarFrame& frame) {
    if (!frame.cloud || frame.cloud->empty()) {
        return current_pose_;
    }

    if (!last_cloud_) {
        last_cloud_ = frame.cloud;
        last_update_ok_ = true;
        trajectory_.push_back({frame.timestamp, current_pose_.position, current_pose_.orientation});
        frames_processed_++;
        return current_pose_;
    }

    const auto t0 = std::chrono::steady_clock::now();
    const IcpResult icp = align_icp(frame.cloud, last_cloud_, icp_config_);
    const auto t1 = std::chrono::steady_clock::now();
    last_icp_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
    total_icp_ms_ += last_icp_ms_;
    icp_count_++;

    last_cloud_ = frame.cloud;

    // If too few pairs, I cannot trust ICP, keep last pose.
    // If not converge but we already have a T, still use it.
    if (icp.correspondences < 3 || icp.iterations == 0) {
        last_update_ok_ = false;
        failed_updates_++;
        std::cerr << "ICP failed at t=" << frame.timestamp
                  << " (correspondences=" << icp.correspondences << ")\n";
        trajectory_.push_back({frame.timestamp, current_pose_.position, current_pose_.orientation});
        frames_processed_++;
        return current_pose_;
    }

    last_update_ok_ = true;
    if (!icp.converged) {
        unconverged_updates_++;
    }
    // T maps current scan into last scan: p_last ≈ T * p_current
    const Pose relative = matrix_to_pose(icp.transform);
    current_pose_ = compose_poses(current_pose_, relative);

    trajectory_.push_back({frame.timestamp, current_pose_.position, current_pose_.orientation});
    frames_processed_++;
    return current_pose_;
}

}  // namespace lio
