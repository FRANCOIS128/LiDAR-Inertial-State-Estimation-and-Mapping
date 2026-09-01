#pragma once

#include "common/types.hpp"

#include <Eigen/Dense>

namespace lio {

struct IcpConfig {
    int max_iterations = 30;
    double max_correspondence_distance = 1.0;  // if NN farther than this (m), drop
    double convergence_threshold = 1e-4;
    double min_overlap_ratio = 0.2;            // too few pairs, ICP is bad
};

struct IcpResult {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    double mean_error = 0.0;
    int iterations = 0;
    int correspondences = 0;
    bool converged = false;
};

// Point-to-point ICP used in odometry.
// T is 4x4, p_target ≈ T * p_source
IcpResult align_icp(const PointCloudPtr& source,
                    const PointCloudPtr& target,
                    const IcpConfig& config = {},
                    const Eigen::Matrix4d& init_guess = Eigen::Matrix4d::Identity());

// PCL ICP, only for compare. Demo does not use this.
IcpResult align_icp_pcl(const PointCloudPtr& source,
                        const PointCloudPtr& target,
                        const IcpConfig& config = {},
                        const Eigen::Matrix4d& init_guess = Eigen::Matrix4d::Identity());

}  // namespace lio
