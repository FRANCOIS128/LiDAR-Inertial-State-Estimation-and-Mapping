#pragma once

#include "common/types.hpp"
#include "registration/icp.hpp"

#include <vector>

namespace lio {

// Only scan-to-scan, no loop closure.
class LidarOdometry {
public:
    explicit LidarOdometry(IcpConfig icp_config = {});

    void set_initial_pose(const Pose& pose);
    // Match this scan to last scan, then chain the relative pose.
    Pose process_frame(const LidarFrame& frame);

    const Pose& pose() const { return current_pose_; }
    const std::vector<TrajectoryEntry>& trajectory() const { return trajectory_; }

    int frames_processed() const { return frames_processed_; }
    double last_icp_ms() const { return last_icp_ms_; }
    double mean_icp_ms() const {
        return icp_count_ == 0 ? 0.0 : total_icp_ms_ / static_cast<double>(icp_count_);
    }
    bool last_update_ok() const { return last_update_ok_; }
    int failed_updates() const { return failed_updates_; }
    int unconverged_updates() const { return unconverged_updates_; }

private:
    IcpConfig icp_config_;
    Pose current_pose_;
    PointCloudPtr last_cloud_;
    std::vector<TrajectoryEntry> trajectory_;
    int frames_processed_ = 0;
    double last_icp_ms_ = 0.0;
    double total_icp_ms_ = 0.0;
    int icp_count_ = 0;
    bool last_update_ok_ = true;
    int failed_updates_ = 0;
    int unconverged_updates_ = 0;
};

}  // namespace lio
