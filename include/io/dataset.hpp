#pragma once

#include "common/types.hpp"

#include <string>
#include <vector>

namespace lio {

struct Dataset {
    std::vector<LidarFrame> lidar_frames;
    std::vector<ImuMeasurement> imu;
    std::vector<TrajectoryEntry> ground_truth;  // same time as lidar
};

// Fake room + circle path, so we can run without a big dataset.
Dataset make_synthetic_dataset();

void write_trajectory_csv(const std::string& path, const std::vector<TrajectoryEntry>& traj);
std::vector<TrajectoryEntry> load_trajectory_csv(const std::string& path);
void write_imu_csv(const std::string& path, const std::vector<ImuMeasurement>& imu);

void write_sample_dataset(const std::string& directory, const Dataset& data);

}  // namespace lio
