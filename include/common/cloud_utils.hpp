#pragma once

#include "common/types.hpp"

#include <string>

namespace lio {

PointCloudPtr load_pcd(const std::string& path);

void save_pcd(const std::string& path, const PointCloudPtr& cloud);

PointCloudPtr voxel_downsample(const PointCloudPtr& cloud, double leaf_size);

// Keep points with range in [min_range, max_range]
PointCloudPtr filter_range(const PointCloudPtr& cloud, double min_range, double max_range);

}  // namespace lio
