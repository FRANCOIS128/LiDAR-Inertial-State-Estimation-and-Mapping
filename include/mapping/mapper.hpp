#pragma once

#include "common/types.hpp"

namespace lio {

// Put transformed scans into one cloud, then voxel.
class Mapper {
public:
    explicit Mapper(double voxel_leaf = 0.15, int downsample_every = 5);

    void add_scan(const Pose& world_pose, const PointCloudPtr& cloud);
    void downsample_now();

    PointCloudPtr map() const { return map_; }
    std::size_t point_count() const { return map_ ? map_->size() : 0; }

private:
    PointCloudPtr map_;
    double voxel_leaf_;
    int downsample_every_;
    int scans_added_ = 0;
};

}  // namespace lio
