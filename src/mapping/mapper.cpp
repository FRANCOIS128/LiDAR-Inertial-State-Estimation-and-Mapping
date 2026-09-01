#include "mapping/mapper.hpp"

#include "common/cloud_utils.hpp"
#include "geometry/transform.hpp"

namespace lio {

Mapper::Mapper(double voxel_leaf, int downsample_every)
    : map_(new PointCloud),
      voxel_leaf_(voxel_leaf),
      downsample_every_(downsample_every) {}

void Mapper::add_scan(const Pose& world_pose, const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) {
        return;
    }

    const PointCloudPtr world_cloud = transform_cloud(cloud, world_pose);
    *map_ += *world_cloud;
    scans_added_++;

    // Voxel every few scans, else map is too big.
    if (downsample_every_ > 0 && scans_added_ % downsample_every_ == 0) {
        downsample_now();
    }
}

void Mapper::downsample_now() {
    map_ = voxel_downsample(map_, voxel_leaf_);
}

}  // namespace lio
