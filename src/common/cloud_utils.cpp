#include "common/cloud_utils.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <cmath>
#include <stdexcept>

namespace lio {

PointCloudPtr load_pcd(const std::string& path) {
    PointCloudPtr cloud(new PointCloud);
    if (pcl::io::loadPCDFile(path, *cloud) < 0) {
        throw std::runtime_error("failed to load PCD: " + path);
    }
    return cloud;
}

void save_pcd(const std::string& path, const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) {
        PointCloud empty;
        empty.width = 0;
        empty.height = 1;
        pcl::io::savePCDFileBinary(path, empty);
        return;
    }
    if (pcl::io::savePCDFileBinary(path, *cloud) < 0) {
        throw std::runtime_error("failed to save PCD: " + path);
    }
}

PointCloudPtr voxel_downsample(const PointCloudPtr& cloud, double leaf_size) {
    PointCloudPtr out(new PointCloud);
    if (!cloud || cloud->empty()) {
        return out;
    }
    // Keep one point in each voxel, else ICP is too slow.
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(static_cast<float>(leaf_size),
                      static_cast<float>(leaf_size),
                      static_cast<float>(leaf_size));
    voxel.filter(*out);
    return out;
}

PointCloudPtr filter_range(const PointCloudPtr& cloud, double min_range, double max_range) {
    PointCloudPtr out(new PointCloud);
    if (!cloud) {
        return out;
    }
    out->reserve(cloud->size());
    for (const auto& p : cloud->points) {
        const double r = std::sqrt(static_cast<double>(p.x) * p.x +
                                   static_cast<double>(p.y) * p.y +
                                   static_cast<double>(p.z) * p.z);
        if (r >= min_range && r <= max_range) {
            out->push_back(p);
        }
    }
    out->width = static_cast<uint32_t>(out->size());
    out->height = 1;
    out->is_dense = true;
    return out;
}

}  // namespace lio
