#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lio {

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;

// position in meters. quaternion is unit, Eigen stores (w,x,y,z)
struct Pose {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct ImuMeasurement {
    double timestamp = 0.0;
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();      // body, m/s^2
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();  // body, rad/s
};

struct LidarFrame {
    double timestamp = 0.0;
    PointCloudPtr cloud;  // points in lidar frame
};

struct TrajectoryEntry {
    double timestamp = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

// IMU navigation state, including accelerometer and gyro bias.
struct NavState {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
};

}  // namespace lio
