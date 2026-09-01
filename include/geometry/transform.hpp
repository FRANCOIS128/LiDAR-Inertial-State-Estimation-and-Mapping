#pragma once

#include "common/types.hpp"

#include <Eigen/Dense>

namespace lio {

// [v]x matrix, for SO(3) and EKF
Eigen::Matrix3d skew(const Eigen::Vector3d& v);

// rotation vector to rotation matrix, R = Exp(omega)
Eigen::Matrix3d exp_so3(const Eigen::Vector3d& omega);

// inverse of exp_so3
Eigen::Vector3d log_so3(const Eigen::Matrix3d& R);

// first yaw (z), then pitch (y), then roll (x)
Eigen::Matrix3d euler_to_rotation(double roll, double pitch, double yaw);

Eigen::Matrix3d quaternion_to_rotation(const Eigen::Quaterniond& q);

Eigen::Quaterniond rotation_to_quaternion(const Eigen::Matrix3d& R);

Eigen::Matrix4d pose_to_matrix(const Pose& pose);

Pose matrix_to_pose(const Eigen::Matrix4d& T);

Eigen::Vector3d transform_point(const Pose& pose, const Eigen::Vector3d& p_local);

PointCloudPtr transform_cloud(const PointCloudPtr& cloud, const Pose& pose);

Pose compose_poses(const Pose& a, const Pose& b);

Pose inverse_pose(const Pose& pose);

double rotation_angle_deg(const Eigen::Matrix3d& R_err);

}  // namespace lio
