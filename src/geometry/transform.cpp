#include "geometry/transform.hpp"

#include <cmath>

namespace lio {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
         -v.y(), v.x(), 0.0;
    return S;
}

Eigen::Matrix3d exp_so3(const Eigen::Vector3d& omega) {
    const double theta = omega.norm();
    if (theta < 1e-12) {
        // small angle: Exp(w) ≈ I + [w]x
        return Eigen::Matrix3d::Identity() + skew(omega);
    }
    return Eigen::AngleAxisd(theta, omega / theta).toRotationMatrix();
}

Eigen::Vector3d log_so3(const Eigen::Matrix3d& R) {
    Eigen::AngleAxisd aa(R);
    return aa.angle() * aa.axis();
}

Eigen::Matrix3d euler_to_rotation(double roll, double pitch, double yaw) {
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

Eigen::Matrix3d quaternion_to_rotation(const Eigen::Quaterniond& q) {
    return q.normalized().toRotationMatrix();
}

Eigen::Quaterniond rotation_to_quaternion(const Eigen::Matrix3d& R) {
    return Eigen::Quaterniond(R).normalized();
}

Eigen::Matrix4d pose_to_matrix(const Pose& pose) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = quaternion_to_rotation(pose.orientation);
    T.block<3, 1>(0, 3) = pose.position;
    return T;
}

Pose matrix_to_pose(const Eigen::Matrix4d& T) {
    Pose pose;
    pose.position = T.block<3, 1>(0, 3);
    pose.orientation = rotation_to_quaternion(T.block<3, 3>(0, 0));
    return pose;
}

Eigen::Vector3d transform_point(const Pose& pose, const Eigen::Vector3d& p_local) {
    // p_world = R * p_local + t
    return quaternion_to_rotation(pose.orientation) * p_local + pose.position;
}

PointCloudPtr transform_cloud(const PointCloudPtr& cloud, const Pose& pose) {
    PointCloudPtr out(new PointCloud);
    if (!cloud) {
        return out;
    }
    out->reserve(cloud->size());
    const Eigen::Matrix3d R = quaternion_to_rotation(pose.orientation);
    const Eigen::Vector3d& t = pose.position;
    for (const auto& pt : cloud->points) {
        const Eigen::Vector3d p(pt.x, pt.y, pt.z);
        const Eigen::Vector3d pw = R * p + t;
        PointT q;
        q.x = static_cast<float>(pw.x());
        q.y = static_cast<float>(pw.y());
        q.z = static_cast<float>(pw.z());
        out->push_back(q);
    }
    out->width = static_cast<uint32_t>(out->size());
    out->height = 1;
    out->is_dense = true;
    return out;
}

Pose compose_poses(const Pose& a, const Pose& b) {
    // T_ac = T_ab * T_bc
    Pose out;
    const Eigen::Matrix3d Ra = quaternion_to_rotation(a.orientation);
    out.orientation = (a.orientation * b.orientation).normalized();
    out.position = a.position + Ra * b.position;
    return out;
}

Pose inverse_pose(const Pose& pose) {
    // R^{-1} = R^T, t^{-1} = -R^T t
    Pose inv;
    inv.orientation = pose.orientation.conjugate().normalized();
    const Eigen::Matrix3d R_inv = quaternion_to_rotation(inv.orientation);
    inv.position = -R_inv * pose.position;
    return inv;
}

double rotation_angle_deg(const Eigen::Matrix3d& R_err) {
    return log_so3(R_err).norm() * 180.0 / M_PI;
}

}  // namespace lio
