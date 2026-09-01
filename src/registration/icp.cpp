#include "registration/icp.hpp"

#include "geometry/transform.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include <cmath>
#include <vector>

namespace lio {
namespace {

struct Correspondence {
    Eigen::Vector3d src;
    Eigen::Vector3d tgt;
};

Eigen::Matrix4d estimate_rigid_svd(const std::vector<Correspondence>& pairs) {
    // Kabsch: find R, t to map src points to tgt points
    Eigen::Vector3d src_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d tgt_mean = Eigen::Vector3d::Zero();
    for (const auto& c : pairs) {
        src_mean += c.src;
        tgt_mean += c.tgt;
    }
    src_mean /= static_cast<double>(pairs.size());
    tgt_mean /= static_cast<double>(pairs.size());

    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (const auto& c : pairs) {
        H += (c.src - src_mean) * (c.tgt - tgt_mean).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R = V * U.transpose();
    if (R.determinant() < 0.0) {
        // SVD may give a reflection. For rotation, det(R) should be +1.
        V.col(2) *= -1.0;
        R = V * U.transpose();
    }

    const Eigen::Vector3d t = tgt_mean - R * src_mean;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}

}  // namespace

IcpResult align_icp(const PointCloudPtr& source,
                    const PointCloudPtr& target,
                    const IcpConfig& config,
                    const Eigen::Matrix4d& init_guess) {
    IcpResult result;
    result.transform = init_guess;

    if (!source || !target || source->empty() || target->empty()) {
        return result;
    }

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(target);

    // PCL nearestKSearch gives squared distance.
    const double max_dist2 = config.max_correspondence_distance * config.max_correspondence_distance;
    Eigen::Matrix4d T = init_guess;

    for (int iter = 0; iter < config.max_iterations; ++iter) {
        std::vector<Correspondence> pairs;
        pairs.reserve(source->size());

        Pose pose = matrix_to_pose(T);
        for (const auto& pt : source->points) {
            const Eigen::Vector3d p_src(pt.x, pt.y, pt.z);
            const Eigen::Vector3d p_tf = transform_point(pose, p_src);

            PointT query;
            query.x = static_cast<float>(p_tf.x());
            query.y = static_cast<float>(p_tf.y());
            query.z = static_cast<float>(p_tf.z());

            std::vector<int> nn_idx(1);
            std::vector<float> nn_dist2(1);
            if (kdtree.nearestKSearch(query, 1, nn_idx, nn_dist2) == 0) {
                continue;
            }
            if (nn_dist2[0] > max_dist2) {
                continue;  // outlier, skip
            }

            const auto& q = target->points[nn_idx[0]];
            // pair is (already transformed src, target). Then T = dT * T.
            pairs.push_back({p_tf, Eigen::Vector3d(q.x, q.y, q.z)});
        }

        result.correspondences = static_cast<int>(pairs.size());
        result.iterations = iter + 1;

        const double overlap = static_cast<double>(pairs.size()) / static_cast<double>(source->size());
        if (pairs.size() < 3 || overlap < config.min_overlap_ratio) {
            result.converged = false;
            break;
        }

        const Eigen::Matrix4d dT = estimate_rigid_svd(pairs);
        T = dT * T;  // update current guess

        const double trans_delta = dT.block<3, 1>(0, 3).norm();
        const double rot_delta = log_so3(dT.block<3, 3>(0, 0)).norm();

        double err = 0.0;
        Pose dpose = matrix_to_pose(dT);
        for (const auto& c : pairs) {
            err += (transform_point(dpose, c.src) - c.tgt).squaredNorm();
        }
        result.mean_error = std::sqrt(err / static_cast<double>(pairs.size()));
        result.transform = T;

        if (trans_delta < config.convergence_threshold && rot_delta < config.convergence_threshold) {
            result.converged = true;
            break;
        }
        // Max iteration is not converge.
    }

    return result;
}

IcpResult align_icp_pcl(const PointCloudPtr& source,
                        const PointCloudPtr& target,
                        const IcpConfig& config,
                        const Eigen::Matrix4d& init_guess) {
    // PCL IterativeClosestPoint, demo does not use it.
    IcpResult result;
    result.transform = init_guess;
    if (!source || !target || source->empty() || target->empty()) {
        return result;
    }

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(config.max_iterations);
    icp.setMaxCorrespondenceDistance(config.max_correspondence_distance);
    icp.setTransformationEpsilon(config.convergence_threshold);

    PointCloud aligned;
    icp.align(aligned, init_guess.cast<float>());

    result.converged = icp.hasConverged();
    result.transform = icp.getFinalTransformation().cast<double>();
    result.mean_error = icp.getFitnessScore();
    result.iterations = config.max_iterations;
    return result;
}

}  // namespace lio
