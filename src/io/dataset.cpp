#include "io/dataset.hpp"

#include "common/cloud_utils.hpp"
#include "geometry/transform.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>

namespace lio {
namespace {

constexpr double kGravity = 9.81;

void add_point(PointCloud& cloud, double x, double y, double z) {
    PointT p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    p.z = static_cast<float>(z);
    cloud.push_back(p);
}

void add_plane(PointCloud& cloud,
               const Eigen::Vector3d& origin,
               const Eigen::Vector3d& u,
               const Eigen::Vector3d& v,
               int nu,
               int nv) {
    for (int i = 0; i < nu; ++i) {
        for (int j = 0; j < nv; ++j) {
            const double su = (nu == 1) ? 0.0 : static_cast<double>(i) / (nu - 1);
            const double sv = (nv == 1) ? 0.0 : static_cast<double>(j) / (nv - 1);
            const Eigen::Vector3d p = origin + su * u + sv * v;
            add_point(cloud, p.x(), p.y(), p.z());
        }
    }
}

PointCloudPtr make_room_map() {
    PointCloudPtr cloud(new PointCloud);

    // A room + some boxes, so ICP can find corners.
    const double L = 12.0;
    const double W = 8.0;
    const double H = 3.0;
    const int nL = 80;
    const int nW = 55;
    const int nH = 22;

    add_plane(*cloud, {0, 0, 0}, {L, 0, 0}, {0, W, 0}, nL, nW);          // floor
    add_plane(*cloud, {0, 0, H}, {L, 0, 0}, {0, W, 0}, nL, nW);          // ceiling
    add_plane(*cloud, {0, 0, 0}, {L, 0, 0}, {0, 0, H}, nL, nH);          // y=0 wall
    add_plane(*cloud, {0, W, 0}, {L, 0, 0}, {0, 0, H}, nL, nH);          // y=W wall
    add_plane(*cloud, {0, 0, 0}, {0, W, 0}, {0, 0, H}, nW, nH);          // x=0 wall
    add_plane(*cloud, {L, 0, 0}, {0, W, 0}, {0, 0, H}, nW, nH);          // x=L wall

    add_plane(*cloud, {3.0, 3.0, 0.0}, {1.2, 0, 0}, {0, 1.2, 0}, 10, 10);
    add_plane(*cloud, {3.0, 3.0, 0.0}, {1.2, 0, 0}, {0, 0, 1.5}, 10, 12);
    add_plane(*cloud, {8.5, 5.5, 0.0}, {0.8, 0, 0}, {0, 0, 1.8}, 8, 14);

    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

PointCloudPtr render_scan(const PointCloudPtr& world,
                          const Pose& lidar_pose,
                          std::mt19937& rng,
                          double noise_std) {
    std::normal_distribution<double> noise(0.0, noise_std);
    const Pose inv = inverse_pose(lidar_pose);
    PointCloudPtr scan(new PointCloud);
    scan->reserve(world->size() / 3);

    for (const auto& pt : world->points) {
        const Eigen::Vector3d p_world(pt.x, pt.y, pt.z);
        Eigen::Vector3d p_body = transform_point(inv, p_world);  // world to lidar
        const double r = p_body.norm();
        if (r < 0.5 || r > 14.0) {
            continue;
        }
        // Simple 360 lidar, cut a bit of vertical FOV.
        const double elev = std::asin(p_body.z() / r);
        if (std::abs(elev) > 0.7) {
            continue;
        }
        p_body.x() += noise(rng);
        p_body.y() += noise(rng);
        p_body.z() += noise(rng);
        add_point(*scan, p_body.x(), p_body.y(), p_body.z());
    }
    scan->width = static_cast<uint32_t>(scan->size());
    scan->height = 1;
    scan->is_dense = true;
    return scan;
}

}  // namespace

Dataset make_synthetic_dataset() {
    Dataset data;
    const PointCloudPtr world = make_room_map();

    const double duration = 8.0;
    const double imu_dt = 0.01;
    const double lidar_dt = 0.1;
    const double radius = 2.0;
    const double omega = 0.35;
    const Eigen::Vector3d center(6.0, 4.0, 1.2);
    const Eigen::Vector3d gravity(0.0, 0.0, -kGravity);

    std::mt19937 rng(7);
    std::normal_distribution<double> acc_noise(0.0, 0.02);
    std::normal_distribution<double> gyro_noise(0.0, 0.001);

    auto pose_at = [&](double t) {
        const double wt = omega * t;
        Pose pose;
        pose.position = center + Eigen::Vector3d(radius * std::cos(wt), radius * std::sin(wt), 0.0);
        const Eigen::Vector3d vel(-radius * omega * std::sin(wt),
                                  radius * omega * std::cos(wt),
                                  0.0);
        const double yaw = std::atan2(vel.y(), vel.x());
        pose.orientation = rotation_to_quaternion(euler_to_rotation(0.0, 0.0, yaw));
        return pose;
    };

    auto acc_at = [&](double t) {
        const double wt = omega * t;
        return Eigen::Vector3d(-radius * omega * omega * std::cos(wt),
                               -radius * omega * omega * std::sin(wt),
                               0.0);
    };

    for (double t = 0.0; t <= duration + 1e-9; t += imu_dt) {
        const Pose pose = pose_at(t);
        const Eigen::Matrix3d R = quaternion_to_rotation(pose.orientation);
        const Eigen::Vector3d acc_world = acc_at(t);
        const Eigen::Vector3d gyro_world(0.0, 0.0, omega);

        ImuMeasurement imu;
        imu.timestamp = t;
        // a_meas = R^T (a_world - g), same as ImuPropagator
        imu.acceleration = R.transpose() * (acc_world - gravity);
        imu.angular_velocity = R.transpose() * gyro_world;
        imu.acceleration.x() += acc_noise(rng);
        imu.acceleration.y() += acc_noise(rng);
        imu.acceleration.z() += acc_noise(rng);
        imu.angular_velocity.x() += gyro_noise(rng);
        imu.angular_velocity.y() += gyro_noise(rng);
        imu.angular_velocity.z() += gyro_noise(rng);
        data.imu.push_back(imu);
    }

    for (double t = 0.0; t <= duration + 1e-9; t += lidar_dt) {
        const Pose pose = pose_at(t);
        LidarFrame frame;
        frame.timestamp = t;
        frame.cloud = render_scan(world, pose, rng, 0.01);
        data.lidar_frames.push_back(frame);

        TrajectoryEntry gt;
        gt.timestamp = t;
        gt.position = pose.position;
        gt.orientation = pose.orientation;
        data.ground_truth.push_back(gt);
    }

    return data;
}

void write_trajectory_csv(const std::string& path, const std::vector<TrajectoryEntry>& traj) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("cannot write " + path);
    }
    out << "timestamp,x,y,z,qx,qy,qz,qw\n";
    out << std::setprecision(12);
    for (const auto& e : traj) {
        out << e.timestamp << ',' << e.position.x() << ',' << e.position.y() << ','
            << e.position.z() << ',' << e.orientation.x() << ',' << e.orientation.y() << ','
            << e.orientation.z() << ',' << e.orientation.w() << '\n';
    }
}

std::vector<TrajectoryEntry> load_trajectory_csv(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot read " + path);
    }
    std::vector<TrajectoryEntry> traj;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        std::stringstream ss(line);
        std::string cell;
        TrajectoryEntry e;
        std::getline(ss, cell, ',');
        e.timestamp = std::stod(cell);
        std::getline(ss, cell, ',');
        e.position.x() = std::stod(cell);
        std::getline(ss, cell, ',');
        e.position.y() = std::stod(cell);
        std::getline(ss, cell, ',');
        e.position.z() = std::stod(cell);
        std::getline(ss, cell, ',');
        e.orientation.x() = std::stod(cell);
        std::getline(ss, cell, ',');
        e.orientation.y() = std::stod(cell);
        std::getline(ss, cell, ',');
        e.orientation.z() = std::stod(cell);
        std::getline(ss, cell, ',');
        e.orientation.w() = std::stod(cell);
        e.orientation.normalize();
        traj.push_back(e);
    }
    return traj;
}

void write_imu_csv(const std::string& path, const std::vector<ImuMeasurement>& imu) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("cannot write " + path);
    }
    out << "timestamp,ax,ay,az,gx,gy,gz\n";
    out << std::setprecision(12);
    for (const auto& m : imu) {
        out << m.timestamp << ',' << m.acceleration.x() << ',' << m.acceleration.y() << ','
            << m.acceleration.z() << ',' << m.angular_velocity.x() << ','
            << m.angular_velocity.y() << ',' << m.angular_velocity.z() << '\n';
    }
}

void write_sample_dataset(const std::string& directory, const Dataset& data) {
    namespace fs = std::filesystem;
    fs::create_directories(directory);
    write_imu_csv(directory + "/imu.csv", data.imu);
    write_trajectory_csv(directory + "/ground_truth.csv", data.ground_truth);

    const std::size_t n = std::min<std::size_t>(3, data.lidar_frames.size());
    for (std::size_t i = 0; i < n; ++i) {
        const auto& frame = data.lidar_frames[i];
        std::ostringstream name;
        name << directory << "/scan_" << std::setw(3) << std::setfill('0') << i << ".pcd";
        save_pcd(name.str(), frame.cloud);
    }

    std::ofstream readme(directory + "/README.txt");
    readme << "Synthetic sample snippet generated by lio_demo.\n"
           << "imu.csv columns: timestamp,ax,ay,az,gx,gy,gz\n"
           << "ground_truth.csv columns: timestamp,x,y,z,qx,qy,qz,qw\n"
           << "scan_XXX.pcd: a few LiDAR frames in the sensor frame.\n"
           << "The full demo runs from in-memory synthetic data; these files\n"
           << "only show the expected on-disk layout.\n";
}

}  // namespace lio
