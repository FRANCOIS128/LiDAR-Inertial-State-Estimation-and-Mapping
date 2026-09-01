#include "common/cloud_utils.hpp"
#include "estimation/ekf.hpp"
#include "io/dataset.hpp"
#include "mapping/mapper.hpp"
#include "odometry/lidar_odometry.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

void print_usage() {
    std::cout << "Usage: lio_demo [--output DIR]\n"
              << "Run synthetic lidar-imu demo, write trajectory and map.\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string output_dir = "output";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage();
            return 0;
        }
        if (arg == "--output" && i + 1 < argc) {
            output_dir = argv[++i];
        }
    }

    std::cout << "Generating synthetic LiDAR / IMU sequence...\n";
    lio::Dataset dataset = lio::make_synthetic_dataset();
    if (dataset.lidar_frames.empty() || dataset.imu.empty()) {
        throw std::runtime_error("synthetic dataset is empty");
    }

    fs::create_directories(output_dir);
    fs::create_directories("data/sample");
    lio::write_sample_dataset("data/sample", dataset);

    lio::IcpConfig icp_cfg;
    icp_cfg.max_iterations = 25;
    icp_cfg.max_correspondence_distance = 0.8;
    icp_cfg.convergence_threshold = 1e-5;

    lio::LidarOdometry odom(icp_cfg);
    lio::PoseEkf ekf;
    lio::Mapper mapper(0.12, 4);

    // First pose from ground truth so the map is in the room frame.
    // Initial velocity is zero.
    const lio::TrajectoryEntry& start = dataset.ground_truth.front();
    lio::Pose init_pose;
    init_pose.position = start.position;
    init_pose.orientation = start.orientation;
    odom.set_initial_pose(init_pose);

    lio::NavState init_state;
    init_state.position = start.position;
    init_state.orientation = start.orientation;
    init_state.velocity = Eigen::Vector3d::Zero();
    ekf.set_state(init_state);

    std::vector<lio::TrajectoryEntry> fused_traj;
    fused_traj.reserve(dataset.lidar_frames.size());

    std::size_t imu_idx = 0;
    double last_imu_time = dataset.imu.front().timestamp;
    double total_frame_ms = 0.0;

    const auto wall_t0 = std::chrono::steady_clock::now();

    for (const auto& raw_frame : dataset.lidar_frames) {
        const auto frame_t0 = std::chrono::steady_clock::now();

        lio::LidarFrame frame;
        frame.timestamp = raw_frame.timestamp;
        frame.cloud = lio::filter_range(raw_frame.cloud, 0.5, 12.0);
        frame.cloud = lio::voxel_downsample(frame.cloud, 0.18);

        // IMU is faster than lidar, predict until this scan time.
        while (imu_idx < dataset.imu.size() &&
               dataset.imu[imu_idx].timestamp <= frame.timestamp) {
            const lio::ImuMeasurement& imu = dataset.imu[imu_idx];
            const double dt = imu.timestamp - last_imu_time;
            ekf.predict(imu, dt);
            last_imu_time = imu.timestamp;
            imu_idx++;
        }

        const lio::Pose lidar_pose = odom.process_frame(frame);
        if (odom.last_update_ok()) {
            ekf.update_pose(lidar_pose);
            mapper.add_scan(ekf.pose(), frame.cloud);
        }
        const lio::Pose fused = ekf.pose();

        fused_traj.push_back({frame.timestamp, fused.position, fused.orientation});

        const auto frame_t1 = std::chrono::steady_clock::now();
        total_frame_ms += std::chrono::duration<double, std::milli>(frame_t1 - frame_t0).count();
    }

    mapper.downsample_now();

    const auto wall_t1 = std::chrono::steady_clock::now();
    const double total_s = std::chrono::duration<double>(wall_t1 - wall_t0).count();
    const int n_frames = odom.frames_processed();
    const double mean_frame_ms = n_frames > 0 ? total_frame_ms / n_frames : 0.0;
    const double fps = total_s > 0.0 ? n_frames / total_s : 0.0;

    lio::write_trajectory_csv(output_dir + "/estimated_trajectory.csv", fused_traj);
    lio::write_trajectory_csv(output_dir + "/lidar_trajectory.csv", odom.trajectory());
    lio::write_trajectory_csv(output_dir + "/ground_truth.csv", dataset.ground_truth);
    lio::save_pcd(output_dir + "/map.pcd", mapper.map());

    {
        std::ofstream mapxy(output_dir + "/map_xy.csv");
        mapxy << "x,y,z\n";
        if (mapper.map()) {
            for (const auto& p : mapper.map()->points) {
                mapxy << p.x << ',' << p.y << ',' << p.z << '\n';
            }
        }
    }

    std::ofstream runtime(output_dir + "/runtime.csv");
    runtime << "metric,value\n";
    runtime << std::setprecision(6);
    runtime << "lidar_frames," << n_frames << "\n";
    runtime << "imu_samples," << dataset.imu.size() << "\n";
    runtime << "mean_icp_ms," << odom.mean_icp_ms() << "\n";
    runtime << "mean_frame_ms," << mean_frame_ms << "\n";
    runtime << "fps," << fps << "\n";
    runtime << "icp_failures," << odom.failed_updates() << "\n";
    runtime << "icp_unconverged," << odom.unconverged_updates() << "\n";
    runtime << "map_points," << mapper.point_count() << "\n";
    runtime << "total_seconds," << total_s << "\n";

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\nDone.\n";
    std::cout << "  LiDAR frames     : " << n_frames << "\n";
    std::cout << "  IMU samples      : " << dataset.imu.size() << "\n";
    std::cout << "  mean ICP time    : " << odom.mean_icp_ms() << " ms\n";
    std::cout << "  mean frame time  : " << mean_frame_ms << " ms\n";
    std::cout << "  approximate FPS  : " << fps << "\n";
    std::cout << "  ICP failures     : " << odom.failed_updates() << "\n";
    std::cout << "  ICP unconverged  : " << odom.unconverged_updates() << "\n";
    std::cout << "  map points       : " << mapper.point_count() << "\n";
    std::cout << "  outputs written to " << output_dir << "/\n";
    return 0;
}
