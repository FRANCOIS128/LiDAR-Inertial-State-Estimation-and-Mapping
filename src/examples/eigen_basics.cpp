#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::Vector3d p(1.0, 2.0, 3.0);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t(0.5, -1.0, 2.0);
    Eigen::Vector3d p_world = R * p + t;  // check Eigen works

    std::cout << "Original point p:\n" << p.transpose() << "\n\n";
    std::cout << "Rotation R (identity):\n" << R << "\n\n";
    std::cout << "Translation t:\n" << t.transpose() << "\n\n";
    std::cout << "Transformed point p_world = R * p + t:\n"
              << p_world.transpose() << "\n";
    return 0;
}
