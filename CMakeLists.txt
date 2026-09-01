cmake_minimum_required(VERSION 3.16)

project(lidar_inertial_odometry LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(APPLE)
    execute_process(
        COMMAND brew --prefix
        OUTPUT_VARIABLE HOMEBREW_PREFIX
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    if(HOMEBREW_PREFIX)
        list(APPEND CMAKE_PREFIX_PATH "${HOMEBREW_PREFIX}")
    endif()
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree filters registration)

add_library(lio_core
    src/geometry/transform.cpp
    src/common/cloud_utils.cpp
    src/registration/icp.cpp
    src/odometry/lidar_odometry.cpp
    src/imu/imu_propagator.cpp
    src/estimation/ekf.cpp
    src/mapping/mapper.cpp
    src/io/dataset.cpp)

target_include_directories(lio_core PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS})

target_link_libraries(lio_core PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES})

target_compile_definitions(lio_core PUBLIC ${PCL_DEFINITIONS})

add_executable(lio_demo src/main.cpp)
target_link_libraries(lio_demo PRIVATE lio_core)

add_executable(eigen_basics src/examples/eigen_basics.cpp)
target_link_libraries(eigen_basics PRIVATE Eigen3::Eigen)
target_include_directories(eigen_basics PRIVATE ${PROJECT_SOURCE_DIR}/include)

enable_testing()

foreach(test_name test_geometry test_icp test_imu test_ekf)
    add_executable(${test_name} tests/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE lio_core)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
