# LiDAR-Inertial State Estimation and Mapping

This is our group project. We write a C++ program for lidar-inertial odometry and mapping.

The steps are: downsample lidar points, match two scans with point-to-point ICP, integrate IMU, fuse with EKF, then put points into a map.

## Method

Rigid transform:

```
p' = R p + t
```

ICP:

```
T* = argmin_T  sum || T p_i - q_i ||^2
```

Nearest point is found by KD-tree. Then SVD gives R and t.

IMU:

```
R = R * Exp((w - bg) * dt)
a = R * (acc - ba) + g
v = v + a * dt
p = p + v * dt
```

EKF uses IMU to predict, and lidar pose to update.

## Result

The demo is a circle path in a room.

ATE RMSE: 0.014 m

![Trajectory](results/trajectory.png)

![Map](results/map.png)

## Build and run

Need cmake, Eigen and PCL.

```bash
cmake -S . -B build
cmake --build build -j
./build/lio_demo --output output
```

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python scripts/plot_trajectory.py
python scripts/evaluate_trajectory.py
```

```bash
cd build && ctest
```

## Files

```
include/   headers
src/       cpp code
tests/
scripts/   plot and error
data/sample/
```

`imu.csv`: `timestamp,ax,ay,az,gx,gy,gz`

`ground_truth.csv`: `timestamp,x,y,z,qx,qy,qz,qw`

LiDAR: one pcd for one scan.
