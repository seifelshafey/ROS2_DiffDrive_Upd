# slam_cpp

Real-time FastSLAM 2.0 with GMapping-style weighting for ROS 2 Humble.

A C++ particle filter SLAM package that subscribes to live odometry and laser scan topics, builds 2D occupancy grid maps in real-time, and publishes the map, particle cloud, and TF corrections. Ported from a Python proof-of-concept and hardened for production use on a differential-drive robot in Ignition Gazebo.

https://github.com/seifelshafey/ROS2_DiffDrive_Upd/raw/master/src/slam_cpp/media/demo.mp4

---

## How It Works

The core algorithm is a Rao-Blackwellized Particle Filter (RBPF): 80 particles each carry an independent pose estimate and occupancy grid. Every SLAM cycle runs three stages:

**Predict** -- Accumulated odometry is decomposed into rotation-translation-rotation and applied to each particle with sampled noise (Probabilistic Robotics, Table 5.6). Covariance is propagated analytically through EKF-style Jacobians.

**Scan Match** -- Each particle aligns the current laser scan against its own map using Levenberg-Marquardt optimization. The inverse Hessian at the optimum defines the proposal covariance -- a Laplace approximation of the local posterior. A coarse grid search seeds the optimizer. Bilinear interpolation with analytic gradients and a precomputed sigmoid LUT keep the inner loop fast.

**Weight & Resample** -- Importance weights use only the GMapping-style gain-scaled scan likelihood. The odometry prior lives inside the LM cost function (no double-counting), and the proposal density ratio is approximately constant across particles when the scan-matched proposal is close to the posterior. Systematic resampling fires only when N_eff drops below N/2, which with GMapping's flat weights happens infrequently -- preserving particle diversity.

---

## Package Structure

```
slam_cpp/
  include/slam_cpp/
    slam_types.hpp          Shared types, config, normalize_angle, result structs
    sigmoid_lut.hpp         256-entry sigmoid LUT for int8 log-odds grids
    occupancy_grid.hpp      Occupancy grid with copy-on-write sharing
    scan_matching.hpp       Coarse search, LM optimizer, GMapping weighting
    particle_filter.hpp     Predict, update_and_resample, systematic resample
  src/
    occupancy_grid.cpp      Bresenham raycasting, log-odds cell updates
    scan_matching.cpp       Bilinear interp, Jacobian, LM loop, weight calc
    particle_filter.cpp     Motion model, Cholesky sampling, adaptive resampling
    slam_node.cpp           ROS 2 node wiring: subs, pubs, TF, timers
  config/
    slam_params.yaml        All tunable parameters
  launch/
    slam_cpp.launch.py      Launch with parameter file
  test/
    test_occupancy_grid.cpp
    test_scan_matching.cpp
    test_particle_filter.cpp
```

## Topics and Transforms

| Topic | Type | Description |
|-------|------|-------------|
| `/odometry/filtered` | `nav_msgs/Odometry` | Input: EKF-fused wheel + IMU odometry |
| `/scan` | `sensor_msgs/LaserScan` | Input: 2D lidar |
| `/slam/map` | `nav_msgs/OccupancyGrid` | Output: best particle's grid (2 Hz) |
| `/slam/particles` | `geometry_msgs/PoseArray` | Output: particle cloud (2 Hz) |
| `/slam/best_pose` | `geometry_msgs/PoseStamped` | Output: highest-weight pose (2 Hz) |
| `map -> odom` | TF | Output: SLAM correction (2 Hz) |

The EKF must publish the `odom -> base_link` TF (`publish_tf: true` in ekf.yaml) so the full chain is `map -> odom -> base_link`.

---

## Design Decisions

**Copy-on-write grids.** Resampling duplicates particles cheaply via shared_ptr refcount. The 250KB deep copy only happens when a particle's grid is actually mutated.

**LM instead of hill-climbing.** GMapping's original scan matcher uses discrete grid search with a finite-difference Hessian. Our LM optimizer produces the exact Gauss-Newton Hessian as a natural byproduct, giving a better-conditioned proposal covariance. Already implemented, so no reason to downgrade.

**Scan-only weights.** Applying GMapping's gain to the scan term while keeping the FastSLAM 2.0 odom-prior and proposal-density terms creates a 10x scale imbalance (the covariance determinant dominates). Dropping those terms matches GMapping's actual implementation and produces healthy weight ratios of ~1.1:1.

**Tight fallback covariance.** When LM fails (sparse maps, featureless areas), the proposal covariance is set to 1e-8*I instead of the odometry covariance. This prevents double-noising -- predict() already added motion model noise, so sampling again from odom_cov would double the scatter and cause irrecoverable drift during bootstrap.

**Adaptive resampling.** With GMapping's flat weights, N_eff stays high and resampling triggers rarely. This is by design: less resampling means less particle depletion, which is the primary failure mode of RBPF.

---

## Build & Run

```bash
# Build (inside the ros2_humble_workspace container)
source /opt/ros/humble/setup.bash && cd /root/workspace
colcon build --packages-select slam_cpp
source install/setup.bash

# Terminal 1: Simulation
ros2 launch teleop_robot sim_ign.launch.py

# Terminal 2: SLAM
ros2 launch slam_cpp slam_cpp.launch.py

# Terminal 3: Teleop
ros2 run teleop_robot teleop
```

In RViz, set Fixed Frame to `map`. Add `/slam/map` (OccupancyGrid), `/slam/best_pose` (PoseStamped), and `/slam/particles` (PoseArray).

## Tests

```bash
colcon test --packages-select slam_cpp
colcon test-result --verbose
# 24 unit tests: grid ops, scan matching convergence, particle filter mechanics
```

---

## Key Parameters

All configurable in `config/slam_params.yaml`:

| Parameter | Default | What it does |
|-----------|---------|--------------|
| `num_particles` | 80 | Particle count. More = better accuracy, more memory. |
| `map_size` | 50.0 | Square map side length in meters |
| `map_resolution` | 0.1 | Meters per grid cell (500x500 grid) |
| `alpha1`--`alpha4` | 0.075, 0.03, 0.075, 0.03 | Motion model noise coefficients |
| `obs_sigma_gain` | 3.0 | GMapping weight flattening. Higher = flatter = safer. |
| `sigma_hit` | 0.05 | Scan matching measurement noise |
| `lm_max_iters` | 10 | LM optimizer iteration budget |
| `motion_threshold_trans` | 0.05 | Min motion to trigger a SLAM update (m) |

---

## Known Limitations

### The Double-Wall Problem

When the robot loops back to a previously mapped area, accumulated heading error places scan endpoints at a slightly different grid position than the original pass. This creates parallel "ghost walls" that Bresenham raycasting can't erase (the old cells sit behind the new surface). No amount of parameter tuning fixes this -- it's structural to RBPF SLAM.

This is the exact limitation that drove the robotics community from particle-filter SLAM (GMapping, FastSLAM) to graph-based SLAM (SLAM Toolbox, Google Cartographer). Graph-based methods store the full trajectory and can retroactively correct all past poses when a loop closure is detected. Particle filters can't look backward.

### Other Limitations

- No explicit loop closure -- scan matching only corrects locally
- 80 particles x 250KB = 20MB grid memory (COW reduces copy cost, not storage)
- Static world assumption -- moving objects leave ghost traces
- 50m map at 0.1m resolution; larger environments need coarser cells or dynamic allocation

---

## What I Learned

This project was built as a deep-dive study into probabilistic robotics. The journey from a 1,000-line Python script with Numba JIT to a real-time C++ ROS 2 node touched: importance sampling theory, Levenberg-Marquardt optimization, Bresenham raycasting, copy-on-write memory management, ROS 2 TF tree mechanics, and the fundamental tradeoffs between particle-filter and graph-based SLAM. The double-wall artifact isn't a bug -- it's the reason the field evolved.
