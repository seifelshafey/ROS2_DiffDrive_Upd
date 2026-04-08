#ifndef SLAM_CPP__SCAN_MATCHING_HPP_
#define SLAM_CPP__SCAN_MATCHING_HPP_

// scan_matching.hpp — Scan matching pipeline for FastSLAM 2.0.
// Coarse grid search, Levenberg-Marquardt refinement, GMapping-style weighting.

#include <Eigen/Dense>

#include "slam_cpp/slam_types.hpp"
#include "slam_cpp/occupancy_grid.hpp"
#include "slam_cpp/sigmoid_lut.hpp"

namespace slam {

// Construct a GridView from an OccupancyGrid and its SigmoidLUT.
GridView make_grid_view(const OccupancyGrid& grid, const SlamConfig& config,
                        const SigmoidLUT& sigmoid);

// Convert raw lidar ranges to a local Cartesian scan matrix (stride-sampled).
Eigen::Matrix<double, Eigen::Dynamic, 2> extract_local_scan(
    const float* ranges, int num_ranges,
    float angle_min, float angle_max, float range_max,
    int stride = 2);

// Extract every stride-th row from a scan matrix.
Eigen::Matrix<double, Eigen::Dynamic, 2> subsample_scan(
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& scans, int stride);

// Convert metric scan points to grid pixel coordinates.
Eigen::Matrix<double, Eigen::Dynamic, 2> to_grid_coords(
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& points_meters,
    double inv_res, double origin);

// Rotate and translate a local scan into world frame at the given pose.
Eigen::Matrix<double, Eigen::Dynamic, 2> transform_scan_to_world(
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans,
    const Eigen::Vector3d& pose);

// Bilinear interpolation over the grid; optionally computes spatial gradients.
BilinearResult bilinear_interpolation(
    const GridView& gv,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& grid_points_pixels,
    bool compute_gradients = false);

// Joint scan + odometry cost at a given pose. scan_indices selects a subset.
double compute_joint_cost(
    const GridView& gv,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans,
    const Eigen::Vector3d& pose,
    const Eigen::Vector3d& odom_pose,
    const Eigen::Matrix3d& odom_info,
    double sigma_hit,
    const std::vector<int>& scan_indices = {});

// Build scan-matching Jacobian at a given pose.
JacobianResult build_scan_jacobian(
    const GridView& gv,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans,
    const Eigen::Vector3d& pose,
    double sigma_hit);

// Fused translate-interp-score with zero heap allocations per candidate.
double coarse_score(
    const GridView& gv,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& rotated_base,
    double cand_x, double cand_y,
    int& valid_count);

// Grid search over translation and rotation candidates.
Eigen::Vector3d coarse_search(
    const GridView& gv,
    double map_quality,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans,
    const Eigen::Vector3d& odom_pose,
    const SlamConfig& config);

// Levenberg-Marquardt refinement of scan-to-map alignment.
LMResult lm_optimize(
    const GridView& gv,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans,
    Eigen::Vector3d curr_pose,
    const Eigen::Vector3d& odom_pose,
    const Eigen::Matrix3d& odom_info,
    const SlamConfig& config);

// GMapping-style gain-scaled log importance weight from scan likelihood.
double calculate_log_weight(
    const GridView& gv,
    const Eigen::Vector3d& final_pose,
    const Eigen::Vector3d& odom_pose,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans,
    const Eigen::Matrix3d& proposal_cov,
    const Eigen::Matrix3d& odom_info,
    const SlamConfig& config);

// Full scan-match pipeline: coarse search, LM, weight. Returns final pose.
ScanMatchResult scan_match_single(
    const OccupancyGrid& grid,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans,
    const Eigen::Vector3d& odom_pose,
    const Eigen::Matrix3d& odom_cov,
    double map_quality,
    const SlamConfig& config,
    const SigmoidLUT& sigmoid);

}  // namespace slam

#endif  // SLAM_CPP__SCAN_MATCHING_HPP_
