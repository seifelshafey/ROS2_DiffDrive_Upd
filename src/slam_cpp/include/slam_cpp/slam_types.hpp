#ifndef SLAM_CPP__SLAM_TYPES_HPP_
#define SLAM_CPP__SLAM_TYPES_HPP_

// slam_types.hpp — Shared type definitions for FastSLAM 2.0.
// Defines normalize_angle, SlamConfig, and the result structs used across
// the scan matching and particle filter pipeline.

#include <cmath>
#include <cstdint>
#include <vector>
#include <Eigen/Dense>

namespace slam {

class SigmoidLUT;  // forward declaration (sigmoid_lut.hpp)

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;

// Wraps any angle to (-pi, pi]. Uses fmod with a sign correction
// because C++ fmod preserves sign of the dividend (unlike Python's %).
inline double normalize_angle(double angle)
{
  angle = std::fmod(angle + kPi, kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  return angle - kPi;
}

// All tunable SLAM parameters, loaded from ROS 2 YAML at startup.
// Tests may construct this directly with custom values.
struct SlamConfig {
  int num_particles = 80;

  // Occupancy grid
  double map_size = 50.0;           // side length in meters (square map)
  double map_resolution = 0.1;      // meters per cell

  // Log-odds scaling: stored_value = int(log_odds * scale)
  double log_odds_scale = 20.0;
  double lo_occupied = 0.85;        // log-odds increment per hit
  double lo_free = -0.4;            // log-odds decrement per free ray
  double lo_max = 5.0;              // clamp ceiling
  double lo_min = -5.0;             // clamp floor

  // Motion model noise (Probabilistic Robotics, Table 5.6)
  double alpha1 = 0.075;            // rotation noise from rotation
  double alpha2 = 0.03;             // rotation noise from translation
  double alpha3 = 0.075;            // translation noise from translation
  double alpha4 = 0.03;             // translation noise from rotation

  // Scan matching
  double sigma_hit = 0.05;          // measurement uncertainty in LM cost
  double sensor_confidence = 5.0;
  int score_stride = 2;             // beam stride for weight scoring
  int lm_max_iters = 10;
  int lm_max_sub_iters = 3;
  double mahalanobis_threshold = 9.0;  // ~97th percentile chi-squared(3 DOF)

  // Levenberg-Marquardt damping schedule
  double lm_lambda_init = 0.01;
  double lm_lambda_min = 1e-7;
  double lm_lambda_max = 1e4;
  double lm_lambda_up = 10.0;       // multiply on step rejection
  double lm_lambda_down = 0.1;      // multiply on step acceptance

  // Motion thresholds for triggering a SLAM update
  double motion_threshold_trans = 0.05;  // meters
  double motion_threshold_rot = 0.05;    // radians

  // Sparse map update gating
  double map_update_dist = 0.20;    // meters
  double map_update_angle = 0.25;   // radians

  // GMapping-style observation gain (Grisetti et al. 2007).
  // gain = 1 / (obs_sigma_gain * num_particles).
  // Higher = flatter weights = less collapse risk.
  double obs_sigma_gain = 3.0;
  double temperature = 0.2;         // legacy, unused in GMapping weighting

  int scan_stride = 2;              // use every Nth beam
  int quality_cache_interval = 5;   // refresh map quality every N iterations

  // Scaled int8 accessors for grid cell values.
  int8_t lo_occupied_scaled() const {
    return static_cast<int8_t>(lo_occupied * log_odds_scale);
  }
  int8_t lo_free_scaled() const {
    return static_cast<int8_t>(lo_free * log_odds_scale);
  }
  int8_t lo_max_scaled() const {
    return static_cast<int8_t>(lo_max * log_odds_scale);
  }
  int8_t lo_min_scaled() const {
    return static_cast<int8_t>(lo_min * log_odds_scale);
  }
};

class OccupancyGrid;  // forward declaration (occupancy_grid.hpp)

// Bilinear interpolation output with optional spatial gradients.
struct BilinearResult {
  Eigen::VectorXd values;
  Eigen::Matrix<double, Eigen::Dynamic, 2> gradients;
  Eigen::VectorXi mask;  // 1 = all four corners in bounds
};

// Levenberg-Marquardt optimization output.
struct LMResult {
  Eigen::Vector3d pose;       // optimized pose [x, y, theta]
  Eigen::Matrix3d hessian;    // accumulated Hessian (scan + odom info)
  bool success;
};

// Full scan matching pipeline output for one particle.
struct ScanMatchResult {
  Eigen::Vector3d pose;            // optimized or fallback pose
  Eigen::Matrix3d proposal_cov;    // proposal distribution covariance
  double log_weight;               // importance weight (log scale)
};

// Scan Jacobian construction output.
struct JacobianResult {
  Eigen::VectorXd residuals;                               // (n_valid,)
  Eigen::Matrix<double, Eigen::Dynamic, 3> jacobian;       // (n_valid, 3)
  std::vector<int> valid_indices;                           // which scan rows are valid
  bool success = false;                                     // false if < 10 valid points
};

// Lightweight read-only view into an OccupancyGrid for scan matching.
// Bundles the six parameters that always travel together, preventing
// rows/cols transposition bugs and reducing function signature clutter.
struct GridView {
  const int8_t* data;
  int rows;
  int cols;
  double inv_res;       // 1.0 / map_resolution
  double origin;        // grid center pixel (cols/2 for square grid)
  const SigmoidLUT* sigmoid;  // shared LUT, not owned
};

// Pre-allocated buffers for scan matching hot path.
// Eliminates per-cycle heap allocations; each OpenMP thread gets its own.
struct ScanMatchWorkspace {
  Eigen::Matrix<double, Eigen::Dynamic, 2> world_pts;
  Eigen::Matrix<double, Eigen::Dynamic, 2> grid_pts;
  Eigen::VectorXd interp_values;
  Eigen::VectorXi interp_mask;
  Eigen::Matrix<double, Eigen::Dynamic, 2> interp_gradients;
  Eigen::VectorXd residuals;
  Eigen::Matrix<double, Eigen::Dynamic, 3> jacobian;
  std::vector<int> valid_indices;

  // Only reallocates when n exceeds current capacity.
  void resize(int n) {
    if (world_pts.rows() < n) {
      world_pts.resize(n, 2);
      grid_pts.resize(n, 2);
      interp_values.resize(n);
      interp_mask.resize(n);
      interp_gradients.resize(n, 2);
      residuals.resize(n);
      jacobian.resize(n, 3);
    }
    valid_indices.clear();
    valid_indices.reserve(static_cast<size_t>(n));
  }
};

}  // namespace slam

#endif  // SLAM_CPP__SLAM_TYPES_HPP_
