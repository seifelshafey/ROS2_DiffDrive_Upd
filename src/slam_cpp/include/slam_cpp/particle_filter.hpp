#ifndef SLAM_CPP__PARTICLE_FILTER_HPP_
#define SLAM_CPP__PARTICLE_FILTER_HPP_

// particle_filter.hpp — FastSLAM 2.0 particle filter.
// Manages N particles, each carrying a pose, covariance, weight, and occupancy grid.
// SOA layout: poses in a 3xN matrix (vectorized trig), grids stored separately (250KB each).
// Per-particle scan matching is parallelized with OpenMP; RNG is sequential for determinism.

#include <random>
#include <vector>
#include <Eigen/Dense>

#include "slam_cpp/slam_types.hpp"
#include "slam_cpp/occupancy_grid.hpp"
#include "slam_cpp/sigmoid_lut.hpp"

namespace slam {

class ParticleFilter {
public:
  // Production constructor: seeds RNG from hardware entropy.
  explicit ParticleFilter(const SlamConfig& config);

  // Test constructor: fixed seed for deterministic results.
  ParticleFilter(const SlamConfig& config, uint64_t seed);

  // Motion update: apply noisy rotation-translation-rotation to all particles.
  // direction: +1.0 forward, -1.0 reverse.
  void predict(double trans, double rot1, double rot2, double direction);

  // Measurement update: scan match, sample from proposal, update weights,
  // conditionally resample (N_eff < N/2), and update occupancy grids.
  void update_and_resample(
      const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans);

  // Index of the highest-weight particle.
  size_t best_particle_index() const;

  // Read-only accessors for the ROS 2 node.
  const Eigen::Matrix<double, 3, Eigen::Dynamic>& poses() const { return poses_; }
  const Eigen::VectorXd& weights() const { return weights_; }
  const std::vector<OccupancyGrid>& grids() const { return grids_; }
  const SlamConfig& config() const { return config_; }
  int global_counter() const { return global_counter_; }

private:
  SlamConfig config_;
  int N_;
  SigmoidLUT sigmoid_;

  // SOA particle state
  Eigen::Matrix<double, 3, Eigen::Dynamic> poses_;  // 3 x N
  Eigen::VectorXd weights_;                          // N x 1
  std::vector<Eigen::Matrix3d> covs_;                // N covariance matrices
  std::vector<OccupancyGrid> grids_;                 // N grids (COW internally)

  // Map quality cache
  Eigen::VectorXd cached_map_quality_;
  int last_quality_update_ = -5;

  // Sparse map update tracking
  Eigen::Vector3d last_map_update_pose_;
  bool has_last_map_update_pose_ = false;

  // RNG (mutable: mutation is an implementation detail, not logical state change)
  mutable std::mt19937 rng_;
  int global_counter_ = 0;

  void update_map_quality_cache();
  std::vector<size_t> systematic_resample() const;
  void init_particles();
};

}  // namespace slam

#endif  // SLAM_CPP__PARTICLE_FILTER_HPP_
