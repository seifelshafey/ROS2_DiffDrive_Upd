#include "slam_cpp/particle_filter.hpp"
#include "slam_cpp/scan_matching.hpp"

#include <algorithm>

namespace slam {

void ParticleFilter::init_particles()
{
  N_ = config_.num_particles;
  poses_ = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, N_);
  weights_ = Eigen::VectorXd::Constant(N_, 1.0 / N_);
  covs_.resize(N_, Eigen::Matrix3d::Zero());
  cached_map_quality_ = Eigen::VectorXd::Zero(N_);

  grids_.reserve(N_);
  for (int i = 0; i < N_; ++i) {
    grids_.emplace_back(
        config_.map_size, config_.map_size, config_.map_resolution, config_);
  }
}

ParticleFilter::ParticleFilter(const SlamConfig& config)
    : config_(config), N_(0), sigmoid_(config.log_odds_scale),
      rng_(std::random_device{}())
{
  init_particles();
}

ParticleFilter::ParticleFilter(const SlamConfig& config, uint64_t seed)
    : config_(config), N_(0), sigmoid_(config.log_odds_scale),
      rng_(seed)
{
  init_particles();
}

// ---------------------------------------------------------------------------
// predict — odometry motion model (rotation-translation-rotation).
// Noise std devs scale with motion magnitude (Prob. Robotics Table 5.6).
// Covariance propagated analytically: P = F*P*F^T + G*M*G^T + reg.
// ---------------------------------------------------------------------------
void ParticleFilter::predict(double trans, double rot1, double rot2,
                             double direction)
{
  // Noise standard deviations (1e-9 floor prevents zero std dev).
  double sd_rot1 = config_.alpha1 * std::abs(rot1)
                 + config_.alpha2 * trans + 1e-9;
  double sd_trans = config_.alpha3 * trans
                  + config_.alpha4 * (std::abs(rot1) + std::abs(rot2)) + 1e-9;
  double sd_rot2 = config_.alpha1 * std::abs(rot2)
                 + config_.alpha2 * trans + 1e-9;

  // Control noise covariance (diagonal — independent noise sources).
  Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
  M(0, 0) = sd_rot1 * sd_rot1;
  M(1, 1) = sd_trans * sd_trans;
  M(2, 2) = sd_rot2 * sd_rot2;

  static const Eigen::Matrix3d reg = Eigen::Matrix3d::Identity() * 1e-6;

  std::normal_distribution<double> dist_rot1(0.0, sd_rot1);
  std::normal_distribution<double> dist_trans(0.0, sd_trans);
  std::normal_distribution<double> dist_rot2(0.0, sd_rot2);

  for (int i = 0; i < N_; ++i) {
    // Sample noisy controls (subtract: Prob. Robotics convention).
    double noisy_rot1  = rot1  - dist_rot1(rng_);
    double noisy_trans = trans - dist_trans(rng_);
    double noisy_rot2  = rot2  - dist_rot2(rng_);

    double old_theta = poses_(2, i);
    double heading = old_theta + noisy_rot1;
    double sin_h = std::sin(heading);
    double cos_h = std::cos(heading);
    double t_dir = noisy_trans * direction;

    // Update pose.
    poses_(0, i) += t_dir * cos_h;
    poses_(1, i) += t_dir * sin_h;
    poses_(2, i) += noisy_rot1 + noisy_rot2;
    poses_(2, i) = normalize_angle(poses_(2, i));

    // State transition Jacobian F = I + [0 0 -t*sin; 0 0 t*cos; 0 0 0].
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -t_dir * sin_h;
    F(1, 2) =  t_dir * cos_h;

    // Control Jacobian G: maps (rot1, trans, rot2) noise to pose change.
    Eigen::Matrix3d G = Eigen::Matrix3d::Zero();
    G(0, 0) = -t_dir * sin_h;
    G(0, 1) =  direction * cos_h;
    G(1, 0) =  t_dir * cos_h;
    G(1, 1) =  direction * sin_h;
    G(2, 0) = 1.0;
    G(2, 2) = 1.0;

    // Propagate covariance.
    covs_[i] = F * covs_[i] * F.transpose()
             + G * M * G.transpose()
             + reg;
  }
}

// ---------------------------------------------------------------------------
// update_and_resample — scan match, sample, weight, resample, update grids.
// ---------------------------------------------------------------------------
void ParticleFilter::update_and_resample(
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans)
{
  // Stage A: Save predicted poses/covariances (scan matching needs these).
  Eigen::Matrix<double, 3, Eigen::Dynamic> odom_poses = poses_;
  std::vector<Eigen::Matrix3d> prior_covs = covs_;

  Eigen::Matrix<double, 3, Eigen::Dynamic> opt_poses(3, N_);
  std::vector<Eigen::Matrix3d> proposed_covs(static_cast<size_t>(N_));
  Eigen::VectorXd log_weights(N_);

  // Stage B: Refresh map quality cache if due.
  if (global_counter_ - last_quality_update_ >= config_.quality_cache_interval) {
    update_map_quality_cache();
  }

  // Stage C: Per-particle scan matching (OpenMP parallel).
#ifdef SLAM_HAS_OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (int i = 0; i < N_; ++i) {
    Eigen::Matrix3d odom_cov =
        prior_covs[static_cast<size_t>(i)]
        + Eigen::Matrix3d::Identity() * 1e-9;

    ScanMatchResult result = scan_match_single(
        grids_[static_cast<size_t>(i)],
        local_scans,
        odom_poses.col(i),
        odom_cov,
        cached_map_quality_(i),
        config_,
        sigmoid_);

    opt_poses.col(i) = result.pose;
    proposed_covs[static_cast<size_t>(i)] = result.proposal_cov;
    log_weights(i) = result.log_weight;
  }

  // Stage D: Cholesky sampling from proposal N(opt_pose, proposal_cov).
  // Per-particle fallback if covariance is non-SPD (unlike Python's batch approach).
  Eigen::Matrix<double, 3, Eigen::Dynamic> sampled_poses(3, N_);
  {
    std::normal_distribution<double> standard_normal(0.0, 1.0);

    for (int i = 0; i < N_; ++i) {
      Eigen::Vector3d z(
          standard_normal(rng_),
          standard_normal(rng_),
          standard_normal(rng_));

      Eigen::LLT<Eigen::Matrix3d> llt(proposed_covs[static_cast<size_t>(i)]);

      if (llt.info() == Eigen::Success) {
        sampled_poses.col(i) = opt_poses.col(i) + llt.matrixL() * z;
      } else {
        // Fallback: independent sampling from diagonal elements.
        const auto& cov = proposed_covs[static_cast<size_t>(i)];
        Eigen::Vector3d stds(
            std::sqrt(cov(0, 0)),
            std::sqrt(cov(1, 1)),
            std::sqrt(cov(2, 2)));
        sampled_poses.col(i) = opt_poses.col(i) + stds.cwiseProduct(z);
      }

      sampled_poses(2, i) = normalize_angle(sampled_poses(2, i));
    }
  }

  // Stage E: Weight accumulation with log-sum-exp for numerical stability.
  // w_new = w_old * exp(log_w - max_log_w). Weights carry forward across
  // iterations where resampling is skipped.
  {
    double max_log_w = log_weights.maxCoeff();
    Eigen::VectorXd weight_update =
        (log_weights.array() - max_log_w).exp();

    weights_ = weights_.cwiseProduct(weight_update);

    double sum_w = weights_.sum();
    if (sum_w < 1e-200 || !weights_.allFinite()) {
      weights_.setConstant(1.0 / N_);
    } else {
      weights_ /= sum_w;
    }
  }

  // Stage F: Sparse map update gating (skip if robot hasn't moved enough).
  size_t best_idx = best_particle_index();
  Eigen::Vector3d curr_best_pose = sampled_poses.col(
      static_cast<Eigen::Index>(best_idx));

  bool do_map_update;
  if (!has_last_map_update_pose_) {
    do_map_update = true;
  } else {
    double dist = (curr_best_pose.head<2>()
                 - last_map_update_pose_.head<2>()).norm();
    double angle_diff = std::abs(normalize_angle(
        curr_best_pose(2) - last_map_update_pose_(2)));
    do_map_update = (dist > config_.map_update_dist)
                 || (angle_diff > config_.map_update_angle);
  }

  // Stage G: Adaptive resampling (N_eff < N/2 triggers systematic resample).
  double n_eff = 1.0 / weights_.squaredNorm();
  bool should_resample = (n_eff < N_ / 2.0);

  if (should_resample) {
    std::vector<size_t> keep_indices = systematic_resample();

    // Update maps for unique ancestors only (avoids redundant raycasting).
    std::vector<size_t> unique_ancestors = keep_indices;
    std::sort(unique_ancestors.begin(), unique_ancestors.end());
    unique_ancestors.erase(
        std::unique(unique_ancestors.begin(), unique_ancestors.end()),
        unique_ancestors.end());

    if (do_map_update) {
      for (size_t idx : unique_ancestors) {
        grids_[idx].update(
            Eigen::Vector3d(sampled_poses.col(static_cast<Eigen::Index>(idx))),
            local_scans);
      }
    }

    // Clone particle set from ancestors (grid copy is O(1) via COW).
    std::vector<OccupancyGrid> new_grids;
    new_grids.reserve(static_cast<size_t>(N_));
    for (int m = 0; m < N_; ++m) {
      size_t ancestor = keep_indices[static_cast<size_t>(m)];
      poses_.col(m) = sampled_poses.col(static_cast<Eigen::Index>(ancestor));
      covs_[static_cast<size_t>(m)] = proposed_covs[ancestor];
      new_grids.push_back(grids_[ancestor]);
    }
    grids_ = std::move(new_grids);

    weights_.setConstant(1.0 / N_);

  } else {
    // No-resample path: keep all particles with accumulated weights.
    for (int m = 0; m < N_; ++m) {
      poses_.col(m) = sampled_poses.col(m);
      covs_[static_cast<size_t>(m)] = proposed_covs[static_cast<size_t>(m)];
    }

    if (do_map_update) {
      for (int i = 0; i < N_; ++i) {
        grids_[static_cast<size_t>(i)].update(
            Eigen::Vector3d(poses_.col(i)), local_scans);
      }
    }
  }

  // Stage H: Finalize.
  if (do_map_update) {
    last_map_update_pose_ = curr_best_pose;
    has_last_map_update_pose_ = true;
  }

  global_counter_++;
}

size_t ParticleFilter::best_particle_index() const
{
  Eigen::Index idx;
  weights_.maxCoeff(&idx);
  return static_cast<size_t>(idx);
}

void ParticleFilter::update_map_quality_cache()
{
  for (int i = 0; i < N_; ++i) {
    cached_map_quality_(i) = grids_[static_cast<size_t>(i)].map_quality();
  }
  last_quality_update_ = global_counter_;
}

// ---------------------------------------------------------------------------
// systematic_resample — O(N) resampling with single random offset.
// Places N evenly-spaced probes on the cumulative weight distribution.
// High-weight particles are hit by multiple probes (duplicated).
// ---------------------------------------------------------------------------
std::vector<size_t> ParticleFilter::systematic_resample() const
{
  std::vector<size_t> indices(static_cast<size_t>(N_));

  double inv_n = 1.0 / static_cast<double>(N_);
  std::uniform_real_distribution<double> dist(0.0, inv_n);
  double r = dist(rng_);

  double c = weights_(0);
  size_t i = 0;

  for (int m = 0; m < N_; ++m) {
    double u = r + m * inv_n;

    while (u > c && i < static_cast<size_t>(N_ - 1)) {
      ++i;
      c += weights_(static_cast<Eigen::Index>(i));
    }

    indices[static_cast<size_t>(m)] = i;
  }

  return indices;
}

}  // namespace slam
