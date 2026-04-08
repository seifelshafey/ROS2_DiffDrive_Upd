#include "slam_cpp/scan_matching.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

namespace slam {

// ---------------------------------------------------------------------------
// GridView factory
// ---------------------------------------------------------------------------
GridView make_grid_view(const OccupancyGrid& grid, const SlamConfig& config,
                        const SigmoidLUT& sigmoid)
{
  return {
      grid.data(),
      grid.rows(),
      grid.cols(),
      1.0 / config.map_resolution,
      static_cast<double>(grid.origin_x()),
      &sigmoid
  };
}

// ---------------------------------------------------------------------------
// extract_local_scan — polar lidar ranges to local Cartesian (x,y) matrix.
// ---------------------------------------------------------------------------
Eigen::Matrix<double, Eigen::Dynamic, 2> extract_local_scan(
    const float* ranges, int num_ranges,
    float angle_min, float angle_max, float range_max,
    int stride)
{
  double angle_inc = 0.0;
  if (num_ranges > 1) {
    angle_inc = static_cast<double>(angle_max - angle_min)
                / static_cast<double>(num_ranges - 1);
  }

  // Collect valid beam indices (r > 0 filters minimum-range self-hits).
  std::vector<int> valid_indices;
  valid_indices.reserve(static_cast<size_t>(num_ranges / stride));
  for (int i = 0; i < num_ranges; i += stride) {
    float r = ranges[i];
    if (std::isfinite(r) && r > 0.0f && r < range_max) {
      valid_indices.push_back(i);
    }
  }

  int n = static_cast<int>(valid_indices.size());
  Eigen::Matrix<double, Eigen::Dynamic, 2> result(n, 2);
  for (int k = 0; k < n; ++k) {
    int i = valid_indices[static_cast<size_t>(k)];
    double r = static_cast<double>(ranges[i]);
    double angle = static_cast<double>(angle_min) + static_cast<double>(i) * angle_inc;
    result(k, 0) = r * std::cos(angle);
    result(k, 1) = r * std::sin(angle);
  }
  return result;
}

// ---------------------------------------------------------------------------
// subsample_scan — extract every stride-th row.
// ---------------------------------------------------------------------------
Eigen::Matrix<double, Eigen::Dynamic, 2> subsample_scan(
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& scans, int stride)
{
  int n_full = static_cast<int>(scans.rows());
  int n_sub = 0;
  for (int i = 0; i < n_full; i += stride) n_sub++;

  Eigen::Matrix<double, Eigen::Dynamic, 2> result(n_sub, 2);
  int k = 0;
  for (int i = 0; i < n_full; i += stride) {
    result.row(k++) = scans.row(i);
  }
  return result;
}

// ---------------------------------------------------------------------------
// to_grid_coords — metric points to grid pixel coordinates.
// ---------------------------------------------------------------------------
Eigen::Matrix<double, Eigen::Dynamic, 2> to_grid_coords(
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& points_meters,
    double inv_res, double origin)
{
  Eigen::Matrix<double, Eigen::Dynamic, 2> result(points_meters.rows(), 2);
  result.col(0) = points_meters.col(0).array() * inv_res + origin;
  result.col(1) = points_meters.col(1).array() * inv_res + origin;
  return result;
}

// ---------------------------------------------------------------------------
// transform_scan_to_world — rotate and translate local scan by pose.
// ---------------------------------------------------------------------------
Eigen::Matrix<double, Eigen::Dynamic, 2> transform_scan_to_world(
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans,
    const Eigen::Vector3d& pose)
{
  double c = std::cos(pose(2));
  double s = std::sin(pose(2));
  int n = static_cast<int>(local_scans.rows());

  Eigen::Matrix<double, Eigen::Dynamic, 2> world(n, 2);
  world.col(0) = local_scans.col(0).array() * c - local_scans.col(1).array() * s + pose(0);
  world.col(1) = local_scans.col(0).array() * s + local_scans.col(1).array() * c + pose(1);
  return world;
}

// ---------------------------------------------------------------------------
// bilinear_interpolation — sub-cell grid lookups with optional gradients.
// ---------------------------------------------------------------------------
BilinearResult bilinear_interpolation(
    const GridView& gv,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& grid_points_pixels,
    bool compute_gradients)
{
  int n = static_cast<int>(grid_points_pixels.rows());
  BilinearResult result;
  result.values = Eigen::VectorXd::Zero(n);
  result.mask = Eigen::VectorXi::Zero(n);
  if (compute_gradients) {
    result.gradients = Eigen::Matrix<double, Eigen::Dynamic, 2>::Zero(n, 2);
  }

  for (int i = 0; i < n; ++i) {
    double px = grid_points_pixels(i, 0);
    double py = grid_points_pixels(i, 1);

    int x0 = static_cast<int>(std::floor(px));
    int y0 = static_cast<int>(std::floor(py));
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    if (x0 < 0 || x1 >= gv.cols || y0 < 0 || y1 >= gv.rows) {
      continue;
    }

    result.mask(i) = 1;

    double Ia = (*gv.sigmoid)(gv.data[y0 * gv.cols + x0]);
    double Ib = (*gv.sigmoid)(gv.data[y1 * gv.cols + x0]);
    double Ic = (*gv.sigmoid)(gv.data[y0 * gv.cols + x1]);
    double Id = (*gv.sigmoid)(gv.data[y1 * gv.cols + x1]);

    double dx = px - x0;
    double dy = py - y0;
    double inv_dx = 1.0 - dx;
    double inv_dy = 1.0 - dy;

    result.values(i) = inv_dx * inv_dy * Ia
                     + inv_dx * dy     * Ib
                     + dx     * inv_dy * Ic
                     + dx     * dy     * Id;

    if (compute_gradients) {
      result.gradients(i, 0) = inv_dy * (Ic - Ia) + dy * (Id - Ib);
      result.gradients(i, 1) = inv_dx * (Ib - Ia) + dx * (Id - Ic);
    }
  }
  return result;
}

// ---------------------------------------------------------------------------
// compute_joint_cost — scan residuals + odometry prior.
// ---------------------------------------------------------------------------
double compute_joint_cost(
    const GridView& gv,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans,
    const Eigen::Vector3d& pose,
    const Eigen::Vector3d& odom_pose,
    const Eigen::Matrix3d& odom_info,
    double sigma_hit,
    const std::vector<int>& scan_indices)
{
  Eigen::Matrix<double, Eigen::Dynamic, 2> eval_scans;
  if (scan_indices.empty()) {
    eval_scans = local_scans;
  } else {
    int ns = static_cast<int>(scan_indices.size());
    eval_scans.resize(ns, 2);
    for (int k = 0; k < ns; ++k) {
      eval_scans.row(k) = local_scans.row(scan_indices[static_cast<size_t>(k)]);
    }
  }

  auto world_pts = transform_scan_to_world(eval_scans, pose);
  auto pix = to_grid_coords(world_pts, gv.inv_res, gv.origin);
  auto interp = bilinear_interpolation(gv, pix);

  int n_pts = static_cast<int>(eval_scans.rows());
  double cost_scan = 0.0;
  for (int k = 0; k < n_pts; ++k) {
    if (interp.mask(k)) {
      double err = (1.0 - interp.values(k)) / sigma_hit;
      cost_scan += err * err;
    }
  }
  cost_scan *= 0.5;

  Eigen::Vector3d diff = pose - odom_pose;
  diff(2) = normalize_angle(diff(2));
  double cost_odom = 0.5 * diff.transpose() * odom_info * diff;

  return cost_scan + cost_odom;
}

// ---------------------------------------------------------------------------
// build_scan_jacobian — Jacobian of scan residuals w.r.t. pose [x, y, theta].
// ---------------------------------------------------------------------------
JacobianResult build_scan_jacobian(
    const GridView& gv,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans,
    const Eigen::Vector3d& pose,
    double sigma_hit)
{
  JacobianResult result;
  double grad_scale = gv.inv_res;

  auto global_scans = transform_scan_to_world(local_scans, pose);
  auto global_pix = to_grid_coords(global_scans, gv.inv_res, gv.origin);
  auto interp = bilinear_interpolation(gv, global_pix, true);

  int n_scans = static_cast<int>(local_scans.rows());
  result.valid_indices.reserve(static_cast<size_t>(n_scans));
  for (int i = 0; i < n_scans; ++i) {
    if (interp.mask(i) && interp.values(i) >= 0.0) {
      result.valid_indices.push_back(i);
    }
  }

  int n_valid = static_cast<int>(result.valid_indices.size());
  if (n_valid < 10) {
    result.success = false;
    return result;
  }
  result.success = true;

  result.residuals.resize(n_valid);
  result.jacobian.resize(n_valid, 3);

  for (int k = 0; k < n_valid; ++k) {
    int i = result.valid_indices[static_cast<size_t>(k)];
    result.residuals(k) = (1.0 - interp.values(i)) / sigma_hit;

    double gx = -interp.gradients(i, 0) * grad_scale / sigma_hit;
    double gy = -interp.gradients(i, 1) * grad_scale / sigma_hit;
    double px_rel = global_scans(i, 0) - pose(0);
    double py_rel = global_scans(i, 1) - pose(1);

    result.jacobian(k, 0) = gx;
    result.jacobian(k, 1) = gy;
    result.jacobian(k, 2) = gx * (-py_rel) + gy * px_rel;  // chain rule for theta
  }
  return result;
}

// ---------------------------------------------------------------------------
// coarse_score — fused translate-interp-score, zero heap allocations.
// ---------------------------------------------------------------------------
double coarse_score(
    const GridView& gv,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& rotated_base,
    double cand_x, double cand_y,
    int& valid_count)
{
  valid_count = 0;
  double score = 0.0;
  int n = static_cast<int>(rotated_base.rows());

  for (int k = 0; k < n; ++k) {
    double wx = rotated_base(k, 0) + cand_x;
    double wy = rotated_base(k, 1) + cand_y;

    double px = wx * gv.inv_res + gv.origin;
    double py = wy * gv.inv_res + gv.origin;

    int x0 = static_cast<int>(std::floor(px));
    int y0 = static_cast<int>(std::floor(py));
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    if (x0 < 0 || x1 >= gv.cols || y0 < 0 || y1 >= gv.rows) {
      continue;
    }

    double Ia = (*gv.sigmoid)(gv.data[y0 * gv.cols + x0]);
    double Ib = (*gv.sigmoid)(gv.data[y1 * gv.cols + x0]);
    double Ic = (*gv.sigmoid)(gv.data[y0 * gv.cols + x1]);
    double Id = (*gv.sigmoid)(gv.data[y1 * gv.cols + x1]);

    double dx_f = px - x0;
    double dy_f = py - y0;
    double val = (1.0 - dx_f) * (1.0 - dy_f) * Ia
               + (1.0 - dx_f) * dy_f          * Ib
               + dx_f         * (1.0 - dy_f)  * Ic
               + dx_f         * dy_f           * Id;

    if (val >= 0.0) {
      valid_count++;
      score += val;
    }
  }
  return score;
}

// ---------------------------------------------------------------------------
// coarse_search — grid search over candidate poses.
// best_score initialized to -inf (not +inf as in Python original).
// ---------------------------------------------------------------------------
Eigen::Vector3d coarse_search(
    const GridView& gv,
    double map_quality,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans,
    const Eigen::Vector3d& odom_pose,
    const SlamConfig& config)
{
  Eigen::Vector3d best_pose = odom_pose;

  std::vector<int> search_steps;
  std::vector<double> search_angles;
  int coarse_stride;

  if (map_quality < 0.2) {
    search_steps = {0, 1, -1, 2, -2, 3, -3};
    search_angles = {0.0, 0.1, -0.1};
    coarse_stride = 5;
  } else {
    search_steps = {0, 1, -1};
    search_angles = {0.0, 0.05, -0.05};
    coarse_stride = 3;
  }

  auto coarse_scans = subsample_scan(local_scans, coarse_stride);
  int n_coarse = static_cast<int>(coarse_scans.rows());

  double best_score = -std::numeric_limits<double>::infinity();

  // Pre-allocate rotated scan buffer (reused across dx/dy candidates).
  Eigen::Matrix<double, Eigen::Dynamic, 2> rotated_base(n_coarse, 2);

  for (double d_th : search_angles) {
    double th_test = odom_pose(2) + d_th;
    double c = std::cos(th_test);
    double s = std::sin(th_test);

    // Rotate once per theta; translation varies per candidate in coarse_score.
    for (int k = 0; k < n_coarse; ++k) {
      double lx = coarse_scans(k, 0);
      double ly = coarse_scans(k, 1);
      rotated_base(k, 0) = lx * c - ly * s;
      rotated_base(k, 1) = lx * s + ly * c;
    }

    for (int dx_step : search_steps) {
      for (int dy_step : search_steps) {
        double cand_x = odom_pose(0) + dx_step * config.map_resolution;
        double cand_y = odom_pose(1) + dy_step * config.map_resolution;

        int valid_count = 0;
        double score = coarse_score(gv, rotated_base, cand_x, cand_y, valid_count);

        if (map_quality > 0.01
            && valid_count < static_cast<int>(n_coarse * 0.2)) {
          continue;
        }

        if (score > best_score) {
          best_score = score;
          best_pose = Eigen::Vector3d(cand_x, cand_y, th_test);
        }
      }
    }
  }
  return best_pose;
}

// ---------------------------------------------------------------------------
// lm_optimize — Levenberg-Marquardt scan-to-map refinement.
// Cost comparison uses same valid_indices as Jacobian for consistency.
// ---------------------------------------------------------------------------
LMResult lm_optimize(
    const GridView& gv,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans,
    Eigen::Vector3d curr_pose,
    const Eigen::Vector3d& odom_pose,
    const Eigen::Matrix3d& odom_info,
    const SlamConfig& config)
{
  double lm_lambda = config.lm_lambda_init;
  Eigen::Matrix3d H_final = odom_info;
  Eigen::Vector3d delta = Eigen::Vector3d::Zero();

  for (int iter = 0; iter < config.lm_max_iters; ++iter) {

    auto jac = build_scan_jacobian(gv, local_scans, curr_pose, config.sigma_hit);
    if (!jac.success) {
      return {curr_pose, H_final, false};
    }

    // Current cost (scan residuals + odometry prior).
    double cost_scan = 0.5 * jac.residuals.squaredNorm();
    Eigen::Vector3d diff = curr_pose - odom_pose;
    diff(2) = normalize_angle(diff(2));
    double cost_odom = 0.5 * diff.transpose() * odom_info * diff;
    double current_cost = cost_scan + cost_odom;

    // Normal equations: H = J^T*J + odom_info, b = J^T*r + odom_info*diff.
    Eigen::Matrix3d H_scan = jac.jacobian.transpose() * jac.jacobian;
    Eigen::Vector3d b_scan = jac.jacobian.transpose() * jac.residuals;
    Eigen::Vector3d b_odom = odom_info * diff;

    Eigen::Matrix3d H_total = H_scan + odom_info;
    Eigen::Vector3d b_total = b_scan + b_odom;

    // LM inner loop: damped solve with adaptive lambda.
    bool valid_step = false;

    for (int sub = 0; sub < config.lm_max_sub_iters; ++sub) {
      Eigen::Matrix3d LHS = H_total
          + Eigen::Matrix3d::Identity() * lm_lambda * (H_total.trace() / 3.0);

      Eigen::LDLT<Eigen::Matrix3d> ldlt(LHS);
      if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
        lm_lambda *= config.lm_lambda_up;
        continue;
      }
      delta = ldlt.solve(-b_total);

      if (!delta.allFinite()) {
        lm_lambda *= config.lm_lambda_up;
        continue;
      }

      if (delta.norm() < 1e-5) {
        valid_step = true;
        break;
      }

      Eigen::Vector3d cand_pose = curr_pose + delta;
      cand_pose(2) = normalize_angle(cand_pose(2));

      // Use same valid_indices for fair cost comparison.
      double cand_cost = compute_joint_cost(
          gv, local_scans, cand_pose, odom_pose, odom_info,
          config.sigma_hit, jac.valid_indices);

      if (cand_cost < current_cost) {
        curr_pose = cand_pose;
        current_cost = cand_cost;
        lm_lambda *= config.lm_lambda_down;
        lm_lambda = std::max(lm_lambda, config.lm_lambda_min);
        valid_step = true;
        H_final = H_total;
        break;
      } else {
        lm_lambda *= config.lm_lambda_up;
        lm_lambda = std::min(lm_lambda, config.lm_lambda_max);
      }
    }

    if (!valid_step) {
      return {curr_pose, H_final, false};
    }
    if (delta.norm() < 1e-4) {
      return {curr_pose, H_final, true};
    }
  }
  return {curr_pose, H_final, true};
}

// ---------------------------------------------------------------------------
// calculate_log_weight — GMapping-style gain-scaled scan likelihood.
//
// Uses only the scan term: gain * sum(log(map_value_at_beam)).
// The odom prior is already inside the LM cost function; including it here
// would double-count it. The proposal density ratio cancels across particles
// when the scan-matched proposal approximates the posterior (Grisetti 2007).
// ---------------------------------------------------------------------------
double calculate_log_weight(
    const GridView& gv,
    const Eigen::Vector3d& final_pose,
    const Eigen::Vector3d& /* odom_pose */,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans,
    const Eigen::Matrix3d& /* proposal_cov */,
    const Eigen::Matrix3d& /* odom_info */,
    const SlamConfig& config)
{
  auto sparse_scans = subsample_scan(local_scans, config.score_stride);
  auto sparse_world = transform_scan_to_world(sparse_scans, final_pose);
  auto sparse_pix = to_grid_coords(sparse_world, gv.inv_res, gv.origin);
  auto interp = bilinear_interpolation(gv, sparse_pix);

  double log_w_scan = 0.0;
  int n_sparse = static_cast<int>(sparse_scans.rows());
  for (int k = 0; k < n_sparse; ++k) {
    double val = std::clamp(interp.values(k), 1e-9, 1.0);
    log_w_scan += std::log(val);
  }

  // GMapping gain: flattens weight distribution to prevent particle collapse.
  // With obs_sigma_gain=3.0 and N=80: gain = 0.0042, compressing a ~27 nat
  // raw range to ~0.11 (weight ratio ~1.12 between best and worst particle).
  double gain = 1.0 / (config.obs_sigma_gain * config.num_particles);
  log_w_scan *= gain;

  return log_w_scan;
}

// ---------------------------------------------------------------------------
// scan_match_single — full pipeline: coarse search, LM, weight.
// ---------------------------------------------------------------------------
ScanMatchResult scan_match_single(
    const OccupancyGrid& grid,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans,
    const Eigen::Vector3d& odom_pose,
    const Eigen::Matrix3d& odom_cov,
    double map_quality,
    const SlamConfig& config,
    const SigmoidLUT& sigmoid)
{
  // Covariance floor: caps odom_info diagonal at ~100, preventing
  // numerical blow-up when motion noise is very small.
  static constexpr double kCovFloor = 0.01;
  Eigen::Matrix3d floored_cov = odom_cov;
  for (int d = 0; d < 3; ++d) {
    if (floored_cov(d, d) < kCovFloor) {
      floored_cov(d, d) = kCovFloor;
    }
  }

  // Invert floored covariance to information matrix.
  Eigen::Matrix3d odom_info;
  Eigen::Matrix3d regularized_cov = floored_cov + Eigen::Matrix3d::Identity() * 1e-9;
  Eigen::LDLT<Eigen::Matrix3d> ldlt_cov(regularized_cov);
  if (ldlt_cov.info() == Eigen::Success && ldlt_cov.isPositive()) {
    odom_info = ldlt_cov.solve(Eigen::Matrix3d::Identity());
  } else {
    odom_info = Eigen::Matrix3d::Identity() * 10.0;
  }

  // No quality gate: scan matching always runs. On empty maps, LM gets
  // zero gradients and returns near-odom naturally.
  (void)map_quality;

  GridView gv = make_grid_view(grid, config, sigmoid);

  Eigen::Vector3d best_start = coarse_search(gv, map_quality, local_scans, odom_pose, config);

  auto [opt_pose, H_final, success] = lm_optimize(
      gv, local_scans, best_start, odom_pose, odom_info, config);

  Eigen::Vector3d final_pose;
  Eigen::Matrix3d proposal_cov;

  if (!success) {
    // Use tiny covariance on fallback: predict() already added motion noise.
    // Reusing odom_cov would double that scatter and cause particle drift.
    final_pose = odom_pose;
    proposal_cov = Eigen::Matrix3d::Identity() * 1e-8;
  } else {
    final_pose = opt_pose;

    Eigen::LDLT<Eigen::Matrix3d> ldlt_h(H_final);
    if (ldlt_h.info() == Eigen::Success && ldlt_h.isPositive()) {
      proposal_cov = ldlt_h.solve(Eigen::Matrix3d::Identity());
      proposal_cov += Eigen::Matrix3d::Identity() * 1e-5;
    } else {
      proposal_cov = Eigen::Matrix3d::Identity() * 1e-8;
    }
  }

  double log_weight = calculate_log_weight(
      gv, final_pose, odom_pose, local_scans,
      proposal_cov, odom_info, config);

  return {final_pose, proposal_cov, log_weight};
}

}  // namespace slam
