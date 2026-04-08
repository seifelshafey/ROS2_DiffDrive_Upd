#include "slam_cpp/occupancy_grid.hpp"

#include <cmath>

namespace slam {

OccupancyGrid::OccupancyGrid(double width, double height, double resolution,
                             const SlamConfig& config)
    : rows_(static_cast<int>(height / resolution)),
      cols_(static_cast<int>(width / resolution)),
      resolution_(resolution),
      width_(width),
      height_(height),
      origin_x_(cols_ / 2),
      origin_y_(rows_ / 2),
      lo_occupied_(config.lo_occupied_scaled()),
      lo_free_(config.lo_free_scaled()),
      lo_max_(config.lo_max_scaled()),
      lo_min_(config.lo_min_scaled()),
      quality_threshold_(static_cast<int8_t>(1.0 * config.log_odds_scale))
{
  data_ = std::make_shared<GridData>();
  data_->cells.resize(static_cast<size_t>(rows_ * cols_), 0);
}

// Convert world coordinates to grid cell indices.
// Uses std::floor (not truncation) for correct negative coordinate mapping.
std::optional<GridCoord> OccupancyGrid::world_to_grid(double x, double y) const
{
  int gx = static_cast<int>(std::floor(x / resolution_)) + origin_x_;
  int gy = static_cast<int>(std::floor(y / resolution_)) + origin_y_;

  if (gx >= 0 && gx < cols_ && gy >= 0 && gy < rows_) {
    return GridCoord{gx, gy};
  }
  return std::nullopt;
}

// Integrate a full laser scan into the grid via Bresenham raycasting.
void OccupancyGrid::update(
    const Eigen::Vector3d& pose,
    const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans)
{
  ensure_unique();

  auto robot_coord = world_to_grid(pose(0), pose(1));
  if (!robot_coord) {
    return;
  }
  int robot_gx = robot_coord->x;
  int robot_gy = robot_coord->y;

  double theta = pose(2);
  double c = std::cos(theta);
  double s = std::sin(theta);

  int n_beams = static_cast<int>(local_scans.rows());
  for (int i = 0; i < n_beams; ++i) {
    // Transform local hit point to world frame.
    double local_x = local_scans(i, 0);
    double local_y = local_scans(i, 1);
    double world_x = local_x * c - local_y * s + pose(0);
    double world_y = local_x * s + local_y * c + pose(1);

    auto hit_coord = world_to_grid(world_x, world_y);
    if (!hit_coord) {
      continue;  // out of bounds — skip (no phantom walls at grid edge)
    }

    bresenham_raycast(robot_gx, robot_gy, hit_coord->x, hit_coord->y);
  }
}

void OccupancyGrid::ensure_unique()
{
  if (data_.use_count() > 1) {
    data_ = std::make_shared<GridData>(*data_);
  }
}

bool OccupancyGrid::is_shared() const
{
  return data_.use_count() > 1;
}

int8_t OccupancyGrid::at(int row, int col) const
{
  return data_->cells[static_cast<size_t>(row * cols_ + col)];
}

const int8_t* OccupancyGrid::data() const
{
  return data_->cells.data();
}

double OccupancyGrid::map_quality() const
{
  int total = data_->n_occupied + data_->n_free;
  if (total == 0) return 0.0;
  return static_cast<double>(data_->n_occupied) / static_cast<double>(total);
}

// Bresenham line: trace from robot (x0,y0) to hit (x1,y1).
// Cells along the ray are marked FREE; the endpoint is OCCUPIED.
void OccupancyGrid::bresenham_raycast(int x0, int y0, int x1, int y1)
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int cx = x0;
  int cy = y0;

  while (true) {
    if (cx == x1 && cy == y1) {
      // Hit cell -> OCCUPIED (bounds-checked).
      if (cx >= 0 && cx < cols_ && cy >= 0 && cy < rows_) {
        update_cell(cy, cx, lo_occupied_);
      }
      break;
    }

    // Free cell along the ray.
    if (cx >= 0 && cx < cols_ && cy >= 0 && cy < rows_) {
      update_cell(cy, cx, lo_free_);
    }

    // Step direction via doubled error term (avoids floating point).
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      cx += sx;
    }
    if (e2 < dx) {
      err += dx;
      cy += sy;
    }
  }
}

// Update a single cell's log-odds and track quality threshold crossings.
void OccupancyGrid::update_cell(int row, int col, int8_t delta)
{
  auto& cell = data_->cells[static_cast<size_t>(row * cols_ + col)];
  int8_t old_val = cell;

  // Add delta and clamp (compute in int to avoid int8 overflow).
  int new_val_int = static_cast<int>(old_val) + static_cast<int>(delta);
  if (new_val_int > static_cast<int>(lo_max_)) new_val_int = lo_max_;
  if (new_val_int < static_cast<int>(lo_min_)) new_val_int = lo_min_;
  int8_t new_val = static_cast<int8_t>(new_val_int);

  cell = new_val;

  // Incremental quality tracking: count cells crossing +-threshold.
  bool was_occupied = old_val > quality_threshold_;
  bool now_occupied = new_val > quality_threshold_;
  bool was_free = old_val < -quality_threshold_;
  bool now_free = new_val < -quality_threshold_;

  if (!was_occupied && now_occupied) data_->n_occupied++;
  if (was_occupied && !now_occupied) data_->n_occupied--;
  if (!was_free && now_free) data_->n_free++;
  if (was_free && !now_free) data_->n_free--;
}

}  // namespace slam
