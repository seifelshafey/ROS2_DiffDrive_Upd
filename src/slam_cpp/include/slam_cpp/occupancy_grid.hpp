#ifndef SLAM_CPP__OCCUPANCY_GRID_HPP_
#define SLAM_CPP__OCCUPANCY_GRID_HPP_

// ============================================================================
// occupancy_grid.hpp — Int8 log-odds occupancy grid with COW and Bresenham.
//
// Maps from: occupancy_grid_new.py (all 152 lines).
//
// WHAT THIS CLASS DOES:
//   Represents a 2D occupancy grid map where each cell stores the log-odds
//   probability of being occupied, quantized to int8 for memory efficiency.
//   The grid is centered at (0,0) — world coordinate (0,0) maps to the
//   center pixel of the grid.
//
//   Construction: OccupancyGrid(50.0, 50.0, 0.1, config) creates a 500x500
//   grid where each cell is 0.1m x 0.1m, covering a 50m x 50m area.
//   All cells start at 0 (unknown, 50% probability of occupied).
//
// COPY-ON-WRITE (COW):
//   During particle filter resampling, high-weight particles are duplicated.
//   Each particle owns a 500x500 grid (250 KB). Naively copying all grids
//   costs 80 * 250 KB = 19.5 MB per resampling step.
//
//   COW defers the copy: when a grid is "copied" (via the copy constructor),
//   it just shares a pointer to the same data. The actual 250 KB memcpy
//   only happens when one of the copies tries to WRITE to the grid
//   (via ensure_unique()). This is implemented using std::shared_ptr —
//   when the reference count is 1, the grid is exclusively owned and can
//   be written directly. When > 1, a real copy is made before writing.
//
// BUG FIXES vs Python:
//   #5: Adds bounds check on Bresenham hit point (occupancy_grid_new.py:36)
//   #6: Uses floor() for coordinate conversion (occupancy_grid_new.py:103)
//   Also: filters out-of-bounds hits instead of clipping to boundary,
//         preventing phantom walls at grid edges.
// ============================================================================

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>
#include <Eigen/Dense>

#include "slam_cpp/slam_types.hpp"

namespace slam {

// Simple (x, y) coordinate pair for grid-space positions (integer pixels).
struct GridCoord {
  int x;  // column index (horizontal)
  int y;  // row index (vertical)
};

class OccupancyGrid {
public:
  // -------------------------------------------------------------------------
  // Constructor: creates a zero-filled grid.
  //
  // Maps from: OccupancyGrid.__init__ (occupancy_grid_new.py:66-99)
  //
  // Parameters:
  //   width:      physical width in meters (e.g., 50.0)
  //   height:     physical height in meters (e.g., 50.0)
  //   resolution: meters per cell (e.g., 0.1 → 500 cells per 50m side)
  //   config:     provides log-odds scaling parameters (SCALE, lo_occupied, etc.)
  //
  // After construction:
  //   - Grid has rows_ * cols_ cells, all initialized to 0 (unknown)
  //   - origin_x_ = cols_/2, origin_y_ = rows_/2 (grid center = world origin)
  //   - Grid is exclusively owned (shared_ptr refcount = 1)
  //   - Incremental quality counters are zero (no known cells yet)
  // -------------------------------------------------------------------------
  OccupancyGrid(double width, double height, double resolution,
                const SlamConfig& config);

  // -------------------------------------------------------------------------
  // COW copy semantics (Rule of Five, all defaulted).
  //
  // Copy constructor/assignment: copies the shared_ptr (shares grid data).
  //   NO 250 KB memcpy — just a pointer copy + atomic refcount increment.
  //   Both the original and copy see the same grid cells until one mutates.
  //
  // Move constructor/assignment: transfers ownership of the shared_ptr.
  //   The source becomes empty (nullptr). Even cheaper than copy — no
  //   refcount change, just a pointer swap.
  //
  // Why all four are declared: Rule of Five — if you declare any special
  // member function, declare all of them to make intent explicit.
  // = default means "use the compiler-generated version, which is correct
  // for our shared_ptr-based COW design."
  // -------------------------------------------------------------------------
  OccupancyGrid(const OccupancyGrid& other) = default;
  OccupancyGrid& operator=(const OccupancyGrid& other) = default;
  OccupancyGrid(OccupancyGrid&& other) = default;
  OccupancyGrid& operator=(OccupancyGrid&& other) = default;

  // -------------------------------------------------------------------------
  // world_to_grid: converts world coordinates (meters) to grid pixels.
  //
  // Maps from: OccupancyGrid.world_to_grid (occupancy_grid_new.py:101-108)
  //
  // Returns std::optional<GridCoord>:
  //   - Has a value if the point is inside the grid → coord->x, coord->y
  //   - Is empty (std::nullopt) if the point is outside the grid
  //   Python equivalent: returns (gx, gy) or (None, None)
  //
  // BUG FIX #6: Uses std::floor() instead of Python's int() truncation.
  //   Python int(-0.5) = 0 (truncates toward zero — WRONG for grid mapping)
  //   std::floor(-0.5) = -1 (truncates toward -infinity — CORRECT)
  //   A point at x = -0.05m should map to the cell LEFT of origin, not ON it.
  // -------------------------------------------------------------------------
  std::optional<GridCoord> world_to_grid(double x, double y) const;

  // -------------------------------------------------------------------------
  // update: integrates a laser scan into the grid via Bresenham raycasting.
  //
  // Maps from: OccupancyGrid.update_with_numba (occupancy_grid_new.py:123-153)
  //
  // Algorithm for each laser beam:
  //   1. Transform the local hit point (robot frame) to world frame
  //      using the particle's pose (rotation + translation).
  //   2. Convert world coordinates to grid pixel coordinates.
  //   3. SKIP beams whose hit point falls outside the grid (instead of
  //      clipping to boundary, which would create phantom walls).
  //   4. Trace a Bresenham line from robot cell to hit cell:
  //      - Each traversed cell gets lo_free added (mark as free space)
  //      - The final hit cell gets lo_occupied added (mark as obstacle)
  //   5. Clamp all updated cells to [lo_min, lo_max] range.
  //
  // Automatically calls ensure_unique() before modifying the grid,
  // so COW sharing is handled transparently.
  // -------------------------------------------------------------------------
  void update(const Eigen::Vector3d& pose,
              const Eigen::Matrix<double, Eigen::Dynamic, 2>& local_scans);

  // -------------------------------------------------------------------------
  // ensure_unique: forces exclusive ownership of the grid data.
  //
  // If shared_ptr refcount > 1 (data is shared with other particles):
  //   → Allocates new memory, copies 250 KB of grid data, takes ownership.
  // If refcount == 1 (already exclusively owned):
  //   → Does nothing (no-op).
  //
  // WHEN TO CALL:
  //   - Automatically called by update() before any mutation.
  //   - Call manually before a parallel OpenMP section if you want all
  //     copies to happen sequentially (avoids concurrent shared_ptr ops).
  // -------------------------------------------------------------------------
  void ensure_unique();

  // True if refcount > 1 (data is shared with at least one other grid).
  bool is_shared() const;

  // -------------------------------------------------------------------------
  // Read-only accessors (do NOT trigger COW — no mutation).
  // -------------------------------------------------------------------------

  // Cell value at (row, col). row = y-axis, col = x-axis (row-major).
  // No bounds checking — caller must ensure indices are valid.
  int8_t at(int row, int col) const;

  // Raw pointer to the underlying int8 array.
  // Used by scan_matching functions for direct bilinear interpolation
  // without going through the OccupancyGrid API (for performance).
  // Layout: row-major, data[row * cols + col].
  const int8_t* data() const;

  // Grid geometry (all immutable after construction)
  int rows() const { return rows_; }
  int cols() const { return cols_; }
  int origin_x() const { return origin_x_; }
  int origin_y() const { return origin_y_; }
  double resolution() const { return resolution_; }

  // -------------------------------------------------------------------------
  // map_quality: fraction of "confident" cells that are occupied.
  //
  // Maps from: fast_slam_2.py:626-629
  //   Python: occupied / (occupied + free) where occupied = cells > 20
  //
  // OPTIMIZATION vs Python:
  //   Python scans all 250,000 cells every 5 SLAM iterations: O(250K).
  //   We maintain incremental counters (n_occupied, n_free) that update
  //   during each Bresenham raycast. This makes map_quality() O(1).
  //   The counters track cells crossing the confidence threshold
  //   (default: ±20 in scaled int8, = ±1.0 in log-odds).
  // -------------------------------------------------------------------------
  double map_quality() const;

private:
  // -------------------------------------------------------------------------
  // GridData: the actual grid storage, shared across COW copies.
  //
  // Held via shared_ptr. When refcount == 1, this particle exclusively
  // owns the data. When refcount > 1, multiple particles share it (and
  // ensure_unique() must be called before any mutation).
  // -------------------------------------------------------------------------
  struct GridData {
    std::vector<int8_t> cells;  // row-major, size = rows * cols (250,000 for 500x500)
    int n_occupied = 0;         // incremental: cells above +quality_threshold
    int n_free = 0;             // incremental: cells below -quality_threshold
  };

  std::shared_ptr<GridData> data_;

  // Grid geometry (set once in constructor, never changes)
  int rows_, cols_;
  double resolution_;
  double width_, height_;
  int origin_x_, origin_y_;  // pixel coordinates of world origin (0,0)

  // Cached log-odds parameters (derived from SlamConfig at construction time).
  // Cached here to avoid re-computing int8_t(float * scale) on every cell update.
  int8_t lo_occupied_;        // e.g., 17 = int8_t(0.85 * 20)
  int8_t lo_free_;            // e.g., -8 = int8_t(-0.4 * 20)
  int8_t lo_max_;             // e.g., 100 = int8_t(5.0 * 20)
  int8_t lo_min_;             // e.g., -100 = int8_t(-5.0 * 20)
  int8_t quality_threshold_;  // e.g., 20 = int8_t(1.0 * 20) — for map_quality counters

  // -------------------------------------------------------------------------
  // bresenham_raycast: traces one ray from (x0,y0) to (x1,y1) in grid space.
  //
  // Maps from: fast_raycast_update (occupancy_grid_new.py:6-63)
  //
  // Algorithm (Bresenham's line algorithm):
  //   - Walks cell-by-cell from the robot position to the laser hit point
  //   - Every cell it passes through is marked FREE (log-odds decreased)
  //   - The final cell (hit point) is marked OCCUPIED (log-odds increased)
  //   - Uses integer-only arithmetic (no floating point in the inner loop)
  //
  // BUG FIX #5: Both free cells AND the hit cell get bounds checks.
  //   Python only checks free cells (line 45) but not the hit (line 36).
  // -------------------------------------------------------------------------
  void bresenham_raycast(int x0, int y0, int x1, int y1);

  // -------------------------------------------------------------------------
  // update_cell: modifies one cell's log-odds and maintains quality counters.
  //
  // This is the ONLY function that writes to data_->cells[].
  // It handles:
  //   - Adding delta to the current cell value
  //   - Clamping the result to [lo_min_, lo_max_]
  //   - Tracking threshold crossings for incremental map_quality()
  //     (e.g., if a cell crosses from 19 to 25, n_occupied increments)
  // -------------------------------------------------------------------------
  void update_cell(int row, int col, int8_t delta);
};

}  // namespace slam

#endif  // SLAM_CPP__OCCUPANCY_GRID_HPP_
