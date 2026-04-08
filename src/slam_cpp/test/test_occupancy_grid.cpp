#include <gtest/gtest.h>
#include <cmath>
#include "slam_cpp/occupancy_grid.hpp"

// ============================================================================
// Test fixture: creates a default config and a 50m x 50m grid (500x500 cells).
// Reused across all occupancy grid tests to avoid boilerplate.
// ============================================================================
class OccupancyGridTest : public ::testing::Test {
protected:
  void SetUp() override {
    config.map_size = 50.0;
    config.map_resolution = 0.1;
    grid = std::make_unique<slam::OccupancyGrid>(
        config.map_size, config.map_size, config.map_resolution, config);
  }

  slam::SlamConfig config;
  std::unique_ptr<slam::OccupancyGrid> grid;
};

// --------------------------------------------------------------------------
// Test 1: Construction creates a zero-filled grid with correct dimensions.
// --------------------------------------------------------------------------
TEST_F(OccupancyGridTest, ConstructionCreatesZeroGrid)
{
  EXPECT_EQ(grid->rows(), 500);
  EXPECT_EQ(grid->cols(), 500);
  EXPECT_EQ(grid->origin_x(), 250);
  EXPECT_EQ(grid->origin_y(), 250);

  // All cells should be 0 (unknown).
  EXPECT_EQ(grid->at(0, 0), 0);
  EXPECT_EQ(grid->at(499, 499), 0);
  EXPECT_EQ(grid->at(250, 250), 0);
}

// --------------------------------------------------------------------------
// Test 2: world_to_grid converts world coordinates correctly.
// Tests Bug Fix #6: std::floor() vs Python int() truncation.
// --------------------------------------------------------------------------
TEST_F(OccupancyGridTest, WorldToGridCenter)
{
  // World origin (0,0) → grid center (250, 250).
  auto c = grid->world_to_grid(0.0, 0.0);
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->x, 250);
  EXPECT_EQ(c->y, 250);

  // 1.0m right of center: floor(1.0/0.1) + 250 = 10 + 250 = 260.
  auto r = grid->world_to_grid(1.0, 0.0);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->x, 260);
  EXPECT_EQ(r->y, 250);

  // Bug Fix #6: -0.05m should map to cell 249, NOT 250.
  // Python int(-0.05/0.1) = int(-0.5) = 0 → 0 + 250 = 250 (WRONG)
  // C++ floor(-0.05/0.1) = floor(-0.5) = -1 → -1 + 250 = 249 (CORRECT)
  auto neg = grid->world_to_grid(-0.05, 0.0);
  ASSERT_TRUE(neg.has_value());
  EXPECT_EQ(neg->x, 249);
  EXPECT_EQ(neg->y, 250);
}

// --------------------------------------------------------------------------
// Test 3: world_to_grid returns nullopt for out-of-bounds points.
// --------------------------------------------------------------------------
TEST_F(OccupancyGridTest, WorldToGridOutOfBounds)
{
  // 50m grid centered at origin → valid range is roughly [-25, +25).
  // 100m is way outside.
  EXPECT_FALSE(grid->world_to_grid(100.0, 0.0).has_value());
  EXPECT_FALSE(grid->world_to_grid(0.0, 100.0).has_value());
  EXPECT_FALSE(grid->world_to_grid(-26.0, 0.0).has_value());
  EXPECT_FALSE(grid->world_to_grid(0.0, -26.0).has_value());

  // Just inside the boundary should work.
  EXPECT_TRUE(grid->world_to_grid(24.9, 0.0).has_value());
  EXPECT_TRUE(grid->world_to_grid(-24.9, 0.0).has_value());
}

// --------------------------------------------------------------------------
// Test 4: Bresenham raycast marks correct cells (free + occupied).
// We call update() with a pose at origin and a single scan beam pointing
// right (+x). The ray should mark cells as free, and the hit cell as occupied.
// --------------------------------------------------------------------------
TEST_F(OccupancyGridTest, BresenhamDrawsCorrectLine)
{
  // Robot at origin, facing +x. One scan beam at (2.0, 0) in robot frame.
  Eigen::Vector3d pose(0.0, 0.0, 0.0);
  Eigen::Matrix<double, Eigen::Dynamic, 2> scans(1, 2);
  scans(0, 0) = 2.0;  // 2m in front
  scans(0, 1) = 0.0;

  grid->update(pose, scans);

  // Hit cell: world (2.0, 0) → grid (270, 250). Should be occupied (> 0).
  EXPECT_GT(grid->at(250, 270), 0);

  // A cell along the ray (e.g., grid (260, 250)) should be free (< 0).
  // World (1.0, 0) → grid (260, 250).
  EXPECT_LT(grid->at(250, 260), 0);

  // A cell far from the ray (e.g., grid (250, 300)) should be untouched (== 0).
  EXPECT_EQ(grid->at(300, 250), 0);
}

// --------------------------------------------------------------------------
// Test 5: Log-odds clamping prevents overflow beyond lo_max / lo_min.
// --------------------------------------------------------------------------
TEST_F(OccupancyGridTest, LogOddsClamping)
{
  // Repeatedly hit the same cell to drive it toward lo_max.
  Eigen::Vector3d pose(0.0, 0.0, 0.0);
  Eigen::Matrix<double, Eigen::Dynamic, 2> scans(1, 2);
  scans(0, 0) = 1.0;
  scans(0, 1) = 0.0;

  // Hit cell: world (1.0, 0) → grid col = floor(1.0/0.1)+250 = 260, row = 250.
  for (int i = 0; i < 30; ++i) {
    grid->update(pose, scans);
  }

  // Cell should be clamped to lo_max_scaled (= 5.0 * 20 = 100).
  int8_t hit_val = grid->at(250, 260);
  EXPECT_EQ(hit_val, config.lo_max_scaled());

  // A cell along the ray should be clamped to lo_min_scaled (= -5.0 * 20 = -100).
  // Cell at grid (255, 250) is between robot and hit: free 30 times.
  int8_t free_val = grid->at(250, 255);
  EXPECT_EQ(free_val, config.lo_min_scaled());
}

// --------------------------------------------------------------------------
// Test 6: COW shares data until a write triggers ensure_unique().
// --------------------------------------------------------------------------
TEST_F(OccupancyGridTest, COWSharesUntilWrite)
{
  // Copy the grid — should share the same underlying data.
  slam::OccupancyGrid copy(*grid);
  EXPECT_TRUE(grid->is_shared());
  EXPECT_TRUE(copy.is_shared());
  EXPECT_EQ(grid->data(), copy.data());  // same pointer

  // Read-only access should NOT trigger a copy.
  (void)copy.at(0, 0);
  EXPECT_EQ(grid->data(), copy.data());  // still same pointer

  // Writing to one should split them.
  Eigen::Vector3d pose(0.0, 0.0, 0.0);
  Eigen::Matrix<double, Eigen::Dynamic, 2> scans(1, 2);
  scans(0, 0) = 1.0;
  scans(0, 1) = 0.0;
  copy.update(pose, scans);

  EXPECT_NE(grid->data(), copy.data());   // different pointers now
  EXPECT_FALSE(grid->is_shared());
  EXPECT_FALSE(copy.is_shared());
  EXPECT_EQ(grid->at(250, 260), 0);       // original unchanged
  EXPECT_GT(copy.at(250, 260), 0);        // copy has the hit
}

// --------------------------------------------------------------------------
// Test 7: map_quality() returns 0 for an empty grid and > 0 after updates.
// --------------------------------------------------------------------------
TEST_F(OccupancyGridTest, MapQualityIncrementalCounters)
{
  // Empty grid: no cells above/below quality threshold → quality 0.
  EXPECT_DOUBLE_EQ(grid->map_quality(), 0.0);

  // Update with a scan to create some occupied and free cells.
  Eigen::Vector3d pose(0.0, 0.0, 0.0);
  Eigen::Matrix<double, Eigen::Dynamic, 2> scans(4, 2);
  scans << 3.0, 0.0,   // +x
           0.0, 3.0,   // +y
          -3.0, 0.0,   // -x
           0.0, -3.0;  // -y

  // Multiple updates to push cells past the quality threshold (±20 scaled).
  for (int i = 0; i < 5; ++i) {
    grid->update(pose, scans);
  }

  double quality = grid->map_quality();
  EXPECT_GT(quality, 0.0);
  EXPECT_LT(quality, 1.0);
}

// --------------------------------------------------------------------------
// Test 8: update() integrates a full scan from a known pose.
// --------------------------------------------------------------------------
TEST_F(OccupancyGridTest, UpdateIntegratesFullScan)
{
  // Robot at (1.0, 1.0, 0). Four beams in cardinal directions, each 2m.
  Eigen::Vector3d pose(1.0, 1.0, 0.0);
  Eigen::Matrix<double, Eigen::Dynamic, 2> scans(4, 2);
  scans << 2.0, 0.0,   // +x in robot frame → world (3.0, 1.0)
           0.0, 2.0,   // +y in robot frame → world (1.0, 3.0)
          -2.0, 0.0,   // -x in robot frame → world (-1.0, 1.0)
           0.0, -2.0;  // -y in robot frame → world (1.0, -1.0)

  grid->update(pose, scans);

  // Hit cells should be occupied (log-odds > 0).
  // world (3.0, 1.0) → grid col=floor(3.0/0.1)+250=280, row=floor(1.0/0.1)+250=260
  EXPECT_GT(grid->at(260, 280), 0);

  // world (1.0, 3.0) → grid col=260, row=280
  EXPECT_GT(grid->at(280, 260), 0);

  // Cells between robot and hit should be free (log-odds < 0).
  // Robot at world (1.0,1.0) → grid (260, 260).
  // Midpoint of +x beam: world (2.0, 1.0) → grid col=270, row=260.
  EXPECT_LT(grid->at(260, 270), 0);

  // Cells far from any ray should remain 0.
  EXPECT_EQ(grid->at(400, 400), 0);
}
