#include <gtest/gtest.h>
#include <cmath>
#include "slam_cpp/scan_matching.hpp"
#include "slam_cpp/occupancy_grid.hpp"
#include "slam_cpp/sigmoid_lut.hpp"

// ============================================================================
// Test fixture: provides a SlamConfig, SigmoidLUT, and helper to build
// a pre-populated grid for scan matching tests.
// ============================================================================
class ScanMatchingTest : public ::testing::Test {
protected:
  void SetUp() override {
    config.map_size = 10.0;       // 10m x 10m (smaller for faster tests)
    config.map_resolution = 0.1;  // 100 x 100 grid
    sigmoid = std::make_unique<slam::SigmoidLUT>(config.log_odds_scale);
  }

  // Helper: create a grid with a wall of occupied cells.
  // Places a horizontal wall at world y=2.0 (grid row 70), from x=-3 to x=3.
  slam::OccupancyGrid make_grid_with_wall()
  {
    slam::OccupancyGrid grid(config.map_size, config.map_size,
                             config.map_resolution, config);
    // Manually update cells along the wall to be highly occupied.
    // We use a scan from a pose directly below the wall.
    Eigen::Vector3d pose(0.0, 0.0, 0.0);
    Eigen::Matrix<double, Eigen::Dynamic, 2> scans(7, 2);
    for (int i = 0; i < 7; ++i) {
      scans(i, 0) = -3.0 + i * 1.0;  // x from -3 to 3
      scans(i, 1) = 2.0;              // all at y=2
    }
    // Hit the wall multiple times to build strong occupancy.
    for (int rep = 0; rep < 10; ++rep) {
      grid.update(pose, scans);
    }
    return grid;
  }

  slam::SlamConfig config;
  std::unique_ptr<slam::SigmoidLUT> sigmoid;
};

// --------------------------------------------------------------------------
// Test 1: extract_local_scan converts polar to Cartesian correctly.
// Also tests Bug Fix #7 (r > 0 filter) and NaN/inf filtering.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, ExtractLocalScanPolarToCartesian)
{
  // 5 beams from -pi/2 to +pi/2 (front semicircle), stride=1.
  int n = 5;
  float angle_min = -static_cast<float>(M_PI) / 2.0f;
  float angle_max = static_cast<float>(M_PI) / 2.0f;
  float range_max = 10.0f;

  // Ranges: [1.0, NaN, 2.0, 0.0, 5.0]
  // Expected valid: beam 0 (r=1.0), beam 2 (r=2.0), beam 4 (r=5.0)
  // beam 1 (NaN) and beam 3 (r=0.0, Bug Fix #7) should be filtered.
  std::vector<float> ranges = {1.0f, std::nanf(""), 2.0f, 0.0f, 5.0f};

  auto result = slam::extract_local_scan(
      ranges.data(), n, angle_min, angle_max, range_max, /*stride=*/1);

  EXPECT_EQ(result.rows(), 3);  // only 3 valid beams

  // Beam 0: angle = -pi/2, range = 1.0 → (0, -1)
  EXPECT_NEAR(result(0, 0), 0.0, 1e-6);   // x ≈ 0
  EXPECT_NEAR(result(0, 1), -1.0, 1e-6);  // y ≈ -1
}

// --------------------------------------------------------------------------
// Test 2: extract_local_scan respects stride parameter.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, ExtractLocalScanStride)
{
  int n = 9;
  float angle_min = 0.0f;
  float angle_max = static_cast<float>(M_PI);
  float range_max = 10.0f;
  std::vector<float> ranges(9, 1.0f);  // all valid, all 1.0m

  auto result = slam::extract_local_scan(
      ranges.data(), n, angle_min, angle_max, range_max, /*stride=*/3);

  // Beams at indices 0, 3, 6 should be kept (3 beams).
  EXPECT_EQ(result.rows(), 3);
}

// --------------------------------------------------------------------------
// Test 3: to_grid_coords scales and offsets correctly.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, ToGridCoordsScaling)
{
  Eigen::Matrix<double, Eigen::Dynamic, 2> pts(2, 2);
  pts << 1.0, 2.0,
         -0.5, 0.0;

  double inv_res = 10.0;  // 1/0.1
  double origin = 50.0;   // center of a 100-cell grid

  auto grid_pts = slam::to_grid_coords(pts, inv_res, origin);

  // (1.0, 2.0) → (1.0*10 + 50, 2.0*10 + 50) = (60, 70)
  EXPECT_NEAR(grid_pts(0, 0), 60.0, 1e-10);
  EXPECT_NEAR(grid_pts(0, 1), 70.0, 1e-10);

  // (-0.5, 0.0) → (-0.5*10 + 50, 0.0*10 + 50) = (45, 50)
  EXPECT_NEAR(grid_pts(1, 0), 45.0, 1e-10);
  EXPECT_NEAR(grid_pts(1, 1), 50.0, 1e-10);
}

// --------------------------------------------------------------------------
// Test 4: transform_scan_to_world applies rotation and translation.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, TransformScanToWorld)
{
  Eigen::Matrix<double, Eigen::Dynamic, 2> local(1, 2);
  local << 1.0, 0.0;  // 1m in front of robot

  // Pose at origin, facing +x (theta=0).
  Eigen::Vector3d pose_fwd(0.0, 0.0, 0.0);
  auto world_fwd = slam::transform_scan_to_world(local, pose_fwd);
  EXPECT_NEAR(world_fwd(0, 0), 1.0, 1e-10);
  EXPECT_NEAR(world_fwd(0, 1), 0.0, 1e-10);

  // Pose at origin, facing +y (theta=pi/2).
  Eigen::Vector3d pose_left(0.0, 0.0, M_PI / 2.0);
  auto world_left = slam::transform_scan_to_world(local, pose_left);
  EXPECT_NEAR(world_left(0, 0), 0.0, 1e-10);
  EXPECT_NEAR(world_left(0, 1), 1.0, 1e-10);

  // Pose at (5, 3), facing +x.
  Eigen::Vector3d pose_offset(5.0, 3.0, 0.0);
  auto world_offset = slam::transform_scan_to_world(local, pose_offset);
  EXPECT_NEAR(world_offset(0, 0), 6.0, 1e-10);
  EXPECT_NEAR(world_offset(0, 1), 3.0, 1e-10);
}

// --------------------------------------------------------------------------
// Test 5: bilinear_interpolation returns known values at grid cell centers.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, BilinearInterpolationKnownValues)
{
  // Create a small grid and populate one cell with a known value.
  slam::OccupancyGrid grid(config.map_size, config.map_size,
                           config.map_resolution, config);

  // Populate a cell by raycasting. After one hit, the hit cell has lo_occupied.
  Eigen::Vector3d pose(0.0, 0.0, 0.0);
  Eigen::Matrix<double, Eigen::Dynamic, 2> scans(1, 2);
  scans << 1.0, 0.0;
  grid.update(pose, scans);

  // Build GridView for querying.
  slam::GridView gv = slam::make_grid_view(grid, config, *sigmoid);

  // Query at the hit cell center: world (1.0, 0) → pixel (60, 50).
  // The cell at (row=50, col=60) should have lo_occupied value.
  Eigen::Matrix<double, Eigen::Dynamic, 2> query_pts(1, 2);
  query_pts << 60.0, 50.0;  // pixel coordinates

  auto result = slam::bilinear_interpolation(gv, query_pts, false);

  // The interpolated value should be the sigmoid of the cell's log-odds.
  // After one hit, the cell has lo_occupied_scaled = 17.
  // sigmoid(17) = 1/(1+exp(-17/20)) ≈ 0.701
  double expected = (*sigmoid)(config.lo_occupied_scaled());
  EXPECT_NEAR(result.values(0), expected, 0.05);
  EXPECT_EQ(result.mask(0), 1);  // valid (in-bounds)
}

// --------------------------------------------------------------------------
// Test 6: bilinear gradients match finite-difference approximation.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, BilinearGradientsMatchFiniteDifferences)
{
  // Build a grid with a wall so there are spatial gradients to measure.
  auto grid = make_grid_with_wall();
  slam::GridView gv = slam::make_grid_view(grid, config, *sigmoid);

  // Query a point near the wall edge where gradients should be non-zero.
  // Wall is at y=2.0 → pixel row = 70. Query just below: pixel (50, 69.5).
  Eigen::Matrix<double, Eigen::Dynamic, 2> pt(1, 2);
  pt << 50.0, 69.5;

  auto result = slam::bilinear_interpolation(gv, pt, /*compute_gradients=*/true);

  // Skip if point is out of bounds.
  if (result.mask(0) == 0) {
    GTEST_SKIP() << "Query point out of bounds";
  }

  // Finite difference approximation: f(x+h) - f(x-h)) / (2h)
  double h = 0.01;
  Eigen::Matrix<double, Eigen::Dynamic, 2> pt_xp(1, 2), pt_xm(1, 2);
  Eigen::Matrix<double, Eigen::Dynamic, 2> pt_yp(1, 2), pt_ym(1, 2);
  pt_xp << 50.0 + h, 69.5;
  pt_xm << 50.0 - h, 69.5;
  pt_yp << 50.0, 69.5 + h;
  pt_ym << 50.0, 69.5 - h;

  auto r_xp = slam::bilinear_interpolation(gv, pt_xp, false);
  auto r_xm = slam::bilinear_interpolation(gv, pt_xm, false);
  auto r_yp = slam::bilinear_interpolation(gv, pt_yp, false);
  auto r_ym = slam::bilinear_interpolation(gv, pt_ym, false);

  if (r_xp.mask(0) && r_xm.mask(0) && r_yp.mask(0) && r_ym.mask(0)) {
    double fd_gx = (r_xp.values(0) - r_xm.values(0)) / (2.0 * h);
    double fd_gy = (r_yp.values(0) - r_ym.values(0)) / (2.0 * h);

    EXPECT_NEAR(result.gradients(0, 0), fd_gx, 0.1);
    EXPECT_NEAR(result.gradients(0, 1), fd_gy, 0.1);
  }
}

// --------------------------------------------------------------------------
// Test 7: coarse_search finds a pose near the true pose on a known map.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, CoarseSearchFindsCorrectPose)
{
  // Build a grid with a known wall.
  auto grid = make_grid_with_wall();
  slam::GridView gv = slam::make_grid_view(grid, config, *sigmoid);

  // The scan that matches the wall from pose (0, 0, 0).
  Eigen::Matrix<double, Eigen::Dynamic, 2> scans(7, 2);
  for (int i = 0; i < 7; ++i) {
    scans(i, 0) = -3.0 + i * 1.0;
    scans(i, 1) = 2.0;
  }

  // Start from a slightly offset odometry pose.
  Eigen::Vector3d odom_pose(0.2, 0.1, 0.05);

  Eigen::Vector3d result = slam::coarse_search(
      gv, grid.map_quality(), scans, odom_pose, config);

  // Coarse search should find a pose reasonably close to origin.
  EXPECT_NEAR(result(0), 0.0, 0.5);  // x within 0.5m
  EXPECT_NEAR(result(1), 0.0, 0.5);  // y within 0.5m
}

// --------------------------------------------------------------------------
// Test 8: compute_joint_cost is lower at the true pose than at an offset.
// --------------------------------------------------------------------------
TEST_F(ScanMatchingTest, ComputeJointCostLowerAtTruePose)
{
  auto grid = make_grid_with_wall();
  slam::GridView gv = slam::make_grid_view(grid, config, *sigmoid);

  Eigen::Matrix<double, Eigen::Dynamic, 2> scans(7, 2);
  for (int i = 0; i < 7; ++i) {
    scans(i, 0) = -3.0 + i * 1.0;
    scans(i, 1) = 2.0;
  }

  Eigen::Vector3d true_pose(0.0, 0.0, 0.0);
  Eigen::Vector3d offset_pose(1.0, 0.5, 0.3);
  Eigen::Vector3d odom_pose(0.0, 0.0, 0.0);
  Eigen::Matrix3d odom_info = Eigen::Matrix3d::Identity();

  double cost_true = slam::compute_joint_cost(
      gv, scans, true_pose, odom_pose, odom_info, config.sigma_hit);
  double cost_offset = slam::compute_joint_cost(
      gv, scans, offset_pose, odom_pose, odom_info, config.sigma_hit);

  EXPECT_LT(cost_true, cost_offset);
}
