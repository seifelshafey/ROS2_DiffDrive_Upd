// slam_node.cpp — ROS 2 node for real-time FastSLAM 2.0.
// Subscribes to /odometry/filtered and /scan.
// Publishes /slam/map, /slam/particles, /slam/best_pose, and map->odom TF.
// Single-threaded executor; all callbacks are mutually exclusive.

#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "slam_cpp/slam_types.hpp"
#include "slam_cpp/particle_filter.hpp"
#include "slam_cpp/scan_matching.hpp"

namespace slam {

// Extract yaw from a quaternion (rotation about Z axis only).
static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
{
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// ============================================================================
// SlamNode — orchestrates the FastSLAM 2.0 pipeline.
//
// Callback flow:
//   odom_callback (~50-100 Hz) → accumulate dx, dy, dth
//   scan_callback (~10 Hz)     → decompose motion, predict, update_and_resample
//   publish_callback (2 Hz)    → publish map, particles, TF
// ============================================================================
class SlamNode : public rclcpp::Node
{
public:
  SlamNode()
  : Node("slam_node")
  {
    SlamConfig config = load_config();

    pf_ = std::make_unique<ParticleFilter>(config);
    RCLCPP_INFO(get_logger(), "ParticleFilter initialized: %d particles, %.1fm map, %.2fm/cell",
                config.num_particles, config.map_size, config.map_resolution);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&SlamNode::odom_callback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SlamNode::scan_callback, this, std::placeholders::_1));

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/slam/map", 1);
    particles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/slam/particles", 1);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/slam/best_pose", 1);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Use node clock (not wall clock) for sim time compatibility.
    double publish_rate = this->get_parameter("map_publish_rate").as_double();
    auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / publish_rate));
    publish_timer_ = rclcpp::create_timer(
        this, this->get_clock(), period,
        std::bind(&SlamNode::publish_callback, this));

    RCLCPP_INFO(get_logger(), "SlamNode ready. Subscribing to [%s] and [%s]",
                odom_topic_.c_str(), scan_topic_.c_str());
  }

private:
  // Load all ROS 2 parameters into SlamConfig.
  SlamConfig load_config()
  {
    SlamConfig c;
    c.num_particles       = this->declare_parameter("num_particles", c.num_particles);
    c.map_size            = this->declare_parameter("map_size", c.map_size);
    c.map_resolution      = this->declare_parameter("map_resolution", c.map_resolution);
    c.log_odds_scale      = this->declare_parameter("log_odds_scale", c.log_odds_scale);
    c.lo_occupied         = this->declare_parameter("lo_occupied", c.lo_occupied);
    c.lo_free             = this->declare_parameter("lo_free", c.lo_free);
    c.lo_max              = this->declare_parameter("lo_max", c.lo_max);
    c.lo_min              = this->declare_parameter("lo_min", c.lo_min);
    c.alpha1              = this->declare_parameter("alpha1", c.alpha1);
    c.alpha2              = this->declare_parameter("alpha2", c.alpha2);
    c.alpha3              = this->declare_parameter("alpha3", c.alpha3);
    c.alpha4              = this->declare_parameter("alpha4", c.alpha4);
    c.sigma_hit           = this->declare_parameter("sigma_hit", c.sigma_hit);
    c.sensor_confidence   = this->declare_parameter("sensor_confidence", c.sensor_confidence);
    c.score_stride        = this->declare_parameter("score_stride", c.score_stride);
    c.lm_max_iters        = this->declare_parameter("lm_max_iters", c.lm_max_iters);
    c.lm_max_sub_iters    = this->declare_parameter("lm_max_sub_iters", c.lm_max_sub_iters);
    c.mahalanobis_threshold = this->declare_parameter("mahalanobis_threshold", c.mahalanobis_threshold);
    c.lm_lambda_init      = this->declare_parameter("lm_lambda_init", c.lm_lambda_init);
    c.lm_lambda_min       = this->declare_parameter("lm_lambda_min", c.lm_lambda_min);
    c.lm_lambda_max       = this->declare_parameter("lm_lambda_max", c.lm_lambda_max);
    c.lm_lambda_up        = this->declare_parameter("lm_lambda_up", c.lm_lambda_up);
    c.lm_lambda_down      = this->declare_parameter("lm_lambda_down", c.lm_lambda_down);
    c.motion_threshold_trans = this->declare_parameter("motion_threshold_trans", c.motion_threshold_trans);
    c.motion_threshold_rot   = this->declare_parameter("motion_threshold_rot", c.motion_threshold_rot);
    c.map_update_dist     = this->declare_parameter("map_update_dist", c.map_update_dist);
    c.map_update_angle    = this->declare_parameter("map_update_angle", c.map_update_angle);
    c.obs_sigma_gain      = this->declare_parameter("obs_sigma_gain", c.obs_sigma_gain);
    c.temperature         = this->declare_parameter("temperature", c.temperature);
    c.scan_stride         = this->declare_parameter("scan_stride", c.scan_stride);
    c.quality_cache_interval = this->declare_parameter("quality_cache_interval", c.quality_cache_interval);

    odom_topic_ = this->declare_parameter("odom_topic", std::string("/odometry/filtered"));
    scan_topic_ = this->declare_parameter("scan_topic", std::string("/scan"));
    this->declare_parameter("map_publish_rate", 2.0);

    return c;
  }

  // Accumulate odometry deltas between laser scans.
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double curr_x = msg->pose.pose.position.x;
    double curr_y = msg->pose.pose.position.y;
    double curr_th = yaw_from_quaternion(msg->pose.pose.orientation);

    if (!odom_initialized_) {
      prev_x_ = curr_x;
      prev_y_ = curr_y;
      prev_th_ = curr_th;
      start_seg_th_ = curr_th;
      odom_initialized_ = true;
      return;
    }

    double dx = curr_x - prev_x_;
    double dy = curr_y - prev_y_;
    double dth = normalize_angle(curr_th - prev_th_);

    accum_dx_ += dx;
    accum_dy_ += dy;
    accum_dth_ += dth;
    accum_dth_ = normalize_angle(accum_dth_);

    prev_x_ = curr_x;
    prev_y_ = curr_y;
    prev_th_ = curr_th;

    // Latest odom for TF computation.
    odom_x_ = curr_x;
    odom_y_ = curr_y;
    odom_th_ = curr_th;
  }

  // Decompose accumulated motion, run SLAM if threshold met.
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!odom_initialized_) {
      return;
    }

    // Motion decomposition: rotation-translation-rotation.
    double delta_trans = std::sqrt(accum_dx_ * accum_dx_ + accum_dy_ * accum_dy_);

    double delta_rot1;
    if (delta_trans < 0.01) {
      delta_rot1 = 0.0;
    } else {
      delta_rot1 = normalize_angle(
          std::atan2(accum_dy_, accum_dx_) - start_seg_th_);
    }

    double delta_rot2 = normalize_angle(accum_dth_ - delta_rot1);

    // Reverse detection: if rot1 > 90 deg, robot is likely driving backward.
    double direction = 1.0;
    if (delta_trans > 0.01 && std::abs(delta_rot1) > (kPi / 2.0)) {
      direction = -1.0;
      delta_rot1 = normalize_angle(
          std::atan2(-accum_dy_, -accum_dx_) - start_seg_th_);
      delta_rot2 = normalize_angle(accum_dth_ - delta_rot1);
    }

    // Run SLAM only when motion exceeds threshold.
    if (delta_trans > pf_->config().motion_threshold_trans ||
        std::abs(accum_dth_) > pf_->config().motion_threshold_rot) {

      auto local_scans = extract_local_scan(
          msg->ranges.data(),
          static_cast<int>(msg->ranges.size()),
          msg->angle_min,
          msg->angle_max,
          msg->range_max,
          pf_->config().scan_stride);

      if (local_scans.rows() > 0) {
        pf_->predict(delta_trans, delta_rot1, delta_rot2, direction);
        pf_->update_and_resample(local_scans);
        slam_counter_++;

        // Throttled status log (at most once every 2 seconds).
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "SLAM #%d | best=%zu | Neff=%.1f | mq=%.4f",
            slam_counter_, pf_->best_particle_index(),
            1.0 / pf_->weights().squaredNorm(),
            pf_->grids()[pf_->best_particle_index()].map_quality());
      }

      // Reset accumulators for next segment.
      start_seg_th_ = prev_th_;
      accum_dx_ = 0.0;
      accum_dy_ = 0.0;
      accum_dth_ = 0.0;
    }
  }

  // Publish map, particles, best pose, and map->odom TF.
  void publish_callback()
  {
    if (slam_counter_ == 0) {
      return;
    }

    // Guard against sim time reset (Gazebo restart).
    rclcpp::Time now = this->get_clock()->now();
    if (last_publish_time_.nanoseconds() > 0 && now < last_publish_time_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Sim time went backward — skipping publish");
      return;
    }
    last_publish_time_ = now;

    size_t best_idx = pf_->best_particle_index();

    publish_map(now, best_idx);
    publish_particles(now);
    publish_best_pose(now, best_idx);
    broadcast_tf(now, best_idx);
  }

  // Convert best particle's grid to nav_msgs::msg::OccupancyGrid.
  // Log-odds [-100,100] mapped to ROS occupancy [-1, 0..100].
  // Published via std::move for zero-copy transfer to middleware.
  void publish_map(const rclcpp::Time& stamp, size_t best_idx)
  {
    const auto& grid = pf_->grids()[best_idx];
    const auto& config = pf_->config();

    auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    msg->header.stamp = stamp;
    msg->header.frame_id = "map";

    msg->info.resolution = static_cast<float>(config.map_resolution);
    msg->info.width = static_cast<uint32_t>(grid.cols());
    msg->info.height = static_cast<uint32_t>(grid.rows());

    double half = config.map_size / 2.0;
    msg->info.origin.position.x = -half;
    msg->info.origin.position.y = -half;
    msg->info.origin.position.z = 0.0;
    msg->info.origin.orientation.w = 1.0;

    int total_cells = grid.rows() * grid.cols();
    msg->data.resize(static_cast<size_t>(total_cells));
    const int8_t* raw = grid.data();

    for (int i = 0; i < total_cells; ++i) {
      int8_t log_odds = raw[i];
      if (log_odds == 0) {
        msg->data[static_cast<size_t>(i)] = -1;
      } else if (log_odds > 0) {
        int occ = 50 + static_cast<int>(log_odds) * 50 / 100;
        if (occ > 100) occ = 100;
        msg->data[static_cast<size_t>(i)] = static_cast<int8_t>(occ);
      } else {
        int occ = 50 + static_cast<int>(log_odds) * 50 / 100;
        if (occ < 0) occ = 0;
        msg->data[static_cast<size_t>(i)] = static_cast<int8_t>(occ);
      }
    }

    map_pub_->publish(std::move(msg));
  }

  void publish_particles(const rclcpp::Time& stamp)
  {
    auto msg = std::make_unique<geometry_msgs::msg::PoseArray>();
    msg->header.stamp = stamp;
    msg->header.frame_id = "map";

    const auto& poses = pf_->poses();
    int n = static_cast<int>(poses.cols());
    msg->poses.resize(static_cast<size_t>(n));

    for (int i = 0; i < n; ++i) {
      auto& p = msg->poses[static_cast<size_t>(i)];
      p.position.x = poses(0, i);
      p.position.y = poses(1, i);
      p.position.z = 0.0;
      double yaw = poses(2, i);
      p.orientation.w = std::cos(yaw / 2.0);
      p.orientation.z = std::sin(yaw / 2.0);
    }

    particles_pub_->publish(std::move(msg));
  }

  void publish_best_pose(const rclcpp::Time& stamp, size_t best_idx)
  {
    const auto& poses = pf_->poses();
    Eigen::Index idx = static_cast<Eigen::Index>(best_idx);

    auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    msg->header.stamp = stamp;
    msg->header.frame_id = "map";

    msg->pose.position.x = poses(0, idx);
    msg->pose.position.y = poses(1, idx);
    double yaw = poses(2, idx);
    msg->pose.orientation.w = std::cos(yaw / 2.0);
    msg->pose.orientation.z = std::sin(yaw / 2.0);

    pose_pub_->publish(std::move(msg));
  }

  // Broadcast map->odom TF.
  // T(map->odom) = T(map->base_link) * T(odom->base_link)^-1
  // = slam_pose * inverse(odom_pose)
  void broadcast_tf(const rclcpp::Time& stamp, size_t best_idx)
  {
    const auto& poses = pf_->poses();
    Eigen::Index idx = static_cast<Eigen::Index>(best_idx);

    double slam_x = poses(0, idx);
    double slam_y = poses(1, idx);
    double slam_th = poses(2, idx);

    double tf_th = slam::normalize_angle(slam_th - odom_th_);
    double cos_tf = std::cos(tf_th);
    double sin_tf = std::sin(tf_th);
    double tf_x = slam_x - (odom_x_ * cos_tf - odom_y_ * sin_tf);
    double tf_y = slam_y - (odom_x_ * sin_tf + odom_y_ * cos_tf);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "map";
    t.child_frame_id = "odom";

    t.transform.translation.x = tf_x;
    t.transform.translation.y = tf_y;
    t.transform.translation.z = 0.0;

    t.transform.rotation.w = std::cos(tf_th / 2.0);
    t.transform.rotation.z = std::sin(tf_th / 2.0);

    tf_broadcaster_->sendTransform(t);
  }

  // --- Members ---
  std::unique_ptr<ParticleFilter> pf_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string odom_topic_;
  std::string scan_topic_;

  // Odometry accumulation (single-threaded: no synchronization needed).
  bool odom_initialized_ = false;
  double prev_x_ = 0.0, prev_y_ = 0.0, prev_th_ = 0.0;
  double start_seg_th_ = 0.0;
  double accum_dx_ = 0.0, accum_dy_ = 0.0, accum_dth_ = 0.0;
  double odom_x_ = 0.0, odom_y_ = 0.0, odom_th_ = 0.0;

  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  int slam_counter_ = 0;
};

}  // namespace slam

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam::SlamNode>());
  rclcpp::shutdown();
  return 0;
}
