// APF Controller implementation.
//
// Reactive obstacle avoidance via Artificial Potential Fields. Reads raw 3D
// LiDAR (projected to 2D) and IMU pitch directly — no costmap is consulted.
//
// Force model:
//   F_total = F_attractive(goal) + Σ F_repulsive(obstacles) + F_slope(IMU pitch)
//
// The resultant force vector is converted to an Ackermann-compatible Twist
// (linear.x, angular.z) respecting minimum turning radius.

#include "bot_navigation/apf_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <pluginlib/class_list_macros.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <nav2_util/node_utils.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/utils.h>

namespace bot_navigation {

void APFController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/) {
  node_ = parent;
  tf_ = tf;
  plugin_name_ = name;

  auto node = node_.lock();
  if (!node) return;
  logger_ = node->get_logger();

  // Declare and read parameters
  nav2_util::declare_parameter_if_not_declared(node, name + ".k_att",
    rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".k_rep",
    rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(node, name + ".influence_distance",
    rclcpp::ParameterValue(4.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".k_slope",
    rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".pitch_threshold",
    rclcpp::ParameterValue(0.21));
  nav2_util::declare_parameter_if_not_declared(node, name + ".max_linear_vel",
    rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(node, name + ".min_turning_radius",
    rclcpp::ParameterValue(3.36));
  nav2_util::declare_parameter_if_not_declared(node, name + ".wheelbase",
    rclcpp::ParameterValue(0.9));
  nav2_util::declare_parameter_if_not_declared(node, name + ".obstacle_height_min",
    rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(node, name + ".obstacle_height_max",
    rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".obstacle_range_max",
    rclcpp::ParameterValue(8.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".scan_topic",
    rclcpp::ParameterValue(std::string("/scan/points")));
  nav2_util::declare_parameter_if_not_declared(node, name + ".imu_topic",
    rclcpp::ParameterValue(std::string("/imu")));
  nav2_util::declare_parameter_if_not_declared(node, name + ".transform_tolerance",
    rclcpp::ParameterValue(2.0));

  node->get_parameter(name + ".k_att", k_att_);
  node->get_parameter(name + ".k_rep", k_rep_);
  node->get_parameter(name + ".influence_distance", influence_distance_);
  node->get_parameter(name + ".k_slope", k_slope_);
  node->get_parameter(name + ".pitch_threshold", pitch_threshold_);
  node->get_parameter(name + ".max_linear_vel", max_linear_vel_);
  node->get_parameter(name + ".min_turning_radius", min_turning_radius_);
  node->get_parameter(name + ".wheelbase", wheelbase_);
  node->get_parameter(name + ".obstacle_height_min", obstacle_height_min_);
  node->get_parameter(name + ".obstacle_height_max", obstacle_height_max_);
  node->get_parameter(name + ".obstacle_range_max", obstacle_range_max_);
  node->get_parameter(name + ".scan_topic", scan_topic_);
  node->get_parameter(name + ".imu_topic", imu_topic_);
  node->get_parameter(name + ".transform_tolerance", transform_tolerance_);

  // Compute max steering angle from Ackermann geometry: δ_max = atan(L / R_min)
  max_steer_angle_ = std::atan(wheelbase_ / min_turning_radius_);

  RCLCPP_INFO(logger_, "APFController configured: k_att=%.2f k_rep=%.2f d0=%.1fm "
              "k_slope=%.2f pitch_thresh=%.1f° max_vel=%.2f R_min=%.2fm δ_max=%.1f°",
              k_att_, k_rep_, influence_distance_,
              k_slope_, pitch_threshold_ * 180.0 / M_PI,
              max_linear_vel_, min_turning_radius_,
              max_steer_angle_ * 180.0 / M_PI);
}

void APFController::activate() {
  auto node = node_.lock();
  if (!node) return;

  // Subscribe to raw 3D LiDAR
  scan_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&APFController::scanCallback, this, std::placeholders::_1));

  // Subscribe to IMU for pitch
  imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(),
    std::bind(&APFController::imuCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "APFController activated: listening on %s and %s",
              scan_topic_.c_str(), imu_topic_.c_str());
}

void APFController::deactivate() {
  scan_sub_.reset();
  imu_sub_.reset();
}

void APFController::cleanup() {
  scan_sub_.reset();
  imu_sub_.reset();
}

void APFController::setPlan(const nav_msgs::msg::Path & path) {
  if (path.poses.empty()) {
    goal_set_ = false;
    return;
  }
  // Use the LAST pose in the path as the goal (straight-line from GoalToPath)
  goal_ = path.poses.back();
  goal_set_ = true;
}

void APFController::setSpeedLimit(const double & speed_limit, const bool & percentage) {
  speed_limit_ = speed_limit;
  speed_limit_is_pct_ = percentage;
}

// ---- Sensor Callbacks ----

void APFController::scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_ = msg;
}

void APFController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Extract pitch from IMU quaternion
  const auto & q = msg->orientation;
  // pitch = asin(2*(qw*qy - qz*qx))
  double siny_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  double cosy_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  double roll = std::atan2(siny_cosp, cosy_cosp);

  double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  double pitch = (std::abs(sinp) >= 1.0)
    ? std::copysign(M_PI / 2.0, sinp)
    : std::asin(sinp);
  (void)roll;  // not used currently

  std::lock_guard<std::mutex> lock(imu_mutex_);
  latest_pitch_ = pitch;
}

// ---- Main Control Loop ----

geometry_msgs::msg::TwistStamped APFController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & /*velocity*/,
    nav2_core::GoalChecker * /*goal_checker*/) {
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = pose.header.stamp;
  cmd.header.frame_id = pose.header.frame_id;

  if (!goal_set_) {
    RCLCPP_WARN_THROTTLE(logger_, *node_.lock()->get_clock(), 2000,
                          "APF: no goal set, commanding zero velocity");
    return cmd;
  }

  // Robot pose in map frame
  const double rx = pose.pose.position.x;
  const double ry = pose.pose.position.y;
  const double ryaw = tf2::getYaw(pose.pose.orientation);

  // Goal in map frame
  const double gx = goal_.pose.position.x;
  const double gy = goal_.pose.position.y;

  // ---- 1. Attractive force (in map frame) ----
  Vec2 f_att = computeAttractive(rx, ry, gx, gy);

  // ---- 2. Repulsive force (from obstacles in robot frame) ----
  std::vector<Vec2> obs_robot;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if (latest_scan_) {
      obs_robot = extractObstacles2D(*latest_scan_);
    }
  }
  Vec2 f_rep = computeRepulsive(rx, ry, ryaw, obs_robot);

  // ---- 3. Slope force (from IMU) ----
  double pitch;
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    pitch = latest_pitch_;
  }
  Vec2 f_slope = computeSlopeForce(pitch);

  // ---- Sum forces ----
  // f_att and f_rep are in map frame; f_slope is in robot frame.
  // Convert f_slope to map frame.
  Vec2 f_slope_map;
  f_slope_map.x = f_slope.x * std::cos(ryaw) - f_slope.y * std::sin(ryaw);
  f_slope_map.y = f_slope.x * std::sin(ryaw) + f_slope.y * std::cos(ryaw);

  // Also convert f_rep from robot frame to map frame
  Vec2 f_rep_map;
  f_rep_map.x = f_rep.x * std::cos(ryaw) - f_rep.y * std::sin(ryaw);
  f_rep_map.y = f_rep.x * std::sin(ryaw) + f_rep.y * std::cos(ryaw);

  Vec2 f_total;
  f_total.x = f_att.x + f_rep_map.x + f_slope_map.x;
  f_total.y = f_att.y + f_rep_map.y + f_slope_map.y;

  // ---- Convert to robot frame ----
  double fx_robot = f_total.x * std::cos(ryaw) + f_total.y * std::sin(ryaw);
  double fy_robot = -f_total.x * std::sin(ryaw) + f_total.y * std::cos(ryaw);

  // ---- Force → Ackermann command ----
  double desired_heading = std::atan2(fy_robot, fx_robot);

  // Clamp steering angle to Ackermann limits
  double steer = std::clamp(desired_heading, -max_steer_angle_, max_steer_angle_);

  // Angular velocity from Ackermann: ω = v * tan(δ) / L
  // Velocity scales with:
  //   - cos(heading_error): slow down for large heading corrections
  //   - proximity to obstacles: natural from force magnitude
  double f_mag = std::sqrt(fx_robot * fx_robot + fy_robot * fy_robot);
  double heading_factor = std::max(0.0, std::cos(desired_heading));

  // If the goal is directly behind (heading error > 90°), stop and let the
  // Ackermann arc bring us around rather than reversing.
  double linear_vel = max_linear_vel_ * heading_factor;

  // Apply speed limit override if set
  if (speed_limit_ > 0.0) {
    double limit = speed_limit_is_pct_
      ? max_linear_vel_ * speed_limit_ / 100.0
      : speed_limit_;
    linear_vel = std::min(linear_vel, limit);
  }

  // Scale velocity down when force magnitude is very small (near equilibrium /
  // trapped between obstacles). This provides natural slow-down near obstacles.
  double vel_scale = std::min(1.0, f_mag / k_att_);
  linear_vel *= vel_scale;

  // Minimum velocity to avoid getting stuck (if we have a goal and heading is OK)
  if (heading_factor > 0.3 && linear_vel < 0.05 && f_mag > 0.1) {
    linear_vel = 0.05;
  }

  double angular_vel = linear_vel * std::tan(steer) / wheelbase_;

  cmd.twist.linear.x = linear_vel;
  cmd.twist.angular.z = angular_vel;

  RCLCPP_DEBUG(logger_, "APF: att=(%.2f,%.2f) rep=(%.2f,%.2f) slope=(%.2f,%.2f) "
               "-> v=%.2f ω=%.2f δ=%.1f° obs=%zu pitch=%.1f°",
               f_att.x, f_att.y, f_rep.x, f_rep.y, f_slope.x, f_slope.y,
               linear_vel, angular_vel, steer * 180.0 / M_PI,
               obs_robot.size(), pitch * 180.0 / M_PI);

  return cmd;
}

// ---- Force Computation ----

APFController::Vec2 APFController::computeAttractive(
    double rx, double ry, double gx, double gy) const {
  double dx = gx - rx;
  double dy = gy - ry;
  double dist = std::sqrt(dx * dx + dy * dy);
  if (dist < 0.01) return {0.0, 0.0};  // at goal

  // Constant-magnitude unit vector toward goal (capped attractive field
  // prevents runaway velocity on distant goals)
  return {k_att_ * dx / dist, k_att_ * dy / dist};
}

APFController::Vec2 APFController::computeRepulsive(
    double /*rx*/, double /*ry*/, double /*ryaw*/,
    const std::vector<Vec2> & obstacles_robot_frame) const {
  Vec2 f_total{0.0, 0.0};
  const double d0 = influence_distance_;

  for (const auto & obs : obstacles_robot_frame) {
    double dist = std::sqrt(obs.x * obs.x + obs.y * obs.y);
    if (dist < 0.01) dist = 0.01;  // avoid division by zero
    if (dist > d0) continue;        // outside influence

    // Repulsive magnitude: k_rep * (1/d - 1/d0) / d²
    double inv_d = 1.0 / dist;
    double inv_d0 = 1.0 / d0;
    double mag = k_rep_ * (inv_d - inv_d0) * inv_d * inv_d;

    // Direction: from obstacle toward robot (i.e., away from obstacle)
    // In robot frame, obstacle is at (obs.x, obs.y), so push direction
    // is (-obs.x, -obs.y) normalized
    double nx = -obs.x / dist;
    double ny = -obs.y / dist;

    f_total.x += mag * nx;
    f_total.y += mag * ny;
  }

  return f_total;
}

APFController::Vec2 APFController::computeSlopeForce(double pitch) const {
  // When pitch exceeds threshold, push backward (negative x in robot frame)
  // to make the robot steer away from steep inclines
  double excess = std::abs(pitch) - pitch_threshold_;
  if (excess <= 0.0) return {0.0, 0.0};

  // Backward force proportional to how much pitch exceeds threshold
  double mag = k_slope_ * excess;
  // Push backward in robot frame (negative x)
  return {-mag, 0.0};
}

// ---- Obstacle Extraction ----

std::vector<APFController::Vec2> APFController::extractObstacles2D(
    const sensor_msgs::msg::PointCloud2 & cloud) const {
  std::vector<Vec2> obstacles;

  // The point cloud comes in the sensor frame (laser_link). We use the raw
  // (x, y, z) in sensor frame and filter by height band. Since laser_link
  // is roughly horizontal and mounted on the robot chassis, the z-axis
  // in sensor frame roughly corresponds to vertical.
  //
  // Points between obstacle_height_min_ and obstacle_height_max_ (in sensor z)
  // are projected to 2D (x, y) as obstacle positions in sensor/robot frame.
  //
  // Note: sensor frame ≈ robot frame for our purposes (small offset). For a
  // precise implementation we'd transform to base_footprint, but the latency
  // savings of using raw sensor frame are worth the ~0.5m offset.

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  const double range_max2 = obstacle_range_max_ * obstacle_range_max_;

  obstacles.reserve(cloud.width * cloud.height / 10);  // rough estimate

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float x = *iter_x;
    const float y = *iter_y;
    const float z = *iter_z;

    // Skip NaN
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

    // Height band filter (in sensor frame, z is roughly vertical)
    if (z < obstacle_height_min_ || z > obstacle_height_max_) continue;

    // Range filter
    double r2 = static_cast<double>(x) * x + static_cast<double>(y) * y;
    if (r2 > range_max2 || r2 < 0.5 * 0.5) continue;  // skip self-returns < 0.5m

    obstacles.push_back({static_cast<double>(x), static_cast<double>(y)});
  }

  return obstacles;
}

}  // namespace bot_navigation

// Register as Nav2 controller plugin
PLUGINLIB_EXPORT_CLASS(bot_navigation::APFController, nav2_core::Controller)
