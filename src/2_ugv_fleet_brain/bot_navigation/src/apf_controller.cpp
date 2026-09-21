// APF Controller implementation.
//
// Reactive obstacle avoidance via Artificial Potential Fields. No costmap is
// consulted — the controller reads sensors directly.
//
// Obstacle input: /scan/obstacles (pre-filtered by ugv_obstacle via DEM-prior
// ground segmentation). This cloud already has dune/slope ground returns
// removed; only genuine obstacles (rocks, bushes, vehicles) remain.
//
// CRITICAL: The point cloud is in sensor frame (laser_link), which is mounted
// at (x=0.60, z=0.84) above base_footprint. All height thresholds are
// specified in "meters above ground" and internally converted to sensor frame:
//     z_sensor = z_ground - sensor_height_
//
// Secondary safety net: per-sector adaptive ground estimation catches leaked
// ground points from ugv_obstacle edge cases (low TRN confidence, close range).
//
// Force model:
//   F_total = F_attractive(goal) + Σ F_repulsive(obstacles) + F_slope(IMU pitch)

#include "bot_navigation/apf_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <pluginlib/class_list_macros.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <nav2_util/node_utils.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/utils.h>

namespace bot_navigation {

// Number of angular sectors for per-sector ground estimation.
// 36 sectors = 10° each. Enough resolution to separate a narrow rock from
// the surrounding slope while keeping computation trivial.
static constexpr int NUM_SECTORS = 36;
static constexpr double SECTOR_WIDTH = 2.0 * M_PI / NUM_SECTORS;

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
  // Sensor geometry from URDF
  nav2_util::declare_parameter_if_not_declared(node, name + ".sensor_height",
    rclcpp::ParameterValue(0.84));
  nav2_util::declare_parameter_if_not_declared(node, name + ".sensor_x_offset",
    rclcpp::ParameterValue(0.60));
  // Obstacle height thresholds — in meters above GROUND (not sensor frame)
  nav2_util::declare_parameter_if_not_declared(node, name + ".obstacle_height_min",
    rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(node, name + ".obstacle_height_max",
    rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".obstacle_range_max",
    rclcpp::ParameterValue(8.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ground_clearance",
    rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(node, name + ".scan_topic",
    rclcpp::ParameterValue(std::string("/scan/obstacles")));
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
  node->get_parameter(name + ".sensor_height", sensor_height_);
  node->get_parameter(name + ".sensor_x_offset", sensor_x_offset_);
  node->get_parameter(name + ".obstacle_height_min", obstacle_height_min_);
  node->get_parameter(name + ".obstacle_height_max", obstacle_height_max_);
  node->get_parameter(name + ".obstacle_range_max", obstacle_range_max_);
  node->get_parameter(name + ".ground_clearance", ground_clearance_);
  node->get_parameter(name + ".scan_topic", scan_topic_);
  node->get_parameter(name + ".imu_topic", imu_topic_);
  node->get_parameter(name + ".transform_tolerance", transform_tolerance_);

  // Compute max steering angle from Ackermann geometry: δ_max = atan(L / R_min)
  max_steer_angle_ = std::atan(wheelbase_ / min_turning_radius_);

  RCLCPP_INFO(logger_, "APFController configured:");
  RCLCPP_INFO(logger_, "  Forces: k_att=%.2f k_rep=%.2f d0=%.1fm k_slope=%.2f pitch_thresh=%.1f°",
              k_att_, k_rep_, influence_distance_, k_slope_, pitch_threshold_ * 180.0 / M_PI);
  RCLCPP_INFO(logger_, "  Ackermann: max_vel=%.2f R_min=%.2fm L=%.2fm δ_max=%.1f°",
              max_linear_vel_, min_turning_radius_, wheelbase_, max_steer_angle_ * 180.0 / M_PI);
  RCLCPP_INFO(logger_, "  Sensor: height=%.2fm x_offset=%.2fm scan=%s",
              sensor_height_, sensor_x_offset_, scan_topic_.c_str());
  RCLCPP_INFO(logger_, "  Obstacle: h_min=%.2fm h_max=%.2fm (above ground) → "
              "z_sensor=[%.2f, %.2f] ground_clearance=%.2fm",
              obstacle_height_min_, obstacle_height_max_,
              obstacle_height_min_ - sensor_height_,
              obstacle_height_max_ - sensor_height_,
              ground_clearance_);
}

void APFController::activate() {
  auto node = node_.lock();
  if (!node) return;

  // Subscribe to obstacle-filtered point cloud (ground already removed by
  // ugv_obstacle via DEM-prior differencing). Falls back gracefully if the
  // topic is raw /scan/points — the per-sector ground filter handles it.
  scan_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&APFController::scanCallback, this, std::placeholders::_1));

  // Subscribe to IMU for slope/pitch detection
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

  // ---- 2. Repulsive force (from obstacles, returned in base_footprint frame) ----
  std::vector<Vec2> obs_robot;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if (latest_scan_) {
      obs_robot = extractObstacles2D(*latest_scan_);
    }
  }
  Vec2 f_rep = computeRepulsive(rx, ry, ryaw, obs_robot);

  // ---- 3. Slope force (from IMU, in robot frame) ----
  double pitch;
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    pitch = latest_pitch_;
  }
  Vec2 f_slope = computeSlopeForce(pitch);

  // ---- Sum forces ----
  // f_att is in map frame; f_rep and f_slope are in robot frame.
  // Convert f_rep and f_slope to map frame.
  Vec2 f_rep_map;
  f_rep_map.x = f_rep.x * std::cos(ryaw) - f_rep.y * std::sin(ryaw);
  f_rep_map.y = f_rep.x * std::sin(ryaw) + f_rep.y * std::cos(ryaw);

  Vec2 f_slope_map;
  f_slope_map.x = f_slope.x * std::cos(ryaw) - f_slope.y * std::sin(ryaw);
  f_slope_map.y = f_slope.x * std::sin(ryaw) + f_slope.y * std::cos(ryaw);

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

  // Velocity scales with:
  //   - cos(heading_error): slow down for large heading corrections
  //   - proximity to obstacles: natural from force magnitude
  double f_mag = std::sqrt(fx_robot * fx_robot + fy_robot * fy_robot);
  double heading_factor = std::max(0.0, std::cos(desired_heading));

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

  // CRITICAL: Always maintain a minimum creep velocity when we have a goal.
  // Without this, when the goal is behind the robot (heading error > 90°),
  // cos(heading) < 0 → velocity = 0 → robot sits still forever → progress
  // checker fires → abort loop. With a small creep velocity and max steering,
  // the Ackermann arc slowly turns the robot toward the goal.
  //
  // This also prevents the progress checker from declaring "no progress" due
  // to TRN localization drift where the goal direction fluctuates randomly.
  constexpr double MIN_CREEP_VEL = 0.08;  // m/s — enough to satisfy progress checker
  if (linear_vel < MIN_CREEP_VEL && f_mag > 0.05) {
    linear_vel = MIN_CREEP_VEL;
  }

  // Angular velocity from Ackermann: ω = v * tan(δ) / L
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
  return {-mag, 0.0};
}

// ---- Obstacle Extraction with Per-Sector Adaptive Ground ----

std::vector<APFController::Vec2> APFController::extractObstacles2D(
    const sensor_msgs::msg::PointCloud2 & cloud) const {
  // Two-pass obstacle extraction with per-sector ground estimation.
  //
  // IMPORTANT: The point cloud is in SENSOR FRAME (laser_link), which is
  // mounted at z = sensor_height_ (0.84m) above ground and x = sensor_x_offset_
  // (0.60m) forward of base_footprint center.
  //
  // In sensor frame:
  //   - Flat ground appears at z ≈ -sensor_height_ (-0.84m)
  //   - A 0.3m rock appears at z ≈ 0.3 - sensor_height_ = -0.54m
  //   - A 1.0m bush appears at z ≈ 1.0 - sensor_height_ = +0.16m
  //
  // obstacle_height_min/max are specified in "meters above ground" and
  // converted to sensor frame here:
  //   z_sensor_min = obstacle_height_min_ - sensor_height_
  //   z_sensor_max = obstacle_height_max_ - sensor_height_
  //
  // The sensor x-offset is added to each point's x to convert from
  // sensor frame to approximate base_footprint frame for the 2D obstacle
  // positions used by the repulsive force computation.
  //
  // Pass 1: For each angular sector, find minimum z → ground proxy
  // Pass 2: Accept points whose z > sector_ground + ground_clearance

  // Convert height thresholds from ground frame to sensor frame
  const float z_sensor_min = static_cast<float>(obstacle_height_min_ - sensor_height_);
  const float z_sensor_max = static_cast<float>(obstacle_height_max_ - sensor_height_);
  // Ground level in sensor frame (used for coarse pre-filter)
  const float z_ground_sensor = static_cast<float>(-sensor_height_);

  struct SectorPoint {
    float x, y, z;    // in sensor frame
    float bx, by;     // in base_footprint frame (2D)
    double range2;     // 2D range from base_footprint center
  };

  // Collect all valid points and assign to sectors
  std::array<float, NUM_SECTORS> sector_min_z;
  sector_min_z.fill(std::numeric_limits<float>::infinity());

  std::vector<SectorPoint> all_points;
  std::vector<int> point_sectors;

  const double range_max2 = obstacle_range_max_ * obstacle_range_max_;
  const double range_min2 = 0.5 * 0.5;  // self-returns exclusion (from base_footprint center)

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  all_points.reserve(cloud.width * cloud.height / 4);
  point_sectors.reserve(cloud.width * cloud.height / 4);

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float sx = *iter_x;  // sensor frame x
    const float sy = *iter_y;  // sensor frame y
    const float sz = *iter_z;  // sensor frame z

    // Skip NaN
    if (!std::isfinite(sx) || !std::isfinite(sy) || !std::isfinite(sz)) continue;

    // Coarse height filter in sensor frame:
    // Reject points clearly below ground (z < ground - 1m margin) or above max
    if (sz < z_ground_sensor - 1.0f || sz > z_sensor_max) continue;

    // Convert to approximate base_footprint frame (2D):
    //   bx = sensor_x + sensor_x_offset  (LiDAR is forward of center)
    //   by = sensor_y                      (LiDAR is on centerline)
    const float bx = sx + static_cast<float>(sensor_x_offset_);
    const float by = sy;

    // Range filter from base_footprint center
    double r2 = static_cast<double>(bx) * bx + static_cast<double>(by) * by;
    if (r2 > range_max2 || r2 < range_min2) continue;

    // Determine angular sector using base_footprint coordinates
    // (so sectors are relative to robot heading, not sensor position)
    double angle = std::atan2(static_cast<double>(by), static_cast<double>(bx));
    int sector = static_cast<int>((angle + M_PI) / SECTOR_WIDTH);
    sector = std::clamp(sector, 0, NUM_SECTORS - 1);

    all_points.push_back({sx, sy, sz, bx, by, r2});
    point_sectors.push_back(sector);

    // Pass 1: track minimum z per sector (ground proxy, in sensor frame)
    if (sz < sector_min_z[sector]) {
      sector_min_z[sector] = sz;
    }
  }

  // Pass 2: accept only points significantly above sector ground
  std::vector<Vec2> obstacles;
  obstacles.reserve(all_points.size() / 4);

  for (size_t i = 0; i < all_points.size(); ++i) {
    const auto & pt = all_points[i];
    const int sector = point_sectors[i];
    const float ground_z = sector_min_z[sector];

    // Adaptive threshold: sector ground + ground_clearance (in sensor frame)
    float threshold = std::isfinite(ground_z)
      ? ground_z + static_cast<float>(ground_clearance_)
      : z_sensor_min;

    // Point must be ABOVE the adaptive ground + clearance threshold
    if (pt.z < threshold) continue;

    // Also enforce the absolute minimum height (belt-and-suspenders)
    // This is in sensor frame: z_sensor_min = obstacle_height_min - sensor_height
    if (pt.z < z_sensor_min) continue;

    // Output in base_footprint 2D frame (for repulsive force computation)
    obstacles.push_back({static_cast<double>(pt.bx), static_cast<double>(pt.by)});
  }

  return obstacles;
}

}  // namespace bot_navigation

// Register as Nav2 controller plugin
PLUGINLIB_EXPORT_CLASS(bot_navigation::APFController, nav2_core::Controller)
