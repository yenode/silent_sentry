// Artificial Potential Field (APF) controller plugin for Nav2.
//
// Replaces RPP + costmap collision detection with a reactive controller that
// reads raw 3D LiDAR and IMU directly. Three force components:
//   1. Attractive: unit vector toward goal
//   2. Repulsive: inverse-square push from nearby obstacle points (2D projected)
//   3. Slope: IMU pitch-based repulsion to avoid non-traversable dunes
//
// Resultant force is converted to Ackermann-compatible (v, δ) with min turning
// radius constraint. No costmap is consulted.
#ifndef BOT_NAVIGATION__APF_CONTROLLER_HPP_
#define BOT_NAVIGATION__APF_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>

namespace bot_navigation {

class APFController : public nav2_core::Controller {
 public:
  APFController() = default;
  ~APFController() override = default;

  // nav2_core::Controller interface
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

 private:
  // Callbacks
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Force computation
  struct Vec2 { double x = 0, y = 0; };
  Vec2 computeAttractive(double robot_x, double robot_y, double goal_x, double goal_y) const;
  Vec2 computeRepulsive(double robot_x, double robot_y, double robot_yaw,
                         const std::vector<Vec2> & obstacles_robot_frame) const;
  Vec2 computeSlopeForce(double pitch) const;

  // Extract 2D obstacle positions in robot frame from the latest point cloud
  std::vector<Vec2> extractObstacles2D(
    const sensor_msgs::msg::PointCloud2 & cloud) const;

  // Node / TF
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("APFController")};

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Latest sensor data (thread-safe)
  sensor_msgs::msg::PointCloud2::SharedPtr latest_scan_;
  std::mutex scan_mutex_;
  double latest_pitch_{0.0};
  std::mutex imu_mutex_;

  // Goal (endpoint of the path from GoalToPath)
  geometry_msgs::msg::PoseStamped goal_;
  bool goal_set_{false};

  // ---- APF Parameters ----
  // Attractive
  double k_att_{1.0};

  // Repulsive
  double k_rep_{0.8};
  double influence_distance_{4.0};     // d0: obstacles beyond this are ignored

  // Slope
  double k_slope_{2.0};
  double pitch_threshold_{0.21};       // ~12 degrees

  // Ackermann
  double max_linear_vel_{0.5};
  double min_turning_radius_{3.36};
  double wheelbase_{0.9};
  double max_steer_angle_;             // computed from wheelbase / min_turning_radius

  // Sensor filtering
  double obstacle_height_min_{0.15};   // above base_footprint z
  double obstacle_height_max_{2.0};
  double obstacle_range_max_{8.0};     // only consider obstacles within this range
  double ground_clearance_{0.3};       // height above per-sector ground estimate to count as obstacle
  std::string scan_topic_{"/scan/obstacles"};  // pre-filtered by ugv_obstacle
  std::string imu_topic_{"/imu"};

  // Speed limit override
  double speed_limit_{0.0};
  bool speed_limit_is_pct_{false};

  // Transform tolerance
  double transform_tolerance_{2.0};
};

}  // namespace bot_navigation

#endif  // BOT_NAVIGATION__APF_CONTROLLER_HPP_
