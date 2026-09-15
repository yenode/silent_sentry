#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

class GoalToPath : public BT::SyncActionNode
{
public:
  GoalToPath(
    const std::string & action_name,
    const BT::NodeConfig & conf)
  : BT::SyncActionNode(action_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal", "Target goal pose"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Path message containing interpolated path to goal")
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("input_goal", goal)) {
      return BT::NodeStatus::FAILURE;
    }

    // Check if goal has changed. If not, return the cached path to prevent
    // continuous action server preemption in BT PipelineSequence.
    if (!first_time_ &&
        std::abs(goal.pose.position.x - prev_goal_.pose.position.x) < 0.01 &&
        std::abs(goal.pose.position.y - prev_goal_.pose.position.y) < 0.01) {
      setOutput("output_path", cached_path_);
      return BT::NodeStatus::SUCCESS;
    }

    prev_goal_ = goal;
    first_time_ = false;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::string global_frame = goal.header.frame_id.empty() ? "map" : goal.header.frame_id;
    std::string robot_base_frame = "base_footprint";

    double start_x = 0.0;
    double start_y = 0.0;
    bool has_robot_pose = false;

    if (config().blackboard->get("tf_buffer", tf_buffer)) {
      (void)config().blackboard->get("global_frame", global_frame);
      (void)config().blackboard->get("robot_base_frame", robot_base_frame);

      try {
        auto transform = tf_buffer->lookupTransform(
          global_frame, robot_base_frame, rclcpp::Time(0), rclcpp::Duration(0, 50000000));
        start_x = transform.transform.translation.x;
        start_y = transform.transform.translation.y;
        has_robot_pose = true;
      } catch (const tf2::TransformException & ex) {
        // Fallback if transform isn't immediately available
      }
    }

    double target_x = goal.pose.position.x;
    double target_y = goal.pose.position.y;

    nav_msgs::msg::Path path;
    path.header = goal.header;
    if (path.header.frame_id.empty()) {
      path.header.frame_id = global_frame;
    }

    if (!has_robot_pose) {
      path.poses.push_back(goal);
      cached_path_ = path;
      setOutput("output_path", path);
      return BT::NodeStatus::SUCCESS;
    }

    double dx = target_x - start_x;
    double dy = target_y - start_y;
    double distance = std::hypot(dx, dy);

    double step_size = 0.1;  // 10 cm resolution
    int num_steps = std::max(2, static_cast<int>(std::ceil(distance / step_size)));
    double yaw = std::atan2(dy, dx);

    double qz = std::sin(yaw / 2.0);
    double qw = std::cos(yaw / 2.0);

    path.poses.reserve(num_steps);
    for (int i = 0; i < num_steps; ++i) {
      double t = static_cast<double>(i) / (num_steps - 1);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = start_x + t * dx;
      pose.pose.position.y = start_y + t * dy;
      pose.pose.position.z = goal.pose.position.z;

      if (i == num_steps - 1) {
        pose.pose.orientation = goal.pose.orientation;
      } else {
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = qz;
        pose.pose.orientation.w = qw;
      }
      path.poses.push_back(pose);
    }

    cached_path_ = path;
    setOutput("output_path", path);
    return BT::NodeStatus::SUCCESS;
  }

private:
  geometry_msgs::msg::PoseStamped prev_goal_;
  nav_msgs::msg::Path cached_path_;
  bool first_time_{true};
};

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalToPath>("GoalToPath");
}
