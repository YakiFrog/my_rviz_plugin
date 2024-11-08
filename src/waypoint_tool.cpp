#include "my_rviz_plugin/waypoint_tool.hpp"

#include <memory>
#include <string>
#include "rviz_common/load_resource.hpp"
#include "rviz_common/display_context.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <visualization_msgs/msg/marker.hpp>

namespace my_rviz_plugin
{

WaypointTool::WaypointTool()
: rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 'w';  // ショートカットキーを設定
  node_ = rclcpp::Node::make_shared("waypoint_tool_node");
  pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("waypoint", 10);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("waypoint_markers", 10);
}

WaypointTool::~WaypointTool()
{
}

void WaypointTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Waypoint Add Tool");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/SetGoal.png")); // アイコンのパスを指定
}

void WaypointTool::onPoseSet(double x, double y, double theta)
{
  auto pose_msg = geometry_msgs::msg::PoseStamped();
  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = 0.0;
  pose_msg.pose.orientation.x = 0.0;
  pose_msg.pose.orientation.y = 0.0;
  pose_msg.pose.orientation.z = sin(theta / 2.0);
  pose_msg.pose.orientation.w = cos(theta / 2.0);
  pose_msg.header.frame_id = context_->getFixedFrame().toStdString();
  pose_msg.header.stamp = rclcpp::Clock().now();

  // publish the pose
  pose_publisher_->publish(pose_msg); 
}

}  // namespace my_rviz_plugin

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(my_rviz_plugin::WaypointTool, rviz_common::Tool)