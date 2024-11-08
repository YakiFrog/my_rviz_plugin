#ifndef MY_RVIZ_PLUGIN_WAYPOINT_TOOL_HPP
#define MY_RVIZ_PLUGIN_WAYPOINT_TOOL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace my_rviz_plugin
{

class WaypointTool : public rviz_default_plugins::tools::PoseTool
{
public:
  WaypointTool();
  ~WaypointTool() override;

  void onInitialize() override;
  void onPoseSet(double x, double y, double theta) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

}  // namespace my_rviz_plugin

#endif  // MY_RVIZ_PLUGIN_WAYPOINT_TOOL_HPP