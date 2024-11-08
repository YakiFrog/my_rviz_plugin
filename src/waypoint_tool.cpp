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
  setName("Waypoint Tool");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/SetGoal.png")); // アイコンのパスを指定
}

void WaypointTool::onPoseSet(double x, double y, double theta)
{
  auto pose_msg = geometry_msgs::msg::PoseStamped();
  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = 0.0;
  pose_msg.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta));
  pose_msg.header.frame_id = context_->getFixedFrame().toStdString();
  pose_msg.header.stamp = rclcpp::Clock().now();

  // publish the pose
  pose_publisher_->publish(pose_msg); 

  // addWaypoint
  // WaypointTool::addWaypoint(x, y, theta);

  // std::string frame_id = context_->getFixedFrame().toStdString();

  // RCLCPP_INFO(rclcpp::get_logger("WaypointTool"), "Waypoint set: x=%f, y=%f, theta=%f, frame_id=%s", x, y, theta, frame_id.c_str());
}

// void WaypointTool::addWaypoint(double x, double y, double theta)
// {
//     // ウェイポイントの位置を設定
//     double new_x = x;
//     double new_y = y;

//     // 矢印マーカーを作成
//     visualization_msgs::msg::Marker marker;
//     marker.header.frame_id = "map"; // フレームIDを設定
//     marker.header.stamp = node_->now
    
//     // 現在のタイムスタンプを設定
//     marker.ns = "waypoints"; // 名前空間を設定
//     marker.id = waypoints.size(); // IDを設定（ユニークである必要があります）
//     marker.type = visualization_msgs::msg::Marker::ARROW; // マーカーのタイプを矢印に設定
//     marker.action = visualization_msgs::msg::Marker::ADD; // マーカーのアクションを追加に設定
//     marker.pose.position.x = new_x; // X座標
//     marker.pose.position.y = new_y; // Y座標
//     marker.pose.position.z = 0.0; // Z座標
//     marker.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta));
//     marker.scale.x = 0.3; // サイズを設定
//     marker.scale.y = 0.1; // 太さ
//     marker.scale.z = 1.0;
//     marker.color.r = 0.0f; // 色を設定
//     marker.color.g = 1.0f;
//     marker.color.b = 0.0f;
//     marker.color.a = 1.0; // アルファ（透明度）を設定

//     // マーカーをパブリッシュ
//     marker_pub_->publish(marker);

//     // ウェイポイントリストに追加
//     waypoints.append(QPair<double, double>(new_x, new_y));

//     // テキストマーカーを作成
//     visualization_msgs::msg::Marker text_marker;
//     text_marker.header.frame_id = "map"; // フレームIDを設定
//     text_marker.header.stamp = node_->now(); // 現在のタイムスタンプを設定
//     text_marker.ns = "waypoints_text"; // 名前空間を設定
//     text_marker.id = waypoints.size(); // IDを設定（ユニークである必要があります）
//     text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING; // テキストマーカーのタイプを設定
//     text_marker.action = visualization_msgs::msg::Marker::ADD; // マーカーのアクションを追加に設定
//     text_marker.pose.position.x = new_x; // X座標
//     text_marker.pose.position.y = new_y; // Y座標
//     text_marker.pose.position.z = 10; // Z座標を少し上げて表示
//     text_marker.pose.orientation.w = 1.0; // 向きの設定
//     text_marker.scale.z = 0.3; // テキストのサイズを設定
//     text_marker.color.r = 1.0f; // 色を設定
//     text_marker.color.g = 1.0f;
//     text_marker.color.b = 1.0f; // 白色
//     text_marker.color.a = 1.0; // アルファ（透明度）を設定
//     text_marker.text = std::to_string(waypoints.size()); // ウェイポイントのインデックス番号をテキストとして設定

//     // テキストマーカーをパブリッシュ
//     marker_pub_->publish(text_marker);

//     // debug output
//     RCLCPP_INFO(rclcpp::get_logger("WaypointTool"), "Waypoint added: x=%f, y=%f", new_x, new_y);
// }

  
}  // namespace my_rviz_plugin

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(my_rviz_plugin::WaypointTool, rviz_common::Tool)