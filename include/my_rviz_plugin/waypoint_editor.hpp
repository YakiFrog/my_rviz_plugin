#ifndef WAYPOINT_EDITOR_HPP_
#define WAYPOINT_EDITOR_HPP_

// 標準ライブラリのインクルード
#include <vector>

// Qt関連
#include <QWidget>
#include <QVector>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QFileDialog>
#include <QTextStream>

// ROS 2関連
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// tf2関連
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

namespace my_rviz_plugin
{

class WaypointEditor : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WaypointEditor(QWidget *parent = nullptr);
  virtual ~WaypointEditor() override;


protected:
  // Add any protected members or methods here

private:
  rclcpp::Node::SharedPtr node_; // ROSノードのインスタンス
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; // マーカーパブリッシャー
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_; // ウェイポイントのサブスクライバー
};
}  // namespace my_rviz_plugin

#endif  // WAYPOINT_EDITOR_HPP_