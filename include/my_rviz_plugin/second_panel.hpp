#ifndef SECOND_PANEL_HPP_
#define SECOND_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace my_rviz_plugin
{

class SecondPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SecondPanel(QWidget* parent = nullptr);
  virtual ~SecondPanel() = default;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace my_rviz_plugin

#endif  // SECOND_PANEL_HPP_