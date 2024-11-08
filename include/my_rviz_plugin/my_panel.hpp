#ifndef MY_RVIZ_PLUGIN__MY_PANEL_HPP_
#define MY_RVIZ_PLUGIN__MY_PANEL_HPP_

#include <QWidget>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace my_rviz_plugin
{

class MyPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MyPanel(QWidget* parent = nullptr);
  virtual ~MyPanel();

  // Override必須のvirtual関数
  virtual void load(const rviz_common::Config& config) override;
  virtual void save(rviz_common::Config config) const override;

protected Q_SLOTS:
  void onButtonClick();

private:
  QPushButton* button_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace my_rviz_plugin

#endif  // MY_RVIZ_PLUGIN__MY_PANEL_HPP_