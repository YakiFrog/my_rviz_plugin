#include <my_rviz_plugin/second_panel.hpp>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <pluginlib/class_list_macros.hpp>

namespace my_rviz_plugin
{

SecondPanel::SecondPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  // GUIの構築
  auto* layout = new QVBoxLayout;
  
  auto* label = new QLabel("This is Second Panel");
  layout->addWidget(label);
  
  auto* button = new QPushButton("Second Panel Button");
  layout->addWidget(button);
  
  setLayout(layout);
  
  // ROSノードの初期化
  node_ = std::make_shared<rclcpp::Node>("second_panel_node");
  
  // ボタンクリック時の処理
  connect(button, &QPushButton::clicked, this, [this]() {
    RCLCPP_INFO(node_->get_logger(), "Second panel button clicked!");
  });
}

}  // namespace my_rviz_plugin

PLUGINLIB_EXPORT_CLASS(my_rviz_plugin::SecondPanel, rviz_common::Panel)