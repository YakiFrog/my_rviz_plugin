#include <my_rviz_plugin/my_panel.hpp>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

namespace my_rviz_plugin
{

MyPanel::MyPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  // レイアウトの作成
  QVBoxLayout* layout = new QVBoxLayout;
  
  // ボタンの作成
  button_ = new QPushButton("Click me!");
  layout->addWidget(button_);
  
  // シグナル・スロットの接続（Callbackの設定）
  connect(button_, SIGNAL(clicked()), this, SLOT(onButtonClick()));
  
  // レイアウトの設定（Panelの表示）
  setLayout(layout);
  
  // ROSノードの初期化
  node_ = std::make_shared<rclcpp::Node>("my_panel_node");
}

MyPanel::~MyPanel()
{
}

void MyPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

void MyPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void MyPanel::onButtonClick()
{
  RCLCPP_INFO(node_->get_logger(), "Button clicked!");
}

}  // namespace my_rviz_plugin

PLUGINLIB_EXPORT_CLASS(my_rviz_plugin::MyPanel, rviz_common::Panel)