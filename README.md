# Waypoint Editor

## セットアップ

ws/srcにクローンする
<!-- setup commands -->
```bash
git clone https://github.com/YakiFrog/my_rviz_plugin.git
```

wsフォルダに移動してビルドする
<!-- build commands -->
```bash
colcon build --packages-select my_rviz_plugin
```

## 実行方法

wsフォルダに移動して以下のコマンドを実行する
<!-- run commands -->
```bash
source install/setup.bash
rviz2
```