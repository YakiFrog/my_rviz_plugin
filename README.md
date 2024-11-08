# Waypoint Editor Made by Kotani

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

## 機能

<!-- 画像表示 -->
![WaypointEditorの画面](https://github.com/user-attachments/assets/382085f3-8cf0-47bb-bdca-6043b608c3c9)

- Waypointの追加
- Waypointの編集
- Waypointの削除
- Waypointの保存
- Waypointの読み込み・表示
- Waypointの非表示
- Mapの読み込み・表示
- 疑似コンソール画面の表示
- 疑似コンソール画面の内容のクリア

## 今後の実装予定

- 指定Waypointからのナビゲーション機能
- Waypoint間のパスの表示
- GUIからNavigation速度の調整
- GUIからNavigationの一時停止
