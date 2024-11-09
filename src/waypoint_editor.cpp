#include <rviz_common/panel.hpp>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QFileDialog>
#include <QTextStream>
#include <QMouseEvent>
#include <QVector>
#include <QProcess>
#include <QDebug>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <pluginlib/class_list_macros.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/render_panel.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <yaml-cpp/yaml.h>
#include <QFileDialog>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>
#include <QTextEdit>

namespace my_rviz_plugin
{
class WaypointEditor : public rviz_common::Panel
{
public:
    struct Waypoint
    {
        double x;
        double y;
        double theta;
    };
    WaypointEditor(QWidget *parent = nullptr)
    : rviz_common::Panel(parent)
    {
        // ROSノードの初期化
        node_ = rclcpp::Node::make_shared("waypoint_editor_node");

        // スレッドを作成してspinを実行
        spin_thread_ = std::thread([this]() { spin(); });

        auto* layout = new QVBoxLayout;

        // コンソール部分
        auto* h_layout_console = new QHBoxLayout;
        h_layout_console->addWidget(new QLabel("コンソール:"));
        
        // コンソールをクリアするボタンを追加
        auto* clear_console_button = new QPushButton("コンソールをクリア");
        h_layout_console->addWidget(clear_console_button);
        connect(clear_console_button, &QPushButton::clicked, [this]() {
            console_output_->clear();
        });
        layout->addLayout(h_layout_console);

        // console output
        console_output_ = new QTextEdit;
        console_output_->setReadOnly(true); // 編集不可にする
        console_output_->setMinimumHeight(150); // コンソールの高さを設定（必要に応じて変更）
        layout->addWidget(console_output_);

        // マップをロードするボタンとWaypointをロードするボタン
        auto* h_layout_load = new QHBoxLayout;
        
        // Mapをロードするボタン
        auto* load_map_button = new QPushButton("Mapをロードする");
        h_layout_load->addWidget(load_map_button);
        
        // Waypointをロードするボタン
        auto* load_waypoints_button = new QPushButton("Waypointsをロードする");
        h_layout_load->addWidget(load_waypoints_button);

        layout->addLayout(h_layout_load);

        // Waypointsを保存するボタン
        auto* save_button = new QPushButton("Waypointsを保存する");
        layout->addWidget(save_button);

        // ID選択用のスピンボックス
        id_selector_ = new QSpinBox;
        id_selector_->setRange(0, 1000); // IDの範囲は適宜変更可能

        auto* h_layout_coords = new QHBoxLayout;
        h_layout_coords->addWidget(new QLabel("Waypoint ID:"));
        h_layout_coords->addWidget(id_selector_);
        layout->addLayout(h_layout_coords);
        // コンストラクタ内でスピンボックスの値が変更されたときに実行されるスロットを接続
        connect(id_selector_, QOverload<int>::of(&QSpinBox::valueChanged), this, 
            [this](int id) {
                if (id < 0 || id >= waypoints.size()) {
                    return;
                } else if (id == 0) {
                    x_input_->setValue(0.0);
                    y_input_->setValue(0.0);
                    theta_input_->setValue(0.0);
                    return;
                }
                x_input_->setValue(waypoints[id - 1].x);
                y_input_->setValue(waypoints[id - 1].y);
                theta_input_->setValue(waypoints[id - 1].theta * 180.0 / M_PI); // rad -> deg
            }
        );

        // x, y, thetaの入力フィールド
        x_input_ = new QDoubleSpinBox;
        y_input_ = new QDoubleSpinBox;
        theta_input_ = new QDoubleSpinBox;
        x_input_->setRange(-1000, 1000);
        y_input_->setRange(-1000, 1000);
        theta_input_->setRange(-180, 180); // 角度の範囲は適宜変更可能
        // 小数点以下の桁数を設定
        x_input_->setDecimals(2); // 0.01 -> 5cm
        y_input_->setDecimals(2); // 0.01 -> 5cm
        theta_input_->setDecimals(2); // 0.01 -> 0.01deg

        h_layout_coords = new QHBoxLayout;
        h_layout_coords->addWidget(new QLabel("x:"));
        h_layout_coords->addWidget(x_input_);
        layout->addLayout(h_layout_coords);

        h_layout_coords = new QHBoxLayout;
        h_layout_coords->addWidget(new QLabel("y:"));
        h_layout_coords->addWidget(y_input_);
        layout->addLayout(h_layout_coords);

        h_layout_coords = new QHBoxLayout;
        h_layout_coords->addWidget(new QLabel("theta [-180 ~ 180deg]:"));
        h_layout_coords->addWidget(theta_input_);
        layout->addLayout(h_layout_coords);

        // 更新ボタン
        auto* update_button = new QPushButton("Waypointを更新する");
        layout->addWidget(update_button);

        // すべてのWaypointsをクリアするボタン
        auto* clear_button = new QPushButton("すべてのWaypointsをクリアする");
        layout->addWidget(clear_button);

        /* Navigation用のWaypoint選択 */
        id_selector_nav_ = new QSpinBox;
        id_selector_nav_->setRange(1, 1000); // IDの範囲は適宜変更可能

        h_layout_coords = new QHBoxLayout;
        h_layout_coords->addWidget(new QLabel("Waypoint ID:"));
        h_layout_coords->addWidget(id_selector_nav_);
        layout->addLayout(h_layout_coords);

        // ナビゲーション開始・停止ボタン (横並び)
        auto* h_layout_nav = new QHBoxLayout;
        auto* start_nav_button = new QPushButton("Nav開始");
        auto* stop_nav_button = new QPushButton("Nav停止");
        h_layout_nav->addWidget(start_nav_button);
        h_layout_nav->addWidget(stop_nav_button);
        layout->addLayout(h_layout_nav);

        // レイアウトを設定
        setLayout(layout);

        // ボタンの接続
        connect(load_map_button, &QPushButton::clicked, this, &WaypointEditor::loadMap);
        connect(load_waypoints_button, &QPushButton::clicked, this, &WaypointEditor::loadWaypoints);
        connect(save_button, &QPushButton::clicked, this, &WaypointEditor::saveWaypoints);
        connect(update_button, &QPushButton::clicked, this, &WaypointEditor::updateWaypointFromUI);
        connect(clear_button, &QPushButton::clicked, this, &WaypointEditor::clearAllWaypoints);
        connect(start_nav_button, &QPushButton::clicked, this, &WaypointEditor::startNavigation);
        connect(stop_nav_button, &QPushButton::clicked, this, &WaypointEditor::stopNavigation);

        // マーカーパブリッシャーの設定
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("waypoint_markers", 10);

        // サブスクライバーの設定
        waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "waypoint",
            10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                double x = msg->pose.position.x;
                double y = msg->pose.position.y;
                double theta = tf2::getYaw(msg->pose.orientation);

                // ウェイポイントリストに追加
                this->waypoints.append({x, y, theta});
                qDebug() << "Added waypoint:" << x << y << sin(theta/2) << cos(theta/2);
                qDebug() << "Orientation:" << msg->pose.orientation.z << msg->pose.orientation.w;

                // add waypoint
                addWaypoint(x, y, theta);
            }
        );
    }

    // コンソールにメッセージを出力する関数を作成
    void logToConsole(const QString& message)
    {
        console_output_->append(message); // テキストを追加
    }

    void clearAllWaypoints()
    {
        // ウェイポイントリストをクリア
        waypoints.clear();

        // マーカーの削除
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node_->now();
        marker.ns = "waypoints";
        marker.action = visualization_msgs::msg::Marker::DELETEALL; // DELETEALLですべてのマーカーを削除
        marker_pub_->publish(marker);

        // テキストマーカーの削除
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = node_->now();
        text_marker.ns = "waypoints_text";
        text_marker.action = visualization_msgs::msg::Marker::DELETEALL;

        // テキストマーカーをパブリッシュ
        marker_pub_->publish(text_marker);

        // ID選択用のスピンボックスをリセット
        id_selector_->setValue(0);

        // x, y, thetaの入力フィールドをリセット
        x_input_->setValue(0.0);
        y_input_->setValue(0.0);
        theta_input_->setValue(0.0);

        // コンソールにメッセージを出力
        qDebug() << "すべてのウェイポイントとマーカーがクリアされました";
        logToConsole("すべてのウェイポイントとマーカーがクリアされました");
    }

    void loadMap()
    {
        // ファイル選択ダイアログを表示し、ユーザーにファイルを選択させる
        QString map_file_path = QFileDialog::getOpenFileName(this, 
                                    tr("地図ファイルを選択"), 
                                    "/home/oitrp/sirius_ws", 
                                    tr("YAML Files (*.yaml)"));
        
        // ユーザーがキャンセルした場合、関数を終了
        if (map_file_path.isEmpty()) {
            qWarning("地図ファイルが選択されませんでした");
            logToConsole("地図ファイルが選択されませんでした");
            return;
        }

        // QProcessを使ってmap_serverを呼び出す
        QProcess* process = new QProcess(this);
        process->start("ros2", QStringList() << "run" << "nav2_map_server" << "map_server" 
                                            << "--ros-args" << "-p" << "yaml_filename:=" + map_file_path);
        
        // プロセスが正常に開始されたかを確認
        if (!process->waitForStarted()) {
            qWarning("地図のロードに失敗しました");
            logToConsole("地図のロードに失敗しました");
            return;
        } else {
            qDebug() << "地図をロードしました";
            logToConsole("地図をロードしました: " + map_file_path);
        }

        // 地図の設定を行う（configureMapServerが必要な場合）
        configureMapServer();
    }

    void loadWaypoints()
    {
        // ファイルダイアログを開いてYAMLファイルを選択
        QString file_name = QFileDialog::getOpenFileName(this, "Load Waypoints", "", "YAML Files (*.yaml);;All Files (*)");
        if (file_name.isEmpty()) {
            return; // ファイル名が指定されていない場合は何もしない
        }

        QFile file(file_name);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            qWarning("ファイルを開けませんでした");
            logToConsole("ファイルを開けませんでした");
            return;
        }

        QTextStream in(&file);
        QString file_contents = in.readAll();
        file.close();

        // YAMLを解析
        YAML::Node yaml;
        try {
            yaml = YAML::Load(file_contents.toStdString());
        } catch (const YAML::Exception& e) {
            qWarning() << "YAMLの解析に失敗しました:" << e.what();
            logToConsole("YAMLの解析に失敗しました");
            return;
        }

        if (!yaml["points"]) {
            qWarning("YAMLファイルにpointsキーがありません");
            logToConsole("YAMLファイルにpointsキーがありません");
            return;
        }

        // 既存のウェイポイントリストをクリア
        waypoints.clear();

        // YAMLからウェイポイントを読み込む
        for (const auto& wp : yaml["points"]) {
            // x, y, thetaを読み込む
            double x = wp[0].as<double>();
            double y = wp[1].as<double>();
            double theta_z = wp[5].as<double>();
            double theta_w = wp[6].as<double>();
            double theta = tf2::getYaw(tf2::Quaternion(0, 0, theta_z, theta_w));
            // ウェイポイントを追加
            waypoints.append({x, y, theta});
            qDebug() << "Loaded waypoint:" << x << y << theta << waypoints.size();
            logToConsole("[" + QString::number(waypoints.size()) + "] " + QString::number(x) + ", " + QString::number(y) + ", " + QString::number(theta));
            // ウェイポイントを表示
            addWaypoint(x, y, theta);
        }
        qDebug() << "Waypoints loaded from" << file_name;
        logToConsole("Waypoints loaded from: " + file_name);
    }

    void saveWaypoints()
    {
        QString file_name = QFileDialog::getSaveFileName(this, "Save Waypoints", "", "YAML Files (*.yaml);;All Files (*)");
        if (file_name.isEmpty()) {
            return; // ファイル名が指定されていない場合は何もしない
        }

        QFile file(file_name);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            qWarning("ファイルを開けませんでした");
            logToConsole("ファイルを開けませんでした");
            return;
        }

        QTextStream out(&file);
        out << "points:\n";
        for (const auto& waypoint : this->waypoints) {
            out << "- - " << waypoint.x << "\n";
            out << "  - " << waypoint.y << "\n";
            out << "  - 0.0\n";
            out << "  - 0.0\n";
            out << "  - 0.0\n";
            out << "  - " << sin(waypoint.theta / 2.0) << "\n";
            out << "  - " << cos(waypoint.theta / 2.0) << "\n";
        }

        file.close();
        qDebug() << "Waypoints saved to" << file_name;
        logToConsole("Waypoints saved to: " + file_name);
    }

    void configureMapServer()
    {
        QProcess* process = new QProcess(this);
        process->start("ros2", QStringList() << "lifecycle" << "set" << "/map_server" << "configure");

        if (!process->waitForFinished()) {
            qWarning("map_serverのconfigureに失敗しました");
            logToConsole("map_serverのconfigureに失敗しました");
            return;
        }

        process->start("ros2", QStringList() << "lifecycle" << "set" << "/map_server" << "activate");

        if (!process->waitForFinished()) {
            qWarning("map_serverのactivateに失敗しました");
            logToConsole("map_serverのactivateに失敗しました");
        }

        qDebug() << "map_serverのconfigureとactivateが完了しました";
        logToConsole("map_serverのconfigureとactivateが完了しました");
    }

    void addWaypoint(double x, double y, double theta)
    {
        // ウェイポイントの位置を設定
        double new_x = x;
        double new_y = y;
        double new_theta = theta;

        // 矢印マーカーを作成
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // フレームIDを設定
        marker.header.stamp = node_->now();
        
        // 現在のタイムスタンプを設定
        marker.ns = "waypoints"; // 名前空間を設定
        marker.id = this->waypoints.size(); // IDを設定（ユニークである必要があります）
        marker.type = visualization_msgs::msg::Marker::ARROW; // マーカーのタイプを矢印に設定
        marker.action = visualization_msgs::msg::Marker::ADD; // マーカーのアクションを追加に設定
        marker.pose.position.x = new_x; // X座標
        marker.pose.position.y = new_y; // Y座標
        marker.pose.position.z = 5.0; // Z座標
        marker.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), new_theta));
        marker.scale.x = 0.3; // サイズを設定
        marker.scale.y = 0.1; // 太さ
        marker.scale.z = 1.0;
        marker.color.r = 0.0f; // 色を設定
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0; // アルファ（透明度）を設定
        // マーカーをパブリッシュ
        marker_pub_->publish(marker);

        // テキストマーカーを作成
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map"; // フレームIDを設定
        text_marker.header.stamp = node_->now(); // 現在のタイムスタンプを設定
        text_marker.ns = "waypoints_text"; // 名前空間を設定
        text_marker.id = this->waypoints.size(); // IDを設定（ユニークである必要があります）
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING; // テキストマーカーのタイプを設定
        text_marker.action = visualization_msgs::msg::Marker::ADD; // マーカーのアクションを追加に設定
        text_marker.pose.position.x = new_x; // X座標
        text_marker.pose.position.y = new_y; // Y座標
        text_marker.pose.position.z = 10; // Z座標を少し上げて表示
        text_marker.pose.orientation.w = 1.0; // 向きの設定
        text_marker.scale.z = 0.5; // テキストのサイズを設定
        text_marker.color.r = 1.0f; // 色を設定
        text_marker.color.g = 0.0f; // 色を設定
        text_marker.color.b = 0.0f; // 白色
        text_marker.color.a = 1.0; // アルファ（透明度）を設定
        text_marker.text = std::to_string(waypoints.size()); // ウェイポイントのインデックス番号をテキストとして設定

        // 背景の半透明丸マーカーの設定
        visualization_msgs::msg::Marker circle_marker;
        circle_marker.header.frame_id = "map";
        circle_marker.header.stamp = node_->now();
        circle_marker.ns = "waypoints_background";
        circle_marker.id = this->waypoints.size();
        circle_marker.type = visualization_msgs::msg::Marker::SPHERE;
        circle_marker.action = visualization_msgs::msg::Marker::ADD;
        circle_marker.pose.position.x = new_x;
        circle_marker.pose.position.y = new_y;
        circle_marker.pose.position.z = 1; // 地面に少し近づける
        circle_marker.scale.x = 0.6; // 半透明円の直径
        circle_marker.scale.y = 0.6; 
        circle_marker.scale.z = 0.05; // 薄い円
        circle_marker.color.r = 0.0f;
        circle_marker.color.g = 0.0f;
        circle_marker.color.b = 0.0f;
        circle_marker.color.a = 0.5; // 半透明設定

        // 背景の半透明丸マーカーをパブリッシュ
        marker_pub_->publish(circle_marker);
        // テキストマーカーをパブリッシュ
        marker_pub_->publish(text_marker);

        // debug output
        // RCLCPP_INFO(rclcpp::get_logger("WaypointTool"), "Waypoint added: x=%f, y=%f, theta=%f", new_x, new_y, new_theta);
    }

    void updateWaypointFromUI()
    {
        int id = id_selector_->value();
        double new_x = x_input_->value();
        double new_y = y_input_->value();
        // -180 ~ 180をradに変換
        double new_theta = theta_input_->value() * M_PI / 180.0;

        // もし，x, y = 0.0の場合は，IDのウェイポイントを削除
        if (new_x == 0.0 && new_y == 0.0 && new_theta == 0.0) {
            if (id == 0) {
                qWarning("ID 0のウェイポイントは削除できません");
                logToConsole("ID 0のウェイポイントは削除できません");
                return;
            }
            deleteWaypoint(id);
            return;
        }

        // ID範囲外のチェック
        if (id < 0 || id >= waypoints.size()) {
            qWarning("指定されたIDが範囲外です");
            logToConsole("指定されたIDが範囲外です");
            return;
        }

        // Waypointの更新
        waypoints[id - 1] = {new_x, new_y, new_theta};
        updateWaypoint(id, new_x, new_y, new_theta);

        // コンソールにメッセージを出力
        qDebug() << "Waypoint updated:" << new_x << new_y << new_theta;
    }

    void updateWaypoint(int id, double new_x, double new_y, double new_theta)
    {
        // マーカーの再作成（IDは既存のものを指定）
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // フレームIDを設定
        marker.header.stamp = node_->now(); // 現在のタイムスタンプを設定
        marker.ns = "waypoints"; // 名前空間を設定
        marker.id = id; // 既存のIDを使用
        marker.type = visualization_msgs::msg::Marker::ARROW; // マーカーのタイプ
        marker.action = visualization_msgs::msg::Marker::MODIFY; // マーカーのアクションを変更に設定
        marker.pose.position.x = new_x; // 新しいX座標
        marker.pose.position.y = new_y; // 新しいY座標
        marker.pose.position.z = 5.0; // Z座標
        marker.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), new_theta)); // 新しい向き
        marker.scale.x = 0.3; // サイズ
        marker.scale.y = 0.1; 
        marker.scale.z = 1.0;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        // マーカーをパブリッシュして更新
        marker_pub_->publish(marker);
        
        // テキストマーカーの再作成
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map"; // フレームIDを設定
        text_marker.header.stamp = node_->now(); // 現在のタイムスタンプを設定
        text_marker.ns = "waypoints_text"; // 名前空間を設定
        text_marker.id = id; // 既存のIDを使用
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING; // テキストマーカーのタイプを設定
        text_marker.action = visualization_msgs::msg::Marker::MODIFY; // マーカーのアクションを変更に設定
        text_marker.pose.position.x = new_x; // X座標
        text_marker.pose.position.y = new_y; // Y座標
        text_marker.pose.position.z = 10; // Z座標を少し上げて表示
        text_marker.pose.orientation.w = 1.0; // 向きの設定
        text_marker.scale.z = 0.5; // テキストのサイズを設定
        text_marker.color.r = 1.0f; // 色を設定
        text_marker.color.g = 0.0f;
        text_marker.color.b = 0.0f; // 白色
        text_marker.color.a = 1.0; // アルファ（透明度）を設定
        text_marker.text = std::to_string(id); // ウェイポイントのインデックス番号をテキストとして設定

        // 背景の半透明丸マーカーの設定
        visualization_msgs::msg::Marker circle_marker;
        circle_marker.header.frame_id = "map";
        circle_marker.header.stamp = node_->now();
        circle_marker.ns = "waypoints_background";
        circle_marker.id = id;
        circle_marker.type = visualization_msgs::msg::Marker::SPHERE;
        circle_marker.action = visualization_msgs::msg::Marker::ADD;
        circle_marker.pose.position.x = new_x;
        circle_marker.pose.position.y = new_y;
        circle_marker.pose.position.z = 1; // 地面に少し近づける
        circle_marker.scale.x = 0.6; // 半透明円の直径
        circle_marker.scale.y = 0.6; 
        circle_marker.scale.z = 0.05; // 薄い円
        circle_marker.color.r = 0.0f;
        circle_marker.color.g = 0.0f;
        circle_marker.color.b = 0.0f;
        circle_marker.color.a = 0.5; // 半透明設定

        // 背景の半透明丸マーカーをパブリッシュ
        marker_pub_->publish(circle_marker);
        // テキストマーカーをパブリッシュ
        marker_pub_->publish(text_marker);
    }

    void deleteWaypoint(int id)
    {
        // ウェイポイントリストから削除
        waypoints.remove(id - 1);

        // マーカーの削除
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node_->now();
        marker.ns = "waypoints";
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::DELETE; // DELETEでマーカーを削除
        marker_pub_->publish(marker);

        // テキストマーカーの削除
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = node_->now();
        text_marker.ns = "waypoints_text";
        text_marker.id = id;
        text_marker.action = visualization_msgs::msg::Marker::DELETE;

        // テキストマーカーをパブリッシュ
        marker_pub_->publish(text_marker);

        // ID選択用のスピンボックスをリセット
        id_selector_->setValue(0);

        // x, y, thetaの入力フィールドをリセット
        x_input_->setValue(0.0);
        y_input_->setValue(0.0);
        theta_input_->setValue(0.0);

        // コンソールにメッセージを出力
        qDebug() << "Waypoint deleted:" << id;
        logToConsole("Waypoint deleted: " + QString::number(id));

        // IDを振り直す
        for (int i = 0; i < waypoints.size(); i++) {
            updateWaypoint(i + 1, waypoints[i].x, waypoints[i].y, waypoints[i].theta);
        }
    }

    void startNavigation()
    {
        int id = id_selector_nav_->value();
        if (id < 1 || id > waypoints.size()) { 
            qWarning("指定されたIDが範囲外です");
            logToConsole("指定されたIDが範囲外です");
            return;
        }

        if (nav_process_ != nullptr) {
            nav_process_->kill();
            nav_process_->deleteLater(); 
            logToConsole("既存のナビゲーションを停止しました");
            return;
        }

        nav_process_ = new QProcess(this);
        connect(nav_process_, &QProcess::readyReadStandardOutput, this, [this]() {
            QByteArray output = nav_process_->readAllStandardOutput();
            logToConsole(output);
        });

        // ナビゲーションを開始するためのコマンドを送信
        QString command = "ros2";
        QStringList arguments = {"run", "sirius_navigation", "move_goal.py", "--count", QString::number(id)};

        nav_process_->start(command, arguments);

        if (!nav_process_->waitForStarted()) {
            qWarning("ナビゲーションの開始に失敗しました");
            logToConsole("ナビゲーションの開始に失敗しました");
            return;
        } else {
            qDebug() << "ナビゲーションを開始しました";
            logToConsole("ナビゲーションを開始しました");
        }
    }

    void stopNavigation()
    {
        if (nav_process_ == nullptr) {
            qWarning("ナビゲーションが開始されていません");
            logToConsole("ナビゲーションが開始されていません");
            return;
        }

        nav_process_->kill();
        nav_process_->deleteLater();
        nav_process_ = nullptr;
        logToConsole("ナビゲーションを停止しました");
    }

    ~WaypointEditor() {
        stop_spinning_ = true;
        if (spin_thread_.joinable()) { // スレッドがjoin可能かどうかを確認
            spin_thread_.join(); // スレッドをjoin
        }
    }

private:
    void spin() // メインスレッドでspinするための関数
    {
        while (!stop_spinning_) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    rclcpp::Node::SharedPtr node_;
    std::thread spin_thread_;
    std::atomic<bool> stop_spinning_{false};
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    QVector<Waypoint> waypoints; // ウェイポイントのリスト
    QSpinBox *id_selector_;
    QDoubleSpinBox *x_input_;
    QDoubleSpinBox *y_input_;
    QDoubleSpinBox *theta_input_;
    rviz_common::RenderPanel* render_panel_; // RenderPanelへの参照
    QTextEdit* console_output_; // コンソール出力用のテキストエディタ
    QSpinBox* id_selector_nav_; // ナビゲーション用のID選択スピンボックス

    QProcess *nav_process_;  // move_goal.py用のプロセス
};

}  // namespace my_rviz_plugin

PLUGINLIB_EXPORT_CLASS(my_rviz_plugin::WaypointEditor, rviz_common::Panel)