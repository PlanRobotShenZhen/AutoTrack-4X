#include "patrol_navigation_panel.h"
#include <yaml-cpp/yaml.h>
#include <ros/console.h>

namespace patrol_navigation_rviz_plugin {

PatrolNavigationPanel::PatrolNavigationPanel(QWidget *parent)
    : rviz::Panel(parent) {
    
    initializeUI();
    initializeROS();
    initializeLogging();
    
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &PatrolNavigationPanel::updateTimer);
    update_timer_->start(TIMER_INTERVAL_MS);
    
    logPatrolEvent(LogEventType::SYSTEM_START, "巡检导航面板初始化完成");
    ROS_INFO("巡检导航面板初始化完成");
}

PatrolNavigationPanel::~PatrolNavigationPanel() {
    if (log_file_.is_open()) {
        logPatrolEvent(LogEventType::SYSTEM_START, "巡检导航面板关闭");
        log_file_.close();
    }
    
    if (ros::ok()) {
        goal_publisher_.shutdown();
        path_publisher_.shutdown();
        marker_publisher_.shutdown();
        cancel_publisher_.shutdown();
        global_plan_publisher_.shutdown();
        local_plan_publisher_.shutdown();
        status_subscriber_.shutdown();
        point_subscriber_.shutdown();
    }
}

void PatrolNavigationPanel::initializeUI() {
    auto* main_layout = new QVBoxLayout(this);

    // 路径管理区域
    auto* path_section = new QGroupBox("路径管理");
    auto* path_layout = new QVBoxLayout;
    
    auto* button_row = new QHBoxLayout;
    add_button_ = new QPushButton("添加点");
    remove_button_ = new QPushButton("删除点");
    clear_button_ = new QPushButton("清空路径");
    reverse_button_ = new QPushButton("反向路径");
    save_button_ = new QPushButton("保存路径");
    load_button_ = new QPushButton("加载路径");
    view_logs_button_ = new QPushButton("查看日志");
    
    QString button_style = 
        "QPushButton {"
        "    background-color: #f0f0f0;"
        "    border: 1px solid #cccccc;"
        "    border-radius: 4px;"
        "    padding: 6px 12px;"
        "}"
        "QPushButton:hover {"
        "    background-color: #e8e8e8;"
        "}"
        "QPushButton:disabled {"
        "    background-color: #f5f5f5;"
        "    color: #aaaaaa;"
        "}";
    
    add_button_->setStyleSheet(button_style);
    remove_button_->setStyleSheet(button_style);
    clear_button_->setStyleSheet(button_style);
    reverse_button_->setStyleSheet(button_style);
    save_button_->setStyleSheet(button_style);
    load_button_->setStyleSheet(button_style);
    view_logs_button_->setStyleSheet(button_style);
    
    button_row->addWidget(add_button_);
    button_row->addWidget(remove_button_);
    button_row->addWidget(clear_button_);
    button_row->addWidget(reverse_button_);
    button_row->addWidget(save_button_);
    button_row->addWidget(load_button_);
    button_row->addWidget(view_logs_button_);
    path_layout->addLayout(button_row);
    
    waypoints_table_ = new QTableWidget(0, 4);
    waypoints_table_->setHorizontalHeaderLabels(
        QStringList() << "X" << "Y" << "朝向(度)" << "停留(秒)");
    waypoints_table_->horizontalHeader()->setStretchLastSection(true);
    waypoints_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    waypoints_table_->setSelectionMode(QAbstractItemView::SingleSelection);
    waypoints_table_->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::SelectedClicked);
    
    path_layout->addWidget(waypoints_table_);
    path_section->setLayout(path_layout);
    main_layout->addWidget(path_section);

    // 控制设置区域
    auto* control_section = new QGroupBox("控制设置");
    auto* control_layout = new QVBoxLayout;
    
    auto* settings_row = new QHBoxLayout;
    settings_row->addWidget(new QLabel("循环间隔时间:"));
    cycle_interval_spinbox_ = new QSpinBox;
    cycle_interval_spinbox_->setRange(0, 300);
    cycle_interval_spinbox_->setSuffix("秒");
    cycle_interval_spinbox_->setValue(5);
    settings_row->addWidget(cycle_interval_spinbox_);
    
    cycle_checkbox_ = new QCheckBox("循环模式");
    settings_row->addWidget(cycle_checkbox_);
    settings_row->addStretch();
    control_layout->addLayout(settings_row);
    
    auto* control_buttons = new QHBoxLayout;
    start_button_ = new QPushButton("开始巡检");
    pause_resume_button_ = new QPushButton("暂停");
    stop_button_ = new QPushButton("停止");
    
    start_button_->setStyleSheet(button_style);
    pause_resume_button_->setStyleSheet(button_style);
    stop_button_->setStyleSheet(button_style);
    
    control_buttons->addWidget(start_button_);
    control_buttons->addWidget(pause_resume_button_);
    control_buttons->addWidget(stop_button_);
    control_buttons->addStretch();
    control_layout->addLayout(control_buttons);
    
    control_section->setLayout(control_layout);
    main_layout->addWidget(control_section);

    // 状态显示区域
    auto* status_section = new QGroupBox("状态");
    auto* status_layout = new QVBoxLayout;
    
    progress_bar_ = new QProgressBar;
    progress_bar_->setRange(0, 100);
    status_layout->addWidget(progress_bar_);
    
    auto* info_row = new QHBoxLayout;
    status_label_ = new QLabel("状态: 就绪");
    current_point_label_ = new QLabel("当前点: 无");
    point_count_label_ = new QLabel("点数: 0");
    
    info_row->addWidget(status_label_);
    info_row->addWidget(current_point_label_);
    info_row->addWidget(point_count_label_);
    info_row->addStretch();
    status_layout->addLayout(info_row);
    
    status_section->setLayout(status_layout);
    main_layout->addWidget(status_section);

    // 信号槽连接
    connect(add_button_, &QPushButton::clicked, this, &PatrolNavigationPanel::handleAddPoint);
    connect(remove_button_, &QPushButton::clicked, this, &PatrolNavigationPanel::handleRemovePoint);
    connect(clear_button_, &QPushButton::clicked, this, &PatrolNavigationPanel::handleClearPoints);
    connect(reverse_button_, &QPushButton::clicked, this, &PatrolNavigationPanel::handleReversePath);
    connect(save_button_, &QPushButton::clicked, this, &PatrolNavigationPanel::handleSavePath);
    connect(load_button_, &QPushButton::clicked, this, &PatrolNavigationPanel::handleLoadPath);
    connect(view_logs_button_, &QPushButton::clicked, this, &PatrolNavigationPanel::handleViewLogs);
    connect(start_button_, &QPushButton::clicked, this, &PatrolNavigationPanel::handleStartPatrol);
    connect(pause_resume_button_, &QPushButton::clicked, this, &PatrolNavigationPanel::handlePauseResume);
    connect(stop_button_, &QPushButton::clicked, this, &PatrolNavigationPanel::handleStopPatrol);
    connect(waypoints_table_, &QTableWidget::cellChanged, this, &PatrolNavigationPanel::handleTableItemChanged);

    setLayout(main_layout);
    refreshUI();
}

void PatrolNavigationPanel::initializeROS() {
    goal_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, true);
    path_publisher_ = node_handle_.advertise<nav_msgs::Path>("patrol_path", 1, true);
    marker_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("patrol_markers", 1, true);
    cancel_publisher_ = node_handle_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    global_plan_publisher_ = node_handle_.advertise<nav_msgs::Path>("/move_base/GlobalPlanner/plan", 1, true);
    local_plan_publisher_ = node_handle_.advertise<nav_msgs::Path>("/move_base/LocalPlanner/plan", 1, true);
    
    status_subscriber_ = node_handle_.subscribe("move_base/status", 1, 
                                               &PatrolNavigationPanel::processGoalStatus, this);
    point_subscriber_ = node_handle_.subscribe("patrol_point", 1,
                                              &PatrolNavigationPanel::processIncomingPoint, this);
    
    clear_costmaps_client_ = node_handle_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
    
    logPatrolEvent(LogEventType::SYSTEM_START, "ROS通信初始化完成");
    ROS_INFO("ROS通信初始化完成");
}

void PatrolNavigationPanel::initializeLogging() {
    QString ros_log_dir = QString("%1/.ros").arg(QDir::homePath());
    if (QDir(ros_log_dir).exists()) {
        log_directory_ = QString("%1/%2").arg(ros_log_dir).arg(LOG_DIR_NAME);
    } else {
        log_directory_ = QString("%1/%2").arg(QDir::homePath()).arg(LOG_DIR_NAME);
    }
    
    QDir log_dir(log_directory_);
    if (!log_dir.exists()) {
        if (log_dir.mkpath(".")) {
            logPatrolEvent(LogEventType::SYSTEM_START, QString("日志目录创建成功: %1").arg(log_directory_));
            ROS_INFO("日志目录创建成功: %s", log_directory_.toStdString().c_str());
        } else {
            logPatrolEvent(LogEventType::ERROR_OCCUR, QString("日志目录创建失败: %1").arg(log_directory_));
            ROS_ERROR("日志目录创建失败: %s", log_directory_.toStdString().c_str());
            showUserMessage("警告", "日志目录创建失败，巡检记录将无法保存", QMessageBox::Warning);
        }
    }
    
    QString timestamp = QDateTime::currentDateTime().toString("yyyy年MM月dd日_hh时mm分ss秒");
    current_log_file_ = QString("%1/巡检日志_%2.log").arg(log_directory_).arg(timestamp);
    
    log_file_.open(current_log_file_.toStdString(), std::ios::out | std::ios::app);
    if (log_file_.is_open()) {
        logPatrolEvent(LogEventType::SYSTEM_START, QString("日志文件创建成功: %1").arg(current_log_file_));
    } else {
        ROS_ERROR("日志文件创建失败: %s", current_log_file_.toStdString().c_str());
        showUserMessage("警告", "日志文件创建失败，巡检记录将无法保存", QMessageBox::Warning);
    }
}

void PatrolNavigationPanel::clearGlobalPlannerPaths() {
    nav_msgs::Path empty_global_path;
    empty_global_path.header.stamp = ros::Time::now();
    empty_global_path.header.frame_id = "map";
    global_plan_publisher_.publish(empty_global_path);
    
    nav_msgs::Path empty_local_path;
    empty_local_path.header.stamp = ros::Time::now();
    empty_local_path.header.frame_id = "map";
    local_plan_publisher_.publish(empty_local_path);
    
    ROS_DEBUG("全局和局部规划路径已清除");
}

void PatrolNavigationPanel::handleTableItemChanged(int row, int column) {
    if (is_programmatic_table_update_) {
        return;
    }
    
    if (row < 0 || row >= static_cast<int>(waypoints_.size())) {
        return;
    }
    
    QTableWidgetItem* item = waypoints_table_->item(row, column);
    if (!item) {
        return;
    }
    
    QString newText = item->text();
    QString oldText;
    const auto& waypoint = waypoints_[row];
    
    switch (column) {
        case 0: oldText = QString::number(waypoint.x, 'f', 2); break;
        case 1: oldText = QString::number(waypoint.y, 'f', 2); break;
        case 2: oldText = QString::number(waypoint.yaw * RADIANS_TO_DEGREES, 'f', 1); break;
        case 3: oldText = QString::number(waypoint.duration); break;
        default: return;
    }
    
    if (newText == oldText) {
        return;
    }
    
    if (current_state_ != PatrolState::Idle) {
        refreshTable();
        showUserMessage("警告", "导航正在进行中，无法编辑点位", QMessageBox::Warning);
        return;
    }
    
    updateWaypointFromTable(row);
    refreshVisualization();
    
    logPatrolEvent(LogEventType::PATH_MODIFY, 
                  QString("点位%1修改 - 列:%2, 原值:%3, 新值:%4")
                  .arg(row+1).arg(column).arg(oldText).arg(newText));
}

void PatrolNavigationPanel::updateWaypointFromTable(int row) {
    if (row < 0 || row >= static_cast<int>(waypoints_.size())) {
        return;
    }
    
    bool conversion_ok = true;
    auto& waypoint = waypoints_[row];
    
    if (auto* x_item = waypoints_table_->item(row, 0)) {
        double x = x_item->text().toDouble(&conversion_ok);
        if (conversion_ok) {
            waypoint.x = x;
        } else {
            showUserMessage("错误", "X坐标格式错误", QMessageBox::Warning);
            refreshTable();
            return;
        }
    }
    
    if (auto* y_item = waypoints_table_->item(row, 1)) {
        double y = y_item->text().toDouble(&conversion_ok);
        if (conversion_ok) {
            waypoint.y = y;
        } else {
            showUserMessage("错误", "Y坐标格式错误", QMessageBox::Warning);
            refreshTable();
            return;
        }
    }
    
    if (auto* yaw_item = waypoints_table_->item(row, 2)) {
        double yaw_degrees = yaw_item->text().toDouble(&conversion_ok);
        if (conversion_ok) {
            waypoint.yaw = yaw_degrees * DEGREES_TO_RADIANS;
        } else {
            showUserMessage("错误", "朝向角度格式错误", QMessageBox::Warning);
            refreshTable();
            return;
        }
    }
    
    if (auto* duration_item = waypoints_table_->item(row, 3)) {
        int duration = duration_item->text().toInt(&conversion_ok);
        if (conversion_ok && duration >= 0) {
            waypoint.duration = duration;
        } else {
            showUserMessage("错误", "停留时间必须为非负整数", QMessageBox::Warning);
            refreshTable();
            return;
        }
    }
    
    QString log_str = QString("点位%1更新: (%2, %3, %4°), 停留: %5秒")
                      .arg(row + 1)
                      .arg(waypoint.x, 0, 'f', 2)
                      .arg(waypoint.y, 0, 'f', 2)
                      .arg(waypoint.yaw * RADIANS_TO_DEGREES, 0, 'f', 1)
                      .arg(waypoint.duration);
    ROS_INFO("%s", log_str.toStdString().c_str());
}

void PatrolNavigationPanel::refreshUI() {
    const bool has_waypoints = !waypoints_.empty();
    const bool is_editable = (current_state_ == PatrolState::Idle);
    const int selected_row = waypoints_table_->currentRow();
    
    add_button_->setEnabled(is_editable);
    remove_button_->setEnabled(is_editable && has_waypoints && selected_row >= 0);
    clear_button_->setEnabled(is_editable && has_waypoints);
    reverse_button_->setEnabled(is_editable && has_waypoints);
    save_button_->setEnabled(is_editable && has_waypoints);
    load_button_->setEnabled(is_editable);
    view_logs_button_->setEnabled(true);
    
    start_button_->setEnabled(has_waypoints && current_state_ == PatrolState::Idle);
    pause_resume_button_->setEnabled(current_state_ == PatrolState::Navigating || 
                                    current_state_ == PatrolState::Waiting ||
                                    current_state_ == PatrolState::Paused);
    stop_button_->setEnabled(current_state_ != PatrolState::Idle);
    
    waypoints_table_->setEnabled(is_editable);
    pause_resume_button_->setText(current_state_ == PatrolState::Paused ? "继续" : "暂停");
    
    QString status_text;
    switch (current_state_) {
        case PatrolState::Idle:
            status_text = "就绪";
            break;
        case PatrolState::Navigating:
            status_text = QString("导航中 -> 点 %1").arg(current_goal_index_ + 1);
            break;
        case PatrolState::Waiting:
            status_text = QString("在点 %1 等待").arg(current_goal_index_ + 1);
            break;
        case PatrolState::Paused:
            status_text = QString("已暂停 (当前点: %1)").arg(current_goal_index_ + 1);
            break;
    }
    status_label_->setText(QString("状态: %1").arg(status_text));
    
    point_count_label_->setText(QString("点数: %1").arg(waypoints_.size()));
    
    if (current_state_ != PatrolState::Idle && has_waypoints) {
        current_point_label_->setText(
            QString("当前点: %1/%2").arg(current_goal_index_ + 1).arg(waypoints_.size()));
    } else {
        current_point_label_->setText("当前点: 无");
    }
    
    if (waypoints_.empty()) {
        progress_bar_->setValue(0);
    } else if (current_state_ != PatrolState::Idle) {
        const int progress = static_cast<int>((current_goal_index_ + 1) * 100.0 / waypoints_.size());
        progress_bar_->setValue(progress);
    } else {
        progress_bar_->setValue(0);
    }
}

void PatrolNavigationPanel::refreshTable() {
    is_programmatic_table_update_ = true;
    
    waypoints_table_->blockSignals(true);
    waypoints_table_->setRowCount(waypoints_.size());
    
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        const auto& point = waypoints_[i];
        
        auto* x_item = new QTableWidgetItem(QString::number(point.x, 'f', 2));
        auto* y_item = new QTableWidgetItem(QString::number(point.y, 'f', 2));
        auto* yaw_item = new QTableWidgetItem(QString::number(point.yaw * RADIANS_TO_DEGREES, 'f', 1));
        auto* duration_item = new QTableWidgetItem(QString::number(point.duration));
        
        if (current_state_ == PatrolState::Idle) {
            x_item->setFlags(x_item->flags() | Qt::ItemIsEditable);
            y_item->setFlags(y_item->flags() | Qt::ItemIsEditable);
            yaw_item->setFlags(yaw_item->flags() | Qt::ItemIsEditable);
            duration_item->setFlags(duration_item->flags() | Qt::ItemIsEditable);
        } else {
            x_item->setFlags(x_item->flags() & ~Qt::ItemIsEditable);
            y_item->setFlags(y_item->flags() & ~Qt::ItemIsEditable);
            yaw_item->setFlags(yaw_item->flags() & ~Qt::ItemIsEditable);
            duration_item->setFlags(duration_item->flags() & ~Qt::ItemIsEditable);
        }
        
        waypoints_table_->setItem(i, 0, x_item);
        waypoints_table_->setItem(i, 1, y_item);
        waypoints_table_->setItem(i, 2, yaw_item);
        waypoints_table_->setItem(i, 3, duration_item);
    }
    
    waypoints_table_->blockSignals(false);
    is_programmatic_table_update_ = false;
    
    refreshUI();
}

void PatrolNavigationPanel::refreshVisualization() {
    if (waypoints_.empty()) {
        clearVisualization();
        return;
    }

    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";
    
    for (const auto& waypoint : waypoints_) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = waypoint.x;
        pose.pose.position.y = waypoint.y;
        pose.pose.position.z = 0.0;
        
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, waypoint.yaw);
        pose.pose.orientation = tf2::toMsg(orientation);
        
        path_msg.poses.push_back(pose);
    }
    
    path_publisher_.publish(path_msg);
    
    visualization_msgs::MarkerArray markers;
    
    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    clear_marker.header.stamp = ros::Time::now();
    markers.markers.push_back(clear_marker);
    
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        const auto& waypoint = waypoints_[i];
        
        visualization_msgs::Marker arrow_marker;
        arrow_marker.header.frame_id = "map";
        arrow_marker.header.stamp = ros::Time::now();
        arrow_marker.ns = "patrol_arrows";
        arrow_marker.id = static_cast<int>(i);
        arrow_marker.type = visualization_msgs::Marker::ARROW;
        arrow_marker.action = visualization_msgs::Marker::ADD;
        
        arrow_marker.pose.position.x = waypoint.x;
        arrow_marker.pose.position.y = waypoint.y;
        arrow_marker.pose.position.z = 0.1;
        
        tf2::Quaternion arrow_orientation;
        arrow_orientation.setRPY(0, 0, waypoint.yaw);
        arrow_marker.pose.orientation = tf2::toMsg(arrow_orientation);
        
        arrow_marker.scale.x = 0.5;
        arrow_marker.scale.y = 0.1;
        arrow_marker.scale.z = 0.1;
        
        if (i == current_goal_index_ && current_state_ != PatrolState::Idle) {
            arrow_marker.color.r = 1.0;
            arrow_marker.color.g = 0.0;
            arrow_marker.color.b = 0.0;
        } else {
            arrow_marker.color.r = 0.0;
            arrow_marker.color.g = 0.5;
            arrow_marker.color.b = 1.0;
        }
        arrow_marker.color.a = 0.8;
        
        markers.markers.push_back(arrow_marker);
        
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "patrol_labels";
        text_marker.id = static_cast<int>(i) + 1000;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        text_marker.pose.position.x = waypoint.x;
        text_marker.pose.position.y = waypoint.y;
        text_marker.pose.position.z = 0.5;
        
        text_marker.scale.z = 0.3;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        text_marker.text = std::to_string(i + 1);
        
        markers.markers.push_back(text_marker);
    }
    
    marker_publisher_.publish(markers);
}

void PatrolNavigationPanel::clearVisualization() {
    nav_msgs::Path empty_path;
    empty_path.header.stamp = ros::Time::now();
    empty_path.header.frame_id = "map";
    path_publisher_.publish(empty_path);
    
    visualization_msgs::MarkerArray clear_markers;
    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    clear_marker.header.stamp = ros::Time::now();
    clear_markers.markers.push_back(clear_marker);
    marker_publisher_.publish(clear_markers);
}

void PatrolNavigationPanel::handleReversePath() {
    if (waypoints_.size() < 2) {
        showUserMessage("提示", "至少需要2个点位才能反向路径");
        return;
    }
    
    if (current_state_ != PatrolState::Idle) {
        showUserMessage("警告", "导航正在进行中，无法反向路径", QMessageBox::Warning);
        return;
    }
    
    reverseWaypoints();
    refreshTable();
    refreshVisualization();
    
    logPatrolEvent(LogEventType::PATH_MODIFY, QString("路径反向成功，总点位数量: %1").arg(waypoints_.size()));
    
    QString log_str = QString("路径反向成功 - 总点位数量: %1").arg(waypoints_.size());
    ROS_INFO("%s", log_str.toStdString().c_str());
    showUserMessage("成功", "路径已反向");
}

void PatrolNavigationPanel::reverseWaypoints() {
    std::reverse(waypoints_.begin(), waypoints_.end());
    
    for (auto& waypoint : waypoints_) {
        waypoint.yaw = fmod(waypoint.yaw + M_PI, 2 * M_PI);
        if (waypoint.yaw < 0) {
            waypoint.yaw += 2 * M_PI;
        }
    }
}

void PatrolNavigationPanel::handleViewLogs() {
    QDir log_dir(log_directory_);
    if (!log_dir.exists()) {
        showUserMessage("提示", "日志目录不存在", QMessageBox::Information);
        return;
    }
    
    bool success = QDesktopServices::openUrl(QUrl::fromLocalFile(log_directory_));
    if (success) {
        QString log_str = QString("日志目录已打开: %1").arg(log_directory_);
        ROS_INFO("%s", log_str.toStdString().c_str());
        logPatrolEvent(LogEventType::SYSTEM_START, log_str);
    } else {
        QString message = QString("无法自动打开日志目录，请手动访问：\n%1").arg(log_directory_);
        showUserMessage("警告", message, QMessageBox::Warning);
        QString log_str = QString("日志目录打开失败: %1").arg(log_directory_);
        ROS_WARN("%s", log_str.toStdString().c_str());
        logPatrolEvent(LogEventType::ERROR_OCCUR, log_str);
    }
}

void PatrolNavigationPanel::startNavigation() {
    if (waypoints_.empty()) {
        showUserMessage("警告", "请先添加巡检点", QMessageBox::Warning);
        return;
    }
    
    if (!ros::ok()) {
        showUserMessage("错误", "ROS节点未运行", QMessageBox::Critical);
        return;
    }
    
    std_srvs::Empty clear_service;
    if (clear_costmaps_client_.exists()) {
        clear_costmaps_client_.call(clear_service);
        ROS_INFO("代价地图清除成功");
    }
    
    resetNavigationState();
    current_goal_index_ = 0;
    navigation_permitted_ = true;
    is_cycling_ = cycle_checkbox_->isChecked();
    
    logPatrolEvent(LogEventType::PATROL_START, 
                  QString("巡检开始 - 点位数量: %1, 循环模式: %2, 循环间隔时间: %3秒")
                  .arg(waypoints_.size())
                  .arg(is_cycling_ ? "开启" : "关闭")
                  .arg(cycle_interval_spinbox_->value()));
    
    QTimer::singleShot(1000, this, [this]() {
        if (navigation_permitted_ && ros::ok()) {
            navigateToWaypoint(current_goal_index_);
        }
    });
    
    refreshUI();
    QString log_str = QString("巡检开始，点位数量: %1, 循环模式: %2, 循环间隔时间: %3秒")
                      .arg(waypoints_.size())
                      .arg(is_cycling_ ? "开启" : "关闭")
                      .arg(cycle_interval_spinbox_->value());
    ROS_INFO("%s", log_str.toStdString().c_str());
}

void PatrolNavigationPanel::navigateToWaypoint(int index) {
    if (!validatePointIndex(index)) {
        QString log_str = QString("无效的点位索引: %1 (总点位数量: %2)").arg(index).arg(waypoints_.size());
        ROS_ERROR("%s", log_str.toStdString().c_str());
        logPatrolEvent(LogEventType::ERROR_OCCUR, log_str);
        return;
    }
    
    const auto& target_waypoint = waypoints_[index];
    
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = target_waypoint.x;
    goal_pose.pose.position.y = target_waypoint.y;
    goal_pose.pose.position.z = 0.0;
    
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, target_waypoint.yaw);
    goal_pose.pose.orientation = tf2::toMsg(orientation);
    
    actionlib_msgs::GoalID cancel_msg;
    cancel_publisher_.publish(cancel_msg);
    
    QTimer::singleShot(500, this, [this, goal_pose, index, target_waypoint]() {
        if (navigation_permitted_ && ros::ok()) {
            goal_publisher_.publish(goal_pose);
            current_goal_id_.id.clear();
            
            refreshVisualization();
            
            ROS_INFO("新目标发布成功（取消旧目标后）");
            logPatrolEvent(LogEventType::SYSTEM_START, 
                          QString("导航至点位%1: (%2, %3), 朝向: %4°, 停留: %5秒")
                          .arg(index+1)
                          .arg(target_waypoint.x, 0, 'f', 2)
                          .arg(target_waypoint.y, 0, 'f', 2)
                          .arg(target_waypoint.yaw * RADIANS_TO_DEGREES, 0, 'f', 1)
                          .arg(target_waypoint.duration));
        }
    });
    
    current_state_ = PatrolState::Navigating;
    highlightCurrentWaypoint(index);
    
    QString log_str = QString("发布目标至点位 %1").arg(index + 1);
    ROS_INFO("%s", log_str.toStdString().c_str());
    
    log_str = QString("位置: (%1, %2), 朝向: %3°, 停留: %4秒")
              .arg(target_waypoint.x, 0, 'f', 2)
              .arg(target_waypoint.y, 0, 'f', 2)
              .arg(target_waypoint.yaw * RADIANS_TO_DEGREES, 0, 'f', 1)
              .arg(target_waypoint.duration);
    ROS_INFO("%s", log_str.toStdString().c_str());
    
    refreshUI();
}

void PatrolNavigationPanel::highlightCurrentWaypoint(int index) {
    for (int i = 0; i < waypoints_table_->rowCount(); ++i) {
        for (int j = 0; j < waypoints_table_->columnCount(); ++j) {
            if (auto* item = waypoints_table_->item(i, j)) {
                item->setBackground(QBrush());
            }
        }
    }
    
    if (index >= 0 && index < waypoints_table_->rowCount()) {
        QColor highlight_color = is_cycling_ ? 
            (completed_cycles_ % 2 == 0 ? QColor(255, 69, 0) : QColor(100, 149, 237)) : 
            QColor(255, 69, 0);
            
        for (int j = 0; j < waypoints_table_->columnCount(); ++j) {
            if (auto* item = waypoints_table_->item(index, j)) {
                item->setBackground(highlight_color);
            }
        }
    }
    
    waypoints_table_->viewport()->update();
}

bool PatrolNavigationPanel::checkGoalStatus(const std::vector<actionlib_msgs::GoalStatus>& status_list) {
    if (status_list.empty()) {
        ROS_DEBUG_THROTTLE(5, "未接收到目标状态");
        return false;
    }
    
    if (current_goal_id_.id.empty()) {
        for (const auto& status : status_list) {
            if (status.status == actionlib_msgs::GoalStatus::ACTIVE) {
                current_goal_id_ = status.goal_id;
                ROS_DEBUG("跟踪活跃目标: %s", current_goal_id_.id.c_str());
                break;
            }
        }
    }
    
    for (const auto& status : status_list) {
        if (status.goal_id == current_goal_id_) {
            switch (status.status) {
                case actionlib_msgs::GoalStatus::SUCCEEDED: {
                    QString log_str = QString("点位%1到达成功").arg(current_goal_index_ + 1);
                    ROS_INFO("%s", log_str.toStdString().c_str());
                    logPatrolEvent(LogEventType::WAYPOINT_ARRIVED, log_str);
                    return true;
                }
                case actionlib_msgs::GoalStatus::ABORTED: {
                    QString log_str = QString("点位%1导航中止").arg(current_goal_index_ + 1);
                    ROS_WARN("%s", log_str.toStdString().c_str());
                    logPatrolEvent(LogEventType::ERROR_OCCUR, log_str);
                    break;
                }
                case actionlib_msgs::GoalStatus::REJECTED: {
                    QString log_str = QString("点位%1目标被拒绝").arg(current_goal_index_ + 1);
                    ROS_WARN("%s", log_str.toStdString().c_str());
                    logPatrolEvent(LogEventType::ERROR_OCCUR, log_str);
                    break;
                }
                case actionlib_msgs::GoalStatus::PREEMPTED: {
                    QString log_str = QString("点位%1目标被抢占").arg(current_goal_index_ + 1);
                    ROS_WARN("%s", log_str.toStdString().c_str());
                    logPatrolEvent(LogEventType::ERROR_OCCUR, log_str);
                    break;
                }
                case actionlib_msgs::GoalStatus::LOST: {
                    QString log_str = QString("点位%1目标丢失").arg(current_goal_index_ + 1);
                    ROS_WARN("%s", log_str.toStdString().c_str());
                    logPatrolEvent(LogEventType::ERROR_OCCUR, log_str);
                    break;
                }
                case actionlib_msgs::GoalStatus::ACTIVE: {
                    QString log_str = QString("点位%1导航中").arg(current_goal_index_ + 1);
                    ROS_DEBUG_THROTTLE(10, "%s", log_str.toStdString().c_str());
                    break;
                }
                default: {
                    QString log_str = QString("点位%1状态: %2").arg(current_goal_index_ + 1).arg(status.status);
                    ROS_DEBUG_THROTTLE(10, "%s", log_str.toStdString().c_str());
                    break;
                }
            }
            break;
        }
    }
    return false;
}

void PatrolNavigationPanel::processGoalStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg) {
    if (current_state_ == PatrolState::Idle || current_state_ == PatrolState::Paused) {
        return;
    }
    
    static bool processing_status = false;
    if (processing_status) {
        return;
    }
    processing_status = true;
    
    bool current_goal_arrived = checkGoalStatus(status_msg->status_list);
    
    if (current_goal_arrived && current_goal_arrived != goal_arrived_ && ros::ok() && navigation_permitted_) {
        goal_arrived_ = current_goal_arrived;
        
        if (is_cycling_) {
            handleCycleNavigation();
        } else {
            handleCompleteNavigation();
        }
    } else {
        goal_arrived_ = current_goal_arrived;
    }
    
    processing_status = false;
}

void PatrolNavigationPanel::handleCompleteNavigation() {
    if (!validatePointIndex(current_goal_index_)) {
        QString log_str = QString("当前目标索引无效: %1").arg(current_goal_index_);
        ROS_ERROR("%s", log_str.toStdString().c_str());
        logPatrolEvent(LogEventType::ERROR_OCCUR, log_str);
        resetNavigationState();
        return;
    }
    
    const int wait_duration = waypoints_[current_goal_index_].duration;
    if (wait_duration > 0) {
        QString log_str = QString("在点位%1停留%2秒").arg(current_goal_index_ + 1).arg(wait_duration);
        ROS_INFO("%s", log_str.toStdString().c_str());
        logPatrolEvent(LogEventType::WAYPOINT_ARRIVED, log_str);
        current_state_ = PatrolState::Waiting;
        
        QTimer::singleShot(wait_duration * 1000, this, [this]() {
            if (current_state_ == PatrolState::Waiting && navigation_permitted_) {
                proceedToNextWaypoint();
            }
        });
    } else {
        proceedToNextWaypoint();
    }
    
    refreshUI();
}

void PatrolNavigationPanel::proceedToNextWaypoint() {
    current_goal_index_++;
    
    if (current_goal_index_ < static_cast<int>(waypoints_.size())) {
        QTimer::singleShot(500, this, [this]() {
            if (navigation_permitted_ && ros::ok()) {
                navigateToWaypoint(current_goal_index_);
            }
        });
    } else {
        if (is_cycling_) {
            handleCycleCompletion();
        } else {
            finishNavigation();
        }
    }
}

void PatrolNavigationPanel::handleCycleNavigation() {
    if (!validatePointIndex(current_goal_index_)) {
        QString log_str = QString("循环导航中目标索引无效: %1").arg(current_goal_index_);
        ROS_ERROR("%s", log_str.toStdString().c_str());
        logPatrolEvent(LogEventType::ERROR_OCCUR, log_str);
        resetNavigationState();
        return;
    }
    
    const int wait_duration = waypoints_[current_goal_index_].duration;
    
    if (wait_duration > 0) {
        current_state_ = PatrolState::Waiting;
        QTimer::singleShot(wait_duration * 1000, this, [this]() {
            if (current_state_ == PatrolState::Waiting && navigation_permitted_) {
                current_goal_index_ = (current_goal_index_ + 1) % waypoints_.size();
                completed_cycles_ = (current_goal_index_ == 0) ? completed_cycles_ + 1 : completed_cycles_;
                
                QTimer::singleShot(500, this, [this]() {
                    if (navigation_permitted_ && ros::ok()) {
                        navigateToWaypoint(current_goal_index_);
                    }
                });
            }
        });
    } else {
        current_goal_index_ = (current_goal_index_ + 1) % waypoints_.size();
        completed_cycles_ = (current_goal_index_ == 0) ? completed_cycles_ + 1 : completed_cycles_;
        
        QTimer::singleShot(500, this, [this]() {
            if (navigation_permitted_ && ros::ok()) {
                navigateToWaypoint(current_goal_index_);
            }
        });
    }
    
    refreshUI();
}

void PatrolNavigationPanel::handleCycleCompletion() {
    completed_cycles_++;
    current_goal_index_ = 0;
    
    const int cycle_interval = cycle_interval_spinbox_->value();
    
    QString log_str = QString("第%1轮循环完成，等待%2秒后开始新循环").arg(completed_cycles_).arg(cycle_interval);
    ROS_INFO("%s", log_str.toStdString().c_str());
    logPatrolEvent(LogEventType::CYCLE_COMPLETE, 
                  QString("第%1轮循环完成，循环间隔时间: %2秒").arg(completed_cycles_).arg(cycle_interval));
    
    if (cycle_interval > 0) {
        current_state_ = PatrolState::Waiting;
        QTimer::singleShot(cycle_interval * 1000, this, [this]() {
            if (navigation_permitted_ && ros::ok()) {
                navigateToWaypoint(current_goal_index_);
            }
        });
    } else {
        QTimer::singleShot(1000, this, [this]() {
            if (navigation_permitted_ && ros::ok()) {
                navigateToWaypoint(current_goal_index_);
            }
        });
    }
}

void PatrolNavigationPanel::finishNavigation() {
    QString log_str = QString("巡检完成，总循环次数: %1").arg(completed_cycles_);
    ROS_INFO("%s", log_str.toStdString().c_str());
    logPatrolEvent(LogEventType::PATROL_FINISH, QString("巡检完成 - 总循环次数: %1").arg(completed_cycles_));
    
    resetNavigationState();
    showUserMessage("完成", QString("所有导航目标已完成\n总共完成 %1 轮循环").arg(completed_cycles_));
}

void PatrolNavigationPanel::resetNavigationState() {
    current_state_ = PatrolState::Idle;
    current_goal_index_ = 0;
    completed_cycles_ = 0;
    navigation_permitted_ = false;
    goal_arrived_ = false;
    current_goal_id_.id.clear();
    
    actionlib_msgs::GoalID cancel_msg;
    cancel_publisher_.publish(cancel_msg);
    
    clearGlobalPlannerPaths();
    QTimer::singleShot(100, this, &PatrolNavigationPanel::clearGlobalPlannerPaths);
    
    highlightCurrentWaypoint(-1);
    
    refreshUI();
    refreshVisualization();
    
    ROS_INFO("导航状态已重置为就绪");
}

void PatrolNavigationPanel::processIncomingPoint(const geometry_msgs::PoseStamped::ConstPtr& point_msg) {
    if (current_state_ != PatrolState::Idle) {
        ROS_WARN("导航进行中，无法添加点位");
        showUserMessage("警告", "导航正在进行中，无法添加点");
        return;
    }
    
    PatrolPoint new_waypoint;
    new_waypoint.x = point_msg->pose.position.x;
    new_waypoint.y = point_msg->pose.position.y;
    
    tf2::Quaternion quaternion;
    tf2::fromMsg(point_msg->pose.orientation, quaternion);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    new_waypoint.yaw = yaw;
    new_waypoint.duration = 5;
    
    waypoints_.push_back(new_waypoint);
    refreshTable();
    refreshVisualization();
    
    logPatrolEvent(LogEventType::PATH_MODIFY, 
                  QString("通过RViz添加点位: (%1, %2, %3°), 停留: %4秒")
                  .arg(new_waypoint.x, 0, 'f', 2)
                  .arg(new_waypoint.y, 0, 'f', 2)
                  .arg(new_waypoint.yaw * RADIANS_TO_DEGREES, 0, 'f', 1)
                  .arg(new_waypoint.duration));
    
    QString log_str = QString("通过RViz添加点位: (%1, %2, %3°), 停留: %4秒")
                      .arg(new_waypoint.x, 0, 'f', 2)
                      .arg(new_waypoint.y, 0, 'f', 2)
                      .arg(new_waypoint.yaw * RADIANS_TO_DEGREES, 0, 'f', 1)
                      .arg(new_waypoint.duration);
    ROS_INFO("%s", log_str.toStdString().c_str());
}

void PatrolNavigationPanel::handleAddPoint() {
    bool dialog_ok = false;
    
    const double x = QInputDialog::getDouble(this, "添加点", "X坐标:", 0, -100, 100, 2, &dialog_ok);
    if (!dialog_ok) return;
    
    const double y = QInputDialog::getDouble(this, "添加点", "Y坐标:", 0, -100, 100, 2, &dialog_ok);
    if (!dialog_ok) return;
    
    const double yaw_degrees = QInputDialog::getDouble(this, "添加点", "朝向(度):", 0, -180, 180, 1, &dialog_ok);
    if (!dialog_ok) return;
    
    PatrolPoint new_waypoint;
    new_waypoint.x = x;
    new_waypoint.y = y;
    new_waypoint.yaw = yaw_degrees * DEGREES_TO_RADIANS;
    new_waypoint.duration = 5;
    
    waypoints_.push_back(new_waypoint);
    refreshTable();
    refreshVisualization();
    
    logPatrolEvent(LogEventType::PATH_MODIFY, 
                  QString("手动添加点位: (%1, %2, %3°), 停留: %4秒")
                  .arg(x, 0, 'f', 2)
                  .arg(y, 0, 'f', 2)
                  .arg(yaw_degrees, 0, 'f', 1)
                  .arg(new_waypoint.duration));
    
    QString log_str = QString("手动添加点位: (%1, %2, %3°), 停留: %4秒")
                      .arg(x, 0, 'f', 2)
                      .arg(y, 0, 'f', 2)
                      .arg(yaw_degrees, 0, 'f', 1)
                      .arg(new_waypoint.duration);
    ROS_INFO("%s", log_str.toStdString().c_str());
}

void PatrolNavigationPanel::handleRemovePoint() {
    const int selected_row = waypoints_table_->currentRow();
    if (selected_row < 0 || selected_row >= static_cast<int>(waypoints_.size())) {
        showUserMessage("警告", "请先选择要删除的点", QMessageBox::Warning);
        return;
    }
    
    const auto& deleted_point = waypoints_[selected_row];
    logPatrolEvent(LogEventType::PATH_MODIFY, 
                  QString("删除点位%1: (%2, %3, %4°), 停留: %5秒")
                  .arg(selected_row+1)
                  .arg(deleted_point.x, 0, 'f', 2)
                  .arg(deleted_point.y, 0, 'f', 2)
                  .arg(deleted_point.yaw * RADIANS_TO_DEGREES, 0, 'f', 1)
                  .arg(deleted_point.duration));
    
    waypoints_.erase(waypoints_.begin() + selected_row);
    
    if (selected_row == current_goal_index_) {
        if (current_goal_index_ >= waypoints_.size() && !waypoints_.empty()) {
            current_goal_index_ = waypoints_.size() - 1;
        } else if (waypoints_.empty()) {
            resetNavigationState();
        }
    } else if (selected_row < current_goal_index_) {
        current_goal_index_--;
    }
    
    refreshTable();
    refreshVisualization();
    QString log_str = QString("删除第%1行点位").arg(selected_row + 1);
    ROS_INFO("%s", log_str.toStdString().c_str());
}

void PatrolNavigationPanel::handleClearPoints() {
    if (waypoints_.empty()) {
        showUserMessage("提示", "路径已为空");
        return;
    }
    
    if (confirmUserAction("确认", "确定要清空所有点吗？")) {
        logPatrolEvent(LogEventType::PATH_MODIFY, QString("清空所有点位，数量: %1").arg(waypoints_.size()));
        
        waypoints_.clear();
        resetNavigationState();
        refreshTable();
        clearVisualization();
        ROS_INFO("清空所有点位");
    }
}

void PatrolNavigationPanel::handleSavePath() {
    if (waypoints_.empty()) {
        showUserMessage("警告", "没有点可保存", QMessageBox::Warning);
        return;
    }
    
    const QString filename = QFileDialog::getSaveFileName(this, "保存路径", 
                                                         QDir::homePath() + "/巡检路径.yaml", 
                                                         "YAML文件 (*.yaml)");
    if (filename.isEmpty()) return;
    
    try {
        YAML::Emitter yaml_emitter;
        yaml_emitter << YAML::BeginMap;
        yaml_emitter << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;
        
        for (const auto& waypoint : waypoints_) {
            yaml_emitter << YAML::BeginMap;
            yaml_emitter << YAML::Key << "x" << YAML::Value << waypoint.x;
            yaml_emitter << YAML::Key << "y" << YAML::Value << waypoint.y;
            yaml_emitter << YAML::Key << "yaw" << YAML::Value << waypoint.yaw;
            yaml_emitter << YAML::Key << "duration" << YAML::Value << waypoint.duration;
            yaml_emitter << YAML::EndMap;
        }
        
        yaml_emitter << YAML::EndSeq;
        yaml_emitter << YAML::EndMap;
        
        std::ofstream file_stream(filename.toStdString());
        if (!file_stream.is_open()) {
            throw std::runtime_error("无法打开文件进行写入");
        }
        file_stream << yaml_emitter.c_str();
        file_stream.close();
        
        logPatrolEvent(LogEventType::PATH_MODIFY, QString("路径保存至: %1").arg(filename));
        
        showUserMessage("成功", "路径保存成功");
        QString log_str = QString("路径保存至: %1").arg(filename);
        ROS_INFO("%s", log_str.toStdString().c_str());
    } catch (const std::exception& error) {
        logPatrolEvent(LogEventType::ERROR_OCCUR, QString("路径保存失败: %1").arg(error.what()));
        
        showUserMessage("错误", QString("保存失败: %1").arg(error.what()), QMessageBox::Critical);
        QString log_str = QString("路径保存失败: %1").arg(error.what());
        ROS_ERROR("%s", log_str.toStdString().c_str());
    }
}

void PatrolNavigationPanel::handleLoadPath() {
    const QString filename = QFileDialog::getOpenFileName(this, "加载路径", 
                                                         QDir::homePath(), 
                                                         "YAML文件 (*.yaml)");
    if (filename.isEmpty()) return;
    
    try {
        const YAML::Node config = YAML::LoadFile(filename.toStdString());
        
        if (!config["waypoints"]) {
            throw YAML::Exception(YAML::Mark::null_mark(), "缺少'waypoints'字段");
        }
        
        std::vector<PatrolPoint> loaded_waypoints;
        const YAML::Node waypoints_node = config["waypoints"];
        
        for (size_t i = 0; i < waypoints_node.size(); ++i) {
            PatrolPoint waypoint;
            waypoint.x = waypoints_node[i]["x"].as<double>();
            waypoint.y = waypoints_node[i]["y"].as<double>();
            waypoint.yaw = waypoints_node[i]["yaw"].as<double>();
            waypoint.duration = waypoints_node[i]["duration"].as<int>();
            loaded_waypoints.push_back(waypoint);
        }
        
        waypoints_ = loaded_waypoints;
        resetNavigationState();
        refreshTable();
        refreshVisualization();
        
        logPatrolEvent(LogEventType::PATH_MODIFY, 
                      QString("路径加载自: %1 (总点位数量: %2)")
                      .arg(filename).arg(waypoints_.size()));
        
        showUserMessage("成功", "路径加载成功");
        QString log_str = QString("路径加载自: %1, 总点位数量: %2")
                          .arg(filename).arg(waypoints_.size());
        ROS_INFO("%s", log_str.toStdString().c_str());
    } catch (const std::exception& error) {
        logPatrolEvent(LogEventType::ERROR_OCCUR, QString("路径加载失败: %1").arg(error.what()));
        
        showUserMessage("错误", QString("加载失败: %1").arg(error.what()), QMessageBox::Critical);
        QString log_str = QString("路径加载失败: %1").arg(error.what());
        ROS_ERROR("%s", log_str.toStdString().c_str());
    }
}

void PatrolNavigationPanel::handleStartPatrol() {
    startNavigation();
}

void PatrolNavigationPanel::handlePauseResume() {
    if (current_state_ == PatrolState::Navigating || current_state_ == PatrolState::Waiting) {
        current_state_ = PatrolState::Paused;
        navigation_permitted_ = false;
        
        actionlib_msgs::GoalID cancel_msg;
        cancel_publisher_.publish(cancel_msg);
        
        clearGlobalPlannerPaths();
        
        logPatrolEvent(LogEventType::PATROL_PAUSE, QString("巡检暂停于点位%1").arg(current_goal_index_+1));
        
        QString log_str = QString("巡检暂停于点位 %1").arg(current_goal_index_ + 1);
        ROS_INFO("%s", log_str.toStdString().c_str());
    } else if (current_state_ == PatrolState::Paused) {
        current_state_ = PatrolState::Navigating;
        navigation_permitted_ = true;
        navigateToWaypoint(current_goal_index_);
        
        logPatrolEvent(LogEventType::PATROL_RESUME, QString("巡检从点位%1恢复").arg(current_goal_index_+1));
        
        QString log_str = QString("巡检从点位 %1 恢复").arg(current_goal_index_ + 1);
        ROS_INFO("%s", log_str.toStdString().c_str());
    }
    
    refreshUI();
}

void PatrolNavigationPanel::handleStopPatrol() {
    if (current_state_ == PatrolState::Idle) {
        showUserMessage("提示", "导航未运行");
        return;
    }
    
    logPatrolEvent(LogEventType::PATROL_STOP, 
                  QString("巡检手动停止 - 当前点位: %1, 已完成循环: %2")
                  .arg(current_goal_index_+1).arg(completed_cycles_));
    
    resetNavigationState();
    showUserMessage("提示", "导航已停止");
    ROS_INFO("巡检手动停止");
}

void PatrolNavigationPanel::updateTimer() {
    if (ros::ok()) {
        ros::spinOnce();
    }
    
    refreshUI();
}

bool PatrolNavigationPanel::validatePointIndex(int index) const {
    return index >= 0 && index < static_cast<int>(waypoints_.size());
}

void PatrolNavigationPanel::showUserMessage(const QString& title, const QString& message, QMessageBox::Icon icon) {
    QMessageBox message_box(icon, title, message, QMessageBox::Ok, this);
    message_box.setWindowModality(Qt::ApplicationModal);
    message_box.exec();
}

bool PatrolNavigationPanel::confirmUserAction(const QString& title, const QString& message) {
    QMessageBox message_box(QMessageBox::Question, title, message, 
                           QMessageBox::Yes | QMessageBox::No, this);
    message_box.setWindowModality(Qt::ApplicationModal);
    return message_box.exec() == QMessageBox::Yes;
}

QString PatrolNavigationPanel::getCurrentTimestamp() {
    return QDateTime::currentDateTime().toString("yyyy年MM月dd日 hh时mm分ss秒");
}

QString PatrolNavigationPanel::logEventTypeToString(LogEventType type) {
    switch (type) {
        case LogEventType::SYSTEM_START:      return "系统启动";
        case LogEventType::PATH_MODIFY:       return "路径修改";
        case LogEventType::PATROL_START:      return "巡检开始";
        case LogEventType::PATROL_PAUSE:      return "巡检暂停";
        case LogEventType::PATROL_RESUME:     return "巡检恢复";
        case LogEventType::PATROL_STOP:       return "巡检停止";
        case LogEventType::WAYPOINT_ARRIVED:  return "到达点位";
        case LogEventType::CYCLE_COMPLETE:    return "循环完成";
        case LogEventType::PATROL_FINISH:     return "巡检完成";
        case LogEventType::ERROR_OCCUR:       return "错误发生";
        default:                              return "未知事件";
    }
}

void PatrolNavigationPanel::logPatrolEvent(LogEventType event_type, const QString& details) {
    if (!log_file_.is_open()) {
        ROS_WARN("日志文件未打开，无法记录事件");
        return;
    }
    
    QString timestamp = getCurrentTimestamp();
    QString event_name = logEventTypeToString(event_type);
    QString log_line = QString("[%1] [%2] %3")
                      .arg(timestamp)
                      .arg(event_name)
                      .arg(details.isEmpty() ? "无详细信息" : details);
    
    log_file_ << log_line.toStdString() << std::endl;
    log_file_.flush();
    
    if (event_type == LogEventType::ERROR_OCCUR) {
        ROS_ERROR("%s", log_line.toStdString().c_str());
    } else {
        ROS_INFO("%s", log_line.toStdString().c_str());
    }
}

} // namespace patrol_navigation_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(patrol_navigation_rviz_plugin::PatrolNavigationPanel, rviz::Panel)
