#ifndef PATROL_NAVIGATION_PANEL_H
#define PATROL_NAVIGATION_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>

#include <QWidget>
#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>
#include <QComboBox>
#include <QSpinBox>
#include <QProgressBar>
#include <QGroupBox>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QHeaderView>
#include <QFileDialog>
#include <QInputDialog>
#include <QtCore>
#include <QDateTime>
#include <QDir>
#include <QDesktopServices>
#include <QUrl>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalID.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <string>
#include <fstream>
#include <ctime>
#include <algorithm>

namespace patrol_navigation_rviz_plugin {

class PatrolNavigationPanel : public rviz::Panel {
    Q_OBJECT

public:
    explicit PatrolNavigationPanel(QWidget *parent = nullptr);
    virtual ~PatrolNavigationPanel();

private Q_SLOTS:
    void handleAddPoint();
    void handleRemovePoint();
    void handleClearPoints();
    void handleSavePath();
    void handleLoadPath();
    void handleStartPatrol();
    void handlePauseResume();
    void handleStopPatrol();
    void updateTimer();
    void handleTableItemChanged(int row, int column);
    void handleReversePath();
    void handleViewLogs();  // 查看日志

private:
    // 巡检点结构体
    struct PatrolPoint {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
        int duration{5}; // 单个点位停留时间（秒）
        
        PatrolPoint() = default;
        PatrolPoint(double x, double y, double yaw, int duration = 5)
            : x(x), y(y), yaw(yaw), duration(duration) {}
    };

    // 巡检状态枚举
    enum class PatrolState { Idle, Navigating, Waiting, Paused };
    
    // 日志事件类型枚举
    enum class LogEventType {
        SYSTEM_START,      // 系统启动
        PATH_MODIFY,       // 路径修改（添加/删除/反转）
        PATROL_START,      // 巡检开始
        PATROL_PAUSE,      // 巡检暂停
        PATROL_RESUME,     // 巡检恢复
        PATROL_STOP,       // 巡检停止
        WAYPOINT_ARRIVED,  // 到达点位
        CYCLE_COMPLETE,    // 循环完成
        PATROL_FINISH,     // 巡检完成
        ERROR_OCCUR        // 错误发生
    };

    // 初始化函数
    void initializeUI();
    void initializeROS();
    void initializeLogging();  // 初始化日志系统
    
    // UI刷新函数
    void refreshUI();
    void refreshTable();
    void refreshVisualization();
    void clearVisualization();
    
    // 导航核心函数
    void navigateToWaypoint(int index);
    void proceedToNextWaypoint();
    void handleCompleteNavigation();
    void handleCycleNavigation();
    void handleCycleCompletion();
    void finishNavigation();
    void highlightCurrentWaypoint(int index);
    void reverseWaypoints();  // 反转路径逻辑
    
    // ROS回调处理
    void processGoalStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg);
    void processIncomingPoint(const geometry_msgs::PoseStamped::ConstPtr& point_msg);
    
    // 辅助函数
    bool checkGoalStatus(const std::vector<actionlib_msgs::GoalStatus>& status_list);
    bool validatePointIndex(int index) const;
    void showUserMessage(const QString& title, const QString& message, QMessageBox::Icon icon = QMessageBox::Information);
    bool confirmUserAction(const QString& title, const QString& message);
    void resetNavigationState();
    void startNavigation();
    void updateWaypointFromTable(int row);
    void clearGlobalPlannerPaths();  // 清除全局/局部规划路径
    
    // 日志相关函数（中文）
    void logPatrolEvent(LogEventType event_type, const QString& details = "");
    QString getCurrentTimestamp();
    QString logEventTypeToString(LogEventType type);

    // UI组件
    QTableWidget* waypoints_table_{nullptr};
    QPushButton* add_button_{nullptr};
    QPushButton* remove_button_{nullptr};
    QPushButton* clear_button_{nullptr};
    QPushButton* save_button_{nullptr};
    QPushButton* load_button_{nullptr};
    QPushButton* reverse_button_{nullptr};  // 反向路径按钮
    QPushButton* view_logs_button_{nullptr}; // 查看日志按钮
    QPushButton* start_button_{nullptr};
    QPushButton* pause_resume_button_{nullptr};
    QPushButton* stop_button_{nullptr};
    QCheckBox* cycle_checkbox_{nullptr};
    QSpinBox* cycle_interval_spinbox_{nullptr}; // 循环间隔时间（秒）
    QProgressBar* progress_bar_{nullptr};
    QLabel* status_label_{nullptr};
    QLabel* current_point_label_{nullptr};
    QLabel* point_count_label_{nullptr};
    QTimer* update_timer_{nullptr};

    // ROS通信相关
    ros::NodeHandle node_handle_;
    ros::Publisher goal_publisher_;
    ros::Publisher path_publisher_;
    ros::Publisher marker_publisher_;
    ros::Publisher cancel_publisher_;
    ros::Publisher global_plan_publisher_;
    ros::Publisher local_plan_publisher_;
    ros::Subscriber status_subscriber_;
    ros::Subscriber point_subscriber_;
    ros::ServiceClient clear_costmaps_client_;

    // 核心数据
    std::vector<PatrolPoint> waypoints_;
    int current_goal_index_{0};
    int completed_cycles_{0};
    PatrolState current_state_{PatrolState::Idle};
    bool navigation_permitted_{false};
    bool is_cycling_{false};
    bool goal_arrived_{false};
    bool is_programmatic_table_update_{false};
    actionlib_msgs::GoalID current_goal_id_;
    
    // 日志相关（中文）
    QString log_directory_;    // 日志目录
    QString current_log_file_; // 当前日志文件路径
    std::ofstream log_file_;   // 日志文件流

    // 常量定义
    static constexpr int TIMER_INTERVAL_MS = 200;
    static constexpr double DEGREES_TO_RADIANS = M_PI / 180.0;
    static constexpr double RADIANS_TO_DEGREES = 180.0 / M_PI;
    static constexpr const char* LOG_DIR_NAME = "巡检日志";  // 中文日志目录名
};

} // namespace patrol_navigation_rviz_plugin

#endif // PATROL_NAVIGATION_PANEL_H
