#include "patrol_point_tool.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

namespace patrol_navigation_rviz_plugin {

PatrolPointTool::PatrolPointTool()
{
  shortcut_key_ = 'p';
  point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("patrol_point", 1);
  
  // 设置工具属性
  setName("Patrol Point Tool");
  setDescription("Click on the map to add patrol points");
}

PatrolPointTool::~PatrolPointTool()
{
}

void PatrolPointTool::onPoseSet(double x, double y, double theta)
{
  // 创建 PoseStamped 消息
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map"; // 默认使用 map 坐标系
  
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  
  // 将角度转换为四元数
  tf::Quaternion quat = tf::createQuaternionFromYaw(theta);
  tf::quaternionTFToMsg(quat, pose.pose.orientation);
  
  // 发布点击的点位
  point_pub_.publish(pose);
  
  std::stringstream ss;
  ss << "Setting patrol point at:\n";
  ss << "  Position: (" << x << ", " << y << ")\n";
  ss << "  Orientation: " << theta << " radians";
  
  ROS_INFO("%s", ss.str().c_str());
  
  // 显示状态消息
  setStatus("Patrol point set! Use the Patrol Navigation Panel to manage points.");
}

} // namespace patrol_navigation_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(patrol_navigation_rviz_plugin::PatrolPointTool, rviz::Tool)
