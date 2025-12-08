#ifndef PATROL_POINT_TOOL_H
#define PATROL_POINT_TOOL_H

#include <ros/ros.h>
#include <rviz/default_plugin/tools/pose_tool.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

namespace patrol_navigation_rviz_plugin {

class PatrolPointTool : public rviz::PoseTool
{
Q_OBJECT
public:
  PatrolPointTool();
  virtual ~PatrolPointTool();

  // 使用正确的函数签名
  virtual void onPoseSet(double x, double y, double theta);

private:
  ros::NodeHandle nh_;
  ros::Publisher point_pub_;
};

} // namespace patrol_navigation_rviz_plugin

#endif // PATROL_POINT_TOOL_H
