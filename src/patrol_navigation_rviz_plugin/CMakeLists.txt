cmake_minimum_required(VERSION 3.0.2)
project(patrol_navigation_rviz_plugin)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(catkin REQUIRED COMPONENTS
  rviz 
  geometry_msgs 
  nav_msgs 
  actionlib_msgs 
  visualization_msgs 
  std_srvs 
  tf 
  roscpp 
  rospy 
  std_msgs 
  pluginlib
)

find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)

# 启用自动MOC
set(CMAKE_AUTOMOC ON)

# 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# catkin包
catkin_package(
  LIBRARIES patrol_navigation_rviz_plugin
  CATKIN_DEPENDS 
    rviz 
    geometry_msgs 
    nav_msgs 
    actionlib_msgs 
    visualization_msgs 
    std_srvs 
    tf 
    roscpp 
    rospy 
    std_msgs 
    pluginlib
)

# 源文件 - 确保所有文件都列出
set(SOURCES
  src/patrol_navigation_panel.cpp
  src/patrol_point_tool.cpp
)

# 创建库
add_library(patrol_navigation_rviz_plugin ${SOURCES})

# 链接 - 确保链接所有必要的库
target_link_libraries(patrol_navigation_rviz_plugin
  ${catkin_LIBRARIES}
  Qt5::Widgets 
  Qt5::Core 
  Qt5::Gui
)

# 设置编译定义
target_compile_definitions(patrol_navigation_rviz_plugin
  PRIVATE 
    QT_NO_KEYWORDS
)

# 安装
install(TARGETS patrol_navigation_rviz_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES patrol_navigation_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
