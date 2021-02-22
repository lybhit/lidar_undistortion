cmake_minimum_required(VERSION 2.8.3)
project(simulate_data)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES LaserUndistortion
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(imu_node src/imu_node.cpp)

target_link_libraries(imu_node
  ${catkin_LIBRARIES}
)

add_executable(scan_node src/scan_node.cpp)

target_link_libraries(scan_node
  ${catkin_LIBRARIES}
)


add_executable(odom_node src/odom_node.cpp)
target_link_libraries(odom_node
  ${catkin_LIBRARIES}
)

add_executable(listen_scan_node src/listen_scan_node.cpp)
target_link_libraries(listen_scan_node ${catkin_LIBRARIES})

