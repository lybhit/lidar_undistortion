cmake_minimum_required(VERSION 3.1)
project(scan_handle)

include(CheckIncludeFile)
include(CheckSymbolExists)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
# SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
# SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

find_package(catkin REQUIRED
  COMPONENTS
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    message_filters
    nav_msgs
    rosbag
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_msgs
    tf2_ros
    pcl_ros
)

#set(GLOG_INCLUDE_DIRS  /usr/include/glog)

find_package(Ceres REQUIRED)

find_package(Boost REQUIRED)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# dynamic reconfigure
# generate_dynamic_reconfigure_options(
    # config/AMCL.cfg
# )

catkin_package(
  CATKIN_DEPENDS
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    rosbag
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_msgs
    tf2_ros
    pcl_ros
  INCLUDE_DIRS include
  LIBRARIES amcl_sensors amcl_map amcl_pf amcl_ceres_scan_match 
)

# add_compile_options(-pg)
# set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)


include_directories(include)
#include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
# include_directories(src/include)

check_include_file(unistd.h HAVE_UNISTD_H)
if (HAVE_UNISTD_H)
  add_definitions(-DHAVE_UNISTD_H)
endif (HAVE_UNISTD_H)

check_symbol_exists(drand48 stdlib.h HAVE_DRAND48)
if (HAVE_DRAND48)
  add_definitions(-DHAVE_DRAND48)
endif (HAVE_DRAND48)

# add_library(amcl_ceres_scan_match STATIC
#                   src/amcl/ceres_scan_match/ceres_matcher.cpp
# )
# target_link_libraries(amcl_ceres_scan_match ${CERES_LIBRARIES})

# add_library(imu_tracker STATIC 
#                               src/amcl/imu_tracker/imu_tracker.cpp)

add_library(lidar_undistortion STATIC
                  src/scan_handle/lidar_undistortion/lidar_undistortion.cpp
)

# add_library(amcl_voxel_filter STATIC
#                   src/amcl/ceres_scan_match/voxel_filter_simple.cpp
# )

# target_link_libraries(amcl_ceres_scan_match  /usr/lib/x86_64-linux-gnu/libglog.so)

# add_library(amcl_pf STATIC
#                     src/amcl/pf/pf.c
#                     src/amcl/pf/pf_kdtree.c
#                     src/amcl/pf/pf_pdf.c
#                     src/amcl/pf/pf_vector.c
#                     src/amcl/pf/eig3.c
#                     src/amcl/pf/pf_draw.c
#                     src/amcl/pf/particle_filter_manager.cpp)


# add_library(amcl_map STATIC
#                     src/amcl/map/map.c
#                     src/amcl/map/map_cspace.cpp
#                     src/amcl/map/map_manager.cpp
#                     src/amcl/map/map_range.c
#                     src/amcl/map/map_store.c
#                     src/amcl/map/map_draw.c
#                     src/amcl/map/dynamic_map.cpp
#                     src/amcl/map/dynamic_map_manager.cpp
#                     src/amcl/range_data/range_data_collator.cpp
#                     src/amcl/map/map_generator.cpp)

# add_library(amcl_sensors STATIC
#                     src/amcl/sensors/laser_manager.cpp
#                     src/amcl/sensors/amcl_sensor.cpp
#                     src/amcl/sensors/amcl_odom.cpp
#                     src/amcl/sensors/amcl_laser.cpp)
# target_link_libraries(amcl_sensors amcl_map amcl_pf)

add_library(scan_manager STATIC
            src/scan_handle/scan_data_handle/scan_manager.cpp)
target_link_libraries(scan_manager lidar_undistortion)


# add_library(pose_cov_manager STATIC
#             src/amcl/pose_covariance_handle/pose_cov_manager.cpp)
# target_link_libraries(pose_cov_manager amcl_pf)

# add_library(loc_convert STATIC
#             src/amcl/loc_convert/loc_convert_manager.cpp
#             src/amcl/icp_match/icp_match.cpp)
# target_link_libraries(loc_convert amcl_map scan_manager ${PCL_LIBRARIES})

# add_library(tf_handle STATIC
#             src/amcl/tf_handle/tf_handle.cpp)
# target_link_libraries(tf_handle amcl_map amcl_pf amcl_ceres_scan_match)

# add_library(publish_handle STATIC
#             src/amcl/publisher_handle/publish_manager.cpp)
# target_link_libraries(publish_handle amcl_map amcl_pf)



# add_executable(amcl
#                        src/amcl_node.cpp src/amcl/amcl.cpp)
# add_dependencies(amcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# target_link_libraries(amcl
#     amcl_sensors amcl_map amcl_pf scan_manager tf_handle pose_cov_manager loc_convert publish_handle imu_tracker
#     ${Boost_LIBRARIES}
#     ${catkin_LIBRARIES}
#     ${CERES_LIBRARIES}
# )

add_executable(scan_test_node src/scan_test_node.cpp src/scan_test.cpp)
target_link_libraries(scan_test_node scan_manager ${catkin_LIBRARIES})

install(TARGETS scan_test_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS 
        lidar_undistortion scan_manager 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# install(TARGETS
#     amcl
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS
#     amcl_sensors amcl_map amcl_pf amcl_ceres_scan_match amcl_voxel_filter lidar_undistortion scan_manager pose_cov_manager loc_convert tf_handle publish_handle
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

install(DIRECTORY include/scan_handle/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# install(DIRECTORY config/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )

## Configure Tests
# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)

#   # Bags
#   catkin_download_test_data(${PROJECT_NAME}_basic_localization_stage_indexed.bag
#     http://download.ros.org/data/amcl/basic_localization_stage_indexed.bag
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#     MD5 41fe43af189ec71e5e48eb9ed661a655)
#   catkin_download_test_data(${PROJECT_NAME}_global_localization_stage_indexed.bag
#     http://download.ros.org/data/amcl/global_localization_stage_indexed.bag
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#     MD5 752f711cf4f6e8d1d660675e2da096b0)
#   catkin_download_test_data(${PROJECT_NAME}_small_loop_prf_indexed.bag
#     http://download.ros.org/data/amcl/small_loop_prf_indexed.bag
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#     MD5 e4ef0fc006872b43f12ed8a7ce7dcd81)
#   catkin_download_test_data(${PROJECT_NAME}_small_loop_crazy_driving_prg_indexed.bag
#     http://download.ros.org/data/amcl/small_loop_crazy_driving_prg_indexed.bag
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#     MD5 4a58d1a7962914009d99000d06e5939c)
#   catkin_download_test_data(${PROJECT_NAME}_texas_greenroom_loop_indexed.bag
#     http://download.ros.org/data/amcl/texas_greenroom_loop_indexed.bag
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#     MD5 6e3432115cccdca1247f6c807038e13d)
#   catkin_download_test_data(${PROJECT_NAME}_texas_willow_hallway_loop_indexed.bag
#     http://download.ros.org/data/amcl/texas_willow_hallway_loop_indexed.bag
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#     MD5 27deb742fdcd3af44cf446f39f2688a8)
#   catkin_download_test_data(${PROJECT_NAME}_rosie_localization_stage.bag
#     http://download.ros.org/data/amcl/rosie_localization_stage.bag
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#     MD5 3347bf3835724cfa45e958c5c1846066)

#   # Maps
#   catkin_download_test_data(${PROJECT_NAME}_willow-full.pgm
#     http://download.ros.org/data/amcl/willow-full.pgm
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#     MD5 b84465cdbbfe3e2fb9eb4579e0bcaf0e)
#   catkin_download_test_data(${PROJECT_NAME}_willow-full-0.05.pgm
#     http://download.ros.org/data/amcl/willow-full-0.05.pgm
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#     MD5 b61694296e08965096c5e78611fd9765)

  # Tests
  # add_rostest(test/set_initial_pose.xml)
  # add_rostest(test/set_initial_pose_delayed.xml)
  # add_rostest(test/basic_localization_stage.xml)
  # add_rostest(test/small_loop_prf.xml)
  # add_rostest(test/small_loop_crazy_driving_prg.xml)
  # add_rostest(test/texas_greenroom_loop.xml)
  # add_rostest(test/rosie_multilaser.xml)
  # add_rostest(test/texas_willow_hallway_loop.xml)

# Not sure when or if this actually passed.
#
# The point of this is that you start with an even probability
# distribution over the whole map and the robot localizes itself after
# some number of iterations of sensing and motion.
#
#  add_rostest(test/global_localization_stage.xml)
# endif()
