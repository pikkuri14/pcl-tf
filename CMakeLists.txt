cmake_minimum_required(VERSION 3.0.2)
project(pcd_viewer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  config/point_cloud_rotator.cfg
)

catkin_package( ## if you dont add this, executables are not found
  INCLUDE_DIRS pcd launch config
  CATKIN_DEPENDS roscpp std_msgs pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcd_opener src/pcd_opener.cpp)
add_executable(pcd_transformer src/pcd_transformer.cpp)
add_executable(pcd_transformer_copy src/pcd_transformer_copy.cpp)

target_link_libraries(pcd_opener ${catkin_LIBRARIES})
target_link_libraries(pcd_transformer ${catkin_LIBRARIES})
target_link_libraries(pcd_transformer_copy ${catkin_LIBRARIES})

add_dependencies(pcd_transformer ${PROJECT_NAME}_gencfg)
add_dependencies(pcd_transformer_copy ${PROJECT_NAME}_gencfg)