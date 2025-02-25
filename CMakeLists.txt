cmake_minimum_required(VERSION 3.5)
project(chair_detector)

# ポリシーの設定
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

set(PCL_DIR "/usr/lib/cmake/pcl")  # 適切なPCLのCMakeディレクトリを指定

# 必要なROS2のパッケージを見つける
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)  # 追加
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)  # tf2を追加
find_package(tf2_ros REQUIRED)  # tf2_rosも追加
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(leg_detector src/leg_detector_v2.cpp)
target_link_libraries(leg_detector 
  ${PCL_LIBRARIES}
)
ament_target_dependencies(leg_detector
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
)

add_executable(scan_to_pointcloud src/scan_to_pointcloud.cpp)
ament_target_dependencies(scan_to_pointcloud rclcpp sensor_msgs laser_geometry)

add_executable(chair_detector src/chair_detector_v3.cpp)
target_link_libraries(chair_detector 
  ${PCL_LIBRARIES}
)
ament_target_dependencies(chair_detector
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# インストールの設定
install(TARGETS
  scan_to_pointcloud
  leg_detector
  chair_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
