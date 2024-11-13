cmake_minimum_required(VERSION 3.5)
project(scan_to_pointcloud)

# 必要な依存パッケージを指定
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)  # tf2を追加
find_package(tf2_ros REQUIRED)  # tf2_rosも追加

# 実行可能なノードのソースファイルを追加
add_executable(scan_to_pointcloud src/scan_to_pointcloud.cpp)

# リンカにライブラリを指定
ament_target_dependencies(scan_to_pointcloud rclcpp sensor_msgs laser_geometry)

# バイナリのインストールターゲットを指定
install(TARGETS
  scan_to_pointcloud
  DESTINATION lib/${PROJECT_NAME})

# パッケージをamentに登録
ament_package()
