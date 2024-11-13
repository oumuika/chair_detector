cmake_minimum_required(VERSION 3.5)
project(detection)

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

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


# 実行可能ファイルの追加
add_executable(detection src/detection.cpp)

target_link_libraries(detection 
  ${PCL_LIBRARIES}
)

# ターゲットへの依存関係の追加
ament_target_dependencies(detection
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs  # ここで依存関係を追加
)

# インストールの設定
install(TARGETS detection
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
