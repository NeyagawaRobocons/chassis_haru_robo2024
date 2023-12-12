cmake_minimum_required(VERSION 3.8)
project(chassis_haru_robo2024)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # tf2_geometry_msgsへの依存関係を追加
find_package(geometry_msgs REQUIRED) # PoseStampedを使うためのパッケージ
find_package(nucleo_agent REQUIRED) # ヌクレオと通信するためのパッケージ

# odometry_nodeの設定
# 実行ファイル名とソースファイルの指定
add_executable(odometry_node 
  src/odometry_node.cpp
  src/odometer_pub_sub.cpp
  src/odometry_class.cpp
)
# インクルードディレクトリの指定
target_include_directories(odometry_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(odometry_node
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nucleo_agent
)
install(TARGETS
  odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
