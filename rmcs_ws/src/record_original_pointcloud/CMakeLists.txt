cmake_minimum_required(VERSION 3.8)
project(record_original_pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED) # 需要 pcl_conversions 来转换 ROS msg 到 PCL

# 可能还需要找到PCL本身，如果pcl_conversions没有完全满足
find_package(PCL REQUIRED)

add_executable(pointcloud_save_node src/point_cloud_save.cpp) # pointcloud_save_node 是你希望生成的可执行文件名

target_include_directories(pointcloud_save_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pointcloud_save_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

# 链接 PCL 库
target_link_libraries(pointcloud_save_node
  ${PCL_LIBRARIES}
)


install(TARGETS pointcloud_save_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
