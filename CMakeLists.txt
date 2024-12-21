cmake_minimum_required(VERSION 3.8)
project(pcd_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# Include PCL directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(ament_dependencies 
  rclcpp
  sensor_msgs
  PCL
  pcl_conversions
)

set(tf_dependencies
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
)

# Create the executable
add_executable(pcd_publisher src/pcd_publisher.cpp) 
ament_target_dependencies(pcd_publisher ${ament_dependencies}) 
target_link_libraries(pcd_publisher ${PCL_LIBRARIES})

add_executable(pcd_subscriber src/pcd_subscriber.cpp)
ament_target_dependencies(pcd_subscriber ${ament_dependencies} ${tf_dependencies})
target_link_libraries(pcd_subscriber ${PCL_LIBRARIES})

# Install the executable
install(TARGETS
  pcd_publisher
  pcd_subscriber
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
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
