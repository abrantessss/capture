cmake_minimum_required(VERSION 3.10.2)
project(capture_gazebo)

# Add -Wno-dev to CLion CMake options to suppress warning about gazebo_dev
find_package(ament_cmake_auto REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
ament_auto_find_build_dependencies()

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Specify where to install the launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

#Gazebo plugin
add_library(gazebo_claw_plugin SHARED src/gazebo_claw_plugin.cpp)

target_link_libraries(gazebo_claw_plugin
    rclcpp::rclcpp
)

ament_target_dependencies(gazebo_claw_plugin
  gazebo_ros
  gazebo_dev
  rclcpp
  capture_msgs
)

install(TARGETS
  gazebo_claw_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

target_include_directories(gazebo_claw_plugin PUBLIC include)
ament_export_libraries(gazebo_claw_plugin)

ament_auto_package()
