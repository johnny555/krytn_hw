cmake_minimum_required(VERSION 3.8)
project(oled_display_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DisplayOutput.msg"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(include)

# Create the C++ node executable
add_executable(oled_display_node_exec src/oled_display_node.cpp) # Renamed target
ament_target_dependencies(oled_display_node_exec # Renamed target
  rclcpp 
  std_msgs
)

# Link against the generated message interfaces
rosidl_target_interfaces(oled_display_node_exec # Renamed target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install C++ executables
install(TARGETS
  oled_display_node_exec # Renamed target
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/display_writer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

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
