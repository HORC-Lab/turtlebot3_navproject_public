cmake_minimum_required(VERSION 3.5)
project(nav2_custom_controller)

# =============================================================
# Basic build and dependency setup
# =============================================================
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Nav2-specific macro for consistent setup
nav2_package()

# Include header files from the include/ directory
include_directories(
  include
)

# =============================================================
# List all dependencies in one place
# =============================================================
set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_util
  nav2_costmap_2d
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# =============================================================
# Create the shared library for the controller plugin
# Students will write their logic in src/custom_controller.cpp
# =============================================================
add_library(${PROJECT_NAME} SHARED
  src/custom_controller.cpp
)

# Prevent pluginlib from using Boost for compatibility
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

# =============================================================
# Install targets and headers
# =============================================================
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# =============================================================
# Export dependencies and plugin info
# DO NOT EDIT THESE unless adding new dependencies
# =============================================================
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_core nav2_custom_controller.xml)

# =============================================================
# Optional linting and test tools
# =============================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # Skip copyright
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

