cmake_minimum_required(VERSION 3.8)
project(buraq_hw)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig REQUIRED)

# Find libserial using pkg-config
pkg_check_modules(SERIAL REQUIRED libserial)

# Include directories
include_directories(
  include
  ${SERIAL_INCLUDE_DIRS}
)

# Define the library target
add_library(
  ${PROJECT_NAME}
  SHARED
  src/diffdrive_4wd.cpp
  src/hw_driver.cpp
  src/wheel.cpp
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
  ${SERIAL_LIBRARIES}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface buraq_hw.xml)

# Install targets
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files if any
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Export dependencies
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Register the package
ament_package()
