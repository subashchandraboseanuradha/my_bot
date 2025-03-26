cmake_minimum_required(VERSION 3.8)
project(my_bot)

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
pkg_check_modules(SERIAL libserial REQUIRED)

# Hardware interface library
add_library(
  ${PROJECT_NAME}
  SHARED
  hardware/diffbot_system.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  hardware/include
  ${SERIAL_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${SERIAL_LIBRARIES}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Install
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY hardware/include/
  DESTINATION include
)

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface my_bot.xml)

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

ament_package() 