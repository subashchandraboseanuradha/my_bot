cmake_minimum_required(VERSION 3.16)
project(my_bot LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find all necessary dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Find libserial using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# Include directories
include_directories(
  hardware/include
  include
  ${LIBSERIAL_INCLUDE_DIRS}
  ${LIBSERIAL_INCLUDE_DIRS}/libserial
)

# Create the hardware plugin library
add_library(
  my_bot
  SHARED
  hardware/diffbot_system.cpp
)

target_include_directories(my_bot
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
)

# Link against dependencies using ament macros
ament_target_dependencies(my_bot
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Link against libserial explicitly
target_link_libraries(my_bot
  ${LIBSERIAL_LIBRARIES}  # Contains all necessary libserial linkage info
)

# Check if my_bot.xml exists
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/my_bot.xml")
  message(FATAL_ERROR "my_bot.xml does not exist in ${CMAKE_CURRENT_SOURCE_DIR}")
endif()

# Export hardware plugins
pluginlib_export_plugin_description_file(${PROJECT_NAME}
  "my_bot.xml"
)

# Add explicit XML file installation
install(FILES my_bot.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY 
    config
    description
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install targets
install(
  TARGETS my_bot
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
