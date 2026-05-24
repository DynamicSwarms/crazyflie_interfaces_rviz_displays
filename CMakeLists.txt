cmake_minimum_required(VERSION 3.20)
project(crazyflie_interfaces_rviz_displays)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(crazyflie_interfaces REQUIRED)

set(CMAKE_AUTOMOC ON)
qt6_wrap_cpp(MOC_FILES
  include/crazyflie_interfaces_rviz_displays/pose_named_array.hpp
)

add_library(pose_named_array
  src/pose_named_array.cpp
  ${MOC_FILES}
)

target_include_directories(pose_named_array PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pose_named_array PUBLIC
  pluginlib::pluginlib
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
  crazyflie_interfaces::crazyflie_interfaces
)

install(TARGETS pose_named_array
  EXPORT export_pose_named_array
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES rviz_common_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(FILES icons/classes/PoseNamedArray.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)


ament_export_include_directories(
  include
)

ament_export_targets(export_pose_named_array)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

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
