cmake_minimum_required(VERSION 3.8)
project(my_rviz_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)


set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_BUILD_TYPE Debug)

set(headers_to_moc
  include/${PROJECT_NAME}/my_panel.hpp
  include/${PROJECT_NAME}/second_panel.hpp
  include/${PROJECT_NAME}/waypoint_editor.hpp
  include/${PROJECT_NAME}/waypoint_tool.hpp
)

qt5_wrap_cpp(MOC_FILES ${headers_to_moc})

set(plugin_sources
  src/my_panel.cpp
  src/second_panel.cpp
  src/waypoint_editor.cpp
  src/waypoint_tool.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} SHARED
  ${plugin_sources}
)

ament_target_dependencies(${PROJECT_NAME}
  "rviz_common"
  "rclcpp"
  "visualization_msgs"
  "interactive_markers"
  "rviz_default_plugins"
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()