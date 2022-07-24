cmake_minimum_required(VERSION 3.5)
project(rviz_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(Eigen3 REQUIRED)

ament_export_dependencies(class_loader REQUIRED)
ament_export_dependencies(pluginlib REQUIRED)
ament_export_dependencies(rclcpp REQUIRED)
ament_export_dependencies(std_msgs REQUIRED)
ament_export_dependencies(rviz2 REQUIRED)
ament_export_dependencies(Eigen3 REQUIRED)

find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
macro(qt_wrap_ui)
  qt5_wrap_ui(${ARGN})
endmacro()
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
add_definitions(-DQT_NO_KEYWORDS)
# cmake settings #
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

include_directories(
  include
)
set(HEADERS
  include/rviz_plugins/yesno_publish_panel.h
)
qt5_wrap_ui(UIC_FILES
  src/ui/yesno_panel.ui
)
set(SOURCE_FILES
  src/yesno_publish_panel.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADERS} ${UIC_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_export_include_directories(include)
ament_export_targets(rviz_plugins HAS_LIBRARY_TARGET)
ament_target_dependencies(${PROJECT_NAME}
  std_msgs
  rviz2
  Qt5
  pluginlib
)
target_include_directories(${PROJECT_NAME} PRIVATE "${OGRE_PREFIX_DIR}/include")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_libraries(
  ${PROJECT_NAME}
  )
ament_package()
