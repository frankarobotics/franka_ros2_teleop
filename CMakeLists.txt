cmake_minimum_required(VERSION 3.8)
project(franka_ros2_teleop)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  franka_msgs
  sensor_msgs
  controller_manager_msgs
  Eigen3
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rclcpp_action
)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
        ${PROJECT_NAME}
        SHARED
        src/teleop_leader_controller.cpp
        src/teleop_follower_controller.cpp
)

target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(
        ${PROJECT_NAME}
        PUBLIC
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(
        controller_interface franka_ros2_teleop.xml)

add_executable(teleop_coordinator src/teleop_coordinator.cpp)
target_include_directories(teleop_coordinator PRIVATE include)
ament_target_dependencies(teleop_coordinator ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(teleop_gripper_node src/teleop_gripper_node.cpp)
target_include_directories(teleop_gripper_node PRIVATE include)
ament_target_dependencies(teleop_gripper_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS
  teleop_coordinator
  teleop_gripper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        controller_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
)

ament_package()
