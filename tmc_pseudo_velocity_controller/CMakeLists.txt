cmake_minimum_required(VERSION 3.5)
project(tmc_pseudo_velocity_controller)

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tmc_manipulation_util REQUIRED)
find_package(tmc_msgs REQUIRED)
find_package(tmc_utils REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)

add_library(${PROJECT_NAME} SHARED src/pseudo_velocity_controller.cpp)
ament_target_dependencies(${PROJECT_NAME}
  message_filters
  sensor_msgs
  rclcpp
  tmc_manipulation_util
  tmc_msgs
  tmc_utils
  trajectory_msgs
  urdf
)

add_executable(pseudo_velocity_controller src/pseudo_velocity_controller_node_main.cpp)
target_link_libraries(pseudo_velocity_controller ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Boost REQUIRED)
  find_package(tmc_manipulation_tests REQUIRED)

  ament_add_gtest(pseudo_velocity_controller_test test/pseudo_velocity_controller-test.cpp)
  target_include_directories(pseudo_velocity_controller_test PRIVATE ${Boost_INCLUDE_DIRS})
  target_link_libraries(pseudo_velocity_controller_test ${PROJECT_NAME})
  ament_target_dependencies(pseudo_velocity_controller_test tmc_manipulation_tests)
endif()

install(TARGETS ${PROJECT_NAME} pseudo_velocity_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories()
ament_export_libraries()
ament_export_dependencies(rclcpp trajectory_msgs)

ament_package()
