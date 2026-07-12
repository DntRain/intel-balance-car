cmake_minimum_required(VERSION 3.8)
project(ld03_lidar)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(ld03_parser src/ld03_parser.cpp)
target_compile_features(ld03_parser PUBLIC cxx_std_17)
target_include_directories(ld03_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(ld03_lidar_node src/ld03_lidar_node.cpp)
target_compile_features(ld03_lidar_node PUBLIC cxx_std_17)
target_link_libraries(ld03_lidar_node ld03_parser)
ament_target_dependencies(ld03_lidar_node rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ld03_parser test/test_ld03_parser.cpp)
  target_link_libraries(test_ld03_parser ld03_parser)
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(TARGETS ld03_parser ld03_lidar_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
