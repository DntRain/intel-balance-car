cmake_minimum_required(VERSION 3.8)
project(balance_bot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config scripts DESTINATION share/${PROJECT_NAME})

ament_package()
