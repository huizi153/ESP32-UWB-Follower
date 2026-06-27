cmake_minimum_required(VERSION 3.8)
project(go2_uwb_follower)

# 如果是 C 则用 ament_cmake, C++ 用 ament_cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_api REQUIRED)

add_executable(go2_uwb_follower go2_uwb_follower.cpp)
ament_target_dependencies(go2_uwb_follower rclcpp unitree_api)

install(TARGETS go2_uwb_follower
        DESTINATION lib/${PROJECT_NAME})

ament_package()
