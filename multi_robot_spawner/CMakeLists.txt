cmake_minimum_required(VERSION 3.8)
project(multi_robot_spawner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(turtlebot3_gazebo)
find_package(turtlebot3_manipulation_bringup)
find_package(turtlebot3_manipulation_description)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
