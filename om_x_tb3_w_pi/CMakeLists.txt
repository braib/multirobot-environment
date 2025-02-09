cmake_minimum_required(VERSION 3.8)
project(om_x_tb3_w_pi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


install(
  DIRECTORY 
  meshes 
  rviz 
  urdf 
  launch 
  worlds 
  config 
  # env-hooks 
  # gazebo 
  # ros2_control
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
