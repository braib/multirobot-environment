cmake_minimum_required(VERSION 3.8)
project(multi_robot)


# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch models rviz urdf worlds params map 
  # meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multi_robot.dsv.in")


ament_package()
