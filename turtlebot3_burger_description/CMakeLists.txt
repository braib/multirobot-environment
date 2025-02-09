cmake_minimum_required(VERSION 3.8)
project(turtlebot3_burger_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(joint_state_broadcaster REQUIRED)

# link_directories(
#   ${GAZEBO_LIBRARY_DIRS}
# )

# include_directories(
#   include
#   ${GAZEBO_INCLUDE_DIRS}
# )

# set(dependencies
#   "geometry_msgs"
#   "nav_msgs"
#   "rclcpp"
#   "sensor_msgs"
#   "tf2"
# )

# include_directories(include)

# add_executable(turtlebot3_drive src/turtlebot3_drive.cpp)
# ament_target_dependencies(turtlebot3_drive ${dependencies})





# error
# add_library(obstacle1 SHARED models/turtlebot3_dqn_world/obstacle_plugin/obstacle1.cc)
# target_link_libraries(obstacle1 ${GAZEBO_LIBRARIES})

# add_library(obstacle2 SHARED models/turtlebot3_dqn_world/obstacle_plugin/obstacle2.cc)
# target_link_libraries(obstacle2 ${GAZEBO_LIBRARIES})

# add_library(obstacles SHARED models/turtlebot3_dqn_world/obstacle_plugin/obstacles.cc)
# target_link_libraries(obstacles ${GAZEBO_LIBRARIES})
# error end




# install(TARGETS turtlebot3_drive
#   DESTINATION lib/${PROJECT_NAME}
# )


install(
  DIRECTORY meshes rviz urdf launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

# install(DIRECTORY include/
#   DESTINATION include/
# )

# ament_export_include_directories(include)

# ament_export_dependencies(geometry_msgs)
# ament_export_dependencies(nav_msgs)
# ament_export_dependencies(rclcpp)
# ament_export_dependencies(sensor_msgs)
# ament_export_dependencies(tf2)


ament_package()
