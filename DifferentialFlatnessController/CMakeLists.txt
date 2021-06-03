cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME differential_flatness_controller)
project(${PROJECT_NAME})
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp  
  std_msgs
  sensor_msgs
  mav_msgs
  tf2_ros
  geometry_msgs
  dynamic_reconfigure
  mavros
  ros_utils_lib
  robot_process
)
find_package(Eigen3)

# Dynamic reconfigure stuff
find_package(dynamic_reconfigure)
generate_dynamic_reconfigure_options(
    cfg/pdcontroller.cfg
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs mav_msgs tf2_ros geometry_msgs dynamic_reconfigure mavros ros_utils_lib robot_process
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
	include
)


add_library(${PROJECT_NAME} src/PD_controller.cpp include/PD_controller.hpp )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )

add_executable(${PROJECT_NAME}_node src/main_nodes.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})
