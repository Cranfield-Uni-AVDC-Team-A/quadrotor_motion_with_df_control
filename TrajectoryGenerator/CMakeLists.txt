cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME trajectory_generator)
project(${PROJECT_NAME})
add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp  
  std_msgs
  sensor_msgs
  mav_msgs
  tf2_ros
  geometry_msgs
  mav_trajectory_generation_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs mav_msgs tf2_ros geometry_msgs  mav_trajectory_generation_ros
  
  )
  
find_package(mav_trajectory_generation_ros)
find_package(Eigen3)

include_directories(
	${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
)

set(CPP_PROJECT_FILES 
    
    src/trajectory_generator.cpp
    src/trajectory_publisher.cpp
    src/eth_traj_wrapper.cpp
    
)
set(HEADER_PROJECT_FILES
    
    include/trajectory_generator.hpp
    include/trajectory_publisher.hpp
    include/eth_traj_wrapper.hpp
)


#  declare library
add_library(${PROJECT_NAME}
  ${CPP_PROJECT_FILES}
  ${HEADER_PROJECT_FILES}
 )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_node src/main_trajectory_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node_aerostack src/main_trajectory_publisher_aerostack_test.cpp)
target_link_libraries(${PROJECT_NAME}_node_aerostack ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_node_aerostack ${catkin_EXPORTED_TARGETS})
