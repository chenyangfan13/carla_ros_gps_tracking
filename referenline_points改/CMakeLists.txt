
cmake_minimum_required(VERSION 2.8.3)
project(referenceline_points)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  nav_msgs
)

find_package(Eigen3 REQUIRED)




catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES car_model
  CATKIN_DEPENDS roscpp rospy std_msgs 
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(referenceline_points src/referenceline_points.cpp)
target_link_libraries(referenceline_points ${catkin_LIBRARIES})
add_dependencies(referenceline_points ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

