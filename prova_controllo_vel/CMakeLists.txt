
cmake_minimum_required(VERSION 2.8.3)
project(prova_controllo_vel)
find_package(catkin REQUIRED
message_generation
roscpp
rospy
std_msgs
geometry_msgs
mavros_msgs
nav_msgs
sensor_msgs
)


 generate_messages(
   DEPENDENCIES  std_msgs
 )

catkin_package(
CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(prova src/prova.cpp)
add_dependencies(prova ${catkin_EXPORTED_TARGETS})
target_link_libraries(prova ${catkin_LIBRARIES})
