cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_controller)
find_package(catkin REQUIRED
message_generation
roscpp
std_msgs
geometry_msgs
mavros_msgs
nav_msgs
sensor_msgs
rospy
)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   PIDparam.msg
   triplePIDparam.msg
 )

 generate_messages(
   DEPENDENCIES  std_msgs mavros_msgs
 )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(basic_PID src/basic_PID.cpp)
add_dependencies(basic_PID ${catkin_EXPORTED_TARGETS})
target_link_libraries(basic_PID ${catkin_LIBRARIES})
