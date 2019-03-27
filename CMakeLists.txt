cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_control)
find_package(catkin REQUIRED
message_generation
roscpp
std_msgs
geometry_msgs
mavros_msgs
nav_msgs
sensor_msgs
)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   PIDparam.msg
   triplePIDparam.msg
#   Message1.msg
#   Message2.msg
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

add_executable(basic_PID src/basic_PID.cpp)
add_dependencies(basic_PID ${catkin_EXPORTED_TARGETS})
target_link_libraries(basic_PID ${catkin_LIBRARIES})
