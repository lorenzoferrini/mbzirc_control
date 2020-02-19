cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_controller)
find_package(catkin REQUIRED
COMPONENTS message_generation
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
   directionalPIDparam.msg
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

add_executable(directional_PID src/directional_PID.cpp)
add_dependencies(directional_PID ${catkin_EXPORTED_TARGETS})
target_link_libraries(directional_PID ${catkin_LIBRARIES})

add_executable(interceptor src/interceptor_controller.cpp)
add_dependencies(interceptor ${catkin_EXPORTED_TARGETS})
target_link_libraries(interceptor ${catkin_LIBRARIES})

add_executable(tgt_sim src/interceptor_tgt_simulator.cpp)
add_dependencies(tgt_sim ${catkin_EXPORTED_TARGETS})
target_link_libraries(tgt_sim ${catkin_LIBRARIES})

add_executable(CV_directional_PID src/CV_directional_PID.cpp)
add_dependencies(CV_directional_PID ${catkin_EXPORTED_TARGETS})
target_link_libraries(CV_directional_PID ${catkin_LIBRARIES})
