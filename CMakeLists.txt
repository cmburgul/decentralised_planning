cmake_minimum_required(VERSION 2.8.3)
project(decentralised_planning)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  geometry_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   GetPath.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES decentralised_planning
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
 add_library(${PROJECT_NAME} src/path_planner.cpp)

## agent_feedback_publisher
add_executable(agent_feedback_publisher src/agent_feedback_publisher.cpp)
add_dependencies(agent_feedback_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(agent_feedback_publisher ${catkin_LIBRARIES})

## agent_feedback_subscriber
add_executable(agent_feedback_subscriber src/agent_feedback_subscriber.cpp)
add_dependencies(agent_feedback_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(agent_feedback_subscriber ${catkin_LIBRARIES})

## get_plan_server
add_executable(get_plan_server src/get_plan_server.cpp)
add_dependencies(get_plan_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_plan_server ${catkin_LIBRARIES})

## get_plan_client
add_executable(get_plan_client src/get_plan_client.cpp)
add_dependencies(get_plan_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_plan_client ${catkin_LIBRARIES})

## update_goal_server
add_executable(update_goal_server src/update_goal_server.cpp)
add_dependencies(update_goal_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(update_goal_server ${catkin_LIBRARIES})

## update_goal_client
add_executable(update_goal_client src/update_goal_client.cpp)
add_dependencies(update_goal_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(update_goal_client ${catkin_LIBRARIES})
