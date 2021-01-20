cmake_minimum_required(VERSION 2.8.3)
project(ur_interface_pkg)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  trajectory_msgs
  message_generation
  kdl_parser
  kdl_conversions
)


generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS 

)
find_package (Eigen3)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include/utils
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


add_executable(ur_ee_manager_node src/ee_manager_node.cpp src/ee_manager.cpp src/qbmove_communications.cpp)
add_dependencies(ur_ee_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ur_ee_manager_node ${catkin_LIBRARIES})

add_executable(ur_manager_node src/ur_manager_node.cpp src/ur_manager.cpp)
add_dependencies(ur_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ur_manager_node ${catkin_LIBRARIES})

add_executable(ur_inv_kin_node src/ur_inv_kin.cpp)
add_dependencies(ur_inv_kin_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ur_inv_kin_node ${catkin_LIBRARIES})

add_executable(ur_homing_node src/ur_homing.cpp)
add_dependencies(ur_homing_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ur_homing_node ${catkin_LIBRARIES})