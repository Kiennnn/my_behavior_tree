cmake_minimum_required(VERSION 3.0.2)
project(my_behavior_tree)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  behaviortree_cpp_v3
  message_generation
  actionlib
  move_base_msgs
)

add_service_files(
  FILES
  TakePackage.srv
  SetDeliveryPoint.srv
)

add_message_files(
  FILES
  OfficePose.msg
  DeliveryPose.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${BEHAVIOR_TREE_LIBRARY}
 CATKIN_DEPENDS roscpp message_runtime
 DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(my_bt_lib src/control/delivery.cpp src/control/charge.cpp
            src/reception/music.cpp src/reception/time_keeping.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(my_bt_lib ${BEHAVIOR_TREE_LIBRARY})

add_executable(move_robot src/move_robot.cpp src/control/delivery.cpp
              src/control/charge.cpp
              src/reception/music.cpp src/reception/time_keeping.cpp)
target_link_libraries(move_robot ${catkin_LIBRARIES})