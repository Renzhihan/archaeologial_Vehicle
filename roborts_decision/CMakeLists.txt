project(roborts_decision)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  nav_msgs
  geometry_msgs
  actionlib
  roborts_common
  roborts_msgs
  yhs_can_msgs
  )

find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  DecisionProtoSrc
  DecisionProtoHds
  ${ProtoFiles}
  )

catkin_package()

add_executable(along_attack_node
 ${DecisionProtoHds}
  ${DecisionProtoSrc}
  action_node/frozen_action.h
  action_node/descend_bucket_action.h
  action_node/go_entrance_action.h
  action_node/go_loading_action.h
  action_node/go_unloading_action.h
  action_node/lift_bucket_action.h
  action_node/Masked_interrupt.h
  action_node/go_electricity.h
  along_attack.cpp
  executor/chassis_executor.cpp
  )

target_link_libraries(along_attack_node
  PRIVATE
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )
add_dependencies(along_attack_node
  roborts_msgs_generate_messages)



  add_executable(test_node
 ${DecisionProtoHds}
  ${DecisionProtoSrc}
  action_node/frozen_action.h
  action_node/descend_bucket_action.h
  action_node/go_entrance_action.h
  action_node/go_loading_action.h
  action_node/go_unloading_action.h
  action_node/lift_bucket_action.h
  action_node/Masked_interrupt.h
  action_node/go_electricity.h
  test.cpp
  executor/chassis_executor.cpp
  )

target_link_libraries(test_node
  PRIVATE
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )
add_dependencies(test_node
  roborts_msgs_generate_messages)


  add_executable(eee
  ${DecisionProtoHds}
   ${DecisionProtoSrc}
   action_node/frozen_action.h
   action_node/descend_bucket_action.h
   action_node/go_entrance_action.h
   action_node/go_loading_action.h
   action_node/go_unloading_action.h
   action_node/lift_bucket_action.h
   action_node/Masked_interrupt.h
   action_node/go_electricity.h
   eeee.cpp
   executor/chassis_executor.cpp
   )
 
 target_link_libraries(eee
   PRIVATE
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   )
 add_dependencies(eee
   roborts_msgs_generate_messages)
