cmake_minimum_required(VERSION 2.8.12)

project(mapping_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    message_generation
    rosgraph_msgs
)

add_message_files(
  FILES
    BoundingBox.msg
    BoundingBoxes.msg
    ObjectCount.msg
    BoxesAndClouds.msg
    ObjectPositions.msg
    FindObjects.msg
    SemanticMap.msg
    SemanticObject.msg
)


generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
    rosgraph_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    sensor_msgs
    message_runtime
    std_msgs
    rosgraph_msgs
)
