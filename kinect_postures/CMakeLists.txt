cmake_minimum_required(VERSION 2.8.3)
project(kinect_postures)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf message_generation)

## Declare ROS messages and services
add_message_files(FILES Move.msg)
# add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp std_msgs message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(simple_hands_detector src/simple_hands_detector.cpp)
add_executable(save_raw src/save_raw.cpp)
add_executable(save_angle src/save_angle.cpp src/transformations.cpp src/modeltree.cpp)
add_executable(postures src/postures.cpp src/posture_tracker.cpp src/transformations.cpp src/classifier.cpp src/modeltree.cpp src/randomforest.cpp src/centroid.cpp src/clustermodel.cpp)
add_executable(movement src/movements.cpp src/movement_tracker.cpp src/transformations.cpp src/classifier.cpp src/modeltree.cpp src/centroid.cpp src/clustermodel.cpp)
target_link_libraries(simple_hands_detector ${catkin_LIBRARIES})
target_link_libraries(save_raw ${catkin_LIBRARIES})
target_link_libraries(save_angle ${catkin_LIBRARIES})
target_link_libraries(postures ${catkin_LIBRARIES})
target_link_libraries(movement ${catkin_LIBRARIES})
