cmake_minimum_required(VERSION 2.8.3)
project(object_detection)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg geometry_msgs sensor_msgs pose_estimation)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
add_service_files(FILES DetectObject.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(CoViS REQUIRED HINTS $ENV{HOME}/CoViS $ENV{HOME}/Code/CoViS $ENV{HOME}/workspace/CoViS ../../../CoViS)
include_directories( ${COVIS_INCLUDE_DIR} )
link_directories( ${COVIS_LIBRARY_DIRS} )
MESSAGE("COVIS_INCLUDE_DIR    ${COVIS_INCLUDE_DIR}")

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES} ${COVIS_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(example example_cpp)

add_executable(service src/service.cpp)
target_link_libraries(service ${catkin_LIBRARIES} ${COVIS_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(service service_cpp)

