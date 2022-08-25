cmake_minimum_required(VERSION 2.8.3)
project(edge_tracker)

######################
### Cmake flags
######################
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

######################
### Packages
######################
find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    rospy
    roslib
    # msg
    std_msgs
    sensor_msgs
    geometry_msgs
    control_msgs
    nav_msgs
    # cv
    cv_bridge
    # pcl
    pcl_conversions
    # msg generation
    message_generation
    livox_ros_driver
)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system timer)


######################
### Message generation
######################
add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  control_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

######################
### Catkin
######################
catkin_package(
    CATKIN_DEPENDS roscpp rospy message_runtime std_msgs nav_msgs geometry_msgs control_msgs
    DEPENDS PCL EIGEN3
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
)

######################
### lidar odometry
######################

# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
# Preprocessing
add_executable(${PROJECT_NAME}_Preprocessing src/Preprocessing.cpp)
target_link_libraries(${PROJECT_NAME}_Preprocessing ${catkin_LIBRARIES})

