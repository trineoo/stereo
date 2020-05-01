cmake_minimum_required(VERSION 2.8.3)
project(lidar_pcl)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    pcl_ros
    geometry_msgs
)

catkin_package(
    # INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)


find_package(PCL 1.7.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#find_package(Eigen3 REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

include_directories(
    ${catkin_INCLUDE_DIRS}
    #include
)

# New node replacing all four previous nodes
add_executable(pcl_obstacle_detector src/pcl_obstacle_detector.cpp)
target_link_libraries(pcl_obstacle_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pcl_obstacle_detector ${catkin_EXPORTED_TARGETS})
