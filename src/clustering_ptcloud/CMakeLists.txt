cmake_minimum_required(VERSION 2.8.3)
project(clustering_ptcloud)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    pcl_ros
    geometry_msgs
    dynamic_reconfigure
    custom_msgs
    custom_libraries
)
find_package(Boost REQUIRED COMPONENTS thread)

generate_dynamic_reconfigure_options(
  cfg/params.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)
include_directories(include)

find_package(OpenCV REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
find_package(PCL 1.7.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
    ${catkin_INCLUDE_DIRS}
)
# New node replacing all four previous nodes
add_executable(pcl_obstacle_detector src/pcl_obstacle_detector.cpp)
add_dependencies(pcl_obstacle_detector ${PROJECT_NAME}_gencfg)
target_link_libraries(pcl_obstacle_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pcl_obstacle_detector ${catkin_EXPORTED_TARGETS})
add_dependencies(pcl_obstacle_detector ${PROJECT_NAME}_gencfg)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
