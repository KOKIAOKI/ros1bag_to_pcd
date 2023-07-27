cmake_minimum_required(VERSION 2.8.3)
project(ros1bag_to_pcd)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        sensor_msgs
        pcl_conversions
        )

catkin_package(
    INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(offline src/offline.cpp)
target_link_libraries(offline ${catkin_LIBRARIES})
