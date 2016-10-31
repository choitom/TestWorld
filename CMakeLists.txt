cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(lidar SHARED LidarPlugin.cc)
target_link_libraries(lidar ${GAZEBO_libraries} ${Boost_LIBRARIES})

add_library(camera SHARED CameraPlugin.cc)
target_link_libraries(camera ${GAZEBO_libararies} ${Boost_LIBRARIES} ${OpenCV_LIBS})
