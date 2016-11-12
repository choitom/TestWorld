cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(LidarInfo SHARED LidarInfo.cc)
target_link_libraries(LidarInfo ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(data SHARED dataProcessing.cc)
target_link_libraries(data LidarInfo ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(lidar SHARED LidarPlugin.cc)
target_link_libraries(lidar data ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(camera SHARED CameraPlugin.cc)
target_link_libraries(camera data ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_library(CarPlugin SHARED CarPlugin.cc)
target_link_libraries(CarPlugin data ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
