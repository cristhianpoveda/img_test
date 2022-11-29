cmake_minimum_required(VERSION 2.8.3)
project(pedestrian)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS
    roscpp
    std_msgs
    cv_bridge
    image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(pedestrian src/pedestrian.cpp)
#set_target_properties(${PROJECT_NAME}_pedestrian PROPERTIES OUTPUT_NAME pedestrian PREFIX "")
add_dependencies(pedestrian ${${PROJECT_NAME}EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pedestrian ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})