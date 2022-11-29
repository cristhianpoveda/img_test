cmake_minimum_required(VERSION 2.8.3)
project(pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(testn src/main.cpp)
#set_target_properties(${PROJECT_NAME}_testn PROPERTIES OUTPUT_NAME testn PREFIX "")
add_dependencies(testn ${${PROJECT_NAME}EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(testn ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})
