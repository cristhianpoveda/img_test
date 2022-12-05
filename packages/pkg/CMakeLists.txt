cmake_minimum_required(VERSION 2.8.3)
project(pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

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

add_executable(led_test src/led.cpp)
#set_target_properties(${PROJECT_NAME}_led_test PROPERTIES OUTPUT_NAME led_test PREFIX "")
add_dependencies(led_test ${${PROJECT_NAME}EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(led_test ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})
