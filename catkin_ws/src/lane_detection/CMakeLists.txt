cmake_minimum_required(VERSION 3.0.2)
project(lane_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs
  nav_msgs
  cv_bridge
  image_transport
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(YAML-CPP REQUIRED)

add_message_files(
  FILES
  xycar_motor.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs visualization_msgs nav_msgs cv_bridge image_transport message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  include
)

add_executable(lane_detection_node src/main.cpp src/LaneDetectionNode.cpp src/HoughTransformLaneDetector.cpp)
target_link_libraries(lane_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(lane_detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
