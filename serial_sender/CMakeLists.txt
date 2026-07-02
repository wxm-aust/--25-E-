cmake_minimum_required(VERSION 2.8.3)
project(serial_sender)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_sender_node src/serial_sender_node.cpp)
target_link_libraries(serial_sender_node
  ${catkin_LIBRARIES}
)

add_executable(serial_receiver_node src/serial_receiver_node.cpp)
target_link_libraries(serial_receiver_node
  ${catkin_LIBRARIES}
)