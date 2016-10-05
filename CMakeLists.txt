cmake_minimum_required(VERSION 2.8.3)
project(dns_controller)

find_package(catkin REQUIRED COMPONENTS std_msgs bupimo_msgs roscpp)

include_directories(${catkin_INCLUDE_DIRS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(
  dns_controller
  src/dns_controller.cpp 
)	 

target_link_libraries(dns_controller ${catkin_LIBRARIES})

