cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11 -o3)

set(CXX_FLAGS "-g -Wall")

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
add_subdirectory(msgs)

include_directories(/usr/include/cppad)
include_directories(/usr/local/include/coin)
link_directories(/usr/local/lib)


# Build the stand-alone test program
add_executable(client_controller client.cpp aimotive_mpc.cpp)

target_link_libraries(client_controller world_state_msgs ${GAZEBO_LIBRARIES} ipopt)

