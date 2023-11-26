
cmake_minimum_required(VERSION 3.19 )

project("demo")
# set(CMAKE_BUILD_TYPE Release)  
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++14 -Ofast")
set(CMAKE_BUILD_TYPE Debug)  # 没有这句话vscode无法单步调试
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++14 -O0")
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# OpenCV
find_package(OpenCV 3.2 REQUIRED) # 版本设置
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV_LIBS:${OpenCV_LIBS}")
message(STATUS "OpenCV_INCLUDE_DIRS:${OpenCV_INCLUDE_DIRS}")

add_executable(laser_undistort laser_undistort.cc ./src/lidar.cc ./src/robot.cc)
add_executable(grid_map draw_grid_map.cc ./src/lidar.cc ./src/robot.cc ./src/cost_map.cc)
add_executable(robot_move robot_move.cc ./src/lidar.cc ./src/robot.cc ./src/cost_map.cc)
add_executable(slam_no_matching slam_no_matching.cc ./src/lidar.cc ./src/robot.cc ./src/cost_map.cc) 

target_link_libraries(laser_undistort ${OpenCV_LIBS} pthread)
target_link_libraries(grid_map ${OpenCV_LIBS} pthread)
target_link_libraries(robot_move ${OpenCV_LIBS} pthread)
target_link_libraries(slam_no_matching ${OpenCV_LIBS} pthread)
