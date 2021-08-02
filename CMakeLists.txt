cmake_minimum_required(VERSION 2.8.12.2)
project(robot-controller)
set(CMAKE_CXX_STANDARD 11)
include_directories(${PROJECT_SOURCE_DIR}/Module/core/include)
include_directories(${PROJECT_SOURCE_DIR}/Module/robot/include)
aux_source_directory(. sourceList)
aux_source_directory(${PROJECT_SOURCE_DIR}/Module/robot/src robot_c_files)
aux_source_directory(${PROJECT_SOURCE_DIR}/Module/core/src core_c_files)
add_executable(controller ${sourceList} ${robot_c_files} ${core_c_files})
target_link_libraries(controller -lpthread)










