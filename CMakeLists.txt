cmake_minimum_required(VERSION 2.8.12.2)
project(robot-controller)
set(CMAKE_CXX_STANDARD 11)
#----------------跟调试相关的语句
SET(CMAKE_BUILD_TYPE "Debug")  
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")  
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall") 
#----------------
find_package(VISP REQUIRED visp_robot visp_vision visp_io visp_gui visp_vs visp_visual_features visp_detection)
include_directories(${PROJECT_SOURCE_DIR}/Module/core/include)
include_directories(${PROJECT_SOURCE_DIR}/Module/robot/include)
include_directories(${PROJECT_SOURCE_DIR}/Module/kernal/include)
aux_source_directory(. sourceList)
aux_source_directory(${PROJECT_SOURCE_DIR}/Module/robot/src robot_c_files)
aux_source_directory(${PROJECT_SOURCE_DIR}/Module/core/src core_c_files)
aux_source_directory(${PROJECT_SOURCE_DIR}/Module/kernal/src kernal_c_files)
add_executable(controller ${sourceList} ${robot_c_files} ${core_c_files} ${kernal_c_files})
set (EXTRA_LIBS ${VISP_LIBRARIES})
target_link_libraries(controller -lpthread -lapi ${EXTRA_LIBS})  

#要用Realsense要添加这段代码
if(VISP_HAVE_REALSENSE2)
  if(UNIX)
    # Add specific build flag to turn off warnings coming from PCL 3rd party
    visp_set_source_file_compile_flag(RobotController.cpp -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-ignored-qualifiers)
  else()
    visp_set_source_file_compile_flag(RobotController.cpp /wd4244 /wd4267)
  endif()
endif()

# project(robot-controller)

# cmake_minimum_required(VERSION 2.8.12.2)

# find_package(VISP REQUIRED visp_robot visp_vision visp_io visp_gui visp_vs visp_visual_features visp_detection)

# set(example_cpp
#   RobotController.cpp
# )


# foreach(cpp ${example_cpp})
#   visp_add_target(${cpp})
#   if(COMMAND visp_add_dependency)
#     visp_add_dependency(${cpp} ./test.cpp "robot-controller")
#   endif()
# endforeach()












