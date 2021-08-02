cmake_minimum_required(VERSION 2.8.12.2)
project(robot-controller)
# find_package(VISP REQUIRED visp_robot visp_vision visp_io visp_gui visp_vs visp_visual_features visp_detection)
set(CMAKE_CXX_STANDARD 11)
include_directories(${PROJECT_SOURCE_DIR}/Module/core/include)
include_directories(${PROJECT_SOURCE_DIR}/Module/robot/include)
aux_source_directory(. sourceList)
aux_source_directory(${PROJECT_SOURCE_DIR}/Module/robot/src robot_c_files)
aux_source_directory(${PROJECT_SOURCE_DIR}/Module/core/src core_c_files)
include_directories(${VISP_INCLUDE_DIRS})
add_executable(controller ${sourceList} ${robot_c_files} ${core_c_files} ${ARGN})
target_link_libraries(controller -lpthread ${VISP_LIBRARIES})
# add_dependencies(visp_${dependency} controller)
# set_target_properties(controller PROPERTIES FOLDER "/home/mingh/robot-controller/")
# project(robot-controller)

# cmake_minimum_required(VERSION 2.8.12.2)

# find_package(VISP REQUIRED visp_robot visp_vision visp_io visp_gui visp_vs visp_visual_features visp_detection)

# set(example_cpp
#   RobotController.cpp
# )

# foreach(cpp ${example_cpp})
#   visp_add_target(${cpp})
#   if(COMMAND visp_add_dependency)
#     visp_add_dependency(${cpp} "robot-controller")
#   endif()
# endforeach()


if(VISP_HAVE_REALSENSE2)
  if(UNIX)
    # Add specific build flag to turn off warnings coming from PCL 3rd party
    visp_set_source_file_compile_flag(RobotController.cpp Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-ignored-qualifiers)
  else()
    visp_set_source_file_compile_flag(RobotController.cpp /wd4244 /wd4267)
  endif()
endif()








