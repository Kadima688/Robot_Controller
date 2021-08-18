cmake_minimum_required(VERSION 2.8.12.2)
project(robot-controller)
set(CMAKE_CXX_STANDARD 11)
#----------------跟调试相关的语句
SET(CMAKE_BUILD_TYPE "Debug")  
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -W -g2 -ggdb") #-w:关闭编译时的警告 -Wall:显示所有警告 -W:只显示编译器认为会出现错误的警告
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -W") 
#----------------添加自定义的宏定义
OPTION(DEFINE_USE_REALSENSE "REALSENSE Option Description" ON)# cmake .. -DDEFINE_USE_REALSENSE=true 默认使用使用realsense的代码
if(DEFINE_USE_REALSENSE)
  ADD_DEFINITIONS(-DUSE_REALSENSE)
endif(DEFINE_USE_REALSENSE)
# OPTION(DEFINE_USE_KERNEL "KERNEL Option Description" ON)
# if(DEFINE_USE_KERNEL)
#   ADD_DEFINITIONS(-DUSE_KERNEL)
# endif(DEFINE_USE_KERNEL)
#根据不同的系统添加不同的宏
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
  ADD_DEFINITIONS(-DLINUX)
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
  ADD_DEFINITIONS(-DWINDOWS)
endif()
#----------------找到visp对应的模块
find_package(VISP REQUIRED visp_robot visp_vision visp_io visp_gui visp_vs visp_visual_features visp_detection)
#---------------找到对应的头文件
include_directories(${PROJECT_SOURCE_DIR}/Module/core/include)
include_directories(${PROJECT_SOURCE_DIR}/Module/robot/include)
include_directories(${PROJECT_SOURCE_DIR}/Module/kernal/include)
#----------------找到对应的源文件
aux_source_directory(. sourceList)
aux_source_directory(${PROJECT_SOURCE_DIR}/Module/robot/src robot_c_files)
aux_source_directory(${PROJECT_SOURCE_DIR}/Module/core/src core_c_files)
aux_source_directory(${PROJECT_SOURCE_DIR}/Module/kernal/src kernal_c_files)
aux_source_directory(${PROJECT_SOURCE_DIR}/test test_files)
add_executable(controller ${sourceList} ${robot_c_files} ${core_c_files} ${kernal_c_files} ${test_files})#添加可执行文件
#--------------将程序连接一些库
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












