cmake_minimum_required (VERSION 3.4)
project(McLib LANGUAGES CXX)
SET(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -fPIC")
aux_source_directory(. DIR_SRCS)
add_library(mclib SHARED  ${DIR_SRCS} )
target_link_libraries(mclib -lpthread)


