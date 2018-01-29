cmake_minimum_required(VERSION 2.8)
project(PathFindCore)

#None Debug Release RelWithDebInfo MinSizeRel
SET(CMAKE_VERBOSE_MAKEFILE TRUE) 
set(CMAKE_BUILD_TYPE Release)
set(LIBRARY_OUTPUT_PATH ../out)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
INCLUDE_DIRECTORIES(include)
aux_source_directory(./src LIB_SRC)
add_library(PathFindCore SHARED ${LIB_SRC})