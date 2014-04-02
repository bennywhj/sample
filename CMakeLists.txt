#project
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(sample)

#set build option
SET(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb -std=c++0x")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
SET(CMAKE_CXX_FLAGS "-msse3")

#include directory
set(SRC_DIR src)
#set(LIBOGP_SRC_DIR ogpsrc)

include_directories("${SRC_DIR}")
#include_directories("${LIBOGP_SRC_DIR}")

# sources
FILE(GLOB SRC_FILES "src/*.cpp")
#FILE(GLOB LIBOGP_SRC_FILES "ogpsrc/*.cpp")

#link
find_package( OpenCV REQUIRED )

#sample program
add_executable( sample sample.cpp ${SRC_FILES})
target_link_libraries( sample ${OpenCV_LIBS})









