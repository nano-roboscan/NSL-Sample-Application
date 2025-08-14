cmake_minimum_required(VERSION 3.1) 
project(nsl-dev VERSION 1.0)            

set(CMAKE_CXX_COMPILIER "/usr/bin/g++")
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g -O3 -Wno-write-strings")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,$ORIGIN")
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_BUILD_TYPE Release)
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#threads
find_package(Threads REQUIRED)

find_path(NSL_LIB_INCLUDE_DIR NAMES nanolib.h PATHS nsl_lib/include REQUIRED)
#find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/linux-7.5/shared REQUIRED)
find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/aarch-7.5/shared REQUIRED)
#find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/windows/shared REQUIRED)

add_compile_options(-Wno-deprecated-declarations)

#sample
add_executable(nsl-dev main.cpp videoSource.cpp)

target_link_libraries(nsl-dev  ${NSL_LIB_PATH} ${PCL_LIBRARIES} ${OpenCV_LIBS} Threads::Threads)
target_include_directories(nsl-dev PRIVATE ${NSL_LIB_INCLUDE_DIR})
