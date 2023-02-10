cmake_minimum_required(VERSION 3.1) 
project(nsl-dev VERSION 1.0)            

set(CMAKE_CXX_COMPILIER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -Wno-write-strings")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,$ORIGIN")
set(THREADS_PREFER_PTHREAD_FLAG ON)


#opencv
find_package(OpenCV REQUIRED)
#threads
find_package(Threads REQUIRED)


#sample
add_executable(nsl-dev main.cpp nsl_driver/NSL1110AA.cpp nsl_driver/NSL3130AA.cpp nsl_driver/videoSource.cpp nsl_driver/lens_transform.cpp nsl_driver/NSL1110AA.h nsl_driver/NSL3130AA.h nsl_driver/NSLFrame.h nsl_driver/videoSource.h nsl_driver/lens_transform.h)
target_link_libraries(nsl-dev ${OpenCV_LIBS} Threads::Threads)
target_include_directories(nsl-dev PRIVATE nsl_driver)

