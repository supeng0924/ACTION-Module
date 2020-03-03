cmake_minimum_required(VERSION 2.6)
set(PROJECT_NAME camerad)
project(${PROJECT_NAME})
# add_definitions(-std=c++11)
set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O3 -pthread" )


FIND_PACKAGE(OpenCV  REQUIRED )
set(LibPATHD435 "/usr/local/lib")
link_directories(${LibPATHD435})
include_directories(src/)
file(GLOB DIR_SRCS "src/*.cpp")


add_executable(${PROJECT_NAME} main.cpp ${DIR_SRCS})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${LibPATHD435}/librealsense2.so)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})


install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)