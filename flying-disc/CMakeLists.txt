cmake_minimum_required(VERSION 3.0)

set(PROJECT_NAME Camera)
project(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

file(GLOB_RECURSE HEADERS *.h)
source_group("Header Files" FILES ${HEADERS})
aux_source_directory(src DIR_SRCS)
add_executable(${PROJECT_NAME} ${DIR_SRCS} ${HEADERS})

# 多核处理
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "/MP")

# 管理员
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "/level='requireAdministrator'") 

# Unicode编码
Add_Definitions(-DUNICODE -D_UNICODE)

# OpenCV Include
set(OPENCV_INCLUDE "$ENV{OPENCV_DIR}\\build\\include;$ENV{OPENCV_DIR}\\build\\include\\opencv2")
include_directories(${OPENCV_INCLUDE})
set(OPENCV_LIB_DIRS $ENV{OPENCV_DIR}/build/x86/vc12/lib)

MACRO(LL LIB_BASE_NAME)
	target_link_libraries(${PROJECT_NAME} debug ${OPENCV_LIB_DIRS}/${LIB_BASE_NAME}d.lib)
	target_link_libraries(${PROJECT_NAME} optimized ${OPENCV_LIB_DIRS}/${LIB_BASE_NAME}.lib)
ENDMACRO()

# OpenCV_LIBS
LL(opencv_core2413)
LL(opencv_features2d2413)
LL(opencv_gpu2413)
LL(opencv_highgui2413)
LL(opencv_imgproc2413)
LL(opencv_legacy2413)
LL(opencv_ml2413)
LL(opencv_flann2413)
LL(opencv_contrib2413)
LL(opencv_calib3d2413)
LL(opencv_videostab2413)
LL(opencv_superres2413)
LL(opencv_stitching2413)
LL(opencv_photo2413)
LL(opencv_ocl2413)
LL(opencv_nonfree2413)
LL(opencv_video2413)
LL(opencv_ts2413)
LL(opencv_objdetect2413)

# Camera Driver
target_link_libraries(${PROJECT_NAME} $ENV{MV_DIR}/MVCAMSDK.lib)