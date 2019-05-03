cmake_minimum_required (VERSION 3.0)

set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package (PkgConfig REQUIRED)
pkg_check_modules (GTK2 REQUIRED gtk+-2.0)


link_directories(
	/usr/lib
	/usr/local/lib
	/usr/lib/arm-linux-gnueabihf
	${GTK2_LIBRARY_DIRS}
)

include_directories(
	/usr/include
	/usr/local/include
	${GTK2_INCLUDE_DIRS}
)
set(projectName "output")

file(GLOB_RECURSE SRCS ./src/*.cpp ./src/*.hpp ./src/*.c ./src/*.h)

project (${projectName})

add_executable(${projectName} ${SRCS})

target_link_libraries(${projectName}
-lopencv_calib3d
-lopencv_contrib
-lopencv_core
-lopencv_features2d
-lopencv_flann
-lopencv_gpu
-lopencv_highgui
-lopencv_imgproc
-lopencv_legacy
-lopencv_ml
-lopencv_objdetect
-lopencv_ocl
-lopencv_photo
-lopencv_stitching
-lopencv_superres
-lopencv_ts
-lopencv_video
-lopencv_videostab
-lpthread
${GTK2_LIBRARIES}
-lboost_system
-lboost_thread
)
