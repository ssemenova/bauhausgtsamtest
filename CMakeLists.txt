cmake_minimum_required(VERSION 3.10)
project(
	"CV_Project"
	VERSION 0.1.0
	LANGUAGES C CXX
	)

SET(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fPIC")#  -O3")

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})

add_executable( Test src/main.cpp )
target_link_libraries( Test  gtsam ${OpenCV_LIBS})
