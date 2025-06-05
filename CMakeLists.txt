cmake_minimum_required(VERSION 3.10)
project(
	"CV_Project"
	VERSION 0.1.0
	LANGUAGES C CXX
	)


find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})

add_executable( Test main.cpp )
target_link_libraries( Test  gtsam )
