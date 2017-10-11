cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)

set (source_files
	${CMAKE_CURRENT_SOURCE_DIR}/src/GenieApp.cpp)
	
add_executable(GenieApp ${source_files})
target_link_libraries(GenieApp GenieLib)
