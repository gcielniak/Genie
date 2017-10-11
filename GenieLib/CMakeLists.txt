cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)

set(extra_include_dirs ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(extra_libs "")

# OpenCV
find_package(OpenCV REQUIRED)
set(extra_include_dirs ${extra_include_dirs} ${OpenCV_INCLUDE_DIRS})
set(extra_libs ${extra_libs} ${OpenCV_LIBS})

# Boost
find_package(Boost REQUIRED COMPONENTS system thread)
set (extra_libs ${extra_libs} ${Boost_LIBRARIES})
add_definitions(-D_SCL_SECURE_NO_WARNINGS)

# Common Vision Blox by Stemmer Imaging: GenICam interface
find_package(CVB REQUIRED)
set (extra_libs ${extra_libs} CVB::CVCDriver CVB::CVCUtilities CVB::CVGenApi)

set (header_files
	${CMAKE_CURRENT_SOURCE_DIR}/include/Camera.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/Parameter.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/AD_130GE.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/GenieNano.h)
	
add_library(GenieLib INTERFACE)

target_link_libraries(GenieLib INTERFACE ${extra_libs})
target_include_directories(GenieLib INTERFACE ${extra_include_dirs})
target_sources(GenieLib INTERFACE ${header_files})
