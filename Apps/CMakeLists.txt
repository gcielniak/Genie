cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)

add_executable(GenieApp ${CMAKE_CURRENT_SOURCE_DIR}/src/GenieApp.cpp)
target_link_libraries(GenieApp GenieLib)

add_executable(GenieCalib ${CMAKE_CURRENT_SOURCE_DIR}/src/GenieCalib.cpp)
target_link_libraries(GenieCalib GenieLib)
