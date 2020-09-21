cmake_minimum_required(VERSION 3.0)

project(demo)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_compile_options(-std=c++11)
add_compile_options(-fPIC)
add_compile_options(-pthread)
add_compile_options(-DASIO_STANDALONE)
add_compile_options(-DASIO_HAS_STD_CHRONO)

ADD_EXECUTABLE(lookup ${CMAKE_CURRENT_SOURCE_DIR}/example/lookup.cpp)
ADD_EXECUTABLE(teach ${CMAKE_CURRENT_SOURCE_DIR}/example/teach.cpp)
ADD_EXECUTABLE(replay ${CMAKE_CURRENT_SOURCE_DIR}/example/replay.cpp)
ADD_EXECUTABLE(feedback ${CMAKE_CURRENT_SOURCE_DIR}/example/feedback.cpp)

target_link_libraries(lookup pthread robotapi.so)
target_link_libraries(teach pthread robotapi.so)
target_link_libraries(replay pthread robotapi.so)
target_link_libraries(feedback pthread robotapi.so)

