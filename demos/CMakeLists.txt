cmake_minimum_required(VERSION 3.0)

project(demo)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../aios/lib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../aios/include)

add_compile_options(-std=c++11)
add_compile_options(-fPIC)
add_compile_options(-pthread)
add_compile_options(-DASIO_STANDALONE)
add_compile_options(-DASIO_HAS_STD_CHRONO)

ADD_EXECUTABLE(lookup ${CMAKE_CURRENT_SOURCE_DIR}/src/lookup.cpp)
ADD_EXECUTABLE(teach ${CMAKE_CURRENT_SOURCE_DIR}/src/teach.cpp)
ADD_EXECUTABLE(replay ${CMAKE_CURRENT_SOURCE_DIR}/src/replay.cpp)
ADD_EXECUTABLE(feedback ${CMAKE_CURRENT_SOURCE_DIR}/src/feedback.cpp)
ADD_EXECUTABLE(config ${CMAKE_CURRENT_SOURCE_DIR}/src/config.cpp)

target_link_libraries(lookup pthread aiosapi.so)
target_link_libraries(teach pthread aiosapi.so libjsoncpp.so)
target_link_libraries(replay pthread aiosapi.so libjsoncpp.so)
target_link_libraries(feedback pthread aiosapi.so libjsoncpp.so)
target_link_libraries(config aiosapi.so libjsoncpp.so)

