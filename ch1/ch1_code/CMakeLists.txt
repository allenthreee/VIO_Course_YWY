cmake_minimum_required(VERSION  3.1.2)
project(ch1)

set(CMAKE_CXX_ STANDARD 11)

find_package(Sophus REQUIRED)

include_directories("/usr/include/eigen3")
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(ch1_rotation_use_libs ch1_rotation_use_libs.cpp)
add_executable(ch1_rotation_handmade ch1_rotation_handmade.cpp)

target_link_libraries(ch1_rotation_use_libs Sophus::Sophus)
target_link_libraries(ch1_rotation_handmade Sophus::Sophus)




