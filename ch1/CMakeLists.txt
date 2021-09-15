cmake_minimum_required(VERSION  3.1.2)
project(ch1)

set(CMAKE_CXX_ STANDARD 11)

find_package(Sophus REQUIRED)

include_directories("/usr/include/eigen3")
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(ch1_rotation ch1_rotation.cpp)

target_link_libraries(ch1_rotation Sophus::Sophus)




