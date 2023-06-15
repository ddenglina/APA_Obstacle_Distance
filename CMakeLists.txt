cmake_minimum_required(VERSION 3.0)

project(Test_Distance)

# set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(OpenCV  REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS})


# 0.1鼠标点的坐标
add_executable(main main.cpp distance_utils.cpp)

target_link_libraries(main ${OpenCV_LIBS})

