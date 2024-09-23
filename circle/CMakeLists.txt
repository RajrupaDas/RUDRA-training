cmake_minimum_required(VERSION 3.10)
project(circle)

find_package(OpenCV REQUIRED)

add_executable(circle circle.cpp)
target_link_libraries(circle ${OpenCV_LIBS})

