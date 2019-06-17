cmake_minimum_required(VERSION 3.14)
project(vio)

find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INLCUDE_DIRS})

add_subdirectory(src)