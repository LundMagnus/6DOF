cmake_minimum_required(VERSION 3.10.0)
project(Code VERSION 0.1.0 LANGUAGES C CXX)

add_library(pca9685 Libraries/pca9685/PCA9685.cpp)
target_include_directories(pca9685 PUBLIC Libraries/pca9685)

add_executable(Code main.cpp)
target_link_libraries(Code PRIVATE pca9685)

include(CTest)
enable_testing()

