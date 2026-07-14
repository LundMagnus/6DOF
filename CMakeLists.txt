cmake_minimum_required(VERSION 3.10.0)
project(Code VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(SDL2 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(ftxui CONFIG REQUIRED)



add_library(PCA9685 Libraries/PCA9685/PCA9685.cpp)
target_include_directories(PCA9685 PUBLIC Libraries/PCA9685)

add_library(Utilities Libraries/Utilities/Utilities.cpp)
target_include_directories(Utilities PUBLIC Libraries/Utilities)
target_link_libraries(PCA9685 PRIVATE Utilities)

add_library(Controller Libraries/Controller/Controller.cpp)
target_include_directories(Controller PUBLIC Libraries/Controller)
target_include_directories(Controller PUBLIC ${SDL2_INCLUDE_DIRS})
target_link_libraries(Controller PUBLIC ${SDL2_LIBRARIES})

add_library(Inverse_Kinematics Libraries/Inverse_Kinematics/Inverse_Kinematics.cpp)
target_include_directories(Inverse_Kinematics PUBLIC ${orocos_kdl_INCLUDE_DIRS})
target_link_libraries(Inverse_Kinematics PRIVATE ${orocos_kdl_LIBRARIES})

add_executable(Code main.cpp)
target_link_libraries(Code PRIVATE PCA9685)
target_link_libraries(Code PRIVATE Controller)
target_link_libraries(Code PRIVATE Inverse_Kinematics)
target_link_libraries(Code PRIVATE ftxui::screen ftxui::dom ftxui::component)

include(CTest)
enable_testing()

