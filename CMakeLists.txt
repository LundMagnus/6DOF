cmake_minimum_required(VERSION 3.10.0)
project(arm_control VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(SDL2 QUIET)
if(TARGET SDL2::SDL2)
    set(SDL2_INCLUDE_DIRS "")
    set(SDL2_LIBRARIES SDL2::SDL2)
elseif(NOT SDL2_FOUND OR NOT SDL2_LIBRARIES)
    find_path(SDL2_INCLUDE_DIR NAMES SDL2/SDL.h)
    find_library(SDL2_LIBRARY NAMES SDL2)
    if(NOT SDL2_INCLUDE_DIR OR NOT SDL2_LIBRARY)
        message(FATAL_ERROR "SDL2 development files not found. Install libsdl2-dev.")
    endif()
    set(SDL2_INCLUDE_DIRS "${SDL2_INCLUDE_DIR}")
    set(SDL2_LIBRARIES "${SDL2_LIBRARY}")
endif()
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

add_executable(arm_control_node
    main.cpp
    Libraries/ArmNode/ArmNode.cpp
)
target_include_directories(arm_control_node PRIVATE Libraries/ArmNode)
target_link_libraries(arm_control_node
    PCA9685
    Controller
    Inverse_Kinematics
    ftxui::screen
    ftxui::dom
    ftxui::component
)
ament_target_dependencies(arm_control_node rclcpp sensor_msgs)
install(TARGETS arm_control_node DESTINATION lib/${PROJECT_NAME})

include(CTest)
enable_testing()

ament_package()

