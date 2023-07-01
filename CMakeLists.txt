cmake_minimum_required(VERSION 3.25)
project(DroneSimProject LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 23)

add_subdirectory(DroneSim)
add_subdirectory(DroneController)
add_subdirectory(PIDTest)
add_subdirectory(Utilities)

