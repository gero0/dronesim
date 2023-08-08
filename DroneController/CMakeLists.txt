cmake_minimum_required(VERSION 3.25)
project(DroneController LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 23)

add_library(
  DroneController_Lib
  src/algebra.h
  src/algebra.cpp
  src/DroneController.h
  src/DroneController.cpp
  src/MotorDriver.h
  src/SensorReader.h
  src/PID.h
  src/PID.cpp)
