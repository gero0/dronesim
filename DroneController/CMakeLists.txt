cmake_minimum_required(VERSION 3.25)
project(DroneController LANGUAGES CXX)

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

set_property(TARGET DroneController_Lib PROPERTY CXX_STANDARD 17)
set_property(TARGET DroneController_Lib PROPERTY CXX_STANDARD_REQUIRED ON)