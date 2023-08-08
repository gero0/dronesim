cmake_minimum_required(VERSION 3.25)
project(DroneSim LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 23)

set(QT_DEBUG_FIND_PACKAGE ON)
if(DEFINED ENV{QT6_PATH})
  message(STATUS "Found QT6 at $ENV{QT6_PATH}")
else ()
  message(FATAL_ERROR "No path to QT found! Set QT6_PATH env variable")
endif()
set(CMAKE_PREFIX_PATH $ENV{QT6_PATH})
include(FetchContent)
find_package(Qt6 REQUIRED COMPONENTS Widgets Gui DataVisualization Charts)
qt_standard_project_setup()

add_subdirectory(../DroneController DroneController)

add_executable(
  DroneSim
  src/DroneModel.cpp
  src/DroneModel.h
  src/MainWindow.cpp
  src/MainWindow.h
  src/MotorDriverMock.h
  src/PIDPlot.cpp
  src/PIDPlot.h
  src/PIDSettings.cpp
  src/PIDSettings.h
  src/PIDWindow.cpp
  src/PIDWindow.h
  src/PlotWindow.cpp
  src/PlotWindow.h
  src/main.cpp)

target_include_directories(DroneSim PRIVATE "../DroneController/src")
target_link_libraries(DroneSim PRIVATE DroneController_Lib)
target_link_libraries(DroneSim PRIVATE Qt6::Widgets Qt6::Gui
                                       Qt6::DataVisualization Qt6::Charts)
