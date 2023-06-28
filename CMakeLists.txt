cmake_minimum_required(VERSION 3.25)
project(plotting LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(QT_DEBUG_FIND_PACKAGE ON)

set(QT_PATH /mnt/HDD/Qt/6.5.1/gcc_64)
#Include the path in prefix so cmake can find qt
set(CMAKE_PREFIX_PATH ${QT_PATH})

include(FetchContent)
find_package(Qt6 REQUIRED COMPONENTS Widgets Gui DataVisualization)
qt_standard_project_setup()

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp src/*.h)

add_executable(plotting ${SOURCES})
target_link_libraries(plotting PRIVATE Qt6::Widgets Qt6::Gui Qt6::DataVisualization)

