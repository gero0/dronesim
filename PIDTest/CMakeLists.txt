set(QT_DEBUG_FIND_PACKAGE ON)
set(QT_PATH /mnt/HDD/Qt/6.5.1/gcc_64)
#Include the path in prefix so cmake can find qt
set(CMAKE_PREFIX_PATH ${QT_PATH})
include(FetchContent)
find_package(Qt6 REQUIRED COMPONENTS Widgets Gui DataVisualization Charts)
qt_standard_project_setup()

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS ./src/*.cpp ./src/*.h)
add_executable(PIDTest ${SOURCES})
target_include_directories(PIDTest PRIVATE "../DroneController/src")
target_link_libraries(PIDTest PRIVATE DroneController)
target_link_libraries(PIDTest PRIVATE Qt6::Widgets Qt6::Gui Qt6::DataVisualization Qt6::Charts)