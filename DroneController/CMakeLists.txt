file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS ./src/*.cpp ./src/*.h)

add_library(DroneController ${SOURCES})
target_include_directories(DroneController PRIVATE "../Utilities")
target_link_libraries(DroneController PRIVATE Utilities)
set_target_properties(DroneController PROPERTIES LINKER_LANGUAGE CXX)