file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS ./src/*.cpp ./src/*.h)

add_library(DroneController ${SOURCES})
set_target_properties(DroneController PROPERTIES LINKER_LANGUAGE CXX)