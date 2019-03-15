# bump version here
set(3dtiles_VERSION 1.0)

define_module(LIBRARY 3dtiles=${3dtiles_VERSION}
  DEPENDS gltf math>=1.4 utility>=1.26 dbglog>=1.4 imgproc>=1.28
  Boost_FILESYSTEM Boost_IOSTREAMS
  )

set(3dtiles_SOURCES
  3dtiles.hpp 3dtiles.cpp
  b3dm.hpp b3dm.cpp
  )

add_library(3dtiles STATIC ${3dtiles_SOURCES})
buildsys_library(3dtiles)

target_link_libraries(3dtiles ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(3dtiles ${MODULE_DEFINITIONS})
