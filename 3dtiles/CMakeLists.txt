# bump version here
set(3dtiles_VERSION 1.0)

set(3dtiles_EXTRA_SOURCES)
set(3dtiles_EXTRA_DEPENDS)
if(MODULE_roarchive_FOUND)
  message(STATUS "3dtiles: compiling in roarchive support")
  list(APPEND 3dtiles_EXTRA_DEPENDS roarchive)
  list(APPEND 3dtiles_EXTRA_SOURCES reader.hpp reader.cpp)
else()
  message(STATUS "3dtiles: compiling without roarchive support")
endif()

define_module(LIBRARY 3dtiles=${3dtiles_VERSION}
  DEPENDS
  ${3dtiles_EXTRA_DEPENDS}
  gltf math>=1.4 utility>=1.26 dbglog>=1.4 imgproc>=1.28
  Boost_FILESYSTEM Boost_IOSTREAMS
  )

set(3dtiles_SOURCES
  3dtiles.hpp 3dtiles.cpp
  b3dm.hpp b3dm.cpp

  ${3dtiles_EXTRA_SOURCES}
  )

add_library(3dtiles STATIC ${3dtiles_SOURCES})
buildsys_library(3dtiles)

target_link_libraries(3dtiles ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(3dtiles ${MODULE_DEFINITIONS})
