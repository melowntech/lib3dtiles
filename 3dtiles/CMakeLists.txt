# bump version here
set(3dtiles_VERSION 1.4)

set(3dtiles_EXTRA_SOURCES)
set(3dtiles_EXTRA_DEPENDS)
if(MODULE_roarchive_FOUND)
  message(STATUS "3dtiles: compiling in roarchive support")
  list(APPEND 3dtiles_EXTRA_DEPENDS roarchive>=1.8)
  list(APPEND 3dtiles_EXTRA_SOURCES reader.hpp reader.cpp)
else()
  message(STATUS "3dtiles: compiling without roarchive support")
endif()

# encoding support (needs VTS libraries)
if(MODULE_vts-libs_FOUND)
  message(STATUS "3dtiles: compiling in encodig support (VTS)")
  list(APPEND 3dtiles_EXTRA_DEPENDS vts-libs)
  list(APPEND 3dtiles_EXTRA_SOURCES
    mesh.hpp mesh.cpp
    encoder.hpp encoder.cpp
    support.hpp support.cpp
    )
else()
  message(STATUS "3dtiles: compiling without encoding support (no VTS)")
endif()

define_module(LIBRARY 3dtiles=${3dtiles_VERSION}
  DEPENDS
  ${3dtiles_EXTRA_DEPENDS}
  jsoncpp>=2.3 gltf>=1.1 math>=1.4 utility>=1.41 dbglog>=1.4 imgproc>=1.28
  Boost_FILESYSTEM Boost_IOSTREAMS
  )

set(3dtiles_SOURCES
  3dtiles.hpp 3dtiles.cpp
  b3dm.hpp b3dm.cpp

  extensions/mlwn/srs.hpp extensions/mlwn/srs.cpp

  ${3dtiles_EXTRA_SOURCES}
  )

add_library(3dtiles STATIC ${3dtiles_SOURCES})
buildsys_library(3dtiles)

target_link_libraries(3dtiles ${MODULE_LIBRARIES})
target_compile_definitions(3dtiles PRIVATE ${MODULE_DEFINITIONS})

if(MODULE_service_FOUND)
  add_subdirectory(tools EXCLUDE_FROM_ALL)
endif()
