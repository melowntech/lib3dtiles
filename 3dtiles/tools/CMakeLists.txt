define_module(BINARY 3dtiles2obj
  DEPENDS geometry 3dtiles service imgproc
  )

set(3dtiles2obj_SOURCES 3dtiles2obj.cpp)
add_executable(3dtiles2obj ${3dtiles2obj_SOURCES})
target_link_libraries(3dtiles2obj ${MODULE_LIBRARIES})
target_compile_definitions(3dtiles2obj PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(3dtiles2obj)

define_module(BINARY clone-3dtiles
  DEPENDS geometry 3dtiles service imgproc
  )

set(clone-3dtiles_SOURCES clone-3dtiles.cpp)
add_executable(clone-3dtiles ${clone-3dtiles_SOURCES})
target_link_libraries(clone-3dtiles ${MODULE_LIBRARIES})
target_compile_definitions(clone-3dtiles PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(clone-3dtiles)

define_module(BINARY b3dm2obj
  DEPENDS geometry 3dtiles service imgproc
  )

set(b3dm2obj_SOURCES b3dm2obj.cpp)
add_executable(b3dm2obj ${b3dm2obj_SOURCES})
target_link_libraries(b3dm2obj ${MODULE_LIBRARIES})
target_compile_definitions(b3dm2obj PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(b3dm2obj)

define_module(BINARY 3dtiles-metadata
  DEPENDS 3dtiles service
  )

set(3dtiles-metadata_SOURCES 3dtiles-metadata.cpp)
add_executable(3dtiles-metadata ${3dtiles-metadata_SOURCES})
target_link_libraries(3dtiles-metadata ${MODULE_LIBRARIES})
target_compile_definitions(3dtiles-metadata PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(3dtiles-metadata)
