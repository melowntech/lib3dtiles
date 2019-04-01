define_module(BINARY 3dtiles2obj
  DEPENDS geometry 3dtiles service imgproc
  )

set(3dtiles2obj_SOURCES 3dtiles2obj.cpp)
add_executable(3dtiles2obj ${3dtiles2obj_SOURCES})
target_link_libraries(3dtiles2obj ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(3dtiles2obj PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(3dtiles2obj)

define_module(BINARY clone-3dtiles
  DEPENDS geometry 3dtiles service imgproc
  )

set(clone-3dtiles_SOURCES clone-3dtiles.cpp)
add_executable(clone-3dtiles ${clone-3dtiles_SOURCES})
target_link_libraries(clone-3dtiles ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(clone-3dtiles PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(clone-3dtiles)
