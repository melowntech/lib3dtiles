define_module(BINARY 3dtiles2obj
  DEPENDS geometry 3dtiles service imgproc
  )

set(3dtiles2obj_SOURCES 3dtiles2obj.cpp)
add_executable(3dtiles2obj ${3dtiles2obj_SOURCES})
target_link_libraries(3dtiles2obj ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(3dtiles2obj PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(3dtiles2obj)
