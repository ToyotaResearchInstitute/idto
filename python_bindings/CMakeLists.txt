find_package(pybind11 CONFIG REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})

pybind11_add_module(pyidto MODULE my_fun_bindings.cc)
target_link_libraries(pyidto PUBLIC my_fun_library drake::drake)
set_target_properties(pyidto PROPERTIES CXX_VISIBILITY_PRESET default)

target_compile_definitions(pyidto
                           PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
