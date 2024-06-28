find_package(pybind11 CONFIG REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/src)

pybind11_add_module(pyidto MODULE my_fun_bindings.cc)
target_link_libraries(pyidto PUBLIC my_fun_library drake::drake)
set_target_properties(pyidto PROPERTIES CXX_VISIBILITY_PRESET default)

add_test(NAME my_fun_bindings_test COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/my_fun_bindings_test.py")
set_tests_properties(my_fun_bindings_test PROPERTIES
    ENVIRONMENT "PYTHONPATH=$<TARGET_FILE_DIR:pyidto>:${PYTHONPATH}")