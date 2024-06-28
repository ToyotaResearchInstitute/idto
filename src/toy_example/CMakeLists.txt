find_package(pybind11 CONFIG REQUIRED)

add_library(my_fun_library my_fun_library.cc my_fun_library.h)
target_link_libraries(my_fun_library drake::drake)

add_executable(my_fun_example my_fun_example.cc)
target_link_libraries(my_fun_example my_fun_library drake::drake)

add_executable(my_fun_test my_fun_test.cc)
target_include_directories(my_fun_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(my_fun_test
  my_fun_library drake::drake gtest
)
add_test(NAME my_fun_test COMMAND my_fun_test)

pybind11_add_module(my_fun_bindings MODULE my_fun_bindings.cc)
target_link_libraries(my_fun_bindings PUBLIC my_fun_library drake::drake)
set_target_properties(my_fun_bindings PROPERTIES CXX_VISIBILITY_PRESET default)

add_test(NAME my_fun_bindings_test COMMAND
    "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/my_fun_bindings_test.py")
set_tests_properties(my_fun_bindings_test PROPERTIES
    ENVIRONMENT "PYTHONPATH=$<TARGET_FILE_DIR:my_fun_bindings>:${PYTHONPATH}")