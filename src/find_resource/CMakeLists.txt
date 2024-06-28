# SPDX-License-Identifier: MIT-0

add_executable(find_resource_example find_resource_example.cc)
target_link_libraries(find_resource_example drake::drake)
add_test(NAME find_resource_example COMMAND find_resource_example)

add_test(NAME find_resource_example_py COMMAND
  "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/find_resource_example.py"
)
set_tests_properties(find_resource_example_py PROPERTIES
  ENVIRONMENT "PYTHONPATH=${PYTHONPATH}"
)
