# SPDX-License-Identifier: MIT-0

add_subdirectory(find_resource)
add_subdirectory(particles)
add_subdirectory(simple_bindings)
add_subdirectory(simple_continuous_time_system)

add_test(NAME import_all_test COMMAND
  "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/import_all_test.py"
)
set_tests_properties(import_all_test PROPERTIES
  ENVIRONMENT "PYTHONPATH=${PYTHONPATH}"
)
