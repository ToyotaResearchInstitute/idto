find_package(pybind11 CONFIG REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})

pybind11_add_module(pyidto MODULE pyidto.cc 
                    problem_definition_py.cc
                    solver_parameters_py.cc 
                    trajectory_optimizer_py.cc
                    trajectory_optimizer_solution_py.cc
                    trajectory_optimizer_stats_py.cc
                    find_resource_py.cc)
target_link_libraries(pyidto PUBLIC 
        trajectory_optimizer drake::drake find_resource)
set_target_properties(pyidto PROPERTIES CXX_VISIBILITY_PRESET default)

target_compile_definitions(pyidto
                           PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})

add_test(NAME problem_definition_test_py COMMAND ${PYTHON_EXECUTABLE} -m pytest
         ${CMAKE_CURRENT_SOURCE_DIR}/test/problem_definition_test.py)
add_test(NAME solver_parameters_test_py COMMAND ${PYTHON_EXECUTABLE} -m pytest
         ${CMAKE_CURRENT_SOURCE_DIR}/test/solver_parameters_test.py)
add_test(NAME trajectory_optimizer_solution_test_py COMMAND ${PYTHON_EXECUTABLE} -m pytest
         ${CMAKE_CURRENT_SOURCE_DIR}/test/trajectory_optimizer_solution_test.py)
add_test(NAME trajectory_optimizer_stats_test_py COMMAND ${PYTHON_EXECUTABLE} -m pytest
         ${CMAKE_CURRENT_SOURCE_DIR}/test/trajectory_optimizer_stats_test.py)
add_test(NAME find_resource_test_py COMMAND ${PYTHON_EXECUTABLE} -m pytest
         ${CMAKE_CURRENT_SOURCE_DIR}/test/find_resource_test.py)
add_test(NAME trajectory_optimizer_test_py COMMAND ${PYTHON_EXECUTABLE} -m pytest
         ${CMAKE_CURRENT_SOURCE_DIR}/test/trajectory_optimizer_test.py)
add_test(NAME warm_start_test_py COMMAND ${PYTHON_EXECUTABLE} -m pytest
         ${CMAKE_CURRENT_SOURCE_DIR}/test/warm_start_test.py)
