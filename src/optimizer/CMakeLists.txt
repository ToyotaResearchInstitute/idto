include_directories(${PROJECT_SOURCE_DIR}/src)

add_library(convergence_criteria_tolerances 
            INTERFACE
            convergence_criteria_tolerances.h)

add_library(inverse_dynamics_partials 
            inverse_dynamics_partials.cc
            inverse_dynamics_partials.h)
target_link_libraries(inverse_dynamics_partials drake::drake)

add_library(penta_diagonal_matrix
            penta_diagonal_matrix.cc
            penta_diagonal_matrix.h)
target_link_libraries(penta_diagonal_matrix drake::drake)

add_library(penta_diagonal_solver
            INTERFACE
            penta_diagonal_solver.h)

add_library(problem_definition
            INTERFACE
            problem_definition.h)

add_library(velocity_partials velocity_partials.cc velocity_partials.h)
target_link_libraries(velocity_partials drake::drake)

# Tests
add_executable(penta_diagonal_solver_test test/penta_diagonal_solver_test.cc)
target_include_directories(penta_diagonal_solver_test PRIVATE 
    "${CMAKE_CURRENT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/src")
target_link_libraries(penta_diagonal_solver_test 
                      penta_diagonal_matrix
                      penta_diagonal_solver
                      drake::drake
                      gtest)
add_test(NAME penta_diagonal_solver_test COMMAND penta_diagonal_solver_test)