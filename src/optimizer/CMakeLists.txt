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

add_library(solver_parameters
            INTERFACE
            solver_parameters.h)

add_library(trajectory_optimizer_solution
            trajectory_optimizer_solution.cc
            trajectory_optimizer_solution.h)
target_link_libraries(trajectory_optimizer_solution drake::drake)

add_library(trajectory_optimizer_state
            trajectory_optimizer_state.cc
            trajectory_optimizer_state.h)
target_link_libraries(trajectory_optimizer_state 
                      drake::drake
                      inverse_dynamics_partials
                      penta_diagonal_matrix
                      trajectory_optimizer_workspace
                      velocity_partials)

add_library(trajectory_optimizer_workspace
            trajectory_optimizer_workspace.cc
            trajectory_optimizer_workspace.h)
target_link_libraries(trajectory_optimizer_workspace drake::drake)

add_library(trajectory_optimizer
            trajectory_optimizer.cc
            trajectory_optimizer.h)
target_link_libraries(trajectory_optimizer 
                      inverse_dynamics_partials
                      penta_diagonal_matrix
                      penta_diagonal_solver
                      problem_definition
                      solver_parameters
                      trajectory_optimizer_solution
                      trajectory_optimizer_state
                      trajectory_optimizer_workspace
                      velocity_partials
                      warm_start
                      drake::drake)

add_library(velocity_partials velocity_partials.cc velocity_partials.h)
target_link_libraries(velocity_partials drake::drake)

add_library(warm_start INTERFACE warm_start.h)

add_executable(penta_diagonal_solver_test test/penta_diagonal_solver_test.cc)
target_include_directories(penta_diagonal_solver_test PRIVATE 
    "${CMAKE_CURRENT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/src")
target_link_libraries(penta_diagonal_solver_test 
                      penta_diagonal_matrix
                      penta_diagonal_solver
                      drake::drake
                      gtest)
add_test(NAME penta_diagonal_solver_test COMMAND penta_diagonal_solver_test)

add_executable(trajectory_optimizer_test test/trajectory_optimizer_test.cc)
target_include_directories(trajectory_optimizer_test PRIVATE 
    "${CMAKE_CURRENT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/src")
target_link_libraries(trajectory_optimizer_test 
                      trajectory_optimizer
                      drake::drake
                      gtest)
add_test(NAME trajectory_optimizer_test COMMAND trajectory_optimizer_test)