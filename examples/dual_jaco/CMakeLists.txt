include_directories(${PROJECT_SOURCE_DIR})

add_executable(dual_jaco dual_jaco.cc)
target_link_libraries(dual_jaco
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(dual_jaco.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/dual_jaco/dual_jaco.yaml COPYONLY)

add_test(NAME dual_jaco_test COMMAND dual_jaco --test)
