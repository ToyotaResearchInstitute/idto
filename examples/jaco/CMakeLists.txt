include_directories(${PROJECT_SOURCE_DIR})

add_executable(jaco jaco.cc)
target_link_libraries(jaco
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(jaco.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/jaco/jaco.yaml COPYONLY)

add_test(NAME jaco_test COMMAND jaco --test)
