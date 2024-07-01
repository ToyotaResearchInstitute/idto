include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(pendulum pendulum.cc)
target_link_libraries(pendulum
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(pendulum.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/pendulum/pendulum.yaml COPYONLY)

add_test(NAME pendulum_test COMMAND pendulum --test)

