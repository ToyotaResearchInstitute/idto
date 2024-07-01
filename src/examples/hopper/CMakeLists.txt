include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(hopper hopper.cc)
target_link_libraries(hopper
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(hopper.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/hopper/hopper.yaml COPYONLY)

add_test(NAME hopper_test COMMAND hopper --test)
