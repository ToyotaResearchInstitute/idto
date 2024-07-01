include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(spinner spinner.cc)
target_link_libraries(spinner
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(spinner.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/spinner/spinner.yaml COPYONLY)

add_test(NAME spinner_test COMMAND spinner --test)
