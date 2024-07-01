include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(punyo punyo.cc)
target_link_libraries(punyo
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(punyo.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/punyo/punyo.yaml COPYONLY)

add_test(NAME punyo_test COMMAND punyo --test)

