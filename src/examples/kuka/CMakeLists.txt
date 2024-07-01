include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(kuka kuka.cc)
target_link_libraries(kuka
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(kuka.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/kuka/kuka.yaml COPYONLY)

add_test(NAME kuka_test COMMAND kuka --test)

