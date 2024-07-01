include_directories(${PROJECT_SOURCE_DIR})

add_executable(airhockey airhockey.cc)
target_link_libraries(airhockey
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(airhockey.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/airhockey/airhockey.yaml COPYONLY)

add_test(NAME airhockey_test COMMAND airhockey --test)
