include_directories(${PROJECT_SOURCE_DIR})

add_executable(mini_cheetah mini_cheetah.cc)
target_link_libraries(mini_cheetah
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(mini_cheetah.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/mini_cheetah/mini_cheetah.yaml COPYONLY)

add_test(NAME mini_cheetah_test COMMAND mini_cheetah --test)
