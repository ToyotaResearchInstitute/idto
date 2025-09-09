include_directories(${PROJECT_SOURCE_DIR})

add_executable(acrobot acrobot.cc)
target_link_libraries(acrobot
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(acrobot.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/acrobot/acrobot.yaml COPYONLY)

add_test(NAME acrobot_test COMMAND acrobot --test)
