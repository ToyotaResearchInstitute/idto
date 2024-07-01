include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(jaco_ball jaco_ball.cc)
target_link_libraries(jaco_ball
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(jaco_ball.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/jaco_ball/jaco_ball.yaml COPYONLY)

add_test(NAME jaco_ball_test COMMAND jaco_ball --test)
