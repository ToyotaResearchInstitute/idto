include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(allegro_hand allegro_hand.cc)
target_link_libraries(allegro_hand
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(allegro_hand.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/allegro_hand/allegro_hand.yaml COPYONLY)
configure_file(allegro_hand_upside_down.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/allegro_hand/allegro_hand_upside_down.yaml COPYONLY)

add_test(NAME allegro_hand_test COMMAND allegro_hand --test)
