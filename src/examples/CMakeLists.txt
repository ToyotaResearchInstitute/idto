include_directories(${PROJECT_SOURCE_DIR}/src)

add_library(yaml_config INTERFACE yaml_config.h)

add_library(pd_plus_controller
            pd_plus_controller.cc
            pd_plus_controller.h)
target_link_libraries(pd_plus_controller drake::drake)

add_library(mpc_controller
            mpc_controller.cc
            mpc_controller.h)
target_link_libraries(mpc_controller 
                      drake::drake
                      pd_plus_controller
                      trajectory_optimizer)

add_library(example_base example_base.cc example_base.h)
target_link_libraries(example_base
                      drake::drake
                      mpc_controller
                      yaml_config
                      find_resource)

add_subdirectory(acrobot)
add_subdirectory(airhockey)
add_subdirectory(allegro_hand)
add_subdirectory(dual_jaco)