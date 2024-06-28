add_library(my_fun_library my_fun_library.cc my_fun_library.h)
target_link_libraries(my_fun_library drake::drake)

add_executable(my_fun_example my_fun_example.cc)
target_link_libraries(my_fun_example my_fun_library drake::drake)