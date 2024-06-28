add_library(my_fun_library my_fun_library.cc my_fun_library.h)
target_link_libraries(my_fun_library drake::drake)

add_executable(my_fun_example my_fun_example.cc)
target_link_libraries(my_fun_example my_fun_library drake::drake)

add_executable(my_fun_test my_fun_test.cc)
target_include_directories(my_fun_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(my_fun_test
  my_fun_library drake::drake gtest
)
add_test(NAME my_fun_test COMMAND my_fun_test)