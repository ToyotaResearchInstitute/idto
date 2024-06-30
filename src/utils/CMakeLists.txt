include_directories(${PROJECT_SOURCE_DIR}/src)

add_library(eigen_matrix_compare
            INTERFACE
            eigen_matrix_compare.h)

add_library(profiler profiler.cc profiler.h)
target_link_libraries(profiler drake::drake)

add_library(find_resource find_resource.cc find_resource.h)
target_link_libraries(find_resource drake::drake)

add_executable(profiler_test test/profiler_test.cc)
target_include_directories(profiler_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(profiler_test
  profiler drake::drake gtest
)
add_test(NAME profiler_test COMMAND profiler_test)

add_executable(find_resource_test test/find_resource_test.cc)
target_include_directories(find_resource_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(find_resource_test
  find_resource drake::drake gtest
)
add_test(NAME find_resource_test COMMAND find_resource_test)
