add_library(profiler profiler.cc profiler.h)
target_link_libraries(profiler drake::drake)

add_executable(profiler_test test/profiler_test.cc)
target_include_directories(profiler_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(profiler_test
  profiler drake::drake gtest
)
add_test(NAME profiler_test COMMAND profiler_test)
