# SPDX-License-Identifier: MIT-0

find_package(Threads)

add_library(gtest STATIC googletest/gtest-all.cc googletest/gtest/gtest.h)
target_include_directories(gtest PUBLIC googletest)
target_link_libraries(gtest Threads::Threads)
