cmake_minimum_required(VERSION 3.10.2)
project(idto)

# N.B. This is a temporary flag. It only really applies to Linux, as Mac
# does not need X11.
option(RUN_X11_TESTS "Run tests that require X11" OFF)

include(CTest)

find_package(drake CONFIG REQUIRED)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_options(-Werror -Wall)
# add_compile_definitions(ENABLE_TIMERS)  # TODO: disable in python

add_subdirectory(src)
add_subdirectory(third_party)
