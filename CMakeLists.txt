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

add_compile_options(-Werror -Wall)  # treat warnings as errors
add_compile_options(-O3)

# add_compile_definitions(ENABLE_TIMERS)  # extra profiling

# Allow us to find resources (e.g. urdfs) in the build directory.
add_compile_definitions(IDTO_BINARY_DIR="${CMAKE_BINARY_DIR}")

add_subdirectory(src)
add_subdirectory(third_party)
