cmake_minimum_required(VERSION 3.10.2)
project(idto)

# N.B. This is a temporary flag. It only really applies to Linux, as Mac
# does not need X11.
option(RUN_X11_TESTS "Run tests that require X11" OFF)

include(CTest)

find_package(drake CONFIG REQUIRED)

find_package(PythonInterp ${drake_PYTHON_VERSION} MODULE REQUIRED EXACT)

find_program(PYTHON_EXECUTABLE NAMES python3
  PATHS "${FIND_PYTHON_EXECUTABLE_PATHS}"
  NO_DEFAULT_PATH
)

execute_process(COMMAND ${PYTHON_EXECUTABLE}-config --exec-prefix
  OUTPUT_VARIABLE PYTHON_EXEC_PREFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
list(APPEND CMAKE_PREFIX_PATH "${PYTHON_EXEC_PREFIX}")
find_package(PythonLibs ${drake_PYTHON_VERSION} MODULE REQUIRED EXACT)

get_filename_component(PYTHONPATH
  "${drake_DIR}/../../python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages"
  REALPATH
)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_options(-Werror -Wall)  # treat warnings as errors
add_compile_options(-O3)
# add_compile_definitions(ENABLE_TIMERS)  # extra profiling

# Allow us to find resources (e.g. urdfs) in the build directory.
add_compile_definitions(IDTO_BINARY_DIR="${CMAKE_BINARY_DIR}")

add_subdirectory(third_party)
add_subdirectory(toy_example)
add_subdirectory(python_bindings)
add_subdirectory(utils)
add_subdirectory(optimizer)
add_subdirectory(models)
add_subdirectory(examples)

