# SPDX-License-Identifier: MIT-0

# Send over the SDF file so the particle example can be run "in place."
configure_file(models/particle.sdf "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)

add_library(particles particle.cc particle.h particle_geometry.cc particle_geometry.h)
target_link_libraries(particles drake::drake)

set(GFLAGS_USE_TARGET_NAMESPACE OFF)
find_package(gflags CONFIG REQUIRED)

add_executable(uniformly_accelerated_particle uniformly_accelerated_particle.cc)
target_link_libraries(uniformly_accelerated_particle particles
  drake::drake gflags
)

add_executable(particle_test particle_test.cc)
target_include_directories(particle_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(particle_test
  particles drake::drake gtest
)
add_test(NAME particle_test COMMAND particle_test)

