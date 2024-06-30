# Copy all files in this directory to idto/models/... in the build directory
file(GLOB_RECURSE SRC_FILES ${CMAKE_SOURCE_DIR}/src/models/*)
foreach(SRC_FILE ${SRC_FILES})
    file(RELATIVE_PATH RELATIVE_PATH ${CMAKE_SOURCE_DIR}/src/models ${SRC_FILE})
    configure_file(
        ${SRC_FILE}
        ${CMAKE_BINARY_DIR}/idto/models/${RELATIVE_PATH}
        COPYONLY
    )
endforeach()
