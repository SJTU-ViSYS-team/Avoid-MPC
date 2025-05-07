# Find casadi package
find_path(Casadi_INCLUDE_DIR casadi/casadi.hpp
    PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../thirdparty/include
    /usr/local/include
    /usr/include
    /use/include
)

find_library(Casadi_LIBRARY NAMES casadi
    PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../thirdparty/lib/
    /usr/local/lib
    /usr/lib
    /use/lib
)

# Check if casadi is found
if(Casadi_INCLUDE_DIR AND Casadi_LIBRARY)
    set(Casadi_FOUND TRUE)
else()
    set(Casadi_FOUND FALSE)
endif()

# Provide information about casadi
if(Casadi_FOUND)
    message(STATUS "Found casadi:")
    message(STATUS "  Include: ${Casadi_INCLUDE_DIR}")
    message(STATUS "  Library: ${Casadi_LIBRARY}")
else()
    message(FATAL_ERROR "casadi not found!")
endif()

# Set include directories
include_directories(${Casadi_INCLUDE_DIR})

# Set libraries
set(LIBS ${Casadi_LIBRARY})
