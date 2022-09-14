cmake_minimum_required(VERSION 3.16)

option(HOMFA_ENABLE_PROFILE "Enable profiling with pprof" OFF)

set(CMAKE_MODULE_PATH
    ${CMAKE_SOURCE_DIR}
    "${CMAKE_SOURCE_DIR}/thirdparty/sanitizers-cmake/cmake"
    ${CMAKE_MODULE_PATH}
)
set(Backward_DIR "${CMAKE_SOURCE_DIR}/thirdparty/backward-cpp")

# Defaults to CMAKE_BUILD_TYPE=Debug
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(homfa LANGUAGES CXX)

find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Sanitizers)
find_package(Backward)

#set(USE_CGGI19 1)
add_subdirectory(thirdparty/TFHEpp)
add_subdirectory(thirdparty/spdlog)

set(HOMFA_CXXFLAGS -Wall -Wextra -pedantic -Wno-sign-compare)
set(HOMFA_CXXFLAGS_DEBUG   ${HOMFA_CXXFLAGS} -O0 -g3)
set(HOMFA_CXXFLAGS_RELEASE ${HOMFA_CXXFLAGS} -O3 -march=native -g3)
set(HOMFA_INCLUDE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/CLI11/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/spdlog/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ThreadPool>
)
set(HOMFA_LIBS tfhe++ Threads::Threads OpenMP::OpenMP_CXX Backward::Backward tbb libspot.a libbddx.a)
set(HOMFA_COMPILE_DEFINITIONS )

if(CMAKE_BUILD_TYPE MATCHES Debug)
    list(APPEND HOMFA_COMPILE_DEFINITIONS HOMFA_BUILD_DEBUG)
endif()

if(HOMFA_ENABLE_PROFILE)
    list(APPEND HOMFA_LIBS -Wl,--no-as-needed -lprofiler -ltcmalloc -Wl,--as-needed)
    list(APPEND HOMFA_COMPILE_DEFINITIONS HOMFA_ENABLE_PROFILE)
endif()

set(CMAKE_C_FLAGS "")
set(CMAKE_C_FLAGS_DEBUG "")
set(CMAKE_C_FLAGS_RELEASE "")
set(CMAKE_CXX_FLAGS "")
set(CMAKE_CXX_FLAGS_DEBUG "")
set(CMAKE_CXX_FLAGS_RELEASE "")

# Get git revision
# Thanks to: https://stackoverflow.com/a/6526533
set(HOMFA_GIT_REVISION "unknown")
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.git)
    find_package(Git)
    if(GIT_FOUND)
        execute_process(
            COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            OUTPUT_VARIABLE "HOMFA_GIT_REVISION"
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE)
        message(STATUS "Git revision: ${HOMFA_GIT_REVISION}")
    endif(GIT_FOUND)
endif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.git)
list(APPEND HOMFA_COMPILE_DEFINITIONS HOMFA_GIT_REVISION="${HOMFA_GIT_REVISION}")

add_subdirectory(src)
