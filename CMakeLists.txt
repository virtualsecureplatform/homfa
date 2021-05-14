cmake_minimum_required(VERSION 3.16)

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
set(HOMFA_CXXFLAGS_DEBUG   ${IYOKAN_CXXFLAGS} -O0 -g3)
set(HOMFA_CXXFLAGS_RELEASE ${IYOKAN_CXXFLAGS} -O3 -march=native -g3)
set(HOMFA_INCLUDE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/spdlog/include>
)
set(HOMFA_LIBS tfhe++ Threads::Threads OpenMP::OpenMP_CXX Backward::Backward tbb)
set(HOMFA_COMPILE_DEFINITIONS )

set(CMAKE_C_FLAGS "")
set(CMAKE_C_FLAGS_DEBUG "")
set(CMAKE_C_FLAGS_RELEASE "")
set(CMAKE_CXX_FLAGS "")
set(CMAKE_CXX_FLAGS_DEBUG "")
set(CMAKE_CXX_FLAGS_RELEASE "")

add_subdirectory(src)
