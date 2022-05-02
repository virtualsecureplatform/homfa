add_executable(benchmark
    backstream_dfa_runner.cpp
    error.cpp
    graph.cpp
    benchmark.cpp
    offline_dfa.cpp
    online_dfa.cpp
    tfhepp_util.cpp
    timeit.cpp
)
add_sanitizers(benchmark)
target_compile_options(benchmark PUBLIC "$<$<CONFIG:DEBUG>:${HOMFA_CXXFLAGS_DEBUG}>")
target_compile_options(benchmark PUBLIC "$<$<CONFIG:RELEASE>:${HOMFA_CXXFLAGS_RELEASE}>")
target_link_libraries(benchmark ${HOMFA_LIBS})
target_include_directories(benchmark PRIVATE ${HOMFA_INCLUDE_DIRS})
target_compile_definitions(benchmark PRIVATE ${HOMFA_COMPILE_DEFINITIONS})
set_target_properties(benchmark
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
install(TARGETS benchmark DESTINATION bin)

add_executable(util
    backstream_dfa_runner.cpp
    error.cpp
    graph.cpp
    util.cpp
    offline_dfa.cpp
    online_dfa.cpp
    tfhepp_util.cpp
    timeit.cpp
)
add_sanitizers(util)
target_compile_options(util PUBLIC "$<$<CONFIG:DEBUG>:${HOMFA_CXXFLAGS_DEBUG}>")
target_compile_options(util PUBLIC "$<$<CONFIG:RELEASE>:${HOMFA_CXXFLAGS_RELEASE}>")
target_link_libraries(util ${HOMFA_LIBS})
target_include_directories(util PRIVATE ${HOMFA_INCLUDE_DIRS})
target_compile_definitions(util PRIVATE ${HOMFA_COMPILE_DEFINITIONS})
set_target_properties(util
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
install(TARGETS util DESTINATION bin)
