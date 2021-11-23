option(HOMFA_BUILD_HOMFA "Build homfa" ON)
option(HOMFA_BUILD_TEST0 "Build test0" ON)
option(HOMFA_BUILD_TEST_PLAIN_RANDOM "Build test_plain_random" OFF)
option(HOMFA_BUILD_TEST_CRYPTO_RANDOM "Build test_crypto_random" OFF)
option(HOMFA_BUILD_BENCHMARK "Build benchmark" OFF)

if(HOMFA_BUILD_HOMFA)
    add_executable(homfa
        backstream_dfa_runner.cpp
        error.cpp
        graph.cpp
        main.cpp
        offline_dfa.cpp
        online_dfa.cpp
        tfhepp_util.cpp
    )
    add_sanitizers(homfa)
    target_compile_options(homfa PUBLIC "$<$<CONFIG:DEBUG>:${HOMFA_CXXFLAGS_DEBUG}>")
    target_compile_options(homfa PUBLIC "$<$<CONFIG:RELEASE>:${HOMFA_CXXFLAGS_RELEASE}>")
    target_link_libraries(homfa ${HOMFA_LIBS})
    target_include_directories(homfa PRIVATE ${HOMFA_INCLUDE_DIRS})
    target_compile_definitions(homfa PRIVATE ${HOMFA_COMPILE_DEFINITIONS})
    set_target_properties(homfa
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
    install(TARGETS homfa DESTINATION bin)
endif()

if(HOMFA_BUILD_TEST0)
    add_executable(test0
        backstream_dfa_runner.cpp
        error.cpp
        graph.cpp
        test0.cpp
        offline_dfa.cpp
        online_dfa.cpp
        tfhepp_util.cpp
    )
    add_sanitizers(test0)
    target_compile_options(test0 PUBLIC "$<$<CONFIG:DEBUG>:${HOMFA_CXXFLAGS_DEBUG}>")
    target_compile_options(test0 PUBLIC "$<$<CONFIG:RELEASE>:${HOMFA_CXXFLAGS_RELEASE}>")
    target_link_libraries(test0 ${HOMFA_LIBS})
    target_include_directories(test0 PRIVATE ${HOMFA_INCLUDE_DIRS})
    target_compile_definitions(test0 PRIVATE ${HOMFA_COMPILE_DEFINITIONS})
    set_target_properties(test0
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
    install(TARGETS test0 DESTINATION bin)
endif()

if(HOMFA_BUILD_TEST_PLAIN_RANDOM)
    add_executable(test_plain_random
        backstream_dfa_runner.cpp
        error.cpp
        graph.cpp
        test_plain_random.cpp
        offline_dfa.cpp
        online_dfa.cpp
        tfhepp_util.cpp
    )
    add_sanitizers(test_plain_random)
    target_compile_options(test_plain_random PUBLIC "$<$<CONFIG:DEBUG>:${HOMFA_CXXFLAGS_DEBUG}>")
    target_compile_options(test_plain_random PUBLIC "$<$<CONFIG:RELEASE>:${HOMFA_CXXFLAGS_RELEASE}>")
    target_link_libraries(test_plain_random ${HOMFA_LIBS})
    target_include_directories(test_plain_random PRIVATE ${HOMFA_INCLUDE_DIRS})
    target_compile_definitions(test_plain_random PRIVATE ${HOMFA_COMPILE_DEFINITIONS})
    set_target_properties(test_plain_random
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
    install(TARGETS test_plain_random DESTINATION bin)
endif()

if(HOMFA_BUILD_TEST_CRYPTO_RANDOM)
    add_executable(test_crypto_random
        backstream_dfa_runner.cpp
        error.cpp
        graph.cpp
        test_crypto_random.cpp
        offline_dfa.cpp
        online_dfa.cpp
        tfhepp_util.cpp
    )
    add_sanitizers(test_crypto_random)
    target_compile_options(test_crypto_random PUBLIC "$<$<CONFIG:DEBUG>:${HOMFA_CXXFLAGS_DEBUG}>")
    target_compile_options(test_crypto_random PUBLIC "$<$<CONFIG:RELEASE>:${HOMFA_CXXFLAGS_RELEASE}>")
    target_link_libraries(test_crypto_random ${HOMFA_LIBS})
    target_include_directories(test_crypto_random PRIVATE ${HOMFA_INCLUDE_DIRS})
    target_compile_definitions(test_crypto_random PRIVATE ${HOMFA_COMPILE_DEFINITIONS})
    set_target_properties(test_crypto_random
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
    install(TARGETS test_crypto_random DESTINATION bin)
endif()

if(HOMFA_BUILD_BENCHMARK)
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
endif()
