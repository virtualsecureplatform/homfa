add_executable(homfa
    dfa.cpp
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
