add_executable(Deadeye_tests
        main.cpp
        hub_pipeline_tests.cpp
        ${CMAKE_SOURCE_DIR}/src/hub_pipeline.cpp
        ${CMAKE_SOURCE_DIR}/src/hub_target_data.cpp
        )

target_include_directories(Deadeye_tests
        PUBLIC
        ${CMAKE_SOURCE_DIR}/src
        )

target_compile_definitions(Deadeye_tests
        PRIVATE
        DEADEYE_TEST_DATA="${CMAKE_CURRENT_LIST_DIR}/data/"
        )

target_link_libraries(Deadeye_tests
        Deadeye::library
        )

include(Catch)
catch_discover_tests(Deadeye_tests)
