cmake_minimum_required(VERSION 3.10)
project(Deadeye_Rapid_React VERSION 0.1 LANGUAGES CXX)

# Project-wide setup
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(deadeye/daemon)

# Build main target
add_executable(Deadeye_daemon
        src/main.cpp
        src/hub_pipeline.cpp
        src/hub_target_data.cpp
        )

set_target_properties(Deadeye_daemon PROPERTIES
        OUTPUT_NAME deadeyed
        EXPORT_NAME deadeyed
        )

target_compile_definitions(Deadeye_daemon
        PUBLIC
        $<$<BOOL:${DEADEYE_CAMERA0_PIPELINE}>:DEADEYE_CAMERA0_PIPELINE=${DEADEYE_CAMERA0_PIPELINE}>
        $<$<BOOL:${DEADEYE_CAMERA1_PIPELINE}>:DEADEYE_CAMERA1_PIPELINE=${DEADEYE_CAMERA1_PIPELINE}>
        $<$<BOOL:${DEADEYE_CAMERA2_PIPELINE}>:DEADEYE_CAMERA2_PIPELINE=${DEADEYE_CAMERA2_PIPELINE}>
        $<$<BOOL:${DEADEYE_CAMERA3_PIPELINE}>:DEADEYE_CAMERA3_PIPELINE=${DEADEYE_CAMERA3_PIPELINE}>
        $<$<BOOL:${DEADEYE_CAMERA4_PIPELINE}>:DEADEYE_CAMERA4_PIPELINE=${DEADEYE_CAMERA4_PIPELINE}>
        )


target_link_libraries(Deadeye_daemon
        PRIVATE
        Deadeye::library
        )

