cmake_minimum_required(VERSION 3.20)

project(VibeStationPS2Lab LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(vibestation_ps2_core STATIC
    core/ps2_system.cpp
    core/ee/ee_cpu.cpp
    core/memory/ee_bus.cpp
    core/memory/ee_ram.cpp
    core/scheduler/scheduler.cpp
)

target_include_directories(vibestation_ps2_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

if(MSVC)
    target_compile_options(vibestation_ps2_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(vibestation_ps2_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(VibeStationPS2Lab main.cpp)
target_link_libraries(VibeStationPS2Lab PRIVATE vibestation_ps2_core)

include(CTest)
if(BUILD_TESTING)
    add_executable(vibestation_ps2_tests
        tests/ps2_smoke_tests.cpp
    )
    target_link_libraries(vibestation_ps2_tests PRIVATE vibestation_ps2_core)

    add_test(
        NAME vibestation_ps2_smoke
        COMMAND $<TARGET_FILE:vibestation_ps2_tests>
    )
endif()
