cmake_minimum_required(VERSION 3.20)

project(VibeStationPS2Lab LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(vibestation_ps2_core STATIC
    core/bios/bios.cpp
    core/cdvd/cdvd_hw.cpp
    core/ps2_system.cpp
    core/ee/ee_cpu.cpp
    core/gs/gs_privileged.cpp
    core/hw/ee_hw.cpp
    core/hw/iop_hw_window.cpp
    core/iop/iop_bus.cpp
    core/iop/iop_cpu.cpp
    core/iop/iop_ram.cpp
    core/memory/ee_bus.cpp
    core/memory/ee_ram.cpp
    core/memory/ee_scratchpad.cpp
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

option(VIBESTATION_PS2_ENABLE_UI "Build the experimental VibeStation PS2 UI" ON)

if(VIBESTATION_PS2_ENABLE_UI)
    include(FetchContent)

    FetchContent_Declare(SDL2
        URL https://github.com/libsdl-org/SDL/releases/download/release-2.30.10/SDL2-2.30.10.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    set(SDL_SHARED OFF CACHE BOOL "" FORCE)
    set(SDL_STATIC ON CACHE BOOL "" FORCE)
    set(SDL_TEST OFF CACHE BOOL "" FORCE)
    set(SDL2_DISABLE_INSTALL ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(SDL2)

    FetchContent_Declare(imgui
        URL https://github.com/ocornut/imgui/archive/refs/tags/v1.91.8.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(imgui)

    if(WIN32)
        set(VIBESTATION_PS2_OPENGL_LIBRARY opengl32)
    else()
        find_package(OpenGL REQUIRED)
        set(VIBESTATION_PS2_OPENGL_LIBRARY OpenGL::GL)
    endif()

    add_library(vibestation_ps2_imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )
    target_include_directories(vibestation_ps2_imgui PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
    )
    target_link_libraries(vibestation_ps2_imgui PUBLIC
        SDL2::SDL2-static
        ${VIBESTATION_PS2_OPENGL_LIBRARY}
    )

    add_executable(VibeStationPS2Lab
        main.cpp
        ui/ps2_app.cpp
        ui/theme_settings.cpp
    )
    target_compile_definitions(VibeStationPS2Lab PRIVATE SDL_MAIN_HANDLED=1)
    target_link_libraries(VibeStationPS2Lab PRIVATE
        vibestation_ps2_core
        vibestation_ps2_imgui
        SDL2::SDL2-static
        ${VIBESTATION_PS2_OPENGL_LIBRARY}
    )
    if(WIN32)
        target_link_libraries(VibeStationPS2Lab PRIVATE comdlg32)
    endif()
endif()

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
