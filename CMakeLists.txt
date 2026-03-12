cmake_minimum_required(VERSION 3.20)
project(VibeStation LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Fetch Dependencies ──────────────────────────────────────────────
include(FetchContent)

# SDL2 (source build)
FetchContent_Declare(SDL2
    URL https://github.com/libsdl-org/SDL/releases/download/release-2.30.10/SDL2-2.30.10.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(SDL_SHARED OFF CACHE BOOL "" FORCE)
set(SDL_STATIC ON  CACHE BOOL "" FORCE)
set(SDL_TEST   OFF CACHE BOOL "" FORCE)
set(SDL2_DISABLE_INSTALL ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(SDL2)

# Dear ImGui
FetchContent_Declare(imgui
    URL https://github.com/ocornut/imgui/archive/refs/tags/v1.91.8.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(imgui)

# ── ImGui static library ────────────────────────────────────────────
add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC SDL2::SDL2-static opengl32)

# ── VibeStation Executable ──────────────────────────────────────────
set(VIBESTATION_SOURCES
    src/main.cpp
    src/core/bios.cpp
    src/core/ram.cpp
    src/core/interrupt.cpp
    src/core/timer.cpp
    src/core/spu.cpp
    src/core/sio.cpp
    src/core/memory_card.cpp
    src/core/pad_controller.cpp
    src/core/cdrom.cpp
    src/core/dma.cpp
    src/core/mdec.cpp
    src/core/gpu.cpp
    src/core/gte.cpp
    src/core/cpu.cpp
    src/core/system.cpp
    src/core/renderer.cpp
    src/input/controller.cpp
    src/input/input_manager.cpp
    src/ui/emu_runner.cpp
    src/ui/app.cpp
)

add_executable(VibeStation ${VIBESTATION_SOURCES})

target_include_directories(VibeStation PRIVATE src)
target_link_libraries(VibeStation PRIVATE imgui_lib SDL2::SDL2-static)

# High warning level + suppress CRT deprecation warnings
if(MSVC)
    target_compile_options(VibeStation PRIVATE /W3 /permissive-)
    target_compile_definitions(VibeStation PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

option(VIBESTATION_ENABLE_IPO "Enable interprocedural optimization for release builds" ON)
if(VIBESTATION_ENABLE_IPO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT VIBESTATION_IPO_SUPPORTED OUTPUT VIBESTATION_IPO_ERROR)
    if(VIBESTATION_IPO_SUPPORTED)
        set_property(TARGET VibeStation PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
        set_property(TARGET VibeStation PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO TRUE)
    else()
        message(STATUS "IPO/LTO not supported: ${VIBESTATION_IPO_ERROR}")
    endif()
endif()