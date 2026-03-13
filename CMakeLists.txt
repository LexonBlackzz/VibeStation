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
    src/integrations/discord_presence.cpp
    src/ui/emu_runner.cpp
    src/ui/app.cpp
)

add_executable(VibeStation ${VIBESTATION_SOURCES})

target_include_directories(VibeStation PRIVATE src)
target_link_libraries(VibeStation PRIVATE imgui_lib SDL2::SDL2-static)

option(VIBESTATION_ENABLE_DISCORD_RPC
    "Enable Discord Rich Presence via the Discord Social SDK when available" ON)
set(VIBESTATION_DISCORD_SDK_ROOT "" CACHE PATH
    "Path to the extracted Discord Social SDK directory")

if(VIBESTATION_ENABLE_DISCORD_RPC)
    find_path(VIBESTATION_DISCORD_INCLUDE_DIR
        NAMES discordpp.h
        PATHS
            ${VIBESTATION_DISCORD_SDK_ROOT}
            ${VIBESTATION_DISCORD_SDK_ROOT}/include
            ${VIBESTATION_DISCORD_SDK_ROOT}/cpp
        NO_DEFAULT_PATH
    )

    find_library(VIBESTATION_DISCORD_LIBRARY
        NAMES discord_partner_sdk
        PATHS
            ${VIBESTATION_DISCORD_SDK_ROOT}
            ${VIBESTATION_DISCORD_SDK_ROOT}/lib
            ${VIBESTATION_DISCORD_SDK_ROOT}/lib/release
            ${VIBESTATION_DISCORD_SDK_ROOT}/lib/debug
            ${VIBESTATION_DISCORD_SDK_ROOT}/lib/x64
            ${VIBESTATION_DISCORD_SDK_ROOT}/lib/x86_64
        NO_DEFAULT_PATH
    )

    if(VIBESTATION_DISCORD_INCLUDE_DIR AND VIBESTATION_DISCORD_LIBRARY)
        target_compile_definitions(VibeStation PRIVATE VIBESTATION_HAS_DISCORD_SDK=1)
        target_include_directories(VibeStation PRIVATE ${VIBESTATION_DISCORD_INCLUDE_DIR})
        target_link_libraries(VibeStation PRIVATE ${VIBESTATION_DISCORD_LIBRARY})

        find_file(VIBESTATION_DISCORD_DLL
            NAMES discord_partner_sdk.dll
            PATHS
                ${VIBESTATION_DISCORD_SDK_ROOT}
                ${VIBESTATION_DISCORD_SDK_ROOT}/bin
                ${VIBESTATION_DISCORD_SDK_ROOT}/bin/release
                ${VIBESTATION_DISCORD_SDK_ROOT}/bin/debug
                ${VIBESTATION_DISCORD_SDK_ROOT}/bin/x64
                ${VIBESTATION_DISCORD_SDK_ROOT}/lib
                ${VIBESTATION_DISCORD_SDK_ROOT}/lib/release
                ${VIBESTATION_DISCORD_SDK_ROOT}/lib/debug
                ${VIBESTATION_DISCORD_SDK_ROOT}/lib/x64
                ${VIBESTATION_DISCORD_SDK_ROOT}/lib/x86_64
            NO_DEFAULT_PATH
        )
        if(WIN32 AND VIBESTATION_DISCORD_DLL)
            file(TO_NATIVE_PATH "${VIBESTATION_DISCORD_DLL}" VIBESTATION_DISCORD_DLL_RC_PATH)
            string(REPLACE "\\" "\\\\" VIBESTATION_DISCORD_DLL_RC_PATH
                "${VIBESTATION_DISCORD_DLL_RC_PATH}")
            configure_file(
                ${CMAKE_CURRENT_SOURCE_DIR}/src/integrations/discord_partner_sdk_embed.rc.in
                ${CMAKE_CURRENT_BINARY_DIR}/discord_partner_sdk_embed.rc
                @ONLY
            )
            target_sources(VibeStation PRIVATE
                ${CMAKE_CURRENT_BINARY_DIR}/discord_partner_sdk_embed.rc)
            target_compile_definitions(VibeStation PRIVATE
                VIBESTATION_HAS_EMBEDDED_DISCORD_DLL=1)
            if(MSVC)
                target_link_libraries(VibeStation PRIVATE delayimp)
                target_link_options(VibeStation PRIVATE /DELAYLOAD:discord_partner_sdk.dll)
            endif()
        endif()

        message(STATUS "Discord Social SDK enabled from ${VIBESTATION_DISCORD_SDK_ROOT}")
    elseif(VIBESTATION_DISCORD_SDK_ROOT)
        message(WARNING
            "Discord Social SDK requested but discordpp.h or discord_partner_sdk.lib "
            "was not found under VIBESTATION_DISCORD_SDK_ROOT='${VIBESTATION_DISCORD_SDK_ROOT}'. "
            "Building with Discord RPC stubs only.")
    else()
        message(STATUS
            "Discord Social SDK root not set; building with Discord RPC stubs only.")
    endif()
endif()

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
