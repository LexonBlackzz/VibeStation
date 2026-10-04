cmake_minimum_required(VERSION 3.20)
project(VibeStation LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CheckCXXSourceCompiles)
check_cxx_source_compiles("\
#if !defined(_M_X64) && !defined(__x86_64__)\n\
#error VibeStation native JIT requires an x64 compiler target\n\
#endif\n\
int main() { return 0; }\n"
    VIBESTATION_COMPILER_TARGETS_X64)

set(VIBESTATION_X64_JIT_DEFAULT OFF)
if(CMAKE_SIZEOF_VOID_P EQUAL 8 AND VIBESTATION_COMPILER_TARGETS_X64)
    set(VIBESTATION_X64_JIT_DEFAULT ON)
endif()
option(VIBESTATION_ENABLE_X64_JIT
    "Enable the native x64 CPU JIT/recompiler when supported" ${VIBESTATION_X64_JIT_DEFAULT})

if(VIBESTATION_ENABLE_X64_JIT AND
        (NOT CMAKE_SIZEOF_VOID_P EQUAL 8 OR
         NOT VIBESTATION_COMPILER_TARGETS_X64))
    message(FATAL_ERROR
        "VIBESTATION_ENABLE_X64_JIT requires an x64 compiler target")
endif()

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

# Dependencies below are fetched as pinned archives rather than git clones:
# cloning (json especially) dominated configure time.

# nlohmann/json (header-only). Official 111 KB release source tarball.
FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

# stb_image (launcher background texture decoding). Pinned to the commit that
# `master` resolved to, so builds are reproducible.
FetchContent_Declare(stb
    URL https://github.com/nothings/stb/archive/2c980bb59875b0d32144a71867fbdebb2f77cd20.zip
    URL_HASH SHA256=8e59f72b0780690cda64726804269f638a3be77b9d1506ea95f443f7964bccf0
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(stb)

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
if(WIN32)
    target_link_libraries(imgui_lib PUBLIC SDL2::SDL2-static opengl32)
else()
    find_package(OpenGL REQUIRED)
    target_link_libraries(imgui_lib PUBLIC SDL2::SDL2-static OpenGL::GL)
endif()

# ── VibeStation Executable ──────────────────────────────────────────
set(VIBESTATION_SOURCES
    src/main.cpp
    src/platform/cpu_backend_compare_runner.cpp
    src/platform/gpu_correctness_runner.cpp
    src/platform/scheduler_self_test.cpp
    src/platform/disc_path_utils.cpp
    src/platform/memory_card_utils.cpp
    src/platform/sample_profiler.cpp
    src/core/bios.cpp
    src/core/ram.cpp
    src/core/interrupt.cpp
    src/core/timer.cpp
    src/core/spu.cpp
    src/core/audio_ring_buffer.cpp
    src/core/sio.cpp
    src/core/memory_card.cpp
    src/core/pad_controller.cpp
    src/core/cdrom.cpp
    src/core/dma.cpp
    src/core/mdec.cpp
    src/core/gpu.cpp
    src/core/gte.cpp
    src/core/cpu.cpp
    src/core/component_serialize.cpp
    src/core/cpu_recompiler.cpp
    src/core/config.cpp
    src/core/rr4_diagnostics.cpp
    src/core/input_recorder.cpp
    src/core/system.cpp
    src/core/system_snapshot.cpp
    src/core/rewind.cpp
    src/core/renderer.cpp
    src/input/controller.cpp
    src/input/input_manager.cpp
    src/integrations/discord_presence.cpp
    src/ui/emu_runner.cpp
    src/ui/embedded_resources.cpp
    src/ui/frame_presentation_worker.cpp
    src/ui/app.cpp
    src/ui/host_window.cpp
    src/ui/app_discord_presence.cpp
    src/ui/input_bindings.cpp
    src/ui/output_resolution_utils.cpp
    src/ui/screenshot_utils.cpp
    src/ui/theme_settings.cpp
    src/ui/definitive/definitive_audio.cpp
    src/ui/definitive/definitive_background.cpp
    src/ui/definitive/definitive_fonts.cpp
    src/ui/definitive/definitive_gameplay_ambient.cpp
    src/ui/definitive/definitive_grim_reaper.cpp
    src/ui/definitive/definitive_intro.cpp
    src/ui/definitive/definitive_settings.cpp
    src/ui/definitive/definitive_home.cpp
    src/ui/definitive/definitive_vs2_switch.cpp
    # VibeStation 2 runs PS1 discs on this (PS1, C++17) core.
    experimental/ps2/ps1/ps1_mode.cpp
    src/ui/panels/grim_reaper_actions.cpp
    src/ui/panels/grim_reaper_runtime.cpp
    src/ui/panels/cpu_backend_panel.cpp
    src/ui/panels/cpu_debug_panel.cpp
    src/ui/panels/about_panel.cpp
    src/ui/panels/emulator_screen_panel.cpp
    src/ui/panels/grim_reaper_panel.cpp
    src/ui/panels/memory_card_panel.cpp
    src/ui/panels/experimental_settings_panel.cpp
    src/ui/panels/audio_panel.cpp
    src/ui/panels/sound_status_panel.cpp
    src/ui/panels/system_panel.cpp
    src/ui/panels/logging_panel.cpp
    src/ui/panels/fmv_diagnostics_panel.cpp
    src/ui/panels/performance_overlay.cpp
    src/ui/panels/bindings_config_panel.cpp
    src/ui/panels/settings_panel.cpp
    src/ui/panels/vram_panel.cpp
)

set(VIBESTATION_VERSION_STRING "v0.6.0")
set(VIBESTATION_BUILD_NUMBER_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/build_number.txt")
set(VIBESTATION_VERSION_SOURCE
    "${CMAKE_CURRENT_BINARY_DIR}/generated/vibestation_version.cpp")

add_custom_command(
    OUTPUT "${VIBESTATION_VERSION_SOURCE}"
    COMMAND ${CMAKE_COMMAND}
        -DVIBESTATION_VERSION_STRING=${VIBESTATION_VERSION_STRING}
        -DVIBESTATION_BUILD_NUMBER_FILE=${VIBESTATION_BUILD_NUMBER_FILE}
        -DVIBESTATION_VERSION_SOURCE=${VIBESTATION_VERSION_SOURCE}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/increment_build_number.cmake"
    # Do not depend on build_number.txt here: the generator increments it,
    # which would otherwise make this command perpetually dirty.
    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/increment_build_number.cmake"
    VERBATIM
    COMMENT "Updating VibeStation build metadata")

list(APPEND VIBESTATION_SOURCES "${VIBESTATION_VERSION_SOURCE}")
set_source_files_properties("${VIBESTATION_VERSION_SOURCE}" PROPERTIES
    GENERATED TRUE
    SKIP_PRECOMPILE_HEADERS ON)

add_executable(VibeStation ${VIBESTATION_SOURCES})

if(WIN32)
    # Embed the application icon plus launcher background, startup icon and UI
    # sounds directly into VibeStation.exe as Windows resources, along with
    # VibeStation 2's sounds and logos.
    target_sources(VibeStation PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/vibestation.rc)
    # Re-embed when one of VibeStation 2's files changes.
    file(GLOB VIBESTATION_VS2_RESOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/experimental/ps2/resources/vs2/*.wav
        ${CMAKE_CURRENT_SOURCE_DIR}/experimental/ps2/resources/vs2/*.ogg
        ${CMAKE_CURRENT_SOURCE_DIR}/experimental/ps2/resources/vs2/*.png)
    set_source_files_properties(
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/vibestation.rc
        PROPERTIES OBJECT_DEPENDS "${VIBESTATION_VS2_RESOURCES}")
endif()

target_include_directories(VibeStation PRIVATE
    src
    "${stb_SOURCE_DIR}"
    # After src: ui/vs2_hosted.h and ps1/ps1_mode.h. The only name both trees
    # share, ui/theme_settings.h, is the same file.
    experimental/ps2)
target_link_libraries(VibeStation PRIVATE imgui_lib SDL2::SDL2-static nlohmann_json::nlohmann_json)
if(WIN32)
    # sample_profiler: DbgHelp symbol lookup and 1 ms timer resolution.
    target_link_libraries(VibeStation PRIVATE dbghelp winmm)
endif()

# ── VibeStation 2 (PS2) ─────────────────────────────────────────────
# The PS2 core and its frontend are C++20 while VibeStation builds as C++17,
# so they live in their own libraries; main.cpp only sees ui/vs2_hosted.h.
# The PS2 lab project provides the core; its standalone UI stays off here.
set(VIBESTATION_PS2_ENABLE_UI OFF CACHE BOOL "" FORCE)
add_subdirectory(experimental/ps2 EXCLUDE_FROM_ALL)

add_library(vibestation_vs2 STATIC
    experimental/ps2/ui/vs2_hosted.cpp
    experimental/ps2/ui/ps2_app.cpp
    experimental/ps2/ui/ps2_gl_gs_backend.cpp
    experimental/ps2/ui/vs2/vs2_assets.cpp
    experimental/ps2/ui/vs2/vs2_audio.cpp
    experimental/ps2/ui/vs2/vs2_boot.cpp
    experimental/ps2/ui/vs2/vs2_fonts.cpp
    experimental/ps2/ui/vs2/vs2_frontend.cpp
    experimental/ps2/ui/vs2/vs2_home.cpp
    experimental/ps2/ui/vs2/vs2_mixer.cpp
    experimental/ps2/ui/vs2/vs2_orbit.cpp
    experimental/ps2/ui/vs2/vs2_screens.cpp
    experimental/ps2/ui/vs2/vs2_edge_light.cpp
    experimental/ps2/ui/vs2/vs2_toolbar.cpp
    experimental/ps2/ui/vs2/vs2_stb_vorbis.cpp
)
set_target_properties(vibestation_vs2 PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)
target_compile_definitions(vibestation_vs2 PRIVATE SDL_MAIN_HANDLED=1
    VIBESTATION_VERSION_LABEL="${VIBESTATION_VERSION_STRING}")
# stb_image itself is compiled into the executable (definitive_background.cpp).
target_include_directories(vibestation_vs2 PRIVATE ${stb_SOURCE_DIR})
target_link_libraries(vibestation_vs2 PUBLIC vibestation_ps2_core imgui_lib SDL2::SDL2-static)
if(WIN32)
    target_link_libraries(vibestation_vs2 PUBLIC comdlg32 ole32 shell32 uuid)
endif()
target_link_libraries(VibeStation PRIVATE vibestation_vs2)
# On Windows VibeStation 2's sounds and logos are embedded (see the .rc
# above); elsewhere they are looked up in resources/vs2 next to the
# executable. See experimental/ps2/resources/vs2/README.md.
if(NOT WIN32)
add_custom_command(TARGET VibeStation POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/experimental/ps2/resources/vs2"
        "$<TARGET_FILE_DIR:VibeStation>/resources/vs2"
    COMMENT "Copying VibeStation 2 resources"
    VERBATIM)
endif()

# Windows builds consume these assets from resources/vibestation.rc, so the
# executable is self-contained. Keep filesystem copies only for non-Windows
# builds where PE resources are unavailable.
if(NOT WIN32)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resources/ui/definitive/background.jpg")
    add_custom_command(TARGET VibeStation POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:VibeStation>/resources/ui/definitive"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/resources/ui/definitive/background.jpg"
            "$<TARGET_FILE_DIR:VibeStation>/resources/ui/definitive/background.jpg"
        COMMENT "Copying definitive launcher background"
        VERBATIM)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resources/icon512x512.png")
    add_custom_command(TARGET VibeStation POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:VibeStation>/resources"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/resources/icon512x512.png"
            "$<TARGET_FILE_DIR:VibeStation>/resources/icon512x512.png"
        COMMENT "Copying VibeStation startup icon"
        VERBATIM)
endif()

set(VIBESTATION_UI_SOUND_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/ui/definitive/sounds")
add_custom_command(TARGET VibeStation POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "$<TARGET_FILE_DIR:VibeStation>/resources/ui/definitive/sounds"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${VIBESTATION_UI_SOUND_DIR}/cursor.wav"
        "$<TARGET_FILE_DIR:VibeStation>/resources/ui/definitive/sounds/cursor.wav"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${VIBESTATION_UI_SOUND_DIR}/open.wav"
        "$<TARGET_FILE_DIR:VibeStation>/resources/ui/definitive/sounds/open.wav"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${VIBESTATION_UI_SOUND_DIR}/close.wav"
        "$<TARGET_FILE_DIR:VibeStation>/resources/ui/definitive/sounds/close.wav"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${VIBESTATION_UI_SOUND_DIR}/logo.wav"
        "$<TARGET_FILE_DIR:VibeStation>/resources/ui/definitive/sounds/logo.wav"
    COMMENT "Copying definitive UI sound effects"
    VERBATIM)
endif()

if(VIBESTATION_ENABLE_X64_JIT)
    FetchContent_Declare(xbyak
        URL https://github.com/herumi/xbyak/archive/refs/tags/v7.37.4.zip
        URL_HASH SHA256=f041ee3827290cac6fa2d88c3c06502b919e748208b458bf90d5cd41d1808392
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(xbyak)
    target_include_directories(VibeStation PRIVATE ${xbyak_SOURCE_DIR})
    target_compile_definitions(VibeStation PRIVATE VIBESTATION_ENABLE_X64_JIT=1)
    message(STATUS "Xbyak available for native x64 CPU JIT work at ${xbyak_SOURCE_DIR}")
else()
    message(STATUS "Native x64 CPU JIT support disabled; optimized CPU modes use non-native fallbacks")
endif()

option(VIBESTATION_ENABLE_DISCORD_RPC
    "Enable Discord Rich Presence via the Discord Social SDK when available" ON)
set(VIBESTATION_DISCORD_DEFAULT_SDK_ROOT "D:/Projects/github/discord_social_sdk")
if(NOT EXISTS "${VIBESTATION_DISCORD_DEFAULT_SDK_ROOT}/include/discordpp.h")
    set(VIBESTATION_DISCORD_DEFAULT_SDK_ROOT "")
endif()
set(VIBESTATION_DISCORD_SDK_ROOT "" CACHE PATH
    "Path to the extracted Discord Social SDK directory")
if(NOT VIBESTATION_DISCORD_SDK_ROOT AND VIBESTATION_DISCORD_DEFAULT_SDK_ROOT)
    set(VIBESTATION_DISCORD_SDK_ROOT "${VIBESTATION_DISCORD_DEFAULT_SDK_ROOT}"
        CACHE PATH "Path to the extracted Discord Social SDK directory" FORCE)
endif()

if(VIBESTATION_ENABLE_DISCORD_RPC)
    # The SDK root can move outside the repo. Clear cached lookup results so
    # reconfigure picks up the current root instead of reusing stale paths.
    unset(VIBESTATION_DISCORD_INCLUDE_DIR CACHE)
    unset(VIBESTATION_DISCORD_LIBRARY CACHE)
    unset(VIBESTATION_DISCORD_DLL CACHE)

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
            ${VIBESTATION_DISCORD_SDK_ROOT}/lib/release
            ${VIBESTATION_DISCORD_SDK_ROOT}/lib/debug
            ${VIBESTATION_DISCORD_SDK_ROOT}/lib/x64
            ${VIBESTATION_DISCORD_SDK_ROOT}/lib/x86_64
            ${VIBESTATION_DISCORD_SDK_ROOT}/lib
            ${VIBESTATION_DISCORD_SDK_ROOT}
        NO_DEFAULT_PATH
    )

    if(VIBESTATION_DISCORD_INCLUDE_DIR AND VIBESTATION_DISCORD_LIBRARY)
        target_compile_definitions(VibeStation PRIVATE VIBESTATION_HAS_DISCORD_SDK=1)
        target_include_directories(VibeStation PRIVATE ${VIBESTATION_DISCORD_INCLUDE_DIR})

        find_file(VIBESTATION_DISCORD_DLL
            NAMES discord_partner_sdk.dll
            PATHS
                ${VIBESTATION_DISCORD_SDK_ROOT}/bin/release
                ${VIBESTATION_DISCORD_SDK_ROOT}/bin/debug
                ${VIBESTATION_DISCORD_SDK_ROOT}/bin/x64
                ${VIBESTATION_DISCORD_SDK_ROOT}/lib
                ${VIBESTATION_DISCORD_SDK_ROOT}/lib/release
                ${VIBESTATION_DISCORD_SDK_ROOT}/lib/debug
                ${VIBESTATION_DISCORD_SDK_ROOT}/lib/x64
                ${VIBESTATION_DISCORD_SDK_ROOT}/lib/x86_64
                ${VIBESTATION_DISCORD_SDK_ROOT}/bin
                ${VIBESTATION_DISCORD_SDK_ROOT}
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

        # Delay-load the SDK so the exe can start without the DLL present and
        # extract the embedded copy first. MSVC uses /DELAYLOAD; MinGW needs a
        # dlltool-generated delay-import library instead of the MSVC .lib.
        set(VIBESTATION_DISCORD_LINK_LIB "${VIBESTATION_DISCORD_LIBRARY}")
        if(MINGW AND VIBESTATION_DISCORD_DLL)
            find_program(VIBESTATION_GENDEF_TOOL gendef)
            find_program(VIBESTATION_DLLTOOL_TOOL dlltool)
            if(VIBESTATION_GENDEF_TOOL AND VIBESTATION_DLLTOOL_TOOL)
                set(VIBESTATION_DISCORD_DELAY_DEF
                    "${CMAKE_CURRENT_BINARY_DIR}/discord_partner_sdk.def")
                set(VIBESTATION_DISCORD_DELAY_LIB
                    "${CMAKE_CURRENT_BINARY_DIR}/libdiscord_partner_sdk_delay.a")
                add_custom_command(
                    OUTPUT ${VIBESTATION_DISCORD_DELAY_LIB}
                    COMMAND ${VIBESTATION_GENDEF_TOOL} ${VIBESTATION_DISCORD_DLL}
                    COMMAND ${VIBESTATION_DLLTOOL_TOOL}
                            -d ${VIBESTATION_DISCORD_DELAY_DEF}
                            -D discord_partner_sdk.dll
                            -y ${VIBESTATION_DISCORD_DELAY_LIB}
                    DEPENDS ${VIBESTATION_DISCORD_DLL}
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                    COMMENT "Generating delay-import library for discord_partner_sdk.dll"
                    VERBATIM)
                add_custom_target(vibestation_discord_delaylib
                    DEPENDS ${VIBESTATION_DISCORD_DELAY_LIB})
                add_dependencies(VibeStation vibestation_discord_delaylib)
                set(VIBESTATION_DISCORD_LINK_LIB
                    "${VIBESTATION_DISCORD_DELAY_LIB}" delayimp)
            else()
                message(WARNING
                    "gendef/dlltool not found; linking Discord SDK import library "
                    "directly. The exe will require discord_partner_sdk.dll at startup.")
            endif()
        endif()
        target_link_libraries(VibeStation PRIVATE ${VIBESTATION_DISCORD_LINK_LIB})

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
    # /MP parallelizes files inside one cl.exe call, which only helps the
    # Visual Studio generator. Ninja already runs one file per process, and
    # keeping /MP off there keeps invocations simple for compiler caches.
    if(CMAKE_GENERATOR MATCHES "Visual Studio")
        target_compile_options(VibeStation PRIVATE /MP)
        target_compile_options(imgui_lib PRIVATE /MP)
    endif()
    target_compile_definitions(VibeStation PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# MinGW: statically link the GCC runtime so the exe does not depend on
# libgcc_s_seh-1.dll / libstdc++-6.dll / libwinpthread-1.dll at runtime.
if(MINGW)
    target_link_options(VibeStation PRIVATE -static -static-libgcc -static-libstdc++)
endif()

option(VIBESTATION_ENABLE_IPO "Enable interprocedural optimization for release builds" ON)
option(VIBESTATION_FAST_ITERATION
    "Favor fast compile/link cycles while developing the definitive UI" OFF)

if(VIBESTATION_ENABLE_IPO AND NOT VIBESTATION_FAST_ITERATION)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT VIBESTATION_IPO_SUPPORTED OUTPUT VIBESTATION_IPO_ERROR)
    if(VIBESTATION_IPO_SUPPORTED)
        set_property(TARGET VibeStation PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
        set_property(TARGET VibeStation PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO TRUE)
    else()
        message(STATUS "IPO/LTO not supported: ${VIBESTATION_IPO_ERROR}")
    endif()
elseif(VIBESTATION_ENABLE_IPO AND VIBESTATION_FAST_ITERATION)
    message(STATUS
        "Fast iteration enabled: skipping IPO/LTO to reduce Release link times")
endif()


include(CTest)
if(BUILD_TESTING)
    add_test(NAME vibestation_gpu_correctness
        COMMAND $<TARGET_FILE:VibeStation> --gpu-self-test)
    set_tests_properties(vibestation_gpu_correctness PROPERTIES TIMEOUT 30)
endif()
