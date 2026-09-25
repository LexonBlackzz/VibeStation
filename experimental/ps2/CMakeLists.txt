cmake_minimum_required(VERSION 3.20)

project(VibeStationPS2Lab LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(vibestation_ps2_core STATIC
    core/bios/bios.cpp
    core/cdvd/cdvd_hw.cpp
    core/dma/gif_dma.cpp
    core/dma/ipu_dma.cpp
    core/dma/sif_dma.cpp
    core/dma/spr_dma.cpp
    core/dma/vif0_dma.cpp
    core/dma/vif1_dma.cpp
    core/ps2_system.cpp
    core/ee/ee_cpu.cpp
    core/ee/ee_jit.cpp
    core/gs/gs_core.cpp
    core/gs/gs_display.cpp
    core/gs/gs_rasterizer.cpp
    core/gs/gs_vram.cpp
    core/gs/gs_privileged.cpp
    core/hw/ee_hw.cpp
    core/hw/iop_hw_window.cpp
    core/input/sio2_pad.cpp
    core/iop/iop_bus.cpp
    core/iop/iop_cpu.cpp
    core/iop/iop_intc.cpp
    core/iop/iop_ram.cpp
    core/memory/ee_bus.cpp
    core/memory/ee_ram.cpp
    core/memory/ee_scratchpad.cpp
    core/scheduler/scheduler.cpp
    core/spu2/spu2.cpp
    core/video/video_timing.cpp
    core/vu/vu1.cpp
)

target_include_directories(vibestation_ps2_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
find_package(Threads REQUIRED)
target_link_libraries(vibestation_ps2_core PUBLIC Threads::Threads)

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
        ui/ps2_gl_gs_backend.cpp
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

add_executable(vibestation_ps2_bios_trace
    tools/bios_trace.cpp
)
target_link_libraries(vibestation_ps2_bios_trace PRIVATE vibestation_ps2_core)

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

    add_executable(vibestation_ps2_bootstrap_tests
        tests/ps2_bootstrap_tests.cpp
    )
    target_link_libraries(vibestation_ps2_bootstrap_tests PRIVATE vibestation_ps2_core)
    add_test(
        NAME vibestation_ps2_bootstrap
        COMMAND $<TARGET_FILE:vibestation_ps2_bootstrap_tests>
    )

    add_executable(vibestation_ps2_vif0_tests
        tests/ps2_vif0_tests.cpp
    )
    target_link_libraries(vibestation_ps2_vif0_tests PRIVATE vibestation_ps2_core)
    add_test(
        NAME vibestation_ps2_vif0
        COMMAND $<TARGET_FILE:vibestation_ps2_vif0_tests>
    )

    add_executable(vibestation_ps2_vif1_tests
        tests/ps2_vif1_tests.cpp
    )
    target_link_libraries(vibestation_ps2_vif1_tests PRIVATE vibestation_ps2_core)
    add_test(
        NAME vibestation_ps2_vif1
        COMMAND $<TARGET_FILE:vibestation_ps2_vif1_tests>
    )

    add_executable(vibestation_ps2_vu1_tests
        tests/ps2_vu1_tests.cpp
    )
    target_link_libraries(vibestation_ps2_vu1_tests PRIVATE vibestation_ps2_core)
    add_test(
        NAME vibestation_ps2_vu1
        COMMAND $<TARGET_FILE:vibestation_ps2_vu1_tests>
    )

    add_executable(vibestation_ps2_sif_tests
        tests/ps2_sif_tests.cpp
    )
    target_link_libraries(vibestation_ps2_sif_tests PRIVATE vibestation_ps2_core)
    add_test(
        NAME vibestation_ps2_sif
        COMMAND $<TARGET_FILE:vibestation_ps2_sif_tests>
    )
endif()

# The interpreter crosses several small EE/IOP/bus translation units for
# each emulated instruction. Release LTO lets the compiler inline those hot
# calls without changing the emulated instruction or device cadence.
include(CheckIPOSupported)
check_ipo_supported(RESULT VIBESTATION_PS2_IPO_SUPPORTED LANGUAGES CXX)
if(VIBESTATION_PS2_IPO_SUPPORTED)
    set_property(TARGET vibestation_ps2_core PROPERTY
        INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
    set_property(TARGET vibestation_ps2_bios_trace PROPERTY
        INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
    if(VIBESTATION_PS2_ENABLE_UI)
        set_property(TARGET VibeStationPS2Lab PROPERTY
            INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
    endif()
endif()
