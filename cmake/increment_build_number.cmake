if(NOT DEFINED VIBESTATION_BUILD_NUMBER_FILE)
    message(FATAL_ERROR "VIBESTATION_BUILD_NUMBER_FILE is required")
endif()

if(NOT DEFINED VIBESTATION_VERSION_SOURCE)
    message(FATAL_ERROR "VIBESTATION_VERSION_SOURCE is required")
endif()

if(NOT DEFINED VIBESTATION_VERSION_STRING)
    set(VIBESTATION_VERSION_STRING "v0.6.0")
endif()

set(VIBESTATION_INITIAL_BUILD_NUMBER 230)

if(EXISTS "${VIBESTATION_BUILD_NUMBER_FILE}")
    file(READ "${VIBESTATION_BUILD_NUMBER_FILE}" build_number)
    string(STRIP "${build_number}" build_number)
    if(build_number STREQUAL "")
        set(build_number "${VIBESTATION_INITIAL_BUILD_NUMBER}")
    endif()
else()
    set(build_number "${VIBESTATION_INITIAL_BUILD_NUMBER}")
endif()

if(NOT build_number MATCHES "^[0-9]+$")
    message(FATAL_ERROR
        "Invalid VibeStation build number '${build_number}' in "
        "${VIBESTATION_BUILD_NUMBER_FILE}")
endif()

math(EXPR next_build_number "${build_number} + 1")
set(full_version_string
    "VibeStation ${VIBESTATION_VERSION_STRING} Build ${build_number}")

get_filename_component(version_source_dir "${VIBESTATION_VERSION_SOURCE}" DIRECTORY)
file(MAKE_DIRECTORY "${version_source_dir}")

set(version_source_content
"#include \"version.h\"

const char* vibestation_version_string() noexcept {
    return \"${VIBESTATION_VERSION_STRING}\";
}

unsigned int vibestation_build_number() noexcept {
    return ${build_number}u;
}

const char* vibestation_full_version_string() noexcept {
    return \"${full_version_string}\";
}
")

set(version_source_tmp "${VIBESTATION_VERSION_SOURCE}.tmp")
file(WRITE "${version_source_tmp}" "${version_source_content}")
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different
    "${version_source_tmp}" "${VIBESTATION_VERSION_SOURCE}"
    COMMAND_ERROR_IS_FATAL ANY)
file(REMOVE "${version_source_tmp}")

file(WRITE "${VIBESTATION_BUILD_NUMBER_FILE}" "${next_build_number}\n")
message(STATUS "Generated ${full_version_string}")
