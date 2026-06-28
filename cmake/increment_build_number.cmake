if(NOT DEFINED VIBESTATION_BUILD_NUMBER_FILE)
    message(FATAL_ERROR "VIBESTATION_BUILD_NUMBER_FILE is required")
endif()

if(NOT DEFINED VIBESTATION_VERSION_HEADER)
    message(FATAL_ERROR "VIBESTATION_VERSION_HEADER is required")
endif()

if(NOT DEFINED VIBESTATION_VERSION_STRING)
    set(VIBESTATION_VERSION_STRING "v0.5.3-dev")
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

get_filename_component(version_header_dir "${VIBESTATION_VERSION_HEADER}" DIRECTORY)
file(MAKE_DIRECTORY "${version_header_dir}")

set(version_header_content
"#pragma once

#define VIBESTATION_VERSION_STRING \"${VIBESTATION_VERSION_STRING}\"
#define VIBESTATION_BUILD_NUMBER ${build_number}
#define VIBESTATION_FULL_VERSION_STRING \"${full_version_string}\"
")

set(version_header_tmp "${VIBESTATION_VERSION_HEADER}.tmp")
file(WRITE "${version_header_tmp}" "${version_header_content}")
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different
    "${version_header_tmp}" "${VIBESTATION_VERSION_HEADER}"
    COMMAND_ERROR_IS_FATAL ANY)
file(REMOVE "${version_header_tmp}")

file(WRITE "${VIBESTATION_BUILD_NUMBER_FILE}" "${next_build_number}\n")
message(STATUS "Generated ${full_version_string}")
