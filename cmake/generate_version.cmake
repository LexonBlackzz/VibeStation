if(NOT DEFINED VIBESTATION_VERSION_SOURCE)
    message(FATAL_ERROR "VIBESTATION_VERSION_SOURCE is required")
endif()

if(NOT DEFINED VIBESTATION_VERSION_STRING)
    set(VIBESTATION_VERSION_STRING "v0.6.0")
endif()

if(NOT DEFINED VIBESTATION_SOURCE_DIR)
    get_filename_component(VIBESTATION_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}" DIRECTORY)
endif()

# The build number is the commit count of HEAD plus a fixed offset, so every
# commit has exactly one build number, wherever it is built (CI or locally).
# CI checkouts must fetch full history (fetch-depth: 0) or the count is 1.
# The offset carries numbering on from the old per-build counter.
set(VIBESTATION_BUILD_NUMBER_OFFSET 100)

find_package(Git QUIET)
set(commit_count "")
if(GIT_FOUND)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" rev-list --count HEAD
        WORKING_DIRECTORY "${VIBESTATION_SOURCE_DIR}"
        OUTPUT_VARIABLE commit_count
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
endif()

if(commit_count MATCHES "^[0-9]+$")
    math(EXPR build_number "${commit_count} + ${VIBESTATION_BUILD_NUMBER_OFFSET}")
else()
    # Not a git checkout (e.g. a source archive): no meaningful build number.
    set(build_number 0)
endif()

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

message(STATUS "Generated ${full_version_string}")
