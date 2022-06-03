cmake_minimum_required(VERSION 3.12)
include(FetchContent)

file(READ ${PROJECT_SOURCE_DIR}/.TOKEN ACCESS_TOKEN_GITLAB)
string(REGEX REPLACE "\n$" "" ACCESS_TOKEN_GITLAB_STRIPPED ${ACCESS_TOKEN_GITLAB})

FetchContent_Declare(
  osi
  GIT_REPOSITORY https://${ACCESS_TOKEN_GITLAB_STRIPPED}@gitlab.setlevel.de/deliverables/architecture/open-simulation-interface.git
  GIT_TAG "sl45/v3.2.2"
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
)

set(FETCHCONTENT_QUIET OFF)
# not available before CMake version 3.14 - Using FetchContent_Populate instead
#FetchContent_MakeAvailable(gRPC)

FetchContent_GetProperties(osi)
if(NOT osi_POPULATED)
  FetchContent_Populate(osi)
  add_subdirectory(${osi_SOURCE_DIR} ${osi_BINARY_DIR} EXCLUDE_FROM_ALL)
endif(NOT osi_POPULATED)
