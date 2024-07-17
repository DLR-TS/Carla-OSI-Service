cmake_minimum_required(VERSION 3.12)
include(FetchContent)

FetchContent_Declare(
  osi
  GIT_REPOSITORY https://github.com/OpenSimulationInterface/open-simulation-interface.git
  GIT_TAG "v3.7.0"
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
