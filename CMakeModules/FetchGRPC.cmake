cmake_minimum_required(VERSION 3.12)
include(FetchContent)

FetchContent_Declare(
  gRPC
  GIT_REPOSITORY https://github.com/grpc/grpc
  GIT_TAG        v1.29.x
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  LOG_CONFIGURE TRUE
  LOG_BUILD TRUE
  LOG_INSTALL TRUE
)

set(gRPC_BUILD_GRPC_CPP_PLUGIN ON CACHE BOOL "Build grpc_cpp_plugin")
set(gRPC_BUILD_GRPC_CSHARP_PLUGIN OFF CACHE BOOL "Build grpc_csharp_plugin")
set(gRPC_BUILD_GRPC_NODE_PLUGIN OFF CACHE BOOL "Build grpc_node_plugin")
set(gRPC_BUILD_GRPC_OBJECTIVE_C_PLUGIN OFF CACHE BOOL "Build grpc_objective_c_plugin")
set(gRPC_BUILD_GRPC_PHP_PLUGIN OFF CACHE BOOL "Build grpc_php_plugin")
set(gRPC_BUILD_GRPC_PYTHON_PLUGIN OFF CACHE BOOL "Build grpc_python_plugin")
set(gRPC_BUILD_GRPC_RUBY_PLUGIN OFF CACHE BOOL "Build grpc_ruby_plugin")


set(FETCHCONTENT_QUIET OFF)
# not available before CMake version 3.14 - Using FetchContent_Populate instead
#FetchContent_MakeAvailable(gRPC)

FetchContent_GetProperties(gRPC)
if(NOT grpc_POPULATED)
	FetchContent_Populate(gRPC)

	if(TARGET zlib OR TARGET CONAN_PKG::zlib)
		message(VERBOSE "Using existing zlib provider")
		set(gRPC_ZLIB_PROVIDER "package" CACHE STRING "Provider of zlib library")
	else()
		set(gRPC_ZLIB_PROVIDER "module" CACHE STRING "Provider of zlib library")
	endif()

	if(TARGET protobuf::protoc OR TARGET CONAN_PKG::protobuf)
		message(VERBOSE "Using existing protobuf provider")
		set(gRPC_PROTOBUF_PROVIDER "package" CACHE STRING "Provider of protobuf library")

		# gRPC uses a different variable to access protoc with it's own find_programm call and therefore suffers from the same search path problem as the default FindProtobuf script, which has difficulties finding protoc inside the conan package
		find_program(Protobuf_PROTOC_EXECUTABLE
			NAMES protoc
			DOC "The Google Protocol Buffers Compiler"
			HINTS
			${CONAN_BIN_DIRS}
		)
		if(Protobuf_PROTOC_EXECUTABLE)
			set(_gRPC_PROTOBUF_PROTOC_EXECUTABLE ${Protobuf_PROTOC_EXECUTABLE})
		endif()
	else()
		set(gRPC_PROTOBUF_PROVIDER "module" CACHE STRING "Provider of protobuf library")
	endif()

	# deactivate abseil-cpp option for building tests
	set(BUILD_TESTING FALSE)

	message(STATUS "Fetched gRPC to ${grpc_SOURCE_DIR}. Will put build results into ${grpc_BINARY_DIR}.")
	if((MSVC OR MINGW) AND BUILD_SHARED_LIBS)
		message(STATUS "Will build gRPC as static library because dll support is currently broken")
		set(BUILD_SHARED_LIBS_TMP TRUE)
		set(BUILD_SHARED_LIBS FALSE)
	endif()
	add_subdirectory(${grpc_SOURCE_DIR} ${grpc_BINARY_DIR} EXCLUDE_FROM_ALL)
	if(BUILD_SHARED_LIBS_TMP)
		set(BUILD_SHARED_LIBS False)
	endif()
	message(STATUS "Added gRPC subdirectory (${grpc_SOURCE_DIR}).")

	if(TARGET CONAN_PKG::protobuf)
		# correct the include directory
		set(_gRPC_PROTOBUF_WELLKNOWN_INCLUDE_DIR ${CONAN_INCLUDE_DIRS_PROTOBUF})
	endif()
	if(MSVC OR MINGW)
		# required compiler definition that is missing in older gRPC versions
		add_compile_definitions(_WIN32_WINNT=0x0600)
	endif()
endif()
