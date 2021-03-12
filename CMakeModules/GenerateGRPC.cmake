cmake_minimum_required(VERSION 3.5)
# based on protobuf_generate_grpc_cpp from root CMakeLists.txt of gRPC and protobuf_generate of FindProtobuf.cmake

#  generate_grpc_cpp
#  --------------------------
#
#   Add custom commands to process ``.proto`` files to C++ using protoc and
#   GRPC plugin::
#
#     generate_grpc_cpp  PROTO_FILES <PROTO_FILES>... [PROTOC_OUT_DIR <protoc_out_dir>] [TARGET <target>] [GRPC_SRCS <grpc_srcs_var>] [GRPC_HDRS <grpc_hdrs_var>] [GRPC_MOCK_HDRS <grpc_mock_hdrs_var>] [GRPC_PB_SRCS <grpc_pb_srcs_var>] [GRPC_PB_HDRS <grpc_pb_hdrs_var>] [IMPORT_DIRS <import_dirs>...]
#
#   ``PROTO_FILES``
#     ``.proto`` files
#
#   ``PROTOC_OUT_DIR``
#     output directory for generated files.
#     Defaults to ${CMAKE_CURRENT_BINARY_DIR}/gens
#
#   ``TARGET``
#     Target name to generate sources from or for
#     Searches the target's target_sources list for ``.proto`` files to process and also appends generated files to the target's target_sources list
#
#   ``GRPC_SRCS``
#     Variable name to store a list of generated ".grpc.cc" files
#
#   ``GRPC_HDRS``
#     Variable name to store a list of generated ".grpc.h" files
#
#   ``GRPC_MOCK_HDRS``
#     Variable name to store a list of generated "_mock.grpc.h" files
#
#   ``GRPC_PB_SRCS``
#     Variable name to store a list of generated ".grpc.pb.cc" files
#
#   ``GRPC_PB_HDRS``
#     Variable name to store a list of generated ".grpc.pb.h" files
#
#   ``IMPORT_DIRS``
#     List of additional protobuffer import dirs to pass to protoc
#
function(generate_grpc_cpp)# PROTO_FILES PROTOC_OUT_DIR GRPC_SRCS GRPC_HDRS GRPC_MOCK_HDRS GRPC_PB_SRCS GRPC_PB_HDRS
  #set(_options APPEND_PATH)
  set(_singleargs PROTOC_OUT_DIR GRPC_SRCS GRPC_HDRS GRPC_MOCK_HDRS GRPC_PB_SRCS GRPC_PB_HDRS)
  if(COMMAND target_sources)
    list(APPEND _singleargs TARGET)
  endif()
  set(_multiargs PROTO_FILES IMPORT_DIRS)

  cmake_parse_arguments(generate_grpc_cpp "${_options}" "${_singleargs}" "${_multiargs}" "${ARGN}")

  if(NOT generate_grpc_cpp_PROTO_FILES AND NOT generate_grpc_cpp_TARGET)
    message(SEND_ERROR "Error: GENERATE_GRPC_CPP() called without any proto files")
    return()
  endif()

  if(NOT generate_grpc_cpp_PROTOC_OUT_DIR)
    set(generate_grpc_cpp_PROTOC_OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/gens")
  endif()

  message(INFO "Will generate gRPC files into ${generate_grpc_cpp_PROTOC_OUT_DIR}")

  if(generate_grpc_cpp_TARGET)
    get_target_property(_source_list ${generate_grpc_cpp_TARGET} SOURCES)
    foreach(_file ${_source_list})
      if(_file MATCHES "proto$")
        list(APPEND generate_grpc_cpp_PROTO_FILES ${_file})
      endif()
    endforeach()
  endif()

  set(_protobuf_include_path -I "${CMAKE_CURRENT_SOURCE_DIR}")
  foreach(DIR ${generate_grpc_cpp_IMPORT_DIRS})
    get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
    list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
    if(${_contains_already} EQUAL -1)
        list(APPEND _protobuf_include_path -I ${ABS_PATH})
    endif()
  endforeach()

  find_program(Protobuf_PROTOC_EXECUTABLE
	NAMES protoc
	DOC "The Google Protocol Buffers Compiler"
	HINTS
	${CONAN_BIN_DIRS}
  )
  
  #if cross-compiling, find host plugin
  if(CMAKE_CROSSCOMPILING)
    find_program(_gRPC_CPP_PLUGIN grpc_cpp_plugin)
  else()
     set(_gRPC_CPP_PLUGIN $<TARGET_FILE:grpc_cpp_plugin>)
  endif()

  foreach(FIL ${generate_grpc_cpp_PROTO_FILES})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)
    file(RELATIVE_PATH REL_FIL ${CMAKE_CURRENT_SOURCE_DIR} ${ABS_FIL})
    get_filename_component(REL_DIR ${REL_FIL} DIRECTORY)
    set(RELFIL_WE "${REL_DIR}/${FIL_WE}")

	set(_grpc_pb_src    "${generate_grpc_cpp_PROTOC_OUT_DIR}/${RELFIL_WE}.grpc.pb.cc")
	set(_grpc_pb_h      "${generate_grpc_cpp_PROTOC_OUT_DIR}/${RELFIL_WE}.grpc.pb.h")
	set(_grpc_pb_h_mock "${generate_grpc_cpp_PROTOC_OUT_DIR}/${RELFIL_WE}_mock.grpc.pb.h")
	set(_pb_src         "${generate_grpc_cpp_PROTOC_OUT_DIR}/${RELFIL_WE}.pb.cc")
	set(_pb_h           "${generate_grpc_cpp_PROTOC_OUT_DIR}/${RELFIL_WE}.pb.h")

	set(_GRPC_PB_SRCS   ${_GRPC_PB_SRCS}   ${_grpc_pb_src})
	set(_GRPC_PB_HDRS   ${_GRPC_PB_HDRS}   ${_grpc_pb_h})
	set(_GRPC_MOCK_HDRS ${_GRPC_MOCK_HDRS} ${_grpc_pb_h_mock})
	set(_GRPC_SRCS      ${_GRPC_SRCS}      ${_pb_src})
	set(_GRPC_HDRS      ${_GRPC_HDRS}      ${_pb_h})

    add_custom_command(
      OUTPUT ${_grpc_pb_src}
             ${_grpc_pb_h}
             ${_grpc_pb_h_mock}
             ${_pb_src}
             ${_pb_h}
      COMMAND ${Protobuf_PROTOC_EXECUTABLE}
      ARGS --grpc_out=generate_mock_code=true:${generate_grpc_cpp_PROTOC_OUT_DIR}
           --cpp_out=${generate_grpc_cpp_PROTOC_OUT_DIR}
           --plugin=protoc-gen-grpc=${_gRPC_CPP_PLUGIN}
           ${_protobuf_include_path}
           ${REL_FIL}
      DEPENDS ${ABS_FIL} ${_gRPC_PROTOBUF_PROTOC} grpc_cpp_plugin
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMENT "Running gRPC C++ protocol buffer compiler on ${FIL}"
      VERBATIM)
  endforeach()

  if(generate_grpc_cpp_GRPC_PB_SRCS)
    set(${generate_grpc_cpp_GRPC_PB_SRCS} ${_GRPC_PB_SRCS} PARENT_SCOPE)
     endif()
  if(generate_grpc_cpp_GRPC_PB_HDRS)
    set(${generate_grpc_cpp_GRPC_PB_HDRS} ${_GRPC_PB_HDRS} PARENT_SCOPE)
  endif()
  if(generate_grpc_cpp_GRPC_MOCK_HDRS)
    set(${generate_grpc_cpp_GRPC_MOCK_HDRS} ${_GRPC_MOCK_HDRS} PARENT_SCOPE)
  endif()
  if(generate_grpc_cpp_GRPC_SRCS)
    set(${generate_grpc_cpp_GRPC_SRCS} ${_GRPC_SRCS} PARENT_SCOPE)
  endif()
  if(generate_grpc_cpp_GRPC_HDRS)
    set(${generate_grpc_cpp_GRPC_HDRS} ${_GRPC_HDRS} PARENT_SCOPE)
  endif()

  if(generate_grpc_cpp_TARGET)
    target_sources(${generate_grpc_cpp_TARGET} PRIVATE ${_GRPC_PB_SRCS} ${_GRPC_SRCS})
    target_sources(${generate_grpc_cpp_TARGET} PUBLIC ${_GRPC_PB_HDRS} ${_GRPC_HDRS} ${_GRPC_MOCK_HDRS})
  endif()
endfunction()