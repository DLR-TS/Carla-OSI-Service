# CMakeList.txt: CMake-Projektdatei der obersten Ebene.
#
cmake_minimum_required (VERSION 3.8)
message("Running with CMake Version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}")

project ("CoSimulationManager")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(GIT_SUBMODULE "Check submodules during build" ON)
if(GIT_SUBMODULE)
	find_package(Git QUIET)
	if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
		message(STATUS "Updating submodules")
		execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
						WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
						RESULT_VARIABLE GIT_SUBMODULE_RESULT)
		if(NOT GIT_SUBMODULE_RESULT EQUAL "0")
			message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMODULE_RESULT}")
		endif()
	endif()
endif()

# Download automatically, you can also just copy the conan.cmake file
if(NOT EXISTS "conan/conan.cmake")
   message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
   file(DOWNLOAD "https://github.com/conan-io/cmake-conan/raw/v0.14/conan.cmake"
                 "conan/conan.cmake")
endif()

include(conan/conan.cmake)

# get dependencies distributed as conan packages
conan_cmake_run(CONANFILE conanfile.txt 
                BASIC_SETUP CMAKE_TARGETS
                BUILD missing)

# FMI4cpp is not distributed as conan package, but still can use conan for dependency management.
conan_cmake_run(CONANFILE lib/FMI4cpp/conanfile.txt
                INSTALL_FOLDER "${CMAKE_BINARY_DIR}/lib/FMI4cpp/"
                BASIC_SETUP
                BUILD missing)
# FMI4CPP_USING_CONAN is faulty because it doesn't use the correct binary dir
#set(FMI4CPP_USING_CONAN ON CACHE BOOL "Build FMI4cpp using conan" FORCE)
include(${CMAKE_CURRENT_BINARY_DIR}/lib/FMI4cpp/conanbuildinfo.cmake)
conan_basic_setup()
add_subdirectory(lib/FMI4cpp/ EXCLUDE_FROM_ALL)


# include protobuf, yaml_cpp, catch2, fakeit
include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
include(${CMAKE_CURRENT_BINARY_DIR}/conan_paths.cmake)
conan_basic_setup()


#main project
# set up variables that are used below
add_subdirectory("src")
add_subdirectory("include")
add_subdirectory("test")
#create library to

add_library(CoSimulationManagerLib STATIC ${COSIMA_SOURCE} ${COSIMA_INCLUDES})
target_include_directories(CoSimulationManagerLib PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(CoSimulationManagerLib ${CONAN_LIBS})#open_simulation_interface_static
# FMI4cpp is not managed by conan, thus it is added here
target_include_directories(CoSimulationManagerLib PUBLIC fmi4cpp)
target_link_libraries(CoSimulationManagerLib fmi4cpp)

######################OSI
#from old config
#set (ENV{PATH} "${CMAKE_SOURCE_DIR}/lib/protobuf/build2/Debug")
#set (ENV{PROTOBUF} "${CMAKE_SOURCE_DIR}/cosima/lib/protobuf")
#set (ENV{PROTOBUF_SRC_ROOT_FOLDER} "%PROTOBUF%")
#set (ENV{CMAKE_INCLUDE_PATH} "${CMAKE_SOURCE_DIR}/lib/protobuf/src")
#set (ENV{CMAKE_LIBRARY_PATH} "${CMAKE_SOURCE_DIR}/lib/protobuf/build2/Debug")

set (ENV{PATH} "C:/.conan/da210a/1/lib")
set (ENV{PROTOBUF} "C:/.conan/da210a/1")
set (ENV{PROTOBUF_SRC_ROOT_FOLDER} "%PROTOBUF%")
set (ENV{CMAKE_INCLUDE_PATH} "C:/.conan/da210a/1/include/google/protobuf")
set (ENV{CMAKE_LIBRARY_PATH} "C:/.conan/da210a/1/lib")

add_subdirectory(lib/open-simulation-interface/ EXCLUDE_FROM_ALL)# --> Unknown CMake command "protobuf_generate_cpp".
target_include_directories(CoSimulationManagerLib PUBLIC open-simulation-interface)
target_link_libraries(CoSimulationManagerLib open-simulation-interface)
#1. OSI herunterladen - check
#2. Protobuf kompilieren
#3. OSI mit Protobuf Libs kombinieren (Umgegungsvariablen)
#4. OSI kompilieren mit Probuf Library
#5. OSI Library einbinden
######################OSI

find_package(Catch2 REQUIRED)
add_executable(test_CoSiMa ${TESTFILES})
target_include_directories(test_CoSiMa PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(test_CoSiMa CoSimulationManagerLib ${CONAN_LIBS} Catch2::Catch2)

#copy test resources to build folder
add_custom_command(TARGET test_CoSiMa PRE_BUILD
                     COMMAND ${CMAKE_COMMAND} -E
                         make_directory $<TARGET_FILE_DIR:test_CoSiMa>/../test/resources/)
#Feedthrough_cs_me.fmu contains definitions for both me and cs => created copies with edited modelDescription.xml
add_custom_command(TARGET test_CoSiMa PRE_BUILD
                     COMMAND ${CMAKE_COMMAND} -E
                         copy_if_different
						 ${CMAKE_CURRENT_LIST_DIR}/test/resources/testconfig1.yaml
						 ${CMAKE_CURRENT_LIST_DIR}/test/resources/Feedthrough_cs.fmu
						 ${CMAKE_CURRENT_LIST_DIR}/test/resources/Feedthrough_me.fmu
						 ${CMAKE_CURRENT_LIST_DIR}/test/resources/Stair.fmu
						 $<TARGET_FILE_DIR:test_CoSiMa>/../test/resources/)

include(CTest)
include(Catch)
catch_discover_tests(test_CoSiMa)


add_executable (CoSimulationManager "${CMAKE_CURRENT_LIST_DIR}/src/CoSiMa.cpp" "${CMAKE_CURRENT_LIST_DIR}/include/CoSiMa.h")
target_link_libraries(CoSimulationManager CoSimulationManagerLib)