# Fetch LibCarla_client and dependencies not packaged for conan
if(NOT COMMAND FetchContent_Declare)
	include(FetchContent)
endif(NOT COMMAND FetchContent_Declare)

# Which version of LibCarla should get fetched? Variable is also needed by Carla itself to generate Version.h
if(NOT DEFINED CARLA_VERSION)# don't overwrite if already defined
	set(CARLA_VERSION 0.9.9.4 )
endif()

# LibCarla itself
FetchContent_Declare(
  LibCarla_client
  GIT_REPOSITORY https://github.com/carla-simulator/carla.git
  GIT_TAG ${CARLA_VERSION}
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  PREFIX lib/Carla
  CMAKE_ARGS "-DCMAKE_BUILD_TYPE=Client"
)

# Dependency of LibCarla_client, usually set up by 'make setup'. Not available as conan package. Carla specific fork of https://github.com/recastnavigation/recastnavigation
FetchContent_Declare(
  libRecast
  GIT_REPOSITORY https://github.com/carla-simulator/recastnavigation.git
  GIT_TAG cdce4e1a270fdf1f3942d4485954cc5e136df1df
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  PREFIX lib/Carla/Build
  CMAKE_ARGS "-DRECASTNAVIGATION_DEMO=False;-DRECASTNAVIGATION_TEST=False"
)

# Dependency of LibCarla_client, usually set up by 'make setup'. Not available as conan package. Carla specific fork of https://github.com/rpclib/rpclib
FetchContent_Declare(
  libRPC
  GIT_REPOSITORY https://github.com/carla-simulator/rpclib.git
  GIT_TAG v2.2.1_c2
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  PREFIX lib/Carla/Build
  CMAKE_ARGS "-DRPCLIB_BUILD_EXAMPLES=OFF"
)

message("CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE}")
message("CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG}")

function(fetch_carla_and_non_conan_dependencies)
	# Using a function for fetching Carla and dependencies not available as conan package and adding it as subdirectory because of its separate variable scope

	message(VERBOSE "Fetching Carla and non-conan dependencies")
	message("CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE}")
	message("CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG}")

	if(WIN32)#TODO This test might be too unprecise
		add_compile_definitions(_WIN32_WINNT=0x0600)
		add_compile_definitions(HAVE_SNPRINTF)
		# Always defined for windows builds, but server specific for linux builds.
		add_compile_definitions(BOOST_ERROR_CODE_HEADER_ONLY)
	endif()

	# Only specified in setup.bat
	add_definitions(-DLIBCARLA_IMAGE_WITH_PNG_SUPPORT)

	#set(BUILD_SHARED_LIBS False)

	FetchContent_GetProperties(libRecast)
	if(NOT librecast_POPULATED)
		FetchContent_Populate(libRecast)
		set(RECASTNAVIGATION_TEST False)
		set(RECASTNAVIGATION_DEMO False)
		message("LibRecast: ${librecast_SOURCE_DIR} ${librecast_BINARY_DIR}")
		add_subdirectory(${librecast_SOURCE_DIR} ${librecast_BINARY_DIR} EXCLUDE_FROM_ALL)
	endif()

	FetchContent_GetProperties(libRPC)
	if(NOT librpc_POPULATED)
		FetchContent_Populate(librpc)
		set(RPCLIB_BUILD_EXAMPLES OFF)
		set(RPCLIB_BUILD_TEST OFF)
		set(RPCLIB_CXX_STANDARD 14)
		add_subdirectory(${librpc_SOURCE_DIR} ${librpc_BINARY_DIR} EXCLUDE_FROM_ALL)
		set_property(TARGET rpc PROPERTY WINDOWS_EXPORT_ALL_SYMBOLS ON)# no explicit exports on windows platform
	endif()


	# LibCarla depends on these variables
	set(BOOST_INCLUDE_PATH ${CONAN_INCLUDE_DIRS_BOOST})
	set(BOOST_LIB_PATH ${CONAN_LIB_DIRS_BOOST})
	#set(RPCLIB_INCLUDE_PATH "%CMAKE_INSTALLATION_DIR%rpclib-install/include")
	get_target_property(RPCLIB_INCLUDE_PATH rpc INTERFACE_INCLUDE_DIRECTORIES)
	#set(RPCLIB_LIB_PATH "%CMAKE_INSTALLATION_DIR%rpclib-install/lib")
	get_target_property(RPCLIB_LIB_PATH rpc LIBRARY_OUTPUT_DIRECTORY)# or BINARY_DIR if not static?
	set(ZLIB_INCLUDE_PATH ${CONAN_INCLUDE_DIRS_ZLIB})
	set(ZLIB_LIB_PATH ${CONAN_LIB_DIRS_ZLIB})
	# files in ${CONAN_INCLUDE_DIRS_LIBPNG} and ${CONAN_INCLUDE_DIRS_LIBPNG}/libpng16 diff as identical (at least on Windows10), but Carla collects headers by globbing all names in ${CONAN_INCLUDE_DIRS_LIBPNG}, resulting in a CMake error because libpng16 is not a filename
	set(LIBPNG_INCLUDE_PATH ${CONAN_INCLUDE_DIRS_LIBPNG}/libpng16)
	set(LIBPNG_LIB_PATH ${CONAN_LIB_DIRS_LIBPNG})

	# Carla's recast variables actually point to directories in which the setup scripts collect the contents of the different include and library folders of recastnavigation
	# This behaviour is imitated here.
	# Needed libraries are: Recast, DetourCrowd, DetourNavMesh, DetourNavMeshBuilder, DetourNavMeshQuery, DetourCommon
	file(MAKE_DIRECTORY "${librecast_BINARY_DIR}/CombinedIncludes")
	file(REMOVE_RECURSE "${librecast_BINARY_DIR}/CombinedIncludes/recast")#remove existing or rename will fail
	file(COPY "${librecast_SOURCE_DIR}/Recast/Include" DESTINATION "${librecast_BINARY_DIR}/CombinedIncludes")
	file(COPY "${librecast_SOURCE_DIR}/DetourCrowd/Include" DESTINATION "${librecast_BINARY_DIR}/CombinedIncludes")
	file(COPY "${librecast_SOURCE_DIR}/Detour/Include" DESTINATION "${librecast_BINARY_DIR}/CombinedIncludes")
	file(RENAME "${librecast_BINARY_DIR}/CombinedIncludes/Include" "${librecast_BINARY_DIR}/CombinedIncludes/recast")
	#file(COPY "${librecast_BINARY_DIR}/CombinedIncludes/Include" DESTINATION "${librecast_BINARY_DIR}/CombinedIncludes/recast")
	set(RECAST_INCLUDE_PATH "${librecast_BINARY_DIR}/CombinedIncludes")

	#file(MAKE_DIRECTORY "${librecast_BINARY_DIR}/LibsCollected")
	#file(COPY	"${librecast_BINARY_DIR}/Recast/Include" 
	#			"${librecast_BINARY_DIR}/DetourCrowd/Include"
	#			"${librecast_BINARY_DIR}/Detour/Include"
	#			DESTINATION "${librecast_BINARY_DIR}/LibsCollected")
	#set(RECAST_LIB_PATH "${librecast_BINARY_DIR}/CombinedIncludes")
	#set(RECAST_LIB_PATH "%RECAST_INSTALL_DIR%/lib")
	#get_target_property(RECAST_LIB_PATH RecastNavigation::Recast INTERFACE_LINK_DIRECTORIES)# or INTERFACE_LINK_LIBRARIES?
	get_target_property(RECAST_LIB_PATH RecastNavigation::Recast LIBRARY_OUTPUT_DIRECTORY)

	#DEBUG OUTPUT
	message("ZLIB_INCLUDE_PATH ${ZLIB_INCLUDE_PATH}")
	message("RECAST_LIB_PATH ${RECAST_LIB_PATH}")
	message("CONAN_INCLUDE_DIRS_LIBPNG ${CONAN_INCLUDE_DIRS_LIBPNG}")

	if(CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE IN_LIST DEBUG_CONFIGURATIONS)
		message(VERBOSE "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} is a debug configuration. CARLA_DEBUG enabled. Will build carla_client_debug instead of carla_client")
		set(CARLA_DEBUG TRUE)
	else()
		message(VERBOSE "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} not found in DEBUG_CONFIGURATIONS (${DEBUG_CONFIGURATIONS}). Will build carla_client in RELEASE configuration")
	endif()
	
	# Carla build system requires a change of CMAKE_BUILD_TYPE to build LibCarla_client
	set(CMAKE_BUILD_TYPE "Client")
	set(LIBCARLA_BUILD_TEST FALSE)
	if(${CARLA_DEBUG})
		set(LIBCARLA_BUILD_RELEASE FALSE)
		set(LIBCARLA_BUILD_DEBUG TRUE)
	else()
		set(LIBCARLA_BUILD_RELEASE TRUE)
		set(LIBCARLA_BUILD_DEBUG FALSE)
	endif()

	set(CMAKE_CXX_STANDARD_REQUIRED ON)
	FetchContent_GetProperties(LibCarla_client)
	if(NOT libcarla_client_POPULATED)
		FetchContent_Populate(LibCarla_client)
		add_subdirectory("${libcarla_client_SOURCE_DIR}/LibCarla/cmake" "${libcarla_client_BINARY_DIR}/LibCarla" EXCLUDE_FROM_ALL)
		
		# Carla changes its library name when including the RSS component in a build
		if(TARGET carla_client_rss)
			add_library(carla_client ALIAS carla_client_rss)
		endif()
		if(TARGET carla_client_rss_debug)
			add_library(carla_client_debug ALIAS carla_client_rss)
		endif()

		#create target inlcuding dependencies to add as dependency of other targets
		add_library(LibCarla_and_deps INTERFACE)
		add_library(LibCarla::LibCarla_client ALIAS LibCarla_and_deps)
		# Carla only defines private cmake include, which don't propagate --> re-add carla headers. Also misuse SYSTEM switch to hide compiler warnings
		target_include_directories(LibCarla_and_deps SYSTEM INTERFACE "${libcarla_client_SOURCE_DIR}/LibCarla/source")
		target_link_libraries(LibCarla_and_deps INTERFACE $<IF:$<BOOL:${CARLA_DEBUG}>,carla_client_debug,carla_client> CONAN_PKG::boost rpc CONAN_PKG::zlib CONAN_PKG::libpng RecastNavigation::Recast)
		if (WIN32)
			# Required for https://docs.microsoft.com/en-us/windows/win32/api/shlwapi/nf-shlwapi-pathmatchspeca used in carla::StringUtil::Match
			target_link_libraries(LibCarla_and_deps INTERFACE Shlwapi)
		endif()
		#target_link_libraries(LibCarla_and_deps INTERFACE carla_client CONAN_PKG::boost rpc CONAN_PKG::zlib CONAN_PKG::libpng RecastNavigation::Recast)
		# includes of link libraries somehow don't propagate. Also misuse SYSTEM switch to hide compiler warnings
		target_include_directories(LibCarla_and_deps SYSTEM INTERFACE ${BOOST_INCLUDE_PATH} ${RPCLIB_INCLUDE_PATH} ${ZLIB_INCLUDE_PATH} ${LIBPNG_INCLUDE_PATH} ${RECAST_INCLUDE_PATH})
	endif()

	#get_target_property(_CARLA_INCLUDES LibCarla_and_deps INTERFACE_INCLUDE_DIRECTORIES)
	#get_target_property(_CARLA_LIBS LibCarla_and_deps INTERFACE_LINK_LIBRARIES)
	#message("LibCarla fetch end INTERFACE_INCLUDE_DIRECTORIES: ${_CARLA_INCLUDES}")
	#message("LibCarla fetch end INTERFACE_LINK_LIBRARIES: ${_CARLA_LIBS}")
endfunction()