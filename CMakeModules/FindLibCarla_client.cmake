# Set path to CARLA source. If not provided, try environment variable CARLA_SOURCE_DIR.
if(NOT CARLA_SOURCE_DIR)
	set(CARLA_SOURCE_DIR "$ENV{CARLA_SOURCE_DIR}")
endif()
if(IS_DIRECTORY "${CARLA_SOURCE_DIR}")
	set(WITH_CARLA TRUE)
	if(NOT LibCarla_client_FIND_QUIETLY)
		message(VERBOSE "Using carla in ${CARLA_SOURCE_DIR}")
	endif()
	
	find_library(LibCarla_client_CARLA_CLIENT_LIBRARY NAMES carla_client libcarla_client PATHS "${CARLA_SOURCE_DIR}" "${CARLA_SOURCE_DIR}/PythonAPI/carla/dependencies/lib" REQUIRED NO_DEFAULT_PATH)
	add_library(LIBCARLA_CLIENT STATIC IMPORTED)
	set_target_properties(LIBCARLA_CLIENT PROPERTIES IMPORTED_LOCATION ${LibCarla_client_CARLA_CLIENT_LIBRARY})
	
	find_library(LibCarla_client_CARLA_CLIENT_DEBUG_LIBRARY NAMES carla_client_debug libcarla_client PATHS "${CARLA_SOURCE_DIR}" "${CARLA_SOURCE_DIR}/PythonAPI/carla/dependencies/lib" NO_DEFAULT_PATH)
	if(CARLA_CLIENT_DEBUG_LIBRARY)
		add_library(LIBCARLA_CLIENT_DEBUG STATIC IMPORTED)
		set_target_properties(LIBCARLA_CLIENT_DEBUG PROPERTIES IMPORTED_LOCATION ${LibCarla_client_CARLA_CLIENT_DEBUG_LIBRARY})
	endif()
	
	find_library(LibCarla_client_RPC_LIBRARY NAMES rpc PATHS "${CARLA_SOURCE_DIR}" "${CARLA_SOURCE_DIR}/PythonAPI/carla/dependencies/lib" REQUIRED NO_DEFAULT_PATH)
	add_library(LIBRPC STATIC IMPORTED)
	set_target_properties(LIBRPC PROPERTIES IMPORTED_LOCATION ${LibCarla_client_RPC_LIBRARY})
	
	find_library(LibCarla_client_RECAST_LIBRARY NAMES Recast PATHS "${CARLA_SOURCE_DIR}" "${CARLA_SOURCE_DIR}/PythonAPI/carla/dependencies/lib" REQUIRED NO_DEFAULT_PATH)
	add_library(LIBRECAST STATIC IMPORTED)
	set_target_properties(LIBRECAST PROPERTIES IMPORTED_LOCATION ${LibCarla_client_RPC_LIBRARY})
	
	find_library(LibCarla_client_DETOUR_LIBRARY NAMES detour PATHS "${CARLA_SOURCE_DIR}" "${CARLA_SOURCE_DIR}/PythonAPI/carla/dependencies/lib" REQUIRED NO_DEFAULT_PATH)
	add_library(LIBDETOUR STATIC IMPORTED)
	set_target_properties(LIBDETOUR PROPERTIES IMPORTED_LOCATION ${LibCarla_client_DETOUR_LIBRARY})
	
	find_library(LibCarla_client_DETOUR_CROWD_LIBRARY NAMES detourCrowd PATHS "${CARLA_SOURCE_DIR}" "${CARLA_SOURCE_DIR}/PythonAPI/carla/dependencies/lib" REQUIRED NO_DEFAULT_PATH)
	add_library(LIBDETOUR_CROWD STATIC IMPORTED)
	set_target_properties(LIBDETOUR_CROWD PROPERTIES IMPORTED_LOCATION ${LibCarla_client_DETOUR_CROWD_LIBRARY})

	find_library(LibCarla_client_DETOUR_TILE_CACHE_LIBRARY NAMES detourTileCache PATHS "${CARLA_SOURCE_DIR}" "${CARLA_SOURCE_DIR}/PythonAPI/carla/dependencies/lib" REQUIRED NO_DEFAULT_PATH)
	add_library(LIBDETOUR_TILE_CACHE STATIC IMPORTED)
	set_target_properties(LIBDETOUR_TILE_CACHE PROPERTIES IMPORTED_LOCATION ${LibCarla_client_DETOUR_TILE_CACHE_LIBRARY})

	# only needed if using IO funtionality; Library distributed by CARLA 0.9.9 on Win10 x64 is named libboost_filesystem-vc141-mt-x64-1_72
	find_library(LibCarla_client_BOOST_FILESYSTEM_LIBRARY NAMES boost_filesystem libboost_filesystem libboost_filesystem* libboost_filesystem-vc141-mt-x64-1_72 PATHS "${CARLA_SOURCE_DIR}" "${CARLA_SOURCE_DIR}/PythonAPI/carla/dependencies/lib" NO_DEFAULT_PATH)
	if(LibCarla_client_BOOST_FILESYSTEM_LIBRARY)
		set(LibCarla_client_BOOST_FILESYSTEM_FOUND TRUE)
		add_library(LIBBOOST_FILESYSTEM STATIC IMPORTED)
		set_target_properties(LIBBOOST_FILESYSTEM PROPERTIES IMPORTED_LOCATION ${LibCarla_client_BOOST_FILESYSTEM_LIBRARY})
	endif()
	
	# only needed if using image processing with png
	find_library(LibCarla_client_PNG_LIBRARY NAMES png libpng PATHS "${CARLA_SOURCE_DIR}" "${CARLA_SOURCE_DIR}/PythonAPI/carla/dependencies/lib" NO_DEFAULT_PATH)
	if(LibCarla_client_PNG_LIBRARY)
		set(LibCarla_client_PNG_FOUND TRUE)
		add_library(LIBPNG STATIC IMPORTED)
		set_target_properties(LIBPNG PROPERTIES IMPORTED_LOCATION ${LibCarla_client_PNG_LIBRARY})
	endif()
	
	# only needed if using image processing with tiff
	find_library(LibCarla_client_TIFF_LIBRARY NAMES tiff PATHS "${CARLA_SOURCE_DIR}" "${CARLA_SOURCE_DIR}/PythonAPI/carla/dependencies/lib" NO_DEFAULT_PATH)
	if(LibCarla_client_TIFF_LIBRARY)
		set(LibCarla_client_TIFF_FOUND TRUE)
		add_library(LIBTIFF STATIC IMPORTED)
		set_target_properties(LIBTIFF PROPERTIES IMPORTED_LOCATION ${LibCarla_client_TIFF_LIBRARY})
	endif()

	# only needed if using image processing with jpeg
	find_library(LibCarla_client_JPEG_LIBRARY NAMES jpeg PATHS "${CARLA_SOURCE_DIR}" "${CARLA_SOURCE_DIR}/PythonAPI/carla/dependencies/lib" NO_DEFAULT_PATH)
	if(LibCarla_client_JPEG_LIBRARY)
		set(LibCarla_client_JPEG_FOUND TRUE)
		add_library(LIBJPEG STATIC IMPORTED)
		set_target_properties(LIBJPEG PROPERTIES IMPORTED_LOCATION ${LibCarla_client_JPEG_LIBRARY})
	endif()

	include(FindPackageHandleStandardArgs)
	if(${CMAKE_VERSION} VERSION_LESS "3.18.0")
		# less options available
		find_package_handle_standard_args(LibCarla_client
			FOUND_VAR LibCarla_client_FOUND
			REQUIRED_VARS 
				LibCarla_client_CARLA_CLIENT_LIBRARY 
				LibCarla_client_RPC_LIBRARY 
				LibCarla_client_RECAST_LIBRARY 
				LibCarla_client_DETOUR_LIBRARY 
				LibCarla_client_DETOUR_CROWD_LIBRARY 
				LibCarla_client_DETOUR_TILE_CACHE_LIBRARY
			HANDLE_COMPONENTS
		)
	else()
		find_package_handle_standard_args(LibCarla_client
			FOUND_VAR LibCarla_client_FOUND
			HANDLE_COMPONENTS
			NAME_MISMATCHED
			REASON_FAILURE_MESSAGE "Is CARLA_SOURCE_DIR pointing to the local carla repository and is the PythonAPI already build?"
		)
	endif()

	##DEBUG
	message(DEBUG "LibCarla_client find_library paths: ${LibCarla_client_CARLA_CLIENT_LIBRARY} ${LibCarla_client_CARLA_CLIENT_DEBUG_LIBRARY} ${LibCarla_client_RPC_LIBRARY} ${LibCarla_client_RECAST_LIBRARY} ${LibCarla_client_DETOUR_LIBRARY} ${LibCarla_client_DETOUR_CROWD_LIBRARY} ${LibCarla_client_DETOUR_TILE_CACHE_LIBRARY} ${LibCarla_client_BOOST_FILESYSTEM_LIBRARY} ${LibCarla_client_PNG_LIBRARY} ${LibCarla_client_TIFF_LIBRARY} ${LibCarla_client_JPEG_LIBRARY}")
	# find_library option REQUIRED is introduced in CMake version 3.18. This is a fallback for older versions
	if(${CMAKE_VERSION} VERSION_LESS "3.18.0") 
	    message("Please consider to switch to CMake 3.18.0 or newer")
		if(NOT (LibCarla_client_CARLA_CLIENT_LIBRARY AND LibCarla_client_RPC_LIBRARY AND LibCarla_client_RECAST_LIBRARY AND LibCarla_client_DETOUR_LIBRARY AND LibCarla_client_DETOUR_CROWD_LIBRARY AND LibCarla_client_DETOUR_TILE_CACHE_LIBRARY AND LibCarla_client_BOOST_FILESYSTEM_LIBRARY))# AND LibCarla_client_PNG_LIBRARY AND LibCarla_client_TIFF_LIBRARY AND LibCarla_client_JPEG_LIBRARY))
			message(FATAL_ERROR "Could not find required libraries in CARLA_SOURCE_DIR/PythonAPI/carla/dependencies/lib: ${CARLA_SOURCE_DIR}/PythonAPI/carla/dependencies/lib \n Has CARLA been build?")
		endif()
	endif()

	if(NOT TARGET LibCarla::LibCarla_client)
		message(DEBUG "Creating INTERFACE library target LibCarla::LibCarla_client")
		add_library(LibCarla::LibCarla_client INTERFACE IMPORTED)

		# use debug carla client for debug builds, if available. Otherwise always use non-debug library.
		if(${LibCarla_client_CARLA_CLIENT_DEBUG_LIBRARY})
			message(DEBUG "LibCarla_client_CARLA_CLIENT_DEBUG_LIBRARY: ${LibCarla_client_CARLA_CLIENT_DEBUG_LIBRARY}")
			target_link_libraries(LibCarla::LibCarla_client INTERFACE
				optimized ${LibCarla_client_CARLA_CLIENT_LIBRARY}
				debug ${LibCarla_client_CARLA_CLIENT_DEBUG_LIBRARY}
			)
			set(LibCarla_client_LIBRARIES $<IF:$<CONFIG:debug>,${CARLA_CLIENT_DEBUG_LIBRARY},${CARLA_CLIENT_LIBRARY}> )
		else()
			target_link_libraries(LibCarla::LibCarla_client INTERFACE 
				${LibCarla_client_CARLA_CLIENT_LIBRARY}
			)
			set(LibCarla_client_LIBRARIES ${CARLA_CLIENT_LIBRARY})
		endif()

		# required dependencies of libcarla_client
		target_link_libraries(LibCarla::LibCarla_client INTERFACE 
			${LibCarla_client_RPC_LIBRARY}
			${LibCarla_client_RECAST_LIBRARY}
			${LibCarla_client_DETOUR_LIBRARY}
			${LibCarla_client_DETOUR_CROWD_LIBRARY}
			${LibCarla_client_DETOUR_TILE_CACHE_LIBRARY}
			)

		set(LibCarla_client_LIBRARIES ${LibCarla_client_LIBRARIES} ${LibCarla_client_RPC_LIBRARY} ${LibCarla_client_RECAST_LIBRARY} ${LibCarla_client_DETOUR_LIBRARY} ${LibCarla_client_DETOUR_CROWD_LIBRARY} ${LibCarla_client_DETOUR_TILE_CACHE_LIBRARY})
		
		list(FIND LibCarla_client_FIND_COMPONENTS BOOST_FILESYSTEM _FIND_BOOST_FILESYSTEM_LIBRARY )
		if(LibCarla_client_BOOST_FILESYSTEM_FOUND AND _FIND_BOOST_FILESYSTEM_LIBRARY GREATER -1)
			target_link_libraries(LibCarla::LibCarla_client INTERFACE ${LibCarla_client_BOOST_FILESYSTEM_LIBRARY})
			set(LibCarla_client_LIBRARIES ${LibCarla_client_LIBRARIES} ${LibCarla_client_BOOST_FILESYSTEM_LIBRARY})
		endif()

		list(FIND LibCarla_client_FIND_COMPONENTS PNG _FIND_PNG_LIBRARY )
		if(LibCarla_client_PNG_FOUND AND _FIND_PNG_LIBRARY GREATER -1)
			target_link_libraries(LibCarla::LibCarla_client INTERFACE ${LibCarla_client_PNG_LIBRARY})
			set(LibCarla_client_LIBRARIES ${LibCarla_client_LIBRARIES} ${LibCarla_client_PNG_LIBRARY})
		endif()

		list(FIND LibCarla_client_FIND_COMPONENTS TIFF _FIND_TIFF_LIBRARY )
		if(LibCarla_client_TIFF_FOUND AND _FIND_TIFF_LIBRARY GREATER -1)
			target_link_libraries(LibCarla::LibCarla_client INTERFACE ${LibCarla_client_TIFF_LIBRARY})
			set(LibCarla_client_LIBRARIES ${LibCarla_client_LIBRARIES} ${LibCarla_client_TIFF_LIBRARY})
		endif()

		list(FIND LibCarla_client_FIND_COMPONENTS JPEG _FIND_JPEG_LIBRARY )
		if(LibCarla_client_JPEG_FOUND AND _FIND_JPEG_LIBRARY GREATER -1)
			target_link_libraries(LibCarla::LibCarla_client INTERFACE ${LibCarla_client_JPEG_LIBRARY})
			set(LibCarla_client_LIBRARIES ${LibCarla_client_LIBRARIES} ${LibCarla_client_JPEG_LIBRARY})
		endif()


		set(LibCarla_client_INCLUDE_DIR ${CARLA_SOURCE_DIR}/PythonAPI/carla/dependencies/include)
		#target_include_directories(LibCarla::LibCarla_client INTERFACE ${CARLA_SOURCE_DIR}/PythonAPI/carla/dependencies/include)
		target_include_directories(LibCarla::LibCarla_client INTERFACE ${LibCarla_client_INCLUDE_DIR}/system ${LibCarla_client_INCLUDE_DIR} )

		#Debug output
		#get_target_property(OUT LibCarla::LibCarla_client INTERFACE_INCLUDE_DIRECTORIES)
		#message("LibCarla INTERFACE_INCLUDE_DIRECTORIES: ${OUT}")
	endif()
else()
	message(FATAL_ERROR "CARLA_SOURCE_DIR does not point to a directory")
endif()

