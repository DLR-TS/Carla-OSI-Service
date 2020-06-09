find_path(Catch2_INCLUDE_DIR NAMES "catch2/catch.hpp" "catch2/catch_reporter_automake.hpp" "catch2/catch_reporter_sonarqube.hpp" "catch2/catch_reporter_tap.hpp" "catch2/catch_reporter_teamcity.hpp" PATHS ${CONAN_INCLUDE_DIRS_Catch2})
# Catch2 is a header only solution
#find_library(Catch2_LIBRARY NAMES ${CONAN_LIBS_Catch2} PATHS ${CONAN_LIB_DIRS_Catch2})

if(Catch2_INCLUDE_DIR STREQUAL "Catch2_INCLUDE_DIR-NOTFOUND")
  message(FATAL_ERROR "Could not find Catch2 Library.")
  set(Catch2_FOUND FALSE)
endif()

set(Catch2_FOUND TRUE)
set(Catch2_INCLUDE_DIRS ${Catch2_INCLUDE_DIR})
# .. still header only
#set(Catch2_LIBRARIES ${Catch2_LIBRARY})
#mark_as_advanced(Catch2_LIBRARY Catch2_INCLUDE_DIR)
mark_as_advanced(Catch2_INCLUDE_DIR)

if(Catch2_FOUND)
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(Catch2
        REQUIRED_VARS Catch2_INCLUDE_DIR
        FOUND_VAR Catch2_FOUND
        FAIL_MESSAGE "Failed to find Catch2")

    if(NOT TARGET Catch2::Catch2)
		# define an imported library as target to hide that Catch2 is an header only implementation
        add_library(Catch2::Catch2 INTERFACE IMPORTED)
        set_target_properties(Catch2::Catch2 PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${Catch2_INCLUDE_DIR})
    endif(NOT TARGET Catch2::Catch2)
	# uncomment and add GLOBAL at the end of the above add_library statement
	## conan defines the target name as above, but just in case add a unqualified name
	#if(NOT TARGET Catch2)
    #    add_library(Catch2 ALIAS Catch2::Catch2)
    #endif(NOT TARGET Catch2)

    mark_as_advanced(Catch2_INCLUDE_DIR)

endif(Catch2_FOUND)