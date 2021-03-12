#source
#subdirectories have to be added before calling set as otherwise CARLA_OSI_LIB_SOURCE will still be empty when writing to the parent scope (parent directory)
add_subdirectory(carla_osi)
#sources
set(CARLA_OSI_LIB_SOURCE ${CARLA_OSI_LIB_SOURCE}
"${CMAKE_CURRENT_LIST_DIR}/CARLA2OSIInterface.cpp" 
"${CMAKE_CURRENT_LIST_DIR}/Utility.cpp" 

PARENT_SCOPE
)