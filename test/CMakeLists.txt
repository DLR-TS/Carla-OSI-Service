set (TESTFILES 
${CMAKE_CURRENT_LIST_DIR}/tests-main.cpp
${CMAKE_CURRENT_LIST_DIR}/tests-yaml_reader.cpp
${CMAKE_CURRENT_LIST_DIR}/test-isimulationdata.cpp
${CMAKE_CURRENT_LIST_DIR}/tests-mapper.cpp
${CMAKE_CURRENT_LIST_DIR}/tests-FMIInterface.cpp
${CMAKE_CURRENT_LIST_DIR}/tests-simulationinterfacefactory.cpp
${CMAKE_CURRENT_LIST_DIR}/tests-OSMPInterface.cpp
${CMAKE_CURRENT_LIST_DIR}/tests-OSIBridge.cpp

$<$<BOOL:${WITH_CARLA}>:${CMAKE_CURRENT_LIST_DIR}/tests-CARLABaseSystemInterface.cpp>

${CMAKE_CURRENT_LIST_DIR}/MockBaseSimulator.cpp
${CMAKE_CURRENT_LIST_DIR}/MockInterfaceSimulator.cpp
${CMAKE_CURRENT_LIST_DIR}/MockMapper.cpp

PARENT_SCOPE)