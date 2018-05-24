cmake_minimum_required(VERSION 2.8)

# set the cmake variables to be used in the source code
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gui/cmakevars.py.in ${CMAKE_CURRENT_SOURCE_DIR}/gui/cmakevars.py)



configure_file(
    visualStates_py.in
    visualStates_py
    @ONLY
)


SET(SOURCE_CPP_RUNTIME
	codegen/cpp/state.cpp
	codegen/cpp/transition.cpp
	codegen/cpp/temporaltransition.cpp
    codegen/cpp/runtimegui.cpp
)

INCLUDE_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/include)
LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib)

ADD_LIBRARY(visualStatesRunTime SHARED ${SOURCE_CPP_RUNTIME})
TARGET_LINK_LIBRARIES(visualStatesRunTime easyiceconfig Ice JderobotInterfaces IceUtil pthread)

INSTALL(TARGETS visualStatesRunTime DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/visualstates_py/ COMPONENT visualStates-python)

#install runtime headers
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/codegen/cpp/
		DESTINATION ${CMAKE_INSTALL_PREFIX}/include/visualstates_py/ COMPONENT visualStates-python
		FILES_MATCHING PATTERN "*.h")

# install cpp sources for ros node generation
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/codegen/cpp/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/visualstates_py/ COMPONENT visualState-python)

#install python runtime dependencies
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/codegen/python/
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/visualStates_py/codegen/python/
		COMPONENT visualStates-python
		PATTERN __pycache__ EXCLUDE )

# install python runtime gui dependencies
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gui
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/visualStates_py/ COMPONENT visualStates-python
		FILES_MATCHING PATTERN "*.py"
		)

# install python module __init__.py
INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/visualStates_py/ COMPONENT visualStates-python)
INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/visualStates_py/codegen/ COMPONENT visualStates-python)
INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/visualStates_py/codegen/python/ COMPONENT visualStates-python)

INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/visualStates.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/visualStates_py/ COMPONENT visualStates-python)

# install getinterfaces.sh bash script to be able to get the interfaces
INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/getinterfaces.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/ PERMISSIONS
OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT visualStates-python)


# Install launcher
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/visualStates_py
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
    DESTINATION bin
    COMPONENT visualStates-python
)
