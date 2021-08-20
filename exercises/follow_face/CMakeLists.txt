
configure_file(
    pantilt_teleop.in
    pantilt_teleop
    @ONLY
)

## INSTALL ##

# install Launcher
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/pantilt_teleop
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
    DESTINATION bin
    COMPONENT panTiltTeleop-python
)

# Install .py
FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*py)
FOREACH(header ${HEADERS_FILES})
	INSTALL(FILES ${header} DESTINATION share/jderobot/python/pantilt_teleop_py/ COMPONENT panTiltTeleop-python)
ENDFOREACH(header)

# Install gui
INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gui DESTINATION share/jderobot/python/pantilt_teleop_py COMPONENT panTiltTeleop-python PATTERN .svn EXCLUDE)

# Install resources
#INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resources DESTINATION share/jderobot/python/pantilt_teleop_py COMPONENT panTiltTeleop-python PATTERN .svn EXCLUDE)
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/pantilt_teleop.yml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf COMPONENT panTiltTeleop-python)
