set(TARGET_NAME "MotionAdaptationToolV2")

if(NOT(MMMCore_FOUND AND MMMTools_FOUND))
    message(STATUS "Required libraries not found for ${TARGET_NAME}")
    return()
endif()

set(LIBRARIES
    RelationshipDescriptor
    MMMCore
    SimoxUtility
    VirtualRobot
    stdc++fs
    MMMViewerLib
    ModelPoseSensor
    KinematicSensor
)

# These headers make up the public interface of the library
# They are installed via "make install" and should not include
# private headers which are only used for implementation.
set(PUBLIC_HEADERS
    MotionAdaptationToolV2Handler.h
    MotionAdaptationToolV2HandlerFactory.h
    MotionAdaptationToolV2HandlerDialog.h
)

set(HEADERS
    ${PUBLIC_HEADERS}
)

set(SOURCES
    MotionAdaptationToolV2Handler.cpp
    MotionAdaptationToolV2HandlerFactory.cpp
    MotionAdaptationToolV2HandlerDialog.cpp
)

set(MOC
    MotionAdaptationToolV2HandlerDialog.h
)

set(UI
    MotionAdaptationToolV2HandlerDialog.ui
)

VirtualRobotQtLibrary(${TARGET_NAME} "${SOURCES}" "${HEADERS}" "${MOC}" "${UI}")
target_link_libraries(${TARGET_NAME} PUBLIC ${LIBRARIES})
set_target_properties(${TARGET_NAME} PROPERTIES PUBLIC_HEADER "${PUBLIC_HEADERS}")
set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${MOTION_HANDLER_PLUGIN_LIB_DIR})

target_compile_definitions(${TARGET_NAME} PUBLIC -D_REENTRANT)
target_include_directories(${TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>)
target_include_directories(${TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>)

install(TARGETS ${TARGET_NAME}
    EXPORT ${TARGET_NAME}Targets
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
    LIBRARY DESTINATION ${MOTION_HANDLER_PLUGIN_LIB_DIR} COMPONENT shlib
    PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/${TARGET_NAME}"
    COMPONENT dev
)


