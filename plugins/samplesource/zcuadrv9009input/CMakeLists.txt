project(zcuadrv9009input)

set(zcuadrv9009input_SOURCES
    zcuadrv9009input.cpp zcuadrv9009inputplugin.cpp
    zcuadrv9009inputsettings.cpp zcuadrv9009inputwebapiadapter.cpp
    zcuadrv9009inputthread.cpp)

set(zcuadrv9009input_HEADERS
    zcuadrv9009input.h zcuadrv9009inputplugin.h zcuadrv9009inputsettings.h
    zcuadrv9009inputwebapiadapter.h zcuadrv9009inputthread.h)

include_directories(${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
                    ${CMAKE_SOURCE_DIR}/devices ${LIBIIO_INCLUDE_DIR})

if(NOT SERVER_MODE)
  set(zcuadrv9009input_SOURCES ${zcuadrv9009input_SOURCES}
                               zcuadrv9009inputgui.cpp zcuadrv9009inputgui.ui)
  set(zcuadrv9009input_HEADERS ${zcuadrv9009input_HEADERS}
                               zcuadrv9009inputgui.h)

  set(TARGET_NAME inputzcuadrv9009)
  set(TARGET_LIB "Qt::Widgets")
  set(TARGET_LIB_GUI "sdrgui")
  set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
else()
  set(TARGET_NAME inputzcuadrv9009srv)
  set(TARGET_LIB "")
  set(TARGET_LIB_GUI "")
  set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

add_library(${TARGET_NAME} SHARED ${zcuadrv9009input_SOURCES})

if(LIBIIO_EXTERNAL)
  add_dependencies(${TARGET_NAME} libiio)
endif()

target_link_libraries(
  ${TARGET_NAME}
  Qt::Core
  ${TARGET_LIB}
  sdrbase
  ${TARGET_LIB_GUI}
  swagger
  ${LIBIIO_LIBRARIES}
  zcuadrv9009device)

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

# Install debug symbols
if(WIN32)
  install(
    FILES $<TARGET_PDB_FILE:${TARGET_NAME}>
    CONFIGURATIONS Debug RelWithDebInfo
    DESTINATION ${INSTALL_FOLDER})
endif()
