project(zcuadrv9009device)

set(zcuadrv9009device_SOURCES
    devicezcuadrv9009.cpp devicezcuadrv9009box.cpp devicezcuadrv9009scan.cpp
    devicezcuadrv9009params.cpp devicezcuadrv9009shared.cpp)

set(zcuadrv9009device_HEADERS
    devicezcuadrv9009.h deviceplutsdrobox.h devicezcuadrv9009scan.h
    devicezcuadrv9009params.h devicezcuadrv9009shared.h)

include_directories(${LIBIIO_INCLUDE_DIR} ${ICONV_INCLUDE_DIR}
                    ${Boost_INCLUDE_DIRS})

add_library(zcuadrv9009device SHARED ${zcuadrv9009device_SOURCES})

set_target_properties(zcuadrv9009device PROPERTIES DEFINE_SYMBOL
                                                   "devices_EXPORTS")

target_link_libraries(zcuadrv9009device sdrbase ${LIBIIO_LIBRARIES}
                      ${ICONV_LIBRARY})

if(LIBIIO_EXTERNAL)
  add_dependencies(zcuadrv9009device libiio)
endif()

install(TARGETS zcuadrv9009device DESTINATION ${INSTALL_LIB_DIR})
