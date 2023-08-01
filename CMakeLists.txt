cmake_minimum_required(VERSION 3.10)

project(hvac)

option(WITH_EMULATOR "Enable emulator support" OFF)

set(board_type "rpi-hvac" CACHE STRING "Board type to build for")
set_property(CACHE board_type PROPERTY STRINGS rpi-bare rpi-hvac)

add_compile_options(-Wall -Werror -include board-${board_type}.h)

set(hvac_SOURCES
	src/main.c
	src/gsdata.c
	src/json.c
	src/network.c
	src/telemetry.c
	src/memcache.c
)
set(hvac_LIBS civetweb cjson pthread m)

if(WITH_EMULATOR)
	list(APPEND hvac_SOURCES src/emulator.c)
else()
	list(APPEND hvac_LIBS gpiod modbus)
endif()

add_executable(hvac ${hvac_SOURCES})
target_include_directories(hvac PRIVATE include)
target_link_libraries(hvac ${hvac_LIBS})

install(TARGETS hvac DESTINATION bin)
install(FILES systemd/hvac.service DESTINATION lib/systemd/system)
