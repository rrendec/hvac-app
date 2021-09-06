cmake_minimum_required(VERSION 3.10)

project(hvac)

option(WITH_EMULATOR "Enable emulator support" OFF)

add_compile_options(-Wall -Werror)

set(hvac_SOURCES src/main.c)
set(hvac_LIBS)
if(WITH_EMULATOR)
	list(APPEND hvac_SOURCES src/emulator.c)
else()
	list(APPEND hvac_LIBS gpiod modbus)
endif()

add_executable(hvac ${hvac_SOURCES})
target_link_libraries(hvac ${hvac_LIBS})
