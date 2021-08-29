cmake_minimum_required(VERSION 3.10)

project(hvac)

add_compile_options(-Wall -Werror)

add_executable(hvac src/main.c)

target_link_libraries(hvac modbus)
