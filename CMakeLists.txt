#
# UART driver
#
# Copyright (C) 2020-2021, HENSOLDT Cyber GmbH
#

cmake_minimum_required(VERSION 3.17)


#-------------------------------------------------------------------------------
#
# Declare UART driver CAmkES Component
#
# Parameters:
#
#   <name>
#       component name
#   <id>
#       ID used in platsupport libs

#
function(UART_DeclareCAmkESComponent name id)

    DeclareCAmkESComponent(
        ${name}
        SOURCES
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/Uart.c
        C_FLAGS
            -Wall
            -Werror
            -DUART_CONFIG_ID=${id}
        LIBS
            os_core_api
            lib_debug
            lib_macros
            lib_io
            platsupport
    )

endfunction()

include("SysCtrl/CMakeLists.txt")

#-------------------------------------------------------------------------------
# declare CAmkES components for all UARTs of a platform
function(DeclareCAmkESComponents_for_UARTs)

    set(dir_plat "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/plat/${PLATFORM}")

    # For now we assume that all platforms have UARTs, so there is always a
    # platform specific folder with CMake files. If it is missing, the build
    # will fail. In case there is ever a platform that has no UARTs, there
    # should be an empty platform file that states this fact in a comment.
    if(NOT IS_DIRECTORY "${dir_plat}")
        message(FATAL_ERROR "unsupported platform: '${PLATFORM}'")
    endif()

    CAmkESAddCPPInclude("${dir_plat}")
    include("${dir_plat}/plat_uart.cmake")

endfunction()
