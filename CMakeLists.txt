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


#-------------------------------------------------------------------------------
# declare CAmkES components for all UARTs of a platform
function(DeclareCAmkESComponents_for_UARTs)

    include(
        "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/plat/${PLATFORM}/plat_uart.cmake"
        OPTIONAL
    )

endfunction()
