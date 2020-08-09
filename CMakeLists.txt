#
# UART
#
# Copyright (C) 2020, Hensoldt Cyber GmbH
#

cmake_minimum_required(VERSION 3.17)


#-------------------------------------------------------------------------------
#
# Declare UART driver CAmkES Component
#
# Parameters:
#
#   <name>
#       component instance name
#
function(DeclareCAmkESComponent_UART
    name
)

    DeclareCAmkESComponent(
        ${name}
        SOURCES
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/Uart.c
        C_FLAGS
            -Wall -Werror
        LIBS
            os_core_api
            os_libs
            platsupport
    )

endfunction()
