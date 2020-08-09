#
# UART
#
# Copyright (C) 2020, Hensoldt Cyber GmbH
#

cmake_minimum_required(VERSION 3.7.2)


# we cannot use CMAKE_CURRENT_LIST_DIR in a function or macro, because that
# would give us the dir of the file that is invoking the macro. The work around
# is that we create a global (ie cached and flagged as internal) variable here
# and then use it in the macro below. As a convenient side effect we don't
# require anybody else to provide any helper variables, this CMake file becomes
# fully self-contained and "copy&paste safe".
set(
    OS_COMPONENT_UART_DIR
    "${CMAKE_CURRENT_LIST_DIR}"
    CACHE INTERNAL ""
)


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
            ${OS_COMPONENT_UART_DIR}/Uart.c
        C_FLAGS
            -Wall -Werror
        LIBS
            os_core_api
            os_libs
            platsupport
    )

endfunction()
