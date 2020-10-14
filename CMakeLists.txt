#
# UART driver
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
#       component name
#   <id>
#       ID used in platsupport libs

#
function(DeclareCAmkESComponent_UART name id)

    CAmkESAddCPPInclude(
        "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/plat/${KernelPlatform}")

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
            os_libs
            platsupport
    )

endfunction()


#-------------------------------------------------------------------------------
# declare CAmkES components for all UARTs of a platform
function(DeclareCAmkESComponents_for_UARTs)

    include("${CMAKE_CURRENT_FUNCTION_LIST_DIR}/plat/${KernelPlatform}/CMakeLists.txt")

    plat_DeclareCAmkESComponents_for_UARTs()

endfunction()
