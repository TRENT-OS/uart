#
# SysCtrl
#
# Copyright (C) 2022, HENSOLDT Cyber GmbH
#

cmake_minimum_required(VERSION 3.7.2)

if(SDK_USE_CAMKES)

    DeclareCAmkESComponent(
        SysCtrl
        SOURCES
            SysCtrl.c
        C_FLAGS
            -Wall
            -Werror
        LIBS
            os_core_api
            lib_debug
            lib_io
    )

endif()  # SDK_USE_CAMKES