#
# UARTs on platform zynq7000
#
# Copyright (C) 2020, Hensoldt Cyber GmbH
#

cmake_minimum_required(VERSION 3.17)


CAmkESAddCPPInclude("${CMAKE_CURRENT_LIST_DIR}")

DeclareCAmkESComponent_UART(UART_0 0)
DeclareCAmkESComponent_UART(UART_1 1)
