#
# UARTs defaults for platform zynq7000
#
# Copyright (C) 2020-2021, HENSOLDT Cyber GmbH
#

cmake_minimum_required(VERSION 3.17)


CAmkESAddCPPInclude("${CMAKE_CURRENT_LIST_DIR}")

UART_DeclareCAmkESComponent(UART_0 0)
UART_DeclareCAmkESComponent(UART_1 1)