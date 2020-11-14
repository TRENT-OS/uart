#
# UART defaults for platfrom sabre (i.MX6 based Sabre Lite board)
#
# Copyright (C) 2020, Hensoldt Cyber GmbH
#

cmake_minimum_required(VERSION 3.17)


CAmkESAddCPPInclude("${CMAKE_CURRENT_LIST_DIR}")

UART_DeclareCAmkESComponent(UART_0 0)
UART_DeclareCAmkESComponent(UART_1 1)
UART_DeclareCAmkESComponent(UART_2 2)
UART_DeclareCAmkESComponent(UART_3 3)
UART_DeclareCAmkESComponent(UART_4 4)
