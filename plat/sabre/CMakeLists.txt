#
# UART defaults for platfrom sabre (i.MX6 based Sabre Lite board)
#
# Copyright (C) 2020, Hensoldt Cyber GmbH
#

cmake_minimum_required(VERSION 3.17)


CAmkESAddCPPInclude("${CMAKE_CURRENT_LIST_DIR}")

DeclareCAmkESComponent_UART(UART_0 0)
DeclareCAmkESComponent_UART(UART_1 1)
DeclareCAmkESComponent_UART(UART_2 2)
DeclareCAmkESComponent_UART(UART_3 3)
DeclareCAmkESComponent_UART(UART_4 4)
