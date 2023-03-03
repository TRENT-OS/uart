#
# UART defaults for the platform jetson-xavier-nx-dev-kit (Nvidia Jetson Xavier NX Developer Kit)
#
# Copyright (C) 2022, HENSOLDT Cyber GmbH
#

cmake_minimum_required(VERSION 3.17)

UART_DeclareCAmkESComponent(UART_0 4)
UART_DeclareCAmkESComponent(UART_1 5)
UART_DeclareCAmkESComponent(UART_2 6)