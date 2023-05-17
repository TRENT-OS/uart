#
# UARTs defaults for platform Odroid-C2
#
# Copyright (C) 2022, HENSOLDT Cyber GmbH
#

cmake_minimum_required(VERSION 3.17)

UART_DeclareCAmkESComponent(UART_A 0)
UART_DeclareCAmkESComponent(UART_B 1)
UART_DeclareCAmkESComponent(UART_C 2)
UART_DeclareCAmkESComponent(UART_AO_A 3)
UART_DeclareCAmkESComponent(UART_AO_B 4)
