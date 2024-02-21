#
# UART defaults for platform rpi4 (bcm2711 based Raspberry Pi board)
# 
# Copyright (C) 2020-2024, HENSOLDT Cyber GmbH
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# For commercial licensing, contact: info.cyber@hensoldt.net
#

cmake_minimum_required(VERSION 3.17)

UART_DeclareCAmkESComponent(UART_0 0)
UART_DeclareCAmkESComponent(UART_1 1)
UART_DeclareCAmkESComponent(UART_2 2)
UART_DeclareCAmkESComponent(UART_3 3)
UART_DeclareCAmkESComponent(UART_4 4)
UART_DeclareCAmkESComponent(UART_5 5)
