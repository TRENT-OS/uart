#
# UART defaults for the platform jetson-tx2-nx-a206 (SeeedStudio A206 Carrier Board + Nvidia Jetson TX2 NX SoM)
#
# 
# Copyright (C) 2022-2024, HENSOLDT Cyber GmbH
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# For commercial licensing, contact: info.cyber@hensoldt.net
#

cmake_minimum_required(VERSION 3.17)

UART_DeclareCAmkESComponent(UART_0 0)
UART_DeclareCAmkESComponent(UART_1 1)
UART_DeclareCAmkESComponent(UART_2 6)