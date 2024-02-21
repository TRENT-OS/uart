#
# UART defaults for the platform aetina-an110-xnx (Aetina AN110 Carrier Board + Nvidia Jetson Xavier NX SoM)
#
# Copyright (C) 2022-2024, HENSOLDT Cyber GmbH
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# For commercial licensing, contact: info.cyber@hensoldt.net
#
cmake_minimum_required(VERSION 3.17)

UART_DeclareCAmkESComponent(UART_0 4)
UART_DeclareCAmkESComponent(UART_1 5)
UART_DeclareCAmkESComponent(UART_2 6)