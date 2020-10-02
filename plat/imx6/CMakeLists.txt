#
# UARTs on platform imx6
#
# Copyright (C) 2020, Hensoldt Cyber GmbH
#

cmake_minimum_required(VERSION 3.17)


function(plat_DeclareCAmkESComponents_for_UARTs)

    DeclareCAmkESComponent_UART(UART_0 0)
    DeclareCAmkESComponent_UART(UART_1 1)
    DeclareCAmkESComponent_UART(UART_2 2)
    DeclareCAmkESComponent_UART(UART_3 3)
    DeclareCAmkESComponent_UART(UART_4 4)

endfunction()
