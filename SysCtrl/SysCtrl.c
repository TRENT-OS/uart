/*
 *  SysCtrl
 *
 *  Copyright (C) 2022, HENSOLDT Cyber GmbH
 */

#include "OS_Error.h"
#include "OS_Dataport.h"
#include "lib_debug/Debug.h"

#include <camkes.h>

#include <string.h>


void sysctrl_rpc_setup(void)
{
    Debug_LOG_DEBUG("setup");
}

//---------------------------------------------------------------------------
void pre_init(void)
{
    Debug_LOG_DEBUG("pre_init");
}


//---------------------------------------------------------------------------
void post_init(void)
{
    Debug_LOG_DEBUG("post_init");
}

int run()
{
    Debug_LOG_DEBUG("run");

    *((volatile uint32_t*)((uintptr_t)dev_car_0 + 0x17c)) = 0; // Set clk source for UART_B
    *((volatile uint32_t*)((uintptr_t)dev_car_0 + 0x320)) = BIT(7); // Enable clock for UART_B
    *((volatile uint32_t*)((uintptr_t)dev_car_0 + 0x304)) = BIT(7); // Clear reset for UART_B

    return 0;
}