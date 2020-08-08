/*
 *  UART
 *
 *  Copyright (C) 2020, Hensoldt Cyber GmbH
 */

#include "LibDebug/Debug.h"
#include "OS_Dataport.h"

#include <platsupport/chardev.h>
#include <platsupport/serial.h>
#include <platsupport/plat/serial.h>

#include <camkes.h>
#include <camkes/io.h>

#include <stdbool.h>

#include "LibIO/FifoDataport.h"
#include "LibUtil/CharFifo.h"
#include "OS_Dataport.h"

#if defined(UART_CONFIG_H_FILE)
#   define Uart_XSTR(d)    Uart_STR(d)
#   define Uart_STR(d)     #d
#   include Debug_XSTR(UART_CONFIG_H_FILE)
#endif


// Number of bytes read at most from the UART FIFO at once
#if !defined(Uart_Config_READ_BUF_SIZE)
#define Uart_Config_READ_BUF_SIZE       32
#endif

// Size of the driver's internal software FIFO, that acts as a backup for the
// FIFO in the dataport.
#if !defined(Uart_Config_BACKUP_FIFO_SIZE)
#   define Uart_Config_BACKUP_FIFO_SIZE    4096 // value found by testing
#endif


#if defined(UART_HAS_BACK_FIFO)
#error "UART_HAS_BACK_FIFO must not be defined here"
#endif

#if defined(Uart_Config_BACKUP_FIFO_SIZE) && (Uart_Config_BACKUP_FIFO_SIZE > 0)
#define UART_HAS_BACK_FIFO
#endif


static struct
{
    bool             isValid;
    ps_io_ops_t      io_ops;
    ps_chardevice_t  ps_cdev;
    FifoDataport*    outputFifo;

#ifdef UART_HAS_BACK_FIFO
    CharFifo         backupFifo;
#endif

} ctx;


//------------------------------------------------------------------------------
void drain_input_fifo(void)
{
    int ret;
    do
    {
        static char readBuf[Uart_Config_READ_BUF_SIZE];
        ret = ctx.ps_cdev.read(&(ctx.ps_cdev),
                               &readBuf, sizeof(readBuf), NULL, NULL);
        if (ret < 0)
        {
            Debug_LOG_ERROR("ctx.ps_cdev.read() failed, code %d", ret);
            return;
        }
        if (ret > Uart_Config_READ_BUF_SIZE)
        {
            Debug_LOG_ERROR("ctx.ps_cdev.read() returned %d (exceeds max %zu)",
                            ret, sizeof(readBuf));
            return;
        }
        if (ret > 0)
        {
            bool isBackupFifoEmpty =
#ifdef UART_HAS_BACK_FIFO
                CharFifo_isEmpty(&ctx.backupFifo);
#else
                false;
#endif

            size_t  i = 0;
            bool    useBackupFifo = !isBackupFifoEmpty;

            while (i < ret && !useBackupFifo)
            {
                size_t toWrite = ret - i;
                size_t written =
                    FifoDataport_write(ctx.outputFifo, &readBuf[i], toWrite);
                if (toWrite < written)
                {
                    useBackupFifo = true;
                }
                else
                {
                    i += written;
                }
            }
            for (size_t j = i; j < ret; j++)
            {

#ifdef UART_HAS_BACK_FIFO
                if (!CharFifo_push(&ctx.backupFifo, &readBuf[j]))
#endif
                {
                    // We do not have an error state in the current Uart to store
                    // there as well what happened here, maybe we have to consider
                    // this for the future
                    Debug_LOG_WARNING("UART input FIFO full, discarding data");
                    Uart_DataAvailable_emit();
                    return;
                }
            }
            /* We are triggering an event after having read READ_BUF_SIZE at
             * most. Supposing we read in 2 loop iterations 128 and 1 byte..
             * we call twice emit() within the same ISR execution.
             * There will be 2 possible cases: either the context switch
             * happened at the first emit() or not.
             * In the first case 128 bytes get consumed and then a second
             * context switch will be needed to consume the remaining 1.
             * In the second case the context switch did not happen and the
             * emit() will be just idempotent. Finally we will consume 129
             * bytes when the context switch will happen.
             * Basically now READ_BUF_SIZE defines the granularity of
             * potential context switches in the scenario of full load.
             */
            Uart_DataAvailable_emit();
        }

#ifdef UART_HAS_BACK_FIFO
        // try to drain the backup FIFO
        size_t backupFifoSize = CharFifo_getSize(&ctx.backupFifo);
        size_t i = 0;
        for (; i < backupFifoSize; i++)
        {
            if (!FifoDataport_write(ctx.outputFifo,
                                    CharFifo_getFirst(&ctx.backupFifo),
                                    1))
            {
                break;
            }
            CharFifo_pop(&ctx.backupFifo);
        }
        if (i > 0)
        {
            Uart_DataAvailable_emit();
        }
#endif

    }
    while (ret > 0
#ifdef UART_HAS_BACK_FIFO
           && !CharFifo_isEmpty(&ctx.backupFifo)
#endif
          );
}


//------------------------------------------------------------------------------
void irq_handle(void)
{
    if (!ctx.isValid)
    {
        Debug_LOG_WARNING("ISR is active while context is not initialised");
    }
    else
    {
        drain_input_fifo();
    }

    int ret = irq_acknowledge();
    if (0 != ret)
    {
        Debug_LOG_ERROR("UART irq_acknowledge() failed, code %d", ret);
    }
}


//------------------------------------------------------------------------------
// Interface UartDrv
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void
UartDrv_write(
    size_t len)
{
    if (!ctx.isValid)
    {
        Debug_LOG_ERROR("UART not initialized");
        return;
    }

    OS_Dataport_t port = OS_DATAPORT_ASSIGN(Uart_inputDataport);
    size_t port_size = OS_Dataport_getSize(port);
    if (len > port_size)
    {
        Debug_LOG_ERROR("write length %zu exceeds port size %zu",
                        len,
                        port_size);
        return;
    }

    ssize_t ret = ctx.ps_cdev.write(
                      &(ctx.ps_cdev),
                      OS_Dataport_getBuf(port),
                      len,
                      NULL,
                      NULL);
    if (ret != len)
    {
        Debug_LOG_ERROR("write error, could only write %d of %zu bytes",
                        ret, len);
    }
}


//------------------------------------------------------------------------------
// CAmkES component
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void post_init(void)
{
    Debug_LOG_INFO("initialize UART");

    ctx.isValid         = false;
    ctx.fifoOverflow    = false;

    OS_Dataport_t out_dp = OS_DATAPORT_ASSIGN(Uart_outputFifoDataport);
    ctx.outputFifo = (FifoDataport*)OS_Dataport_getBuf(out_dp);
    size_t fifoCapacity = OS_Dataport_getSize(out_dp)
                          - offsetof(FifoDataport, data);
    if (!FifoDataport_ctor(ctx.outputFifo, fifoCapacity))
    {
        Debug_LOG_ERROR("FifoDataport_ctor() failed");
        return;
    }

#ifdef UART_HAS_BACK_FIFO

    static char backupBuf[Uart_Config_BACKUP_FIFO_SIZE];
    if (!CharFifo_ctor(&ctx.backupFifo, backupBuf, sizeof(backupBuf)))
    {
        Debug_LOG_ERROR("CharFifo_ctor() failed");
        return;
    }

#endif

    int ret = camkes_io_ops( &(ctx.io_ops) );
    if (0 != ret)
    {
        Debug_LOG_ERROR("camkes_io_ops() failed, code %d", ret);
        return;
    }

    // We can't use ps_cdev_init(), because it expects an ID, usually simply
    // set to PS_SERIAL_DEFAULT to use the default UART.
    //
    // ps_chardevice_t* dev = ps_cdev_init(
    //                            PS_SERIAL_DEFAULT,
    //                            &(ctx.io_ops),
    //                            &(ctx.ps_cdev);
    //
    // All we have from CAmkES is "regBase" with the virtual address of the
    // registers. Since the structure behind the ID just contains the physical
    // address, there is no change to correlate this. Luckily, some platforms
    // implement ps_cdev_static_init(), where we can pass the virtual address
    // directly.
    ps_chardevice_t* dev = ps_cdev_static_init(
                               &(ctx.io_ops),
                               &(ctx.ps_cdev),
                               regBase);
    if (dev != &(ctx.ps_cdev))
    {
        Debug_LOG_ERROR("ps_cdev_init() failed, returned dev=%p", dev);
        return;
    }

    // this is not a console, so we don't want that every CR (\n) is
    // automatically turned into CR LF (\r\n)
    ctx.ps_cdev.flags &= ~SERIAL_AUTO_CR;

    ctx.isValid = true;

    Debug_LOG_INFO("initialize UART ok");
}
