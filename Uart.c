/*
 *  UART
 *
 *  Copyright (C) 2020, Hensoldt Cyber GmbH
 */

#include "LibDebug/Debug.h"

#include <platsupport/chardev.h>
#include <platsupport/serial.h>
#include <platsupport/plat/serial.h>

#include <camkes.h>
#include <camkes/io.h>

#include <stdbool.h>
#include "LibIO/FifoDataport.h"
#include "LibUtil/CharFifo.h"

#if defined(UART_CONFIG_H_FILE)
#   define Uart_XSTR(d)    Uart_STR(d)
#   define Uart_STR(d)     #d
#   include Debug_XSTR(UART_CONFIG_H_FILE)
#else
// defines, in the ISR loop, how many bytes per iteration we try to read before
// to signal data available
#   define Uart_Config_READ_BUF_SIZE       32
// defines the size of a backup FIFO. The aim of the backup FIFO is to allow
// flexible amount of bytes retained in the UART. The size of the main shared
// FIFO (FifoDataport) could be indeed constrained by system constraints and not
// sufficient to avoid data-loss
#   define Uart_Config_BACKUP_FIFO_SIZE    4096
#endif

static struct
{
    bool             isValid;
    ps_io_ops_t      io_ops;
    ps_chardevice_t  ps_cdev;
    FifoDataport*    outputFifo;
#if Uart_Config_BACKUP_FIFO_SIZE > 0
    CharFifo         backupFifo;
#endif
} ctx;


//------------------------------------------------------------------------------
void irq_handle(void)
{
    int ret = irq_acknowledge();
    if (0 != ret)
    {
        Debug_LOG_FATAL("%s: UART irq_acknowledge() error, code %d",
                        __func__, ret);
        // we do not return here as there could be anyway data to retrieve
    }

    if (!ctx.isValid)
    {
        Debug_LOG_WARNING("%s: UART ISR is active while UART context is not initialised",
                          __func__);
        return;
    }

    do
    {
        static char readBuf[Uart_Config_READ_BUF_SIZE];
        ret = ctx.ps_cdev.read(&(ctx.ps_cdev),
                               &readBuf, sizeof(readBuf), NULL, NULL);
        if (ret < 0)
        {
            Debug_LOG_ERROR("%s: UART read error, code %d", __func__, ret);
            return;
        }
        if (ret > Uart_Config_READ_BUF_SIZE)
        {
            Debug_LOG_FATAL("%s: UART platform read() returned an amount of %zu when %zu is maximum expected",
                            __func__, ret, Uart_Config_READ_BUF_SIZE);
            return;
        }
        if (ret > 0)
        {
            bool isBackupFifoEmpty =
#if Uart_Config_BACKUP_FIFO_SIZE > 0
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
#if Uart_Config_BACKUP_FIFO_SIZE > 0
                if (!CharFifo_push(&ctx.backupFifo, &readBuf[j]))
#endif
                {
                    // We do not have an error state in the current Uart to store
                    // there as well what happened here, maybe we have to consider
                    // this for the future
                    Debug_LOG_WARNING("%s: UART FIFO is full, bytes will be discarded", __func__);
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
#if Uart_Config_BACKUP_FIFO_SIZE > 0
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
#if Uart_Config_BACKUP_FIFO_SIZE > 0
           && !CharFifo_isEmpty(&ctx.backupFifo)
#endif
          );
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

    ssize_t ret = ctx.ps_cdev.write(
                      &(ctx.ps_cdev),
                      Uart_inputDataport,
                      len,
                      NULL,
                      NULL);
    if (ret != len)
    {
        Debug_LOG_ERROR("write error, could only write %zd of %zu bytes",
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
    ctx.outputFifo      = (FifoDataport*) Uart_outputFifoDataport;
    size_t fifoCapacity =
        sizeof( *(Uart_outputFifoDataport) ) - offsetof(FifoDataport, data);
#if Uart_Config_BACKUP_FIFO_SIZE > 0
    static char backupBuf[Uart_Config_BACKUP_FIFO_SIZE];
#endif
    if (!FifoDataport_ctor(ctx.outputFifo, fifoCapacity))
    {
        Debug_LOG_ERROR("FifoDataport_ctor() failed in %s", __FILE__);
        return;
    }
#if Uart_Config_BACKUP_FIFO_SIZE > 0
    if (!CharFifo_ctor(&ctx.backupFifo, backupBuf, Uart_Config_BACKUP_FIFO_SIZE))
    {
        Debug_LOG_ERROR("CharFifo_ctor() failed in %s", __FILE__);
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
        Debug_LOG_ERROR("ps_cdev_init() failed, code %p", dev);
        return;
    }

    // this is not a console, so we don't want that every CR (\n) is
    // automatically turned into CR LF (\r\n)
    ctx.ps_cdev.flags &= ~SERIAL_AUTO_CR;

    ctx.isValid = true;

    Debug_LOG_INFO("initialize UART ok");
}
