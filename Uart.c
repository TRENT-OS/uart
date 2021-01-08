/*
 *  UART
 *
 *  Copyright (C) 2020, Hensoldt Cyber GmbH
 */

#include "lib_debug/Debug.h"
#include "OS_Dataport.h"

#include <platsupport/chardev.h>
#include <platsupport/serial.h>
#include <platsupport/plat/serial.h>
#include <sel4/sel4.h> // needed for seL4_yield()

#include <camkes.h>
#include <camkes/io.h>

#include <stdbool.h>

#include "lib_io/FifoDataport.h"
#include "OS_Dataport.h"

// UART ID used by lib-platsupport must be set
#if !defined(UART_CONFIG_ID)
#error "UART_CONFIG_ID missing"
#endif

// Number of bytes read at most from the UART FIFO at once
#if !defined(Uart_Config_READ_BUF_SIZE)
#define Uart_Config_READ_BUF_SIZE       512
#endif

// Here below the reason not to simply use OS_Dataport_getSize(port) is that
// this does not work for the cases of user defined dataport size. Camkes
// internally changes its behaviour making assumption on which the function
// works decaying.
#if !defined(Uart_INPUT_FIFO_DATAPORT_SIZE)
#   define INPUT_FIFO_DATAPORT_SIZE(port) OS_Dataport_getSize(port)
#else
#   define INPUT_FIFO_DATAPORT_SIZE(port) Uart_INPUT_FIFO_DATAPORT_SIZE
#endif

typedef struct
{
    bool             isValid;
    char*            fifoOverflow;
    ps_io_ops_t      io_ops;
    ps_chardevice_t  ps_cdev;
    FifoDataport*    inputFifo;
} ctx_t;

static ctx_t ctx;


//------------------------------------------------------------------------------
void
setOverflow(
    ctx_t* ctx,
    bool  isOverflow)
{
    *(ctx->fifoOverflow) = isOverflow ? (char)1 : (char)0;
}


//------------------------------------------------------------------------------
bool
isOverflow(
    ctx_t* ctx)
{
    return (0 != *(ctx->fifoOverflow));
}


//------------------------------------------------------------------------------
void
trigger_event(void)
{
    Uart_DataAvailable_emit();

    // give up our time slice, the upper layer may run now
    seL4_Yield();
}


//------------------------------------------------------------------------------
static inline bool
lowLevelRead(
    ctx_t* ctx,
    char readBuf[],
    size_t readBufSize,
    size_t* read)
{
    int ret = ctx->ps_cdev.read(
                      &(ctx->ps_cdev),
                      readBuf,
                      readBufSize,
                      NULL,
                      NULL);
    if (ret < 0)
    {
        Debug_LOG_ERROR("ctx.ps_cdev.read() failed, code %d", ret);
        return false;
    }

    if (ret > readBufSize)
    {
        Debug_LOG_ERROR("ctx.ps_cdev.read() returned %d (exceeds max %zu)",
                        ret, readBufSize);
        return false;
    }

    // ret holds the number of bytes read into the buffer
    *read = (size_t) ret;
    return true;
}


//------------------------------------------------------------------------------
void
drain_input_fifo(
    ctx_t* ctx)
{
    for (;;)
    {
        // Uart_Config_READ_BUF_SIZE defines the amount of bytes we will read
        // from the UART's FIFO at once. We trigger an event then, thus this
        // value can be used to control granularity of potential context
        // switches where the upper layer might run to pick up the data.
        static char readBuf[Uart_Config_READ_BUF_SIZE];

        size_t bytesRead = 0;
        if (!lowLevelRead(ctx, readBuf, sizeof(readBuf), &bytesRead))
        {
            return;
        }

        // do nothing on overflow
        // ToDo: we need an API where the upper layer has to reset this.
        if (isOverflow(ctx))
        {
            if (0 == bytesRead)
            {
                // UART FIFO is empty, all data has been discarded. Trigger
                // event, so upper layer wakes up and sees the error.
                trigger_event();
                return;
            }

            continue; // keep draining the FIFO
        }

        // if there is no new data, we are done
        if (0 == bytesRead)
        {
            return;
        }

        // if we are here, data has been read from the UART's FIFO and it must
        // be transferred to the upper layer

        // there is data from the UART, write it into the dataport FIFO
        size_t written = FifoDataport_write(
                             ctx->inputFifo,
                             readBuf,
                             bytesRead);
        assert( written <= bytesRead );
        if (written < bytesRead)
        {
            // dataport FIFO is full, we have to discard the remaining data
            setOverflow(ctx, true);

            Debug_LOG_ERROR(
                "dataport FIFO (capacity %zu) full, discarding %zu bytes",
                FifoDataport_getCapacity(ctx->inputFifo),
                bytesRead - written);
        }

        // notify the upper layer that there is new data.
        trigger_event();

    } // end for (;;)
}


//------------------------------------------------------------------------------
void
dev_irq_handle(
    ps_irq_t *irq)
{
    if (!ctx.isValid)
    {
        Debug_LOG_WARNING("ISR is active while context is not initialised");
    }
    else
    {
        ctx.ps_cdev.handle_irq(&(ctx.ps_cdev));
        drain_input_fifo(&ctx);
    }

    int ret = dev_irq_acknowledge(irq);
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

    OS_Dataport_t port = OS_DATAPORT_ASSIGN(Uart_outputDataport);
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
void
post_init(void)
{
    Debug_LOG_INFO("initialize UART");

    ctx.isValid = false;

    OS_Dataport_t port = OS_DATAPORT_ASSIGN(Uart_inputFifoDataport);

    // the last byte of the dataport holds an overflow flag
    ctx.fifoOverflow = (char*)( (uintptr_t)OS_Dataport_getBuf(port)
                                + INPUT_FIFO_DATAPORT_SIZE(port) - 1 );

    setOverflow(&ctx, false);

    ctx.inputFifo = (FifoDataport*)OS_Dataport_getBuf(port);

    // the last bytes is used a overflow indicator
    size_t fifoCapacity = INPUT_FIFO_DATAPORT_SIZE(port)
                          - offsetof(FifoDataport, data) - 1;

    if (!FifoDataport_ctor(ctx.inputFifo, fifoCapacity))
    {
        Debug_LOG_ERROR("FifoDataport_ctor() failed");
        return;
    }

    // Get a special set of io operations that are aware of the CAmkES memory
    // mappings, ie they can't really map anything, but look up the mapping in
    // the existing CAmkES mappings instead.
    int ret = camkes_io_ops( &(ctx.io_ops) );
    if (0 != ret)
    {
        Debug_LOG_ERROR("camkes_io_ops() failed, code %d", ret);
        return;
    }

    // This will get the physical address of the UART peripheral from the
    // UART ID and then maps it using the passed io_ops. Since these ops are
    // just dummies, we expect them to to find the existing CAmkES mapping of
    // the UART.
    ps_chardevice_t* dev = ps_cdev_init(
                               UART_CONFIG_ID,
                               &(ctx.io_ops),
                               &(ctx.ps_cdev));
    if (dev != &(ctx.ps_cdev))
    {
        Debug_LOG_ERROR("ps_cdev_init() failed, returned dev=%p", dev);
        return;
    }

    // This is not a console, so we don't want that every CR (\n) is
    // automatically turned into CR LF (\r\n).
    ctx.ps_cdev.flags &= ~SERIAL_AUTO_CR;

    ctx.isValid = true;

    Debug_LOG_INFO("initialize UART ok");
}
