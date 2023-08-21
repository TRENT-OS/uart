/*
 *  UART
 *
 *  Copyright (C) 2020, HENSOLDT Cyber GmbH
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

// #define UART_DRIVER_PROFILING

typedef struct
{
    bool             isValid;
    char*            fifoOverflow;
    ps_io_ops_t      io_ops;
    ps_chardevice_t  ps_cdev;
    FifoDataport*    inputFifo;
#ifdef UART_DRIVER_PROFILING
    size_t           cnt_intr;
    size_t           cnt_data;
    size_t           cnt_data_delta;
#endif
} ctx_t;

static ctx_t ctx = {0};


//------------------------------------------------------------------------------
static void
setOverflow(
    ctx_t* ctx,
    bool  isOverflow)
{
    *(ctx->fifoOverflow) = isOverflow ? (char)1 : (char)0;
}


//------------------------------------------------------------------------------
static bool
isOverflow(
    ctx_t* ctx)
{
    return (0 != *(ctx->fifoOverflow));
}


//------------------------------------------------------------------------------
static void
drain_input_fifo(
    ctx_t* ctx)
{
    if (isOverflow(ctx))
    {
        Debug_LOG_ERROR("isOverflow");
        return;
    }

    for (;;)
    {
        // The number of bytes we can read from the hardware FIFO is limited by
        // the number of bytes that are free in the dataport FIFO. Since it is
        // a circular buffer, the free part might be split into two parts, that
        // can only be accessed one at a time as a contiguous buffer each.
        void* buffer = NULL;
        size_t size = FifoDataport_getContiguousFree(ctx->inputFifo, &buffer);
        if (0 == size)
        {
            /* There is no space left in the dataport FIFO, we are in an
             * overflow condition now.
             */
            Debug_LOG_ERROR("dataport FIFO full");
            setOverflow(ctx, true);
            return;
        }

        int ret = ctx->ps_cdev.read(
                      &(ctx->ps_cdev),
                      buffer,
                      size,
                      NULL,
                      NULL);
        if (ret < 0)
        {
            Debug_LOG_ERROR("ctx.ps_cdev.read() failed, code %d", ret);
            return;
        }

        // ret holds the number of bytes read into the buffer, we are done if
        // there is no new data
        size_t bytesRead = (size_t)ret;
        if (0 == bytesRead)
        {
            return;
        }

        // We read some data, do a sanity check if the lower layer is behaving
        // well.
        if (bytesRead > size)
        {
            Debug_LOG_ERROR("ctx.ps_cdev.read() returned %d (exceeds max %zu)",
                            ret, size);
            // In debug builds this is fatal, in release builds we pretend that
            // we have read nothing and return. Actually, we should report a
            // fatal error here and stop the driver as there might be memory
            // corruption.
            assert(0);
            return;
        }

#ifdef UART_DRIVER_PROFILING
        ctx->cnt_data += bytesRead;
        // Log a message roughly every MiB, this works well enough to see some
        // statistics in the throughput test.
        ctx->cnt_data_delta += bytesRead;
        if (ctx->cnt_data_delta > 1024 * 1024)
        {
            ctx->cnt_data_delta = 0;
            Debug_LOG_INFO(
                "data: %zu (0x%zx), num intr: %zu, bytes/intr: %zu",
                ctx->cnt_data, ctx->cnt_data,
                ctx->cnt_intr,
                ctx->cnt_data / ctx->cnt_intr);
        }
#endif

        FifoDataport_add(ctx->inputFifo, bytesRead);
        Uart_DataAvailable_emit();

    } // end for (;;)
}


//------------------------------------------------------------------------------
void
dev_irq_handle(
    ps_irq_t* irq)
{
    if (!ctx.isValid)
    {
        Debug_LOG_WARNING("ISR is active while context is not initialised");
    }
    else
    {
#ifdef UART_DRIVER_PROFILING
        ctx.cnt_intr++;
#endif
        ctx.ps_cdev.handle_irq(&(ctx.ps_cdev));
        drain_input_fifo(&ctx);

        if (isOverflow(&ctx))
        {
            /* ToDo: save interrupt and ack once overflow is cleaned */
            Debug_LOG_ERROR("overflow detected, don't ack interrupt");
            return;
        }
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

    ssize_t ret;
    do {
        ret = ctx.ps_cdev.write(
                      &(ctx.ps_cdev),
                      OS_Dataport_getBuf(port),
                      len,
                      NULL,
                      NULL);
    } while (ret != len);
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

    size_t dataport_size = Uart_inputFifoDataport_get_size();
    void* dataport_base = Uart_inputFifoDataport;

    // The last byte of the dataport is used a overflow indicator.
    ctx.fifoOverflow = (char*)((uintptr_t)dataport_base + (dataport_size - 1) );
    setOverflow(&ctx, false);

    // the rest of the dataport can be used by the FIFO
    ctx.inputFifo = (FifoDataport*)dataport_base;
    if (!FifoDataport_ctor(
            ctx.inputFifo,
            (dataport_size - 1) - offsetof(typeof(*ctx.inputFifo), data) ))
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
    // just dummies, we expect them to find the UART's existing CAmkES mapping.
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
