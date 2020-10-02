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
#include <sel4/sel4.h> // needed for seL4_yield()

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
#define Uart_Config_READ_BUF_SIZE       512
#endif

// Size of the driver's internal software FIFO, that sits between the UART's
// hardware FIFO and the dataport FIFO
#if !defined(Uart_Config_INTERNAL_FIFO_SIZE)
#   define Uart_Config_INTERNAL_FIFO_SIZE    0
#endif


#if defined(UART_USE_INTERNAL_FIFO)
#error "UART_USE_INTERNAL_FIFO must not be defined here"
#endif

#if defined(Uart_Config_INTERNAL_FIFO_SIZE) && (Uart_Config_INTERNAL_FIFO_SIZE > 0)
#define UART_USE_INTERNAL_FIFO
#endif


typedef struct
{
    bool             isValid;
    char*            fifoOverflow;
    ps_io_ops_t      io_ops;
    ps_chardevice_t  ps_cdev;
    FifoDataport*    outputFifo;

#ifdef UART_USE_INTERNAL_FIFO
    CharFifo         internalFifo;
#endif

} ctx_t;

static ctx_t ctx;


//------------------------------------------------------------------------------
#ifdef UART_USE_INTERNAL_FIFO

// return the number of bytes written into the internal FIFO
static size_t internalFifoWrite(
    ctx_t* ctx,
    char* buf,
    size_t len)
{
    for (size_t pos = 0; pos < len; pos++)
    {
        if (!CharFifo_push(&ctx->internalFifo, &buf[pos]))
        {
            Debug_LOG_WARNING(
                "internal FIFO full, discarding %zu bytes",
                len - pos);

            // failure to push indicate the FIFO is full
            assert( CharFifo_isFull(&ctx->internalFifo) );

            return pos;
        }
    }

    return len;
}

#endif // UART_USE_INTERNAL_FIFO


//------------------------------------------------------------------------------
#ifdef UART_USE_INTERNAL_FIFO

// this is a separate function to structure the code better. Currently it is
// called when the internal FIFO has data, but within the function we don't
// assume this is always the case.
static size_t internalFifoDrainToDataPortFifo(
    ctx_t* ctx)
{
    size_t internalFifoSize = CharFifo_getSize(&ctx->internalFifo);
    for (size_t pos = 0; pos < internalFifoSize; pos++)
    {
        if (!FifoDataport_write(
                ctx->outputFifo,
                CharFifo_getFirst(&ctx->internalFifo),
                1))
        {
            // dataport FIFO is full, can't write more data from the internal
            // FIFO there. Note that the dataport FIFO is accessed by different
            // threads, so we can't do something like
            //     assert( CharFifo_isFull(ctx->outputFifo) );
            // here. Immediately after we've found it to be full, the other
            // thread my have read something from it, so it is no longer full.
            return pos;
        }

        CharFifo_pop(&ctx->internalFifo);
    }

    // if we arrive here, the internal FIFO must be empty
    Debug_LOG_DEBUG("internal FIFO empty again");

    assert( CharFifo_isEmpty(&ctx->internalFifo) );

    return internalFifoSize;
}
#endif // UART_USE_INTERNAL_FIFO

//------------------------------------------------------------------------------
void setOverflow(
    ctx_t* ctx,
    bool  isOverflow)
{
    *(ctx->fifoOverflow) = isOverflow ? (char)1 : (char)0;
}


//------------------------------------------------------------------------------
bool isOverflow(
    ctx_t* ctx)
{
    return (0 != *(ctx->fifoOverflow));
}


//------------------------------------------------------------------------------
void trigger_event(void)
{
    Uart_DataAvailable_emit();

    // give up out time slice, the upper layer may run now
    seL4_Yield();
}

//------------------------------------------------------------------------------
#ifdef UART_USE_INTERNAL_FIFO
static inline bool
internalFifoDrainToDataPortFifoAndNotify(
    ctx_t* ctx)
{
    // if there is something in the internal FIFO, then try to drain it
    // into the dataport FIFO and trigger the event if there are new bytes in
    // the dataport FIFO
    if (CharFifo_isEmpty(&ctx->internalFifo))
    {
        return false;
    }

    size_t written = internalFifoDrainToDataPortFifo(ctx);
    if (0 == written)
    {
        return false;
    }

    // we've written data, so trigger the event
    trigger_event();
    return true;
}
#endif // UART_USE_INTERNAL_FIFO

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
void drain_input_fifo(
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

#ifdef UART_USE_INTERNAL_FIFO
        // if there is something in the internal FIFO, then try to drain it
        // into the dataport FIFO first. Afterwards, if the internal FIFO is
        // empty, all new UART data goes into the dataport directly while there
        // is space. Otherwise the dataport FIFO is still full and all new data
        // goes into the internal FIFO
        if (internalFifoDrainToDataPortFifoAndNotify(ctx))
        {
            return;
        }
#endif // UART_USE_INTERNAL_FIFO

        // if there is no new data, we are done
        if (0 == bytesRead)
        {
            return;
        }

        // if we are here, data has been read from the UART's FIFO and it must
        // be transferred to the upper layer

#ifdef UART_USE_INTERNAL_FIFO

        // If the internal FIFO is not empty, this implies the dataport FIFO is
        // full - otherwise we would have drained more data into it. So all new
        // UART data goes into the internal FIFO then. What we can't put there
        // will be discarded.
        if (!CharFifo_isEmpty(&ctx->internalFifo))
        {
            size_t written = internalFifoWrite(ctx, readBuf, bytesRead);
            assert( written <= bytesRead );
            if (written < bytesRead)
            {
                // all FIFOs are full, so we have to discard the remaining data
                setOverflow(ctx, true);

                Debug_LOG_ERROR(
                    "internal FIFO full, discarding %zu bytes",
                    bytesRead - written);
            }

            // even if no data was written to the internal FIFO, we raise the
            // signal. Rationale is, that the dataport FIFO is full anyway, so
            // the upper layer shall drain it.
            trigger_event();

            // continue draining the UART FIFO, if all FIFOs are full it means
            // we discard the data.
            continue;
        }

        // if we are here and the internal FIFO is empty, any data from the
        // UART must be written into the dataport FIFO. It's basically the same
        // situation as if there was no internal FIFO.

#endif // UART_USE_INTERNAL_FIFO

        // there is data from the UART, write it into the dataport FIFO
        size_t written = FifoDataport_write(
                             ctx->outputFifo,
                             readBuf,
                             bytesRead);
        assert( written <= bytesRead );
        if (written < bytesRead)
        {

#ifdef UART_USE_INTERNAL_FIFO

            // if we arrive here, the dataport FIFO is full, so write the
            // remaining new UART data to the internal FIFO.
            Debug_LOG_DEBUG("dataport FIFO full, filling internal FIFO");

            // sanity check, the internal FIFO must be empty.
            assert( CharFifo_isEmpty(&ctx->internalFifo) );

            size_t lenLeft = bytesRead - written;
            size_t writtenInternal = internalFifoWrite(
                                         ctx,
                                         &readBuf[written],
                                         bytesRead - written);
            assert( writtenInternal <= lenLeft );
            if (writtenInternal < lenLeft)
            {
                // all FIFOs are full, so we have to discard the remaining data
                setOverflow(ctx, true);

                Debug_LOG_ERROR(
                    "internal FIFO full, discarding %zu bytes",
                    lenLeft - writtenInternal);
            }

#else // not UART_USE_INTERNAL_FIFO

            // all FIFO are full, so we have to discard the remaining data
            setOverflow(ctx, true);

            Debug_LOG_ERROR(
                "dataport FIFO full, discarding %zu bytes",
                bytesRead - written);

#endif // [not] UART_USE_INTERNAL_FIFO

        }

        // notify the upper layer that there is new data.
        trigger_event();

    } // end for (;;)
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
        ctx.ps_cdev.handle_irq(&(ctx.ps_cdev));
        drain_input_fifo(&ctx);
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

    ctx.isValid = false;

    OS_Dataport_t out_dp = OS_DATAPORT_ASSIGN(Uart_outputFifoDataport);

    // the last byte of the dataport holds an overflow flag
    ctx.fifoOverflow = (char*)( (uintptr_t)OS_Dataport_getBuf(out_dp)
                                + OS_Dataport_getSize(out_dp) - 1 );

    setOverflow(&ctx, false);

    ctx.outputFifo = (FifoDataport*)OS_Dataport_getBuf(out_dp);

    // the last bytes is used a overflow indicator
    size_t fifoCapacity = OS_Dataport_getSize(out_dp)
                          - offsetof(FifoDataport, data) - 1;

    if (!FifoDataport_ctor(ctx.outputFifo, fifoCapacity))
    {
        Debug_LOG_ERROR("FifoDataport_ctor() failed");
        return;
    }

#ifdef UART_USE_INTERNAL_FIFO

    static char internalBuf[Uart_Config_INTERNAL_FIFO_SIZE];
    if (!CharFifo_ctor(&ctx.internalFifo, internalBuf, sizeof(internalBuf)))
    {
        Debug_LOG_ERROR("CharFifo_ctor() failed");
        return;
    }

#endif // UART_USE_INTERNAL_FIFO

    // get a special set of io operations that are aware of the CAmkES memory
    // mappings, ie they can't really map anything, but look up the mapping in
    // the existing CAmkES mappings instead.
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
