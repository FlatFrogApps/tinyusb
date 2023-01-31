#include "dcd_num487.h"

#if USE_FS_PORT == 1
#include "dcd_num487_fs.h"
#endif

#if USE_HS_PORT == 1
#include "dcd_num487_hs.h"
#endif

#include <assert.h>

#define ARRAY_LENGTH(array) (sizeof(array) / sizeof(array[0]))

const struct num487_dcd_driver * const drivers[] =
{
#if USE_FS_PORT == 1
    &fs_driver,
#endif

#if USE_HS_PORT == 1
    &hs_driver,
#endif
};

_Static_assert( ARRAY_LENGTH(drivers)>0,"No drivers defined!");

//--------------------------------------------------------------------+
// Controller API
//--------------------------------------------------------------------+

void dcd_init(uint8_t rhport)
{

    assert(rhport < ARRAY_LENGTH(drivers));
    drivers[rhport]->init(rhport);
}

void dcd_int_enable(uint8_t rhport)
{
    assert(rhport < ARRAY_LENGTH(drivers));
    drivers[rhport]->int_enable();
}

void dcd_int_disable(uint8_t rhport)
{
    assert(rhport < ARRAY_LENGTH(drivers));
    drivers[rhport]->int_disable();
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
    assert(rhport < ARRAY_LENGTH(drivers));
    drivers[rhport]->set_address(dev_addr);
}

void dcd_remote_wakeup(uint8_t rhport)
{
    assert(rhport < ARRAY_LENGTH(drivers));
    drivers[rhport]->remote_wakeup();
}

void dcd_connect(uint8_t rhport)
{
    assert(rhport < ARRAY_LENGTH(drivers));

    if( drivers[rhport]->connect )
    {
        drivers[rhport]->connect();
    }
}

void dcd_disconnect(uint8_t rhport)
{
    assert(rhport < ARRAY_LENGTH(drivers));

    if( drivers[rhport]->disconnect )
    {
        drivers[rhport]->disconnect();
    }
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
    assert(rhport < ARRAY_LENGTH(drivers));
    drivers[rhport]->sof_enable(en);
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const * request)
{
    assert(rhport < ARRAY_LENGTH(drivers));

    if( drivers[rhport]->edpt0_status_complete )
    {
        drivers[rhport]->edpt0_status_complete(request);
    }
}

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * desc_ep)
{
    assert(rhport < ARRAY_LENGTH(drivers));
    return drivers[rhport]->open(desc_ep);
}

void dcd_edpt_close_all(uint8_t rhport)
{
    assert(rhport < ARRAY_LENGTH(drivers));
    drivers[rhport]->close_all();
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
    assert(rhport < ARRAY_LENGTH(drivers));

    if( drivers[rhport]->close )
    {
        drivers[rhport]->close(ep_addr);
    }
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
    assert(rhport < ARRAY_LENGTH(drivers));
    return drivers[rhport]->xfer(ep_addr, buffer, total_bytes);
}

bool dcd_edpt_xfer_fifo(uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes)
{
    assert(rhport < ARRAY_LENGTH(drivers));

    if( drivers[rhport]->xfer_fifo )
    {
        return drivers[rhport]->xfer_fifo(ep_addr, ff, total_bytes);
    }
    return false;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
    assert(rhport < ARRAY_LENGTH(drivers));
    drivers[rhport]->stall(ep_addr);
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
    assert(rhport < ARRAY_LENGTH(drivers));
    drivers[rhport]->clear_stall(ep_addr);
}
