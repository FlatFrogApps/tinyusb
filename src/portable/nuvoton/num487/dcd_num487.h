#ifndef DCD_NUM487_H
#define DCD_NUM487_H

#include <stdint.h>

#include "device/dcd.h"

// TODO: This should move to build config
#define USE_FS_PORT 1
#define USE_HS_PORT 1

#ifndef USE_FS_PORT
#define USE_FS_PORT 0
#endif

#ifndef USE_HS_PORT
#define USE_HS_PORT 0
#endif

enum ep_enum
{
  PERIPH_EPA = 0,
  PERIPH_EPB = 1,
  PERIPH_EPC = 2,
  PERIPH_EPD = 3,
  PERIPH_EPE = 4,
  PERIPH_EPF = 5,
  PERIPH_EPG = 6,
  PERIPH_EPH = 7,
  PERIPH_EPI = 8,
  PERIPH_EPJ = 9,
  PERIPH_EPK = 10,
  PERIPH_EPL = 11,
  PERIPH_MAX_EP,
};

enum portno
{
#if USE_FS_PORT == 1
    USB_FS_PORT,
#endif
#if USE_HS_PORT == 1
    USB_HS_PORT,
#endif
    USB_NUM_PORTS
};

struct num487_dcd_driver
{
    // Device API
    void(*init)(uint8_t portno);
    void(*int_enable)(void);
    void(*int_disable)(void);
    void(*set_address)(uint8_t dev_addr);
    void(*remote_wakeup)(void);
    void(*connect)(void);
    void(*disconnect)(void);
    void(*sof_enable)(bool enable);

    // Endpoint API
    bool(*open)(tusb_desc_endpoint_t const * ep_desc);
    void(*close)(uint8_t ep_addr);
    void(*close_all)(void);
    bool(*xfer)(uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes);
    bool(*xfer_fifo)(uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes);
    void(*stall)(uint8_t ep_addr);
    void(*clear_stall)(uint8_t ep_addr);

    void(*edpt0_status_complete)(tusb_control_request_t const * request);
};


#endif // DCD_NUM487_H
