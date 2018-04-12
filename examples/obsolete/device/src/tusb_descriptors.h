/**************************************************************************/
/*!
    @file     tusb_descriptors.h
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, hathach (tinyusb.org)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    This file is part of the tinyusb stack.
*/
/**************************************************************************/

#ifndef _TUSB_DESCRIPTORS_H_
#define _TUSB_DESCRIPTORS_H_

#include "tusb.h"

//--------------------------------------------------------------------+
// Descriptors Value (calculated by enabled Classes)
//--------------------------------------------------------------------+
#define CFG_VENDORID            0xCAFE
//#define CFG_PRODUCTID           0x4567 // use auto product id to prevent conflict with pc's driver

// each combination of interfaces need to have a unique productid, as windows will bind & remember device driver after the first plug.
// Auto ProductID layout's Bitmap: (MSB) MassStorage | Generic | Mouse | Key | CDC (LSB)
#ifndef CFG_PRODUCTID
  #define PRODUCTID_BITMAP(interface, n)  ( (CFG_TUD_##interface) << (n) )
  #define CFG_PRODUCTID                   (0x4000 | ( PRODUCTID_BITMAP(CDC, 0) | PRODUCTID_BITMAP(HID_KEYBOARD, 1) | \
                                           PRODUCTID_BITMAP(HID_MOUSE, 2) | PRODUCTID_BITMAP(HID_GENERIC, 3) | \
                                           PRODUCTID_BITMAP(MSC, 4) ) )
#endif

#define ITF_NUM_CDC           0
#define INTERFACE_NO_HID_KEYBOARD (ITF_NUM_CDC          + 2*(CFG_TUD_CDC ? 1 : 0) )
#define INTERFACE_NO_HID_MOUSE    (INTERFACE_NO_HID_KEYBOARD + CFG_TUD_HID_KEYBOARD    )
#define INTERFACE_NO_HID_GENERIC  (INTERFACE_NO_HID_MOUSE    + CFG_TUD_HID_MOUSE       )
#define ITF_NUM_MSC          (INTERFACE_NO_HID_GENERIC  + CFG_TUD_HID_GENERIC     )

#define ITF_TOTAL           (2*CFG_TUD_CDC + CFG_TUD_HID_KEYBOARD + CFG_TUD_HID_MOUSE + \
                                   CFG_TUD_HID_GENERIC + CFG_TUD_MSC)

#if (CFG_TUSB_MCU == OPT_MCU_LPC11UXX || CFG_TUSB_MCU == OPT_MCU_LPC13UXX) && (ITF_TOTAL > 4)
  #error These MCUs do not have enough number of endpoints for the current configuration
#endif

//--------------------------------------------------------------------+
// Endpoints Address & Max Packet Size
//--------------------------------------------------------------------+
#define EDPT_IN(x)    (0x80 | (x))
#define EDPT_OUT(x)   (x)

#if CFG_TUSB_MCU == OPT_MCU_LPC175X_6X // MCUs's endpoint number has a fixed type

  //------------- CDC -------------//
  #define CDC_EDPT_NOTIF            EDPT_IN (1)
  #define CDC_EDPT_NOTIFICATION_PACKETSIZE      64

  #define CDC_EDPT_OUT                EDPT_OUT(2)
  #define CDC_EDPT_IN                 EDPT_IN (2)
  #define CDC_EDPT_SIZE              64

  //------------- HID Keyboard -------------//
  #define HID_KEYBOARD_EDPT_ADDR                EDPT_IN (4)
  #define HID_KEYBOARD_EDPT_PACKETSIZE          8

  //------------- HID Mouse -------------//
  #define HID_MOUSE_EDPT_ADDR                   EDPT_IN (7)
  #define HID_MOUSE_EDPT_PACKETSIZE             8

  //------------- HID Generic -------------//

  //------------- Mass Storage -------------//
  #define MSC_EDPT_OUT_ADDR                     EDPT_OUT(5)
  #define MSC_EDPT_IN_ADDR                      EDPT_IN (5)

#else

  //------------- CDC -------------//
  #define CDC_EDPT_NOTIF            EDPT_IN (ITF_NUM_CDC+1)
  #define CDC_EDPT_NOTIFICATION_PACKETSIZE      64

  #define CDC_EDPT_OUT                EDPT_OUT(ITF_NUM_CDC+2)
  #define CDC_EDPT_IN                 EDPT_IN (ITF_NUM_CDC+2)
  #define CDC_EDPT_SIZE              64

  //------------- HID Keyboard -------------//
  #define HID_KEYBOARD_EDPT_ADDR                EDPT_IN (INTERFACE_NO_HID_KEYBOARD+1)
  #define HID_KEYBOARD_EDPT_PACKETSIZE          8

  //------------- HID Mouse -------------//
  #define HID_MOUSE_EDPT_ADDR                   EDPT_IN (INTERFACE_NO_HID_MOUSE+1)
  #define HID_MOUSE_EDPT_PACKETSIZE             8

  //------------- HID Generic -------------//

  //------------- Mass Storage -------------//
  #define MSC_EDPT_OUT_ADDR                     EDPT_OUT(ITF_NUM_MSC+1)
  #define MSC_EDPT_IN_ADDR                      EDPT_IN (ITF_NUM_MSC+1)

#endif

#define MSC_EDPT_PACKETSIZE                   (CFG_TUSB_MCU == OPT_MCU_LPC43XX ? 512 : 64)

//--------------------------------------------------------------------+
// CONFIGURATION DESCRIPTOR
//--------------------------------------------------------------------+
typedef struct ATTR_PACKED
{
  tusb_desc_configuration_t              configuration;

  //------------- CDC -------------//
#if CFG_TUD_CDC
  tusb_desc_interface_assoc_t      cdc_iad;

  //CDC Control Interface
  tusb_desc_interface_t                  cdc_comm_interface;
  cdc_desc_func_header_t                       cdc_header;
  cdc_desc_func_call_management_t              cdc_call;
  cdc_desc_func_acm_t  cdc_acm;
  cdc_desc_func_union_t                        cdc_union;
  tusb_desc_endpoint_t                   cdc_endpoint_notification;

  //CDC Data Interface
  tusb_desc_interface_t                  cdc_data_interface;
  tusb_desc_endpoint_t                   cdc_endpoint_out;
  tusb_desc_endpoint_t                   cdc_endpoint_in;
#endif

  //------------- HID Keyboard -------------//
#if CFG_TUD_HID_KEYBOARD
  tusb_desc_interface_t                    keyboard_interface;
  tusb_hid_descriptor_hid_t                      keyboard_hid;
  tusb_desc_endpoint_t                     keyboard_endpoint;
#endif

//------------- HID Mouse -------------//
#if CFG_TUD_HID_MOUSE
  tusb_desc_interface_t                    mouse_interface;
  tusb_hid_descriptor_hid_t                      mouse_hid;
  tusb_desc_endpoint_t                     mouse_endpoint;
#endif

//------------- Mass Storage -------------//
#if CFG_TUD_MSC
  tusb_desc_interface_t                    msc_interface;
  tusb_desc_endpoint_t                     msc_endpoint_in;
  tusb_desc_endpoint_t                     msc_endpoint_out;
#endif

} app_descriptor_configuration_t;

#endif
