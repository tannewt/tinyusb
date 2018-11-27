/**************************************************************************/
/*!
    @file     dcd_valentyusb.c
    @author   mithro

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Scott Shawcroft for Adafruit Industries
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
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    This file is part of the tinyusb stack.
*/
/**************************************************************************/

#include "generated/csr.h"

#include "tusb_option.h"

#include "device/dcd.h"

// Setup the control endpoint 0.
/*
static void bus_reset(void) {
    // Prepare for setup packet
    //dcd_edpt_xfer(0, 0, _setup_packet, sizeof(_setup_packet));
}*/


/*------------------------------------------------------------------*/
/* Controller API
 *------------------------------------------------------------------*/
bool dcd_init (uint8_t rhport)
{
  (void) rhport;
  return true;
}

void dcd_connect (uint8_t rhport)
{
}
void dcd_disconnect (uint8_t rhport)
{
}

void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
  (void) rhport;

  // Wait for EP0 to finish before switching the address.
}

void dcd_set_config (uint8_t rhport, uint8_t config_num)
{
  (void) rhport;
  (void) config_num;
  // Nothing to do
}

/*------------------------------------------------------------------*/
/* DCD Endpoint port
 *------------------------------------------------------------------*/

bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * desc_edpt)
{
  (void) rhport;
  (void) desc_edpt;

  //uint8_t const epnum = edpt_number(desc_edpt->bEndpointAddress);
  ///uint8_t const dir   = edpt_dir(desc_edpt->bEndpointAddress);

  return true;
}

/*
static void poll_usb_endpoints()
{
  // Endpoint zero has data.
  if (usb_ep_0_out_empty_read()) {
    usb_ep_0_out_head_write(0);           // Push the FIFO forward by one
    //buffer[i] = usb_ep_0_out_head_read(); // Read the data
    //dcd_edpt_xfer(0, 0, _setup_packet, sizeof(_setup_packet));
    //dcd_event_setup_received(0, (uint8_t*), true);
    //dcd_event_bus_signal(0, DCD_EVENT_SOF, true);
  } else {
    if (usb_ep_0_in_empty_read()) {
      // FIXME: The endpoint has finished sending the bytes...
      dcd_event_xfer_complete(0, ep_addr, total_bytes, XFER_RESULT_SUCCESS, true);
    }
  }
}
*/

bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  (void) rhport;

  uint8_t const epnum = edpt_number(ep_addr);
  if (epnum != 0) {
    return false;
  }
  uint8_t const dir   = edpt_dir(ep_addr);

  if ( dir == TUSB_DIR_OUT ) {
    for(uint16_t i = 0; i < total_bytes; i++) {
      while (usb_ep_0_out_empty_read());    // Poll for data in the FIFO
      usb_ep_0_out_head_write(0);           // Push the FIFO forward by one
      buffer[i] = usb_ep_0_out_head_read(); // Read the data
    }
    dcd_event_xfer_complete(0, ep_addr, total_bytes, XFER_RESULT_SUCCESS, true);
    return true;
  } else {
    // If the buffer isn't empty, we can't write new data.
    if (!usb_ep_0_in_empty_read()) {
      return false;
    }
    // Push the data into the outgoing FIFO
    for(uint16_t i = 0; i < total_bytes; i++) {
      usb_ep_0_in_head_write(buffer[i]);
    }
    // FIXME: Arm the endpoint here..
    return true;
  }
}

bool dcd_edpt_stalled (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  return false;
}

void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  (void) ep_addr;
}

void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  (void) ep_addr;
}

bool dcd_edpt_busy (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  uint8_t const epnum = edpt_number(ep_addr);
  if (epnum != 0) {
    return true;
  }
  uint8_t const dir   = edpt_dir(ep_addr);

  if ( dir == TUSB_DIR_OUT ) {
    // ???
    return false;
  } else {
    return !usb_ep_0_in_empty_read();
  }
}

/*------------------------------------------------------------------*/
