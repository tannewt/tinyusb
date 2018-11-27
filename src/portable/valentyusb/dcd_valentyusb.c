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

#include "tusb_option.h"

#include "device/dcd.h"

/*------------------------------------------------------------------*/
/* MACRO TYPEDEF CONSTANT ENUM
 *------------------------------------------------------------------*/
static ATTR_ALIGNED(4) uint8_t _setup_packet[8];

// Setup the control endpoint 0.
static void bus_reset(void) {
    // Prepare for setup packet
    dcd_edpt_xfer(0, 0, _setup_packet, sizeof(_setup_packet));
}


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

  uint8_t const epnum = edpt_number(desc_edpt->bEndpointAddress);
  uint8_t const dir   = edpt_dir(desc_edpt->bEndpointAddress);

  uint32_t size_value = 0;
  while (size_value < 7) {
    if (1 << (size_value + 3) == desc_edpt->wMaxPacketSize.size) {
      break;
    }
    size_value++;
  }

  // unsupported endpoint size
  if ( size_value == 7 && desc_edpt->wMaxPacketSize.size != 1023 ) return false;

  if ( dir == TUSB_DIR_OUT )
  {
  }else
  {
  }

  return true;
}

bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  (void) rhport;

  uint8_t const epnum = edpt_number(ep_addr);
  uint8_t const dir   = edpt_dir(ep_addr);

  // A setup token can occur immediately after an OUT STATUS packet so make sure we have a valid
  // buffer for the control endpoint.
  if (epnum == 0 && dir == 0 && buffer == NULL) {
    buffer = _setup_packet;
  }

  if ( dir == TUSB_DIR_OUT )
  {
  } else
  {
  }

  return true;
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

  // USBD shouldn't check control endpoint state
  if ( 0 == ep_addr ) return false;

  uint8_t const epnum = edpt_number(ep_addr);

  if (edpt_dir(ep_addr) == TUSB_DIR_IN) {
  }
}

/*------------------------------------------------------------------*/

static bool maybe_handle_setup_packet(void) {
    //if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.RXSTP)
    //{
        // This copies the data elsewhere so we can reuse the buffer.
        dcd_event_setup_received(0, (uint8_t*) sram_registers[0][0].ADDR.reg, true);
        return true;
    //}
    return false;
}
/*
 *------------------------------------------------------------------*/
void USB_0_Handler(void) {
  /*------------- Interrupt Processing -------------*/
  if ( false )
  {
    bus_reset();
    dcd_event_bus_signal(0, DCD_EVENT_BUS_RESET, true);
  }

  // Setup packet received.
  maybe_handle_setup_packet();
}
/* USB_SOF_HSOF */
void USB_1_Handler(void) {
    dcd_event_bus_signal(0, DCD_EVENT_SOF, true);
}

void transfer_complete(uint8_t direction) {
    uint32_t epints = USB->DEVICE.EPINTSMRY.reg;
    for (uint8_t epnum = 0; epnum < USB_EPT_NUM; epnum++) {
        if ((epints & (1 << epnum)) == 0) {
            continue;
        }

        if (direction == TUSB_DIR_OUT && maybe_handle_setup_packet()) {
            continue;
        }
        uint16_t total_transfer_size = 0;

        uint8_t ep_addr = epnum;
        if (direction == TUSB_DIR_IN) {
            ep_addr |= TUSB_DIR_IN_MASK;
        }
        dcd_event_xfer_complete(0, ep_addr, total_transfer_size, XFER_RESULT_SUCCESS, true);

        // just finished status stage (total size = 0), prepare for next setup packet
        if (epnum == 0 && total_transfer_size == 0) {
            dcd_edpt_xfer(0, 0, _setup_packet, sizeof(_setup_packet));
        }

    }
}

// Bank zero is for OUT and SETUP transactions.
void USB_2_Handler(void) {
    transfer_complete(TUSB_DIR_OUT);
}

// Bank one is used for IN transactions.
void USB_3_Handler(void) {
    transfer_complete(TUSB_DIR_IN);
}

#endif
