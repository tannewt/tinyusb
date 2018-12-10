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

#include <irq.h>
#include <stdio.h>
#include <string.h>
#include <console.h>
#include <uart.h>

#include "generated/csr.h"

#include "tusb_option.h"

#include "device/dcd.h"

static uint8_t _setup_packet[8];

struct {
  uint8_t* buffer;
  uint8_t  len;
} out_ep_buffers[2];

struct {
  int8_t  len;
} in_ep_buffers[2];


void isr(void);
__attribute__ ((used)) void isr(void)
{
	unsigned int irqs;

	irqs = irq_pending() & irq_getmask();

	if(irqs & (1 << UART_INTERRUPT))
		uart_isr();
}

void board_init(void)
{
	irq_setmask(0);
	irq_setie(1);
	uart_init();

  puts("Continue? ");
  readchar();
}

uint32_t tusb_hal_millis(void)
{
  return 0;
}


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
  // Clear all the buffer pointers.
  out_ep_buffers[0].buffer = NULL;
  out_ep_buffers[0].len = 255;

  out_ep_buffers[1].buffer = NULL;
  out_ep_buffers[1].len = 255;

  in_ep_buffers[0].len = 255;
  in_ep_buffers[1].len = 255;

  // Allow the USB to start
  usb_pullup_out_write(1);

  printf("init\n");

  // Prepare for setup packet
  //dcd_edpt_xfer(0, 0, _setup_packet, sizeof(_setup_packet));
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

enum PID {
  PID_OUT     = 0b0001, // Token
  PID_ACK     = 0b0010,
  PID_DATA0   = 0b0011, // Data
              //0b0100,
  PID_SOF     = 0b0101, // Token
              //0b0110,
              //0b0111,
              //0b1000,
  PID_IN      = 0b1001, // Token
  PID_NAK     = 0b1010,
  PID_DATA1   = 0b1011,
              //0b1100,
  PID_SETUP   = 0b1101, // Token
  PID_STALL   = 0b1110,
              //0b1111,
};

enum TOK {
                //XX01
  PID_OUT     = 0b0001,
  PID_SOF     = 0b0101,
  PID_IN      = 0b1001,
  PID_SETUP   = 0b1101,
};

void ep0_xfer_complete(uint8_t len) {
  if (len != 255) {
    dcd_event_xfer_complete(0, 0, len, XFER_RESULT_SUCCESS, false);
  }
  if (len == 0) {
    //printf("queuing setup\r\n");
    // setup->wLength = ENDIAN_BE16(setup->wLength);
    //dcd_edpt_xfer(0, 0, _setup_packet, sizeof(_setup_packet));
  }
}

void dcd_poll(uint8_t rhport)
{
  // Have we finished sending the packet?
  if (usb_ep_0_in_ev_pending_read()) {
    uint8_t len = in_ep_buffers[0].len;
    in_ep_buffers[0].len = 255;
    if (len != 255) {
      if (!usb_ep_0_in_empty_read()) {
        printf("not empty !?\r\n");
      }
      printf("dcd_poll ep0  in complete %d\r\n", len);
      ep0_xfer_complete(len);
    }
  }

  //printf("poll ");
  // Endpoint zero has data.
  if (usb_ep_0_out_ev_pending_read()) {
    printf("\r\n");
    enum PID last_pid = usb_last_pid_read();
    printf("pid ");
    switch(last_pid) {
    case PID_OUT:
      printf("out ");
      break;
    case PID_IN:
      printf("in ");
      break;
    case PID_SOF:
      printf("sof ");
      break;
    case PID_SETUP:
      printf("set ");
      break;
    case PID_DATA0:
      printf("d0 ");
      break;
    case PID_DATA1:
      printf("d1 ");
      break;
    case PID_ACK:
      printf("ack ");
      break;
    case PID_NAK:
      printf("nak ");
      break;
    case PID_STALL:
      printf("stl ");
      break;
    default:
      printf("%x ", last_pid);
    }
    printf("\r\n");

    uint8_t* buffer;
    uint8_t  len;
    if (last_pid == PID_OUT) {
      printf("--out\r\n");
      buffer = out_ep_buffers[0].buffer;
      len    = out_ep_buffers[0].len;
      // Not expecting anything on ep0 at the moment
      if (len == 255) {
        printf("ERROR: Unexpected OUT!\r\n");
	return;
      }
      // Clear the pointers
      out_ep_buffers[0].buffer = NULL;
      out_ep_buffers[0].len = 255;
    } else if (last_pid == PID_SETUP) {
      printf("--setup\r\n");
      buffer = &(_setup_packet[0]);
      len = sizeof(_setup_packet);
    } else {
      printf("--???\r\n");
      printf("ERROR: Invalid PID!\r\n");
    }

    // Read in the data
    unsigned i = 0;
    while (!usb_ep_0_out_empty_read()) {
      uint8_t byte = usb_ep_0_out_head_read(); // Read the data
      usb_ep_0_out_head_write(0);              // Push the FIFO forward by one
      if (i < len) {
        buffer[i] = byte;
      }
      i++;
    }
    if (i < len) {
      printf("ERROR: short packet!\r\n");
    }
    if (i > len) {
      printf("ERROR: long packet!\r\n");
    }

    // Tell tinyusb about it
    if (&(_setup_packet[0]) == buffer) {
      printf("ep0 out got setup g:%u w:%u ", i, len);
      dcd_event_setup_received(0, buffer, false);
    } else {
      printf("ep0 out g:%u w:%u ", i, len);
      ep0_xfer_complete(i);
    }
    for (uint8_t j = 0; j < i; j++) {
      printf("%x ", (unsigned)(buffer[j]));
    }
    printf("\r\n");
    usb_ep_0_out_ev_pending_write(0xff);
  }

}

bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  (void) rhport;

  uint8_t const dir   = edpt_dir(ep_addr);
  uint8_t const epnum = edpt_number(ep_addr);

  printf("start xfer d:%s e:%u l:%u b:%p ", (dir == TUSB_DIR_OUT) ? "o" : "i", epnum, total_bytes, buffer);
  if ( dir == TUSB_DIR_OUT ) {
    // Endpoint is in use?
    if (out_ep_buffers[epnum].buffer != NULL) {
      printf("oep.buffer != NULL\r\n");
      return false;
    }
    if (out_ep_buffers[epnum].len != 255) {
      printf("oep.len != 255\r\n");
      return false;
    }
    out_ep_buffers[epnum].buffer = buffer;
    out_ep_buffers[epnum].len = total_bytes;
    printf(" ..\r\n");
  } else if ( dir == TUSB_DIR_IN ) {
    // Endpoint is in use?
    if (!usb_ep_0_in_ev_pending_read()) {
      printf("iep not pending\r\n");
      return false;
    }
    if (in_ep_buffers[epnum].len != 255) {
      printf("iep.len != 255\r\n");
      return false;
    }
    if (!usb_ep_0_in_empty_read()) {
      printf("not empty\r\n");
      return false;
    }

    // FIXME: Check total_bytes...

    // Push the data into the outgoing FIFO
    for(uint16_t i = 0; i < total_bytes; i++) {
      usb_ep_0_in_head_write(buffer[i]);
    }
    in_ep_buffers[0].len = total_bytes;
    usb_ep_0_in_ev_pending_write(0xff);
    printf(" ..\r\n");
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
  printf("stall %x\r\n", ep_addr);
  //dcd_edpt_xfer(0, 0, _setup_packet, sizeof(_setup_packet));
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
  puts("busy\n");
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
