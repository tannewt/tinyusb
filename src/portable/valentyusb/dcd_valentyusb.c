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
} out_ep_buffers[3];

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

  out_ep_buffers[2].buffer = NULL;
  out_ep_buffers[2].len = 255;

  in_ep_buffers[0].len = 255;
  in_ep_buffers[1].len = 255;

  // Allow the USB to start
  usb_pullup_out_write(1);

  printf("init\n");

  // Prepare for setup packet
  //dcd_edpt_xfer(0, 0, _setup_packet, sizeof(_setup_packet));
  usb_ep_0_out_ev_pending_write(0xff);
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
  printf("set_config #:%u\r\n", config_num);
}

/*------------------------------------------------------------------*/
/* DCD Endpoint port
 *------------------------------------------------------------------*/

bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * desc_edpt)
{
  (void) rhport;
  (void) desc_edpt;

  uint8_t const epnum = edpt_number(desc_edpt->bEndpointAddress);
  uint8_t const dir   = edpt_dir(desc_edpt->bEndpointAddress);
  printf("edpt_open d:%u #:%u\r\n", epnum, dir);

  return true;
}

enum TOK {
                //XX01
  TOK_OUT     = 0b00,
  TOK_SOF     = 0b01,
  TOK_IN      = 0b10,
  TOK_SETUP   = 0b11,
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

  if (usb_ep_1_in_ev_pending_read()) {
    uint8_t len = in_ep_buffers[1].len;
    in_ep_buffers[1].len = 255;
    if (len != 255) {
      if (!usb_ep_1_in_empty_read()) {
        printf("not empty !?\r\n");
      }
      printf("dcd_poll ep1  in complete %d\r\n", len);
      dcd_event_xfer_complete(0, 1 | TUSB_DIR_IN_MASK, len, XFER_RESULT_SUCCESS, false);
    }
  }

  // Endpoint zero has data.
  if (usb_ep_0_out_ev_pending_read()) {
    printf("\r\n");
    enum TOK last_tok = usb_last_tok_read();
    printf("tok ");
    switch(last_tok) {
    case TOK_OUT:
      printf("out ");
      break;
    case TOK_IN:
      printf("in ");
      break;
    case TOK_SOF:
      printf("sof ");
      break;
    case TOK_SETUP:
      printf("set ");
      break;
    default:
      printf("%x ", last_tok);
    }
    printf("\r\n");

    uint8_t* buffer;
    uint8_t  len;
    if (last_tok == TOK_OUT) {
      buffer = out_ep_buffers[0].buffer;
      len    = out_ep_buffers[0].len;
      // Not expecting anything on ep0 at the moment
      if (len == 255) {
        printf("ERROR: OUT!\r\n");
	return;
      }
      // Clear the pointers
      out_ep_buffers[0].buffer = NULL;
      out_ep_buffers[0].len = 255;
    } else if (last_tok == TOK_SETUP) {
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
      printf("ERROR: short!\r\n");
    }
    if (i > len) {
      printf("ERROR: long!\r\n");
    }

    // Tell tinyusb about it
    if (&(_setup_packet[0]) == buffer) {
      printf("ep0 set g:%u w:%u ", i, len);
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

  if (usb_ep_2_out_ev_pending_read()) {
    uint8_t* buffer = out_ep_buffers[2].buffer;
    uint8_t  len    = out_ep_buffers[2].len;

    // Not expecting anything on ep0 at the moment
    if (len == 255) {
      printf("ERROR: OUT!\r\n");
      return;
    }
    // Clear the pointers
    out_ep_buffers[2].buffer = NULL;
    out_ep_buffers[2].len    = 255;

    // Read in the data
    unsigned i = 0;
    while (!usb_ep_2_out_empty_read()) {
      uint8_t byte = usb_ep_2_out_head_read(); // Read the data
      usb_ep_2_out_head_write(0);              // Push the FIFO forward by one
      if (i < len) {
        buffer[i] = byte;
      }
      i++;
    }
    if (i < len) {
      printf("ERROR: short!\r\n");
    }
    if (i > len) {
      printf("ERROR: long!\r\n");
    }

    printf("ep2 out g:%u w:%u ", i, len);
    // Tell tinyusb about it
    dcd_event_xfer_complete(0, 2, len, XFER_RESULT_SUCCESS, false);
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

  uint8_t const dir   = edpt_dir(ep_addr);
  uint8_t const epnum = edpt_number(ep_addr);

  if ( dir == TUSB_DIR_OUT ) {
    if (epnum == 0) {
      return usb_ep_0_out_stall_read() == 0;
    } else if (epnum == 1) {
      //return usb_ep_1_out_stall_read() == 0;
    } else if (epnum == 2) {
      return usb_ep_2_out_stall_read() == 0;
    }
  } else if ( dir == TUSB_DIR_IN ) {
    if (epnum == 0) {
      return usb_ep_0_in_stall_read() == 0;
    } else if (epnum == 1) {
      return usb_ep_1_in_stall_read() == 0;
    } else if (epnum == 2) {
      //return usb_ep_2_in_stall_read() == 0;
    }
  } else {
    printf("stall error\r\n");
  }
  return false;
}

void dcd_edpt_stall_set (uint8_t rhport, uint8_t ep_addr, uint8_t value)
{
  (void) rhport;
  uint8_t const dir   = edpt_dir(ep_addr);
  uint8_t const epnum = edpt_number(ep_addr);

  if ( dir == TUSB_DIR_OUT ) {
    printf("stall out %x\r\n", ep_addr);
    if (epnum == 0) {
      //usb_ep_0_out_stall_write(value);
    } else if (epnum == 1) {
      //usb_ep_1_out_stall_write(value);
    } else if (epnum == 2) {
      usb_ep_2_out_stall_write(value);
    }
  } else if ( dir == TUSB_DIR_IN ) {
    printf("stall in %x\r\n", ep_addr);
    if (epnum == 0) {
      usb_ep_0_in_stall_write(value);
    } else if (epnum == 1) {
      usb_ep_1_in_stall_write(value);
    } else if (epnum == 2) {
      //usb_ep_2_in_stall_write(value);
    }
  } else {
    printf("stall error\r\n");
  }
}

void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
  dcd_edpt_stall_set(rhport, ep_addr, 1);
}

void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
  printf("un");
  dcd_edpt_stall_set(rhport, ep_addr, 0);
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
