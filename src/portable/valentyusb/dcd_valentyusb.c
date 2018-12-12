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

enum ENDPOINT_RESPONSE {
  ENDPOINT_ACK = 0b00,
  ENDPOINT_NAK = 0b01,
  ENDPOINT_STA = 0b11,
  ENDPOINT_IGN = 0b10,
};

enum TOK {
                //XX01
  TOK_OUT     = 0b00,
  TOK_SOF     = 0b01,
  TOK_IN      = 0b10,
  TOK_SETUP   = 0b11,
};

void printf_tok(enum TOK tok) {
    printf("tok ");
    switch(tok) {
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
      printf("%x ", tok);
    }
}

static uint8_t _setup_packet[8];

struct {
  uint8_t* buffer;
  uint8_t  len;
} ep_buffers[3];

typedef struct {
  // Control functions
  void    (*set_response)(uint8_t);
  uint8_t (*get_response)(void);
  uint8_t (*pending_read)(void);
  void    (*pending_clear)(uint8_t);
  // Input FIFO
  uint8_t (*d2h_empty)(void);
  void    (*d2h_push)(uint8_t);
  // Output FIFO
  uint8_t (*h2d_empty)(void);
  uint8_t (*h2d_read)(void);
  void    (*h2d_pop)(uint8_t);
} ep_func_t;

ep_func_t ep_funcs[] = {
  // Endpoint 0 -- bidirectional
#ifdef CSR_USB_EP_0_EV_STATUS_ADDR
  {
    // Control functions
    .set_response  = &usb_ep_0_respond_write,
    .get_response  = &usb_ep_0_respond_read,
    .pending_read  = &usb_ep_0_ev_pending_read,
    .pending_clear = &usb_ep_0_ev_pending_write,

    // Input FIFO
#ifdef CSR_USB_EP_0_IBUF_HEAD_ADDR
    .d2h_empty     = &usb_ep_0_ibuf_empty_read,
    .d2h_push      = &usb_ep_0_ibuf_head_write,
#else
    .d2h_empty     = NULL,
    .d2h_push      = NULL,
#endif

    // Output FIFO
#ifdef CSR_USB_EP_0_OBUF_HEAD_ADDR
    .h2d_empty     = &usb_ep_0_obuf_empty_read,
    .h2d_read      = &usb_ep_0_obuf_head_read,
    .h2d_pop       = &usb_ep_0_obuf_head_write,
#else
    .h2d_empty     = NULL,
    .h2d_read      = NULL,
    .h2d_pop       = NULL,
#endif
  },
#endif
  // Endpoint 1
#ifdef CSR_USB_EP_1_EV_STATUS_ADDR
  {
    // Control functions
    .set_response  = &usb_ep_1_respond_write,
    .get_response  = &usb_ep_1_respond_read,
    .pending_read  = &usb_ep_1_ev_pending_read,
    .pending_clear = &usb_ep_1_ev_pending_write,

    // Input FIFO
#ifdef CSR_USB_EP_1_IBUF_HEAD_ADDR
    .d2h_empty     = &usb_ep_1_ibuf_empty_read,
    .d2h_push      = &usb_ep_1_ibuf_head_write,
#else
    .d2h_empty     = NULL,
    .d2h_push      = NULL,
#endif

    // Output FIFO
#ifdef CSR_USB_EP_1_OBUF_HEAD_ADDR
    .h2d_empty     = &usb_ep_1_obuf_empty_read,
    .h2d_read      = &usb_ep_1_obuf_head_read,
    .h2d_pop       = &usb_ep_1_obuf_head_write,
#else
    .h2d_empty     = NULL,
    .h2d_read      = NULL,
    .h2d_pop       = NULL,
#endif
  },
#endif
  // Endpoint 2
#ifdef CSR_USB_EP_2_EV_STATUS_ADDR
  {
    // Control functions
    .set_response  = &usb_ep_2_respond_write,
    .get_response  = &usb_ep_2_respond_read,
    .pending_read  = &usb_ep_2_ev_pending_read,
    .pending_clear = &usb_ep_2_ev_pending_write,

    // Input FIFO
#ifdef CSR_USB_EP_2_IBUF_HEAD_ADDR
    .d2h_empty     = &usb_ep_2_ibuf_empty_read,
    .d2h_push      = &usb_ep_2_ibuf_head_write,
#else
    .d2h_empty     = NULL,
    .d2h_push      = NULL,
#endif

    // Output FIFO
#ifdef CSR_USB_EP_2_OBUF_HEAD_ADDR
    .h2d_empty     = &usb_ep_2_obuf_empty_read,
    .h2d_read      = &usb_ep_2_obuf_head_read,
    .h2d_pop       = &usb_ep_2_obuf_head_write,
#else
    .h2d_empty     = NULL,
    .h2d_read      = NULL,
    .h2d_pop       = NULL,
#endif
  },
#endif
#ifdef CSR_USB_EP_3_EV_STATUS_ADDR
  // Endpoint 2 -- Out (host to device) endpoint
  {
    // Control functions
    .set_response  = &usb_ep_3_respond_write,
    .get_response  = &usb_ep_3_respond_read,
    .pending_read  = &usb_ep_3_ev_pending_read,
    .pending_clear = &usb_ep_3_ev_pending_write,

    // Input FIFO
#ifdef CSR_USB_EP_3_IBUF_HEAD_ADDR
    .d2h_empty     = &usb_ep_3_ibuf_empty_read,
    .d2h_push      = &usb_ep_3_ibuf_head_write,
#else
    .d2h_empty     = NULL,
    .d2h_push      = NULL,
#endif

    // Output FIFO
#ifdef CSR_USB_EP_3_OBUF_HEAD_ADDR
    .h2d_empty     = &usb_ep_3_obuf_empty_read,
    .h2d_read      = &usb_ep_3_obuf_head_read,
    .h2d_pop       = &usb_ep_3_obuf_head_write,
#else
    .h2d_empty     = NULL,
    .h2d_read      = NULL,
    .h2d_pop       = NULL,
#endif
  },
#endif
};

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
  for (unsigned ep = 0; ep < (sizeof(ep_buffers)/sizeof(ep_buffers[0])); ep++) {
    ep_buffers[ep].buffer = NULL;
    ep_buffers[ep].len = 255;
  }

  // Allow the USB to start
  usb_pullup_out_write(1);

  printf("init\n");

  // Prepare for setup packet
  for (unsigned ep = 0; ep < (sizeof(ep_funcs)/sizeof(ep_funcs[0])); ep++) {
    ep_funcs[ep].pending_clear(0xff);
  }
  ep_funcs[0].set_response(ENDPOINT_ACK);
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
  printf("edpt_open d:%s e:%u\r\n", (dir == TUSB_DIR_OUT) ? "o" : "i", epnum, dir);

  ep_funcs[epnum].set_response(ENDPOINT_NAK);
  ep_buffers[epnum].buffer = NULL;
  ep_buffers[epnum].len = 255;

  return true;
}

void dcd_poll(uint8_t rhport)
{
  for (unsigned ep = 0; ep < (sizeof(ep_funcs)/sizeof(ep_funcs[0])); ep++) {
    ep_func_t epf = ep_funcs[ep];
    if (!epf.pending_read()) {
      continue;
    }

    // FIXME: last_tok is global?
    enum TOK last_tok = usb_last_tok_read();
    uint8_t* buffer = NULL;
    uint8_t  len    = 255;

    switch(last_tok) {
    case TOK_SETUP:
      buffer = &(_setup_packet[0]);
      len = sizeof(_setup_packet);
      break;
    case TOK_OUT:
    case TOK_IN:
      buffer = ep_buffers[ep].buffer;
      len    = ep_buffers[ep].len;
      break;
    case TOK_SOF:
      printf("ep%u: SOF!", ep);
      return;
    }
    // Clear the pointers
    ep_buffers[ep].buffer = NULL;
    ep_buffers[ep].len = 255;

    printf("ep%u: ", ep);
    printf_tok(last_tok);
    printf("w:%u ", len);

    if (len == 255) {
      printf(" -- ERR unexpected");
      goto fail;
    }

    uint8_t t = 255;
    switch(last_tok) {
    case TOK_IN:
      if (epf.d2h_empty == NULL) {
        printf("ERR - No input funcs!");
        goto fail;
      }
      // Check FIFO has been emptied
      if (!epf.d2h_empty()) {
        printf("not empty!");
      } else {
        t = len;
      }
      break;
    case TOK_SETUP:
    case TOK_OUT:
      if (epf.h2d_empty == NULL) {
        printf("ERR - No output funcs!\r\n");
        goto fail;
      }
      // Copy the data out of the FIFO
      t = 0;
      while (!(epf.h2d_empty())) {
        uint8_t byte = epf.h2d_read(); // Read the data
        epf.h2d_pop(0);                // Push the FIFO forward by one
        printf("%x ", byte);
        if (t < len) {
          buffer[t] = byte;
        }
        t++;
      }

      break;
    case TOK_SOF:
      // NOT_REACHED();
      break;
    }

    printf("g:%u ", t);

    if (t < len) {
      printf("-- ERR short!");
    }
    if (t > len) {
      printf("-- ERR long!");
    }

    epf.set_response(ENDPOINT_NAK);
    epf.pending_clear(0xff);
    printf("!\r\n");

    switch(last_tok) {
    case TOK_SETUP:
      dcd_event_setup_received(0, buffer, false);
      break;
    case TOK_IN:
      ep = ep | TUSB_DIR_IN_MASK;
      /* fall through */
    case TOK_OUT:
      dcd_event_xfer_complete(0, ep, len, XFER_RESULT_SUCCESS, false);
      break;
    case TOK_SOF:
      // NOT_REACHED();
      break;
    }
    continue;

fail:
    epf.set_response(ENDPOINT_STA);
    epf.pending_clear(0xff);
    printf(":-(\r\n");
  }

}

bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  (void) rhport;

  uint8_t const dir   = edpt_dir(ep_addr);
  uint8_t const epnum = edpt_number(ep_addr);

  printf("start xfer d:%s e:%u l:%u b:%p ", (dir == TUSB_DIR_OUT) ? "o" : "i", epnum, total_bytes, buffer);

  // Endpoint is in use?
  if (ep_funcs[epnum].pending_read()) {
    printf("pending packet!\r\n");
    return false;
  }
  if (ep_buffers[epnum].buffer != NULL) {
    printf("ep.buffer != NULL (%p)\r\n", ep_buffers[epnum].buffer);
    return false;
  }
  if (ep_buffers[epnum].len != 255) {
    printf("ep.len != 255 (%u)\r\n", ep_buffers[epnum].len);
    return false;
  }
  if (ep_funcs[epnum].get_response() != ENDPOINT_NAK) {
    printf("ep.state != NAK (%x)\r\n", ep_funcs[epnum].get_response());
    return false;
  }

  if ( dir == TUSB_DIR_OUT ) {
    if (!ep_funcs[epnum].h2d_empty()) {
      printf("not empty\r\n");
      return false;
    }
    ep_buffers[epnum].buffer = buffer;
    ep_buffers[epnum].len = total_bytes;
  } else if ( dir == TUSB_DIR_IN ) {
    if (!ep_funcs[epnum].d2h_empty()) {
      printf("not empty\r\n");
      return false;
    }
    // FIXME: Check total_bytes...

    // Push the data into the outgoing FIFO
    ep_funcs[epnum].set_response(ENDPOINT_NAK);
    for(uint16_t i = 0; i < total_bytes; i++) {
      ep_funcs[epnum].d2h_push(buffer[i]);
    }
    ep_buffers[epnum].buffer = NULL;
    ep_buffers[epnum].len = total_bytes;

  } else {
    return false;
  }

  ep_funcs[epnum].set_response(ENDPOINT_ACK);
  printf(" ..\r\n");
  return true;
}

bool dcd_edpt_stalled (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  uint8_t const epnum = edpt_number(ep_addr);
  return (ep_funcs[epnum].get_response() == ENDPOINT_STA);
}

void dcd_edpt_set_response(uint8_t rhport, uint8_t ep_addr, enum ENDPOINT_RESPONSE e)
{
  (void) rhport;
  uint8_t const epnum = edpt_number(ep_addr);
  ep_funcs[epnum].set_response(e);
  printf("stall %x\r\n", ep_addr);
}

void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
  dcd_edpt_set_response(rhport, ep_addr, ENDPOINT_STA);
}

void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
  printf("un");
  dcd_edpt_set_response(rhport, ep_addr, ENDPOINT_NAK);
}

bool dcd_edpt_busy (uint8_t rhport, uint8_t ep_addr)
{
  puts("busy\n");
  (void) rhport;

  uint8_t const epnum = edpt_number(ep_addr);
  return (ep_buffers[epnum].len != 255) || (ep_funcs[epnum].pending_read());
}

/*------------------------------------------------------------------*/
