/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2021 Ha Thach (tinyusb.org) for Double Buffered
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if TUSB_OPT_HOST_ENABLED && CFG_TUSB_MCU == OPT_MCU_RP2040

#include "pico.h"
#include "rp2040_usb.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/structs/systick.h"
#include "pico/multicore.h"

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "osal/osal.h"

#include "host/hcd.h"
#include "host/usbh.h"

// In this port, rhport is the pin offset where into pins 0-32. DP is
// the lower numbered of the two pins.

static volatile uint32_t _idle_state;
static bool core1_active = false;
static volatile uint32_t _frame_number;

void __not_in_flash_func(core1_usb)(void) {
    // One pass through the loop is a USB frame.
    _frame_number = 0;

    pico_trace("core1 started\n");

    systick_hw->rvr = clock_get_hz(clk_sys) / 1000 - 1;
    systick_hw->csr = 1 << M0PLUS_SYST_CSR_CLKSOURCE_LSB |
                      M0PLUS_SYST_CSR_ENABLE_BITS;
    size_t clocks_per_usb_bit = clock_get_hz(clk_sys) / 12000000;

    gpio_set_dir(12, GPIO_OUT);
    gpio_put(12, false);
    while (true) {
        _idle_state = 0;
        gpio_put(12, true);
        // Send start of frame for each port.


        // Poll all interrupt endpoints that have an interval that is % == 0 of
        // the frame number.

        // Send queued transactions based on the FIFO.

        // Check that we have enough time for the next transaction.
        while (systick_hw->cvr > 80 * clocks_per_usb_bit) {

        }
        gpio_put(12, false);
        // Otherwise, just wait.
        _frame_number++;
        // Wait
        while ((systick_hw->csr & M0PLUS_SYST_CSR_COUNTFLAG_BITS) == 0) {}
    }
}

//--------------------------------------------------------------------+
// HCD API
//--------------------------------------------------------------------+
bool pio_init(uint8_t rhport)
{
    pico_trace("pio_init %d\n", rhport);
    // Connect to the core through the FIFO. We probably want to have
    // inited already because we can pick any set of pins.

    if (!core1_active) {
        multicore_reset_core1();
        // all USB task run in core1
        multicore_launch_core1(core1_usb);
        return true;
    }
    return false;
}

void pio_port_reset(uint8_t rhport)
{
    // TODO: Reset everything.
}

bool pio_port_connect_status(uint8_t rhport)
{
    return (_idle_state >> rhport) & 0x3;
}

tusb_speed_t pio_port_speed_get(uint8_t rhport)
{
    switch ((_idle_state >> rhport) & 0x3)
    {
        case 1: // Full speed has DP pulled up.
            return TUSB_SPEED_FULL;
        case 2: // Low speed has DM pulled up.
            return TUSB_SPEED_LOW;
        default:
            panic("Invalid speed\n");
            return TUSB_SPEED_INVALID;
    }
}

// Close all opened endpoint belong to this device
void pio_device_close(uint8_t rhport, uint8_t dev_addr)
{
}

uint32_t pio_frame_number(uint8_t rhport)
{
    // share a frame number between all ports
    return 0;
}

void pio_int_enable(uint8_t rhport)
{
    // TODO: Use the FIFO's SIO IRQ
    // irq_set_enabled(USBCTRL_IRQ, true);
}

void pio_int_disable(uint8_t rhport)
{
    // TODO: Use the FIFO's SIO IRQ
    // irq_set_enabled(USBCTRL_IRQ, false);
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

bool pio_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const * ep_desc)
{
    // TODO: Implement open.
    return false;
}

bool pio_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t * buffer, uint16_t buflen)
{
    (void) rhport;

    // TODO: xfer
    // Queue up the transfer if the FIFO to the other core isn't full.

    return false;
}

bool pio_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8])
{
    // TODO: Implement this first.
    return false;
}

bool pio_edpt_clear_stall(uint8_t dev_addr, uint8_t ep_addr)
{
    (void) dev_addr;
    (void) ep_addr;

    // TODO: Implement false.
    return false;
}

#endif
