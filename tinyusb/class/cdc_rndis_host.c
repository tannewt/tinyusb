/**************************************************************************/
/*!
    @file     cdc_rndis_host.c
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
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    This file is part of the tinyusb stack.
*/
/**************************************************************************/

#include "tusb_option.h"

#if (MODE_HOST_SUPPORTED && TUSB_CFG_HOST_CDC && TUSB_CFG_HOST_CDC_RNDIS)

#define _TINY_USB_SOURCE_FILE_

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "common/common.h"
#include "cdc_host.h"
#include "cdc_rndis_host.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
static uint8_t msg_notification[TUSB_CFG_HOST_DEVICE_MAX][8] TUSB_CFG_ATTR_USBRAM;

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
static tusb_error_t rndis_body_subtask(void);

//--------------------------------------------------------------------+
// IMPLEMENTATION
//--------------------------------------------------------------------+

// To enable the TASK_ASSERT style (quick return on false condition) in a real RTOS, a task must act as a wrapper
// and is used mainly to call subtasks. Within a subtask return statement can be called freely, the task with
// forever loop cannot have any return at all.
OSAL_TASK_FUNCTION(cdch_rndis_task) (void* p_task_para)
{
  OSAL_TASK_LOOP_BEGIN

  rndis_body_subtask();

  OSAL_TASK_LOOP_END
}

static tusb_error_t rndis_body_subtask(void)
{
  static uint8_t relative_addr;

  OSAL_SUBTASK_BEGIN

  for (relative_addr = 0; relative_addr < TUSB_CFG_HOST_DEVICE_MAX; relative_addr++)
  {

  }

  osal_task_delay(100);

  OSAL_SUBTASK_END
}



tusb_error_t rndish_open_subtask(uint8_t dev_addr, cdch_data_t *p_cdc)
{
  tusb_error_t error;

  static rndis_msg_initialize_t msg_init =
  {
      .type          = RNDIS_MSG_INITIALIZE,
      .length        = sizeof(rndis_msg_initialize_t),
      .request_id    = 1, // TODO should use some magic number
      .major_version = 1,
      .minor_version = 0,
      .max_xfer_size = 0x4000 // TODO mimic windows
  };

  static tusb_control_request_t control_request =
  {
      .bmRequestType = {
          .direction = TUSB_DIR_HOST_TO_DEV,
          .type      = TUSB_REQUEST_TYPE_CLASS,
          .recipient = TUSB_REQUEST_RECIPIENT_INTERFACE
      },
      .bRequest = SEND_ENCAPSULATED_COMMAND,
      .wLength   = sizeof(rndis_msg_initialize_t)
//      .wIndex   = p_cdc->interface_number,
  };

  OSAL_SUBTASK_BEGIN

  control_request.wIndex   = p_cdc->interface_number;

  OSAL_SUBTASK_INVOKED_AND_WAIT ( usbh_control_xfer_subtask(dev_addr, &control_request, (uint8_t*)&msg_init), error ) ;

  if ( tusbh_cdc_rndis_mounted_cb )
  {
    tusbh_cdc_rndis_mounted_cb(dev_addr);
  }

  OSAL_SUBTASK_END
}

#endif