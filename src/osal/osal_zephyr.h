/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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

#ifndef TUSB_OSAL_ZEPHYR_H_
#define TUSB_OSAL_ZEPHYR_H_

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct k_sem osal_semaphore_def_t;
typedef struct k_mutex osal_mutex_def_t;
typedef struct k_msgq osal_queue_def_t;

typedef osal_semaphore_def_t* osal_semaphore_t;
typedef osal_mutex_def_t* osal_mutex_t;
typedef osal_queue_def_t* osal_queue_t;

// _int_set is not used with an RTOS
#define OSAL_QUEUE_DEF(_int_set, _name, _depth, _type) \
  K_MSGQ_DEFINE(_name, sizeof(_type), _depth, sizeof(size_t) /* alignment */)

//--------------------------------------------------------------------+
// TASK API
//--------------------------------------------------------------------+

TU_ATTR_ALWAYS_INLINE static inline k_timeout_t _osal_ms2timeout(uint32_t msec) {
  if ( msec == OSAL_TIMEOUT_WAIT_FOREVER ) return K_FOREVER;
  if ( msec == 0 ) return K_NO_WAIT;

  return K_MSEC(msec);
}

TU_ATTR_ALWAYS_INLINE static inline void osal_task_delay(uint32_t msec) {
  k_msleep(msec);
}

//--------------------------------------------------------------------+
// Semaphore API
//--------------------------------------------------------------------+

TU_ATTR_ALWAYS_INLINE static inline osal_semaphore_t osal_semaphore_create(osal_semaphore_def_t *semdef) {
  return k_sem_init(semdef, 0, K_SEM_MAX_LIMIT);
}

TU_ATTR_ALWAYS_INLINE static inline bool osal_semaphore_delete(osal_semaphore_t semd_hdl) {
  k_sem_reset(semd_hdl);
  return true;
}

TU_ATTR_ALWAYS_INLINE static inline bool osal_semaphore_post(osal_semaphore_t sem_hdl, bool in_isr) {
  k_sem_give(sem_hdl);
}

TU_ATTR_ALWAYS_INLINE static inline bool osal_semaphore_wait(osal_semaphore_t sem_hdl, uint32_t msec) {
  return k_sem_take(sem_hdl, _osal_ms2timeout(msec));
}

TU_ATTR_ALWAYS_INLINE static inline void osal_semaphore_reset(osal_semaphore_t const sem_hdl) {
  k_sem_reset(sem_hdl);
}

//--------------------------------------------------------------------+
// MUTEX API (priority inheritance)
//--------------------------------------------------------------------+

TU_ATTR_ALWAYS_INLINE static inline osal_mutex_t osal_mutex_create(osal_mutex_def_t *mdef) {
  return k_mutex_init(mdef);
}

TU_ATTR_ALWAYS_INLINE static inline bool osal_mutex_delete(osal_mutex_t mutex_hdl) {
  return true;
}

TU_ATTR_ALWAYS_INLINE static inline bool osal_mutex_lock(osal_mutex_t mutex_hdl, uint32_t msec) {
  return k_mutex_lock(mutex_hdl, _osal_ms2timeout(msec));
}

TU_ATTR_ALWAYS_INLINE static inline bool osal_mutex_unlock(osal_mutex_t mutex_hdl) {
  return k_mutex_unlock(mutex_hdl);
}

//--------------------------------------------------------------------+
// QUEUE API
//--------------------------------------------------------------------+

TU_ATTR_ALWAYS_INLINE static inline osal_queue_t osal_queue_create(osal_queue_def_t* qdef) {
  // Statically init already by K_MSGQ_DEFINE
  return qdef;
}

TU_ATTR_ALWAYS_INLINE static inline bool osal_queue_delete(osal_queue_t qhdl) {
  return true;
}

TU_ATTR_ALWAYS_INLINE static inline bool osal_queue_receive(osal_queue_t qhdl, void* data, uint32_t msec) {
  return k_msgq_get(qhdl, data, _osal_ms2timeout(msec)) == 0;
}

TU_ATTR_ALWAYS_INLINE static inline bool osal_queue_send(osal_queue_t qhdl, void const *data, bool in_isr) {
  k_timeout_t timeout = in_isr ? K_NO_WAIT : K_FOREVER;
  return k_msgq_put(qhdl, data, timeout) == 0;
}

TU_ATTR_ALWAYS_INLINE static inline bool osal_queue_empty(osal_queue_t qhdl) {
  return k_msgq_num_used_get(qhdl) == 0;
}

#ifdef __cplusplus
}
#endif

#endif
