/*
 * Availink AVL6261 DVB-S/S2/S2X demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (opensource@availink.com)
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License along
 *    with this program; if not, write to the Free Software Foundation, Inc.,
 *    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _AVL_BSP_H_
#define _AVL_BSP_H_

#include <linux/i2c.h>
#include <linux/dvb/frontend.h>
#include "AVL_Types.h"

#ifdef AVL_CPLUSPLUS
extern "C"
{
#endif

  extern AVL_semaphore gsemI2C;

  struct i2c_adapter *AVL_IBSP_I2C_Adapter(AVL_uint16 uiSlaveAddr, struct i2c_adapter *setPtr);

  AVL_uint32 AVL_IBSP_Initialize(void);

  AVL_uint32 AVL_IBSP_Reset(void);

  AVL_uint32 AVL_IBSP_Delay(AVL_uint32 uiDelay_ms);
  AVL_uint32 AVL_IBSP_I2C_Read(AVL_uint16 uiSlaveAddr, AVL_puchar pucBuff, AVL_puint16 puiSize);
  AVL_uint32 AVL_IBSP_I2C_Write(AVL_uint16 uiSlaveAddr, AVL_puchar pucBuff, AVL_puint16 puiSize);
  AVL_uint32 AVL_IBSP_Dispose(void);
  AVL_uint32 AVL_IBSP_InitSemaphore(AVL_psemaphore pSemaphore);
  AVL_uint32 AVL_IBSP_ReleaseSemaphore(AVL_psemaphore pSemaphore);
  AVL_uint32 AVL_IBSP_WaitSemaphore(AVL_psemaphore pSemaphore);
  AVL_uint32 AVL_SERIAL_Initialize(bool valid);
  void AVL_SERIAL_ByPassOn(void);
  void AVL_SERIAL_ByPassOff(void);

#ifdef AVL_CPLUSPLUS
}
#endif

#endif
