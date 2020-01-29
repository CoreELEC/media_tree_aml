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

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/bitrev.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include "avl_bsp.h"

AVL_semaphore AVL_IBSP_I2C_sem;

struct i2c_adapter *AVL_IBSP_I2C_Adapter(AVL_uint16 uiSlaveAddr, struct i2c_adapter *setPtr)
{
  static struct i2c_adapter *my_i2c_adapter[8] = {
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL};
  if (setPtr != NULL)
  {
    my_i2c_adapter[uiSlaveAddr >> 8] = setPtr;
  }
  return my_i2c_adapter[uiSlaveAddr >> 8];
}

AVL_uint32 AVL_IBSP_Initialize(void)
{
  return (0);
}

AVL_uint32 AVL_IBSP_Reset()
{
  return (0);
}

AVL_uint32 AVL_IBSP_Delay(AVL_uint32 uiDelay_ms)
{
  //msleep(uiDelay_ms);
  //https://www.kernel.org/doc/Documentation/timers/timers-howto.txt
  usleep_range(uiDelay_ms * 1000, uiDelay_ms * 2000);
  return (0);
}

AVL_uint32 AVL_IBSP_I2C_Read(AVL_uint16 uiSlaveAddr, AVL_puchar pucBuff, AVL_puint16 puiSize)
{
  int ret;
  struct i2c_msg msg = {
      .addr = (uiSlaveAddr & 0xff),
      .flags = I2C_M_RD,
      .len = *puiSize,
      .buf = pucBuff};
  ret = i2c_transfer(AVL_IBSP_I2C_Adapter(uiSlaveAddr, NULL), &msg, 1);
  if (ret == 1)
  {
    ret = 0;
  }
  else
  {
    dev_warn(&(AVL_IBSP_I2C_Adapter(uiSlaveAddr, NULL)->dev), "%s: i2c rd failed=%d "
                                                              "len=%d\n",
             KBUILD_BASENAME, ret, *puiSize);
    ret = -EREMOTEIO;
  }
  return ret;
}

AVL_uint32 AVL_IBSP_I2C_Write(AVL_uint16 uiSlaveAddr, AVL_puchar pucBuff, AVL_puint16 puiSize)
{
  int ret;
  struct i2c_msg msg = {
      .addr = (uiSlaveAddr & 0xff),
      .flags = 0,
      .buf = pucBuff,
      .len = *puiSize};
  ret = i2c_transfer(AVL_IBSP_I2C_Adapter(uiSlaveAddr, NULL), &msg, 1);
  if (ret == 1)
  {
    ret = 0;
  }
  else
  {
    dev_warn(&(AVL_IBSP_I2C_Adapter(uiSlaveAddr, NULL)->dev), "%s: i2c wr failed=%d "
                                                              "len=%d\n",
             KBUILD_BASENAME, ret, *puiSize);
    ret = -EREMOTEIO;
  }
  return ret;
}

AVL_uint32 AVL_IBSP_Dispose(void)
{
  return (0);
}

AVL_uint32 AVL_IBSP_InitSemaphore(AVL_psemaphore pSemaphore)
{
  return (0);
}

AVL_uint32 AVL_IBSP_ReleaseSemaphore(AVL_psemaphore pSemaphore)
{
  return (0);
}

AVL_uint32 AVL_IBSP_WaitSemaphore(AVL_psemaphore pSemaphore)
{
  return (0);
}

EXPORT_SYMBOL_GPL(AVL_IBSP_I2C_Adapter);
EXPORT_SYMBOL_GPL(AVL_IBSP_Initialize);
EXPORT_SYMBOL_GPL(AVL_IBSP_Reset);
EXPORT_SYMBOL_GPL(AVL_IBSP_Delay);
EXPORT_SYMBOL_GPL(AVL_IBSP_I2C_Read);
EXPORT_SYMBOL_GPL(AVL_IBSP_I2C_Write);
EXPORT_SYMBOL_GPL(AVL_IBSP_Dispose);
EXPORT_SYMBOL_GPL(AVL_IBSP_InitSemaphore);
EXPORT_SYMBOL_GPL(AVL_IBSP_ReleaseSemaphore);
EXPORT_SYMBOL_GPL(AVL_IBSP_WaitSemaphore);
EXPORT_SYMBOL_GPL(AVL_IBSP_I2C_sem);

MODULE_DESCRIPTION("Board support driver for Availink demod/tuner drivers");
MODULE_AUTHOR("Availink, Inc. (opensource@availink.com)");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
