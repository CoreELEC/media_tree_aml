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

#include "AVL62X1_Internal.h"
#include "avl_bsp.h"

extern AVL_semaphore AVL_IBSP_I2C_sem;

AVL_uint16 DeChunk16_AVL62X1(const AVL_puchar pBuff)
{
  AVL_uint16 uiData = 0;
  uiData = pBuff[0];
  uiData = (AVL_uint16)(uiData << 8) + pBuff[1];

  return uiData;
}

AVL_uint32 DeChunk32_AVL62X1(const AVL_puchar pBuff)
{
  AVL_uint32 uiData = 0;
  uiData = pBuff[0];
  uiData = (uiData << 8) + pBuff[1];
  uiData = (uiData << 8) + pBuff[2];
  uiData = (uiData << 8) + pBuff[3];

  return uiData;
}

void Chunk16_AVL62X1(AVL_uint16 uidata, AVL_puchar pBuff)
{
  pBuff[0] = (AVL_uchar)(uidata >> 8);
  pBuff[1] = (AVL_uchar)(uidata & 0xff);

  return;
}

void Chunk32_AVL62X1(AVL_uint32 uidata, AVL_puchar pBuff)
{
  pBuff[0] = (AVL_uchar)(uidata >> 24);
  pBuff[1] = (AVL_uchar)(uidata >> 16);
  pBuff[2] = (AVL_uchar)(uidata >> 8);
  pBuff[3] = (AVL_uchar)(uidata);

  return;
}

void ChunkAddr_AVL62X1(AVL_uint32 uiaddr, AVL_puchar pBuff)
{
  pBuff[0] = (AVL_uchar)(uiaddr >> 16);
  pBuff[1] = (AVL_uchar)(uiaddr >> 8);
  pBuff[2] = (AVL_uchar)(uiaddr);

  return;
}

void Add32To64_AVL62X1(AVL62X1_uint64 *pSum, AVL_uint32 uiAddend)
{
  AVL_uint32 uiTemp = 0;

  uiTemp = pSum->uiLowWord;
  pSum->uiLowWord += uiAddend;
  pSum->uiLowWord &= 0xFFFFFFFF;

  if (pSum->uiLowWord < uiTemp)
  {
    pSum->uiHighWord++;
  }
}

AVL_uint32 Divide64_AVL62X1(AVL62X1_uint64 y, AVL62X1_uint64 x)
{
  AVL_uint32 uFlag = 0x0;
  AVL_uint32 uQuto = 0x0;
  AVL_uint32 i = 0;
  AVL_uint32 dividend_H = x.uiHighWord;
  AVL_uint32 dividend_L = x.uiLowWord;
  AVL_uint32 divisor_H = y.uiHighWord;
  AVL_uint32 divisor_L = y.uiLowWord;

  if (((divisor_H == 0x0) && (divisor_L == 0x0)) || (dividend_H / divisor_L))
  {
    return 0;
  }
  else if ((divisor_H == 0x0) && (dividend_H == 0x0))
  {
    return (dividend_L / divisor_L);
  }
  else
  {
    if (divisor_H != 0)
    {
      while (divisor_H)
      {
        dividend_L /= 2;
        if (dividend_H % 2)
        {
          dividend_L += 0x80000000;
        }
        dividend_H /= 2;

        divisor_L /= 2;
        if (divisor_H % 2)
        {
          divisor_L += 0x80000000;
        }
        divisor_H /= 2;
      }
    }
    for (i = 0; i <= 31; i++)
    {
      uFlag = (AVL_int32)dividend_H >> 31;
      dividend_H = (dividend_H << 1) | (dividend_L >> 31);
      dividend_L <<= 1;
      uQuto <<= 1;

      if ((dividend_H | uFlag) >= divisor_L)
      {
        dividend_H -= divisor_L;
        uQuto++;
      }
    }
    return uQuto;
  }
}

AVL_uint32 GreaterThanOrEqual64_AVL62X1(AVL62X1_uint64 a, AVL62X1_uint64 b)
{
  AVL_uint32 result = 0;

  if ((a.uiHighWord == b.uiHighWord) && (a.uiLowWord == b.uiLowWord))
  {
    result = 1;
  }
  if (a.uiHighWord > b.uiHighWord)
  {
    result = 1;
  }
  else if (a.uiHighWord == b.uiHighWord)
  {
    if (a.uiLowWord > b.uiLowWord)
    {
      result = 1;
    }
  }

  return result;
}

void Subtract64_AVL62X1(AVL62X1_uint64 *pA, AVL62X1_uint64 b)
{
  AVL62X1_uint64 a = {0, 0};
  AVL62X1_uint64 temp = {0, 0};

  a.uiHighWord = pA->uiHighWord;
  a.uiLowWord = pA->uiLowWord;

  temp.uiHighWord = a.uiHighWord - b.uiHighWord;
  if (a.uiLowWord >= b.uiLowWord)
  {
    temp.uiLowWord = a.uiLowWord - b.uiLowWord;
  }
  else
  {
    temp.uiLowWord = b.uiLowWord - a.uiLowWord;
    temp.uiHighWord >>= 1;
  }

  pA->uiHighWord = temp.uiHighWord;
  pA->uiLowWord = temp.uiLowWord;
}

void Multiply32_AVL62X1(AVL62X1_uint64 *pDst, AVL_uint32 m1, AVL_uint32 m2)
{
  pDst->uiLowWord = (m1 & 0xFFFF) * (m2 & 0xFFFF);
  pDst->uiHighWord = 0;

  AddScaled32To64_AVL62X1(pDst, (m1 >> 16) * (m2 & 0xFFFF));
  AddScaled32To64_AVL62X1(pDst, (m2 >> 16) * (m1 & 0xFFFF));

  pDst->uiHighWord += (m1 >> 16) * (m2 >> 16);
}

void AddScaled32To64_AVL62X1(AVL62X1_uint64 *pDst, AVL_uint32 a)
{
  AVL_uint32 saved = 0;

  saved = pDst->uiLowWord;
  pDst->uiLowWord += (a << 16);

  pDst->uiLowWord &= 0xFFFFFFFF;
  pDst->uiHighWord += ((pDst->uiLowWord < saved) ? 1 : 0) + (a >> 16);
}

AVL_uint32 Min32_AVL62X1(AVL_uint32 a, AVL_uint32 b)
{
  if (a < b)
  {
    return (a);
  }
  else
  {
    return (b);
  }
}

AVL_uint32 Max32_AVL62X1(AVL_uint32 a, AVL_uint32 b)
{
  if (a > b)
  {
    return (a);
  }
  else
  {
    return (b);
  }
}

AVL_ErrorCode AVL_II2C_Initialize(void)
{
  AVL_ErrorCode r = AVL_EC_OK;
  static AVL_uchar gI2CSem_inited = 0;
  if (0 == gI2CSem_inited)
  {
    gI2CSem_inited = 1;
    r = AVL_IBSP_InitSemaphore(&AVL_IBSP_I2C_sem);
  }
  return r;
}

AVL_ErrorCode II2C_Read_AVL62X1(AVL_uint16 uiSlaveAddr, AVL_uint32 uiOffset, AVL_puchar pucBuff, AVL_uint32 uiSize)
{
  AVL_ErrorCode r = AVL_EC_OK;
  AVL_uchar pucBuffTemp[3] = {0};
  AVL_uint16 us1 = 0;
  AVL_uint32 ui2 = 0;
  AVL_uint16 usSize = 0;

  r = AVL_IBSP_WaitSemaphore(&(AVL_IBSP_I2C_sem));
  if (AVL_EC_OK == r)
  {
    ChunkAddr_AVL62X1(uiOffset, pucBuffTemp);
    us1 = 3;
    r = AVL_IBSP_I2C_Write(uiSlaveAddr, pucBuffTemp, &us1);
    if (AVL_EC_OK == r)
    {
      usSize = uiSize;
      while (usSize > MAX_II2C_READ_SIZE)
      {
        us1 = MAX_II2C_READ_SIZE;
        r |= AVL_IBSP_I2C_Read(uiSlaveAddr, pucBuff + ui2, &us1);
        ui2 += MAX_II2C_READ_SIZE;
        usSize -= MAX_II2C_READ_SIZE;
      }

      if (0 != usSize)
      {
        r |= AVL_IBSP_I2C_Read(uiSlaveAddr, pucBuff + ui2, &usSize);
      }
    }
  }
  r |= AVL_IBSP_ReleaseSemaphore(&(AVL_IBSP_I2C_sem));

  return (r);
}

AVL_ErrorCode II2C_Read8_AVL62X1(AVL_uint16 uiSlaveAddr, AVL_uint32 uiAddr, AVL_puchar puiData)
{
  AVL_ErrorCode r = AVL_EC_OK;
  AVL_uchar Data = 0;

  r = II2C_Read_AVL62X1(uiSlaveAddr, uiAddr, &Data, 1);
  if (AVL_EC_OK == r)
  {
    *puiData = Data;
  }

  return (r);
}

AVL_ErrorCode II2C_Read16_AVL62X1(AVL_uint16 uiSlaveAddr, AVL_uint32 uiAddr, AVL_puint16 puiData)
{
  AVL_ErrorCode r = AVL_EC_OK;
  AVL_uchar pBuff[2] = {0};

  r = II2C_Read_AVL62X1(uiSlaveAddr, uiAddr, pBuff, 2);
  if (AVL_EC_OK == r)
  {
    *puiData = DeChunk16_AVL62X1(pBuff);
  }

  return (r);
}

AVL_ErrorCode II2C_Read32_AVL62X1(AVL_uint16 uiSlaveAddr, AVL_uint32 uiAddr, AVL_puint32 puiData)
{
  AVL_ErrorCode r = AVL_EC_OK;
  AVL_uchar pBuff[4] = {0};

  r = II2C_Read_AVL62X1(uiSlaveAddr, uiAddr, pBuff, 4);
  if (AVL_EC_OK == r)
  {
    *puiData = DeChunk32_AVL62X1(pBuff);
  }

  return (r);
}

AVL_ErrorCode II2C_ReadDirect_AVL62X1(AVL_uint16 uiSlaveAddr, AVL_puchar pucBuff, AVL_uint16 uiSize)
{
  AVL_ErrorCode r = AVL_EC_OK;
  AVL_uint16 ui1 = 0;
  AVL_uint32 ui2 = 0;
  AVL_uint16 iSize = 0;

  r = AVL_IBSP_WaitSemaphore(&(AVL_IBSP_I2C_sem));
  if (AVL_EC_OK == r)
  {
    iSize = uiSize;
    while (iSize > MAX_II2C_READ_SIZE)
    {
      ui1 = MAX_II2C_READ_SIZE;
      r |= AVL_IBSP_I2C_Read(uiSlaveAddr, pucBuff + ui2, &ui1);
      ui2 += MAX_II2C_READ_SIZE;
      iSize -= MAX_II2C_READ_SIZE;
    }

    if (0 != iSize)
    {
      r |= AVL_IBSP_I2C_Read(uiSlaveAddr, pucBuff + ui2, &iSize);
    }
  }
  r |= AVL_IBSP_ReleaseSemaphore(&(AVL_IBSP_I2C_sem));

  return (r);
}

AVL_ErrorCode II2C_WriteDirect_AVL62X1(AVL_uint16 uiSlaveAddr, AVL_puchar pucBuff, AVL_uint16 uiSize)
{
  AVL_ErrorCode r = AVL_EC_OK;
  AVL_uint16 ui1 = 0;
  AVL_uint32 ui2 = 0;
  AVL_uint32 uTemp = 0;
  AVL_uint32 iSize = 0;

  r = AVL_IBSP_WaitSemaphore(&(AVL_IBSP_I2C_sem));
  if (AVL_EC_OK == r)
  {
    iSize = uiSize;
    uTemp = (MAX_II2C_WRITE_SIZE - 3) & 0xfffe;
    while (iSize > uTemp)
    {
      ui1 = uTemp;
      r |= AVL_IBSP_I2C_Write(uiSlaveAddr, pucBuff + ui2, &ui1);
      ui2 += uTemp;
      iSize -= uTemp;
    }
    ui1 = iSize;
    r |= AVL_IBSP_I2C_Write(uiSlaveAddr, pucBuff + ui2, &ui1);
    ui2 += iSize;
  }
  r |= AVL_IBSP_ReleaseSemaphore(&(AVL_IBSP_I2C_sem));

  return (r);
}

AVL_ErrorCode II2C_Write_AVL62X1(AVL_uint16 uiSlaveAddr, AVL_puchar pucBuff, AVL_uint32 uiSize)
{
  AVL_ErrorCode r = AVL_EC_OK;
  AVL_uchar pucBuffTemp[5] = {0};
  AVL_uint16 ui1 = 0;
  AVL_uint32 ui2 = 0;
  AVL_uint16 uTemp = 0;
  AVL_uint32 iSize = 0;
  AVL_uint32 uAddr = 0;

  if (uiSize < 3)
  {
    return (AVL_EC_GENERAL_FAIL); //at least 3 bytes
  }

  uiSize -= 3; //actual data size
  r = AVL_IBSP_WaitSemaphore(&(AVL_IBSP_I2C_sem));
  if (AVL_EC_OK == r)
  {
    //dump address
    uAddr = pucBuff[0];
    uAddr = uAddr << 8;
    uAddr += pucBuff[1];
    uAddr = uAddr << 8;
    uAddr += pucBuff[2];

    iSize = uiSize;

    uTemp = (MAX_II2C_WRITE_SIZE - 3) & 0xfffe; //how many bytes data we can transfer every time

    ui2 = 0;
    while (iSize > uTemp)
    {
      ui1 = uTemp + 3;
      //save the data
      pucBuffTemp[0] = pucBuff[ui2];
      pucBuffTemp[1] = pucBuff[ui2 + 1];
      pucBuffTemp[2] = pucBuff[ui2 + 2];
      ChunkAddr_AVL62X1(uAddr, pucBuff + ui2);
      r |= AVL_IBSP_I2C_Write(uiSlaveAddr, pucBuff + ui2, &ui1);
      //restore data
      pucBuff[ui2] = pucBuffTemp[0];
      pucBuff[ui2 + 1] = pucBuffTemp[1];
      pucBuff[ui2 + 2] = pucBuffTemp[2];
      uAddr += uTemp;
      ui2 += uTemp;
      iSize -= uTemp;
    }
    ui1 = iSize + 3;
    //save the data
    pucBuffTemp[0] = pucBuff[ui2];
    pucBuffTemp[1] = pucBuff[ui2 + 1];
    pucBuffTemp[2] = pucBuff[ui2 + 2];
    ChunkAddr_AVL62X1(uAddr, pucBuff + ui2);
    r |= AVL_IBSP_I2C_Write(uiSlaveAddr, pucBuff + ui2, &ui1);
    //restore data
    pucBuff[ui2] = pucBuffTemp[0];
    pucBuff[ui2 + 1] = pucBuffTemp[1];
    pucBuff[ui2 + 2] = pucBuffTemp[2];
    uAddr += iSize;
    ui2 += iSize;
  }
  r |= AVL_IBSP_ReleaseSemaphore(&(AVL_IBSP_I2C_sem));

  return (r);
}

AVL_ErrorCode II2C_Write8_AVL62X1(AVL_uint16 uiSlaveAddr, AVL_uint32 uiAddr, AVL_uchar ucData)
{
  AVL_ErrorCode r = AVL_EC_OK;
  AVL_uchar pBuff[4] = {0};

  ChunkAddr_AVL62X1(uiAddr, pBuff);
  pBuff[3] = ucData;
  r = II2C_Write_AVL62X1(uiSlaveAddr, pBuff, 4);

  return (r);
}

AVL_ErrorCode II2C_Write16_AVL62X1(AVL_uint16 uiSlaveAddr, AVL_uint32 uiAddr, AVL_uint16 uiData)
{
  AVL_ErrorCode r = AVL_EC_OK;
  AVL_uchar pBuff[5] = {0};

  ChunkAddr_AVL62X1(uiAddr, pBuff);
  Chunk16_AVL62X1(uiData, pBuff + 3);
  r = II2C_Write_AVL62X1(uiSlaveAddr, pBuff, 5);

  return (r);
}

AVL_ErrorCode II2C_Write32_AVL62X1(AVL_uint16 uiSlaveAddr, AVL_uint32 uiAddr, AVL_uint32 uiData)
{
  AVL_ErrorCode r = AVL_EC_OK;
  AVL_uchar pBuff[7] = {0};

  ChunkAddr_AVL62X1(uiAddr, pBuff);
  Chunk32_AVL62X1(uiData, pBuff + 3);
  r = II2C_Write_AVL62X1(uiSlaveAddr, pBuff, 7);

  return (r);
}

AVL_uchar patch_read8_AVL62X1(AVL_puchar pPatchBuf, AVL_uint32 *idx)
{
  AVL_uchar tmp = 0;
  tmp = pPatchBuf[*idx];
  *idx += 1;
  return tmp;
}
AVL_uint16 patch_read16_AVL62X1(AVL_puchar pPatchBuf, AVL_uint32 *idx)
{
  AVL_uint16 tmp = 0;
  tmp = (pPatchBuf[*idx + 0] << 8) | (pPatchBuf[*idx + 1]);
  *idx += 2;
  return tmp;
}
AVL_uint32 patch_read32_AVL62X1(AVL_puchar pPatchBuf, AVL_uint32 *idx)
{
  AVL_uint32 tmp = 0;
  tmp = (pPatchBuf[*idx + 0] << 24) | (pPatchBuf[*idx + 1] << 16) | (pPatchBuf[*idx + 2] << 8) | pPatchBuf[*idx + 3];
  *idx += 4;
  return tmp;
}
//XLFSR read32 from s_AVL62X1_S2X_PLScramKey_iaddr
AVL_uint32 Convert_XLFSRToN_AVL62X1(AVL_uint32 XLFSR)
{
  AVL_uint32 i = 0;
  AVL_uint32 reg_data = 0x0001; //Initial FLSR is 0x0001;
  XLFSR &= ~(1 << 18);          //clear x(18) just to be safe
  reg_data &= ~(1 << 18);       //clear x(18) just to be safe

  for (i = 0; i <= 262141; i++)
  {
    if (reg_data == XLFSR)
    {
      return i;
    }
    reg_data |= ((reg_data & 0x1) ^ ((reg_data >> 7) & 0x1)) << 18; //XOR x(0) and x(7), assign to x(18)
    reg_data >>= 1;                                                 //shift everything towards x(0).  x(18) becomes x(17)
  }

  return AVL_EC_ConvertXLFSRToN_FAIL; // error ,no find PLS code
}

// example -> Convert_NToXLFSR_AVL62X1(1,1310701),XLFSR=16416 ---> Gold=131070,Root is 16416;
AVL_uint32 Convert_NToXLFSR_AVL62X1(AVL_uint32 reg_data, AVL_uint32 offset)
{

  reg_data &= ~(1 << 18); //clear x(18) just to be safe
  while (offset-- > 0)
  {
    reg_data |= ((reg_data & 0x1) ^ ((reg_data >> 7) & 0x1)) << 18; //XOR x(0) and x(7), assign to x(18)
    reg_data >>= 1;                                                 //shift everything towards x(0).  x(18) becomes x(17)
  }

  return reg_data;
}
