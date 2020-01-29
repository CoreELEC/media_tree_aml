/*
 * Airoha AV201x tuner driver
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

#include "av201x_avl_drv.h"

e_AV201X_Model g_eAV201X_TunerModel = e_AV201X_Model_AV2018;

extern AVL_semaphore AVL_IBSP_I2C_sem;

static unsigned char gs_u8AV201X_init_regs[50] =
    {
        (char)(0x38), (char)(0x00), (char)(0x00), (char)(0x54), (char)(0x1f),
        (char)(0xa3), (char)(0xfd), (char)(0x58), (char)(0x0e), (char)(0x82),
        (char)(0x88), (char)(0xB4), (char)(0xd6), (char)(0x40), (char)(0x94),
        (char)(0x4a), (char)(0x66), (char)(0x40), (char)(0x80), (char)(0x2b),
        (char)(0x6a), (char)(0x50), (char)(0x91), (char)(0x27), (char)(0x8f),
        (char)(0xcc), (char)(0x21), (char)(0x10), (char)(0x80), (char)(0xee),
        (char)(0xf5), (char)(0x7f), (char)(0x4a), (char)(0x9b), (char)(0xe0),
        (char)(0xe0), (char)(0x36), (char)(0x00), (char)(0xab), (char)(0x97),
        (char)(0xc5), (char)(0xa8)};

static s_AV201X_Params gs_AV201X_Params = {
    AV201X_DEFAULT_USE_DEFAULTS,
    AV201X_DEFALUT_XTAL_FREQ,
    AV201X_DEFALUT_IQ_MODE,
    AV201X_DEFALUT_BB_GAIN,
    AV201X_DEFAULT_LOOP_THROUGH,
    AV201X_DEFAULT_FT_STATE};

AVL_uint32 AV201X_GetLockStatus(struct AVL_Tuner *pTuner)
{
  AVL_uint32 r = 0;
  AVL_uchar uilock = 11;
  AVL_uint16 size;

  size = 1;

  r = AVL_IBSP_WaitSemaphore(&(AVL_IBSP_I2C_sem));
  r |= AVL_IBSP_I2C_Write((AVL_uchar)(pTuner->usTunerI2CAddr), &uilock, &size);
  r |= AVL_IBSP_I2C_Read((AVL_uchar)(pTuner->usTunerI2CAddr), &uilock, &size);
  r |= AVL_IBSP_ReleaseSemaphore(&(AVL_IBSP_I2C_sem));

  if (0 == r)
  {
    if ((uilock & 0x3) != (AV201X_R11_LPF_LOCK | AV201X_R11_PLL_LOCK))
    {
      pTuner->ucTunerLocked = 0;
      r = 1;
    }
    else
    {
      pTuner->ucTunerLocked = 1;
    }
  }
  return (r);
}

AVL_uint32 AV201X_Initialize(struct AVL_Tuner *pTuner)
{
  AVL_uint32 r = 0;

  if (gs_AV201X_Params.u1UseDefaults)
  {
    gs_AV201X_Params.eXTAL_Freq = AV201X_DEFALUT_XTAL_FREQ;
    gs_AV201X_Params.eIQmode = AV201X_DEFALUT_IQ_MODE;
    gs_AV201X_Params.eBBG = AV201X_DEFALUT_BB_GAIN;
    gs_AV201X_Params.eLoopTh = AV201X_DEFAULT_LOOP_THROUGH;
    gs_AV201X_Params.u1FT = AV201X_DEFAULT_FT_STATE;
  }

  if (g_eAV201X_TunerModel != e_AV201X_Model_AV2018)
  {
    gs_u8AV201X_init_regs[0] = 0x38;
    gs_u8AV201X_init_regs[1] = 0x00;
    gs_u8AV201X_init_regs[2] = 0x00;
    gs_u8AV201X_init_regs[3] = 0x54;
    gs_u8AV201X_init_regs[29] = 0x02;
  }
  else
  {
    gs_u8AV201X_init_regs[0] = 0x50;
    gs_u8AV201X_init_regs[1] = 0xa1;
    gs_u8AV201X_init_regs[2] = 0x2f;
    gs_u8AV201X_init_regs[3] = 0x50;
    gs_u8AV201X_init_regs[29] = 0xee;
  }

  //init/poweron sequence
  //Reg0 thru 11
  r |= AV201X_I2C_write(pTuner, 0, &(gs_u8AV201X_init_regs[0]), 12);
  AV201X_Time_DELAY_MS(1);
  //Reg13 thru 24
  r |= AV201X_I2C_write(pTuner, 13, &(gs_u8AV201X_init_regs[13]), 12);
  AV201X_Time_DELAY_MS(1);
  //Reg25 thru 35
  r |= AV201X_I2C_write(pTuner, 25, &(gs_u8AV201X_init_regs[25]), 11);
  AV201X_Time_DELAY_MS(1);
  //Reg36 thru 41
  r |= AV201X_I2C_write(pTuner, 36, &(gs_u8AV201X_init_regs[36]), 6);
  AV201X_Time_DELAY_MS(1);
  //Reg12
  r |= AV201X_I2C_write(pTuner, 12, &(gs_u8AV201X_init_regs[12]), 1);
  AV201X_Time_DELAY_MS(10);

  return (r);
}

void AV201X_Time_DELAY_MS(UINT32 ms)
{
  AVL_IBSP_Delay(ms);
}

AVL_uint32 AV201X_I2C_write(struct AVL_Tuner *pTuner, UINT8 reg_start, UINT8 *buff, UINT8 len)
{
  AVL_uint32 r = 0;
  //AVL_uint16 uiTimeOut = 0;
  static AVL_uchar ucTemp[50];
  int i = 0;
  AVL_uint16 size;

  ucTemp[0] = reg_start;

  for (i = 1; i < len + 1; i++)
  {
    ucTemp[i] = *(buff + i - 1);
  }

  size = len + 1;

  r = AVL_IBSP_WaitSemaphore(&(AVL_IBSP_I2C_sem));
  r = AVL_IBSP_I2C_Write((AVL_uchar)pTuner->usTunerI2CAddr, ucTemp, &size);
  r = AVL_IBSP_ReleaseSemaphore(&(AVL_IBSP_I2C_sem));

  return (r);
}

/* read one register */
AVL_uint32 AV201X_I2C_read(struct AVL_Tuner *pTuner, UINT8 addr, UINT8 *data)
{
  AVL_uint32 r = 0;
  AVL_uint16 size = 1;
  AVL_uchar b = addr;

  r = AVL_IBSP_WaitSemaphore(&(AVL_IBSP_I2C_sem));
  r |= AVL_IBSP_I2C_Write((AVL_uchar)(pTuner->usTunerI2CAddr), &b, &size);
  r |= AVL_IBSP_I2C_Read((AVL_uchar)(pTuner->usTunerI2CAddr), data, &size);
  r |= AVL_IBSP_ReleaseSemaphore(&(AVL_IBSP_I2C_sem));

  return r;
}

AVL_uint32 AV201X_GetMaxLPF(struct AVL_Tuner *pTuner, AVL_uint32 *puiMaxLPFHz)
{
  AVL_uint32 r = 0;
  *puiMaxLPFHz = (AVL_uint32)(AV201X_FILTER_BANDWIDTH_MAX * 1000 * 1000);
  return (r);
}

AVL_uint32 AV201X_GetMinLPF(struct AVL_Tuner *pTuner, AVL_uint32 *puiMinLPFHz)
{
  AVL_uint32 r = 0;
  *puiMinLPFHz = (AVL_uint32)(AV201X_FIX_LPF_MIN * 1000 * 1000);
  return (r);
}

AVL_uint32 AV201X_Lock(struct AVL_Tuner *pTuner)
{
  AVL_uint32 r = 0;
  unsigned int u32Frequency_kHz;
  unsigned int u32FilterBandwidth_kHz;
  unsigned int fracN;
  unsigned short tuner_crystal_kHz;
  unsigned char reg_temp[4] = {0, 0, 0, 0};
  //Channel Frequency Calculation.
  u32Frequency_kHz = pTuner->uiRFFrequencyHz / 1000;
  tuner_crystal_kHz = gs_AV201X_Params.eXTAL_Freq;
  fracN = (u32Frequency_kHz + ((unsigned int)tuner_crystal_kHz) / 2) / (unsigned int)tuner_crystal_kHz;
  if (fracN > 0xff)
  {
    fracN = 0xff;
  }
  reg_temp[0] = (char)(fracN & 0xff);
  u32Frequency_kHz = u32Frequency_kHz % ((unsigned int)tuner_crystal_kHz);
  fracN = ((u32Frequency_kHz << 17) / ((unsigned int)tuner_crystal_kHz));
  reg_temp[1] = (char)((fracN >> 9) & 0xff);
  reg_temp[2] = (char)((fracN >> 1) & 0xff);

  //reg[3]_D7 is frac<0>
  reg_temp[3] = gs_u8AV201X_init_regs[3] | (char)((fracN << 7) & 0x80);

  //reg[3]_D2 is IQ mode for 2011/2012
  if ((g_eAV201X_TunerModel != e_AV201X_Model_AV2018) && (gs_AV201X_Params.eIQmode == e_AV201X_IQ_DIFF))
  {
    reg_temp[3] &= ~0x04;
  }
  else
  {
    reg_temp[3] |= 0x04;
  }

  //send R0-R3
  r |= AV201X_I2C_write(pTuner, 0, &(reg_temp[0]), 4);
  AV201X_Time_DELAY_MS(10);

  //set LPF bandwidth
  if (pTuner->ucBlindScanMode)
  {
    //Set tuner LPF wide open
    pTuner->fpGetMaxLPF(pTuner, &pTuner->uiLPFHz);
  }
  u32FilterBandwidth_kHz = (unsigned int)(pTuner->uiLPFHz) / ((unsigned int)1000);
  if (u32FilterBandwidth_kHz < ((unsigned int)(AV201X_FIX_LPF_MIN * 1000)))
    u32FilterBandwidth_kHz = (unsigned int)(AV201X_FIX_LPF_MIN * 1000);
  if (u32FilterBandwidth_kHz < ((unsigned int)(AV201X_FILTER_BANDWIDTH_MIN * 1000)))
    u32FilterBandwidth_kHz = (unsigned int)(AV201X_FILTER_BANDWIDTH_MIN * 1000);
  else if (u32FilterBandwidth_kHz > ((unsigned int)(AV201X_FILTER_BANDWIDTH_MAX * 1000)))
    u32FilterBandwidth_kHz = (unsigned int)(AV201X_FILTER_BANDWIDTH_MAX * 1000);

  //BW(MHz) * 1.27 / 211Khz
  reg_temp[0] = (unsigned char)((u32FilterBandwidth_kHz * ((unsigned int)127) + ((unsigned int)(21100 / ((unsigned int)2)))) / ((unsigned int)21100));
  //send R5
  r |= AV201X_I2C_write(pTuner, 5, &(reg_temp[0]), 1);
  AV201X_Time_DELAY_MS(10);

  //set up FT (fine tune)
  //R37[2:0] = {FT_Block, FT_Enable, FT_Hold}
  reg_temp[0] = (gs_u8AV201X_init_regs[37] & 0xF8);
  if (gs_AV201X_Params.u1UseDefaults == 0)
  {
    reg_temp[0] |= (gs_AV201X_Params.u1FT & 0x7);
  }
  else
  {
    if (pTuner->ucBlindScanMode)
    {
      reg_temp[0] |= 0x5; //block=1,enable=0,hold=1
    }
    else
    {
      reg_temp[0] |= 0x6; //block=1,enable=1,hold=0
    }
  }
  r |= AV201X_I2C_write(pTuner, 37, &(reg_temp[0]), 1);

  //configure BB gain R8[6:3]
  //NOTE: Prev version of driver allowed R8[7] to be set arbitrarily thru a AV201X parameter.
  //      This version of the driver keeps it at its default value (0).
  reg_temp[0] = (gs_u8AV201X_init_regs[8] & ~0x78);
  reg_temp[0] |= (((unsigned char)gs_AV201X_Params.eBBG & 0xF) << 3);
  r |= AV201X_I2C_write(pTuner, 8, &(reg_temp[0]), 1);

  //configure loop-thru
  //R12[6]
  reg_temp[0] = (gs_u8AV201X_init_regs[12] & ~0x40);
  reg_temp[0] |= (((unsigned char)gs_AV201X_Params.eLoopTh & 0x1) << 6);
  r |= AV201X_I2C_write(pTuner, 12, &(reg_temp[0]), 1);

  return (r);
}
