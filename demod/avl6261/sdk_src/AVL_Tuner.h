/*
 * Availink AVL6261 DVB-S/S2/S2X demodulator driver - tuner abstraction
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

#ifndef _AVL_TUNER_H_
#define _AVL_TUNER_H_

#include "avl_bsp.h"

#ifdef AVL_CPLUSPLUS
extern "C"
{
#endif

#define AVL_TUNER_EC_OK 0 // There is no error.

  typedef struct AVL_Tuner
  {
    AVL_uint16 usTunerI2CAddr;
    AVL_uchar ucTunerLocked; //1 Lock;   0 unlock

    AVL_uint32 uiRFFrequencyHz;
    AVL_uint32 uiLPFHz; //only valid for satellite tuner

    AVL_uchar ucBlindScanMode;

    void *vpMorePara;

    AVL_uint32 (*fpInitializeFunc)(struct AVL_Tuner *);
    AVL_uint32 (*fpLockFunc)(struct AVL_Tuner *);
    AVL_uint32 (*fpGetLockStatusFunc)(struct AVL_Tuner *);
    AVL_uint32 (*fpGetRFStrength)(struct AVL_Tuner *, AVL_int32 *);

    //Maximum tuner low pass filter bandwidth in Hz
    AVL_uint32 (*fpGetMaxLPF)(struct AVL_Tuner *, AVL_uint32 *);

    //Minimum tuner low pass filter bandwidth in Hz
    AVL_uint32 (*fpGetMinLPF)(struct AVL_Tuner *, AVL_uint32 *);

    //Low pass filter bandwidth step size in Hz
    AVL_uint32 (*fpGetLPFStepSize)(struct AVL_Tuner *, AVL_uint32 *);

    //Tuner AGC gain slope in dB per Volt (dB/V).
    //Tuners with non-inverted AGC sense have a positive slope.
    //Tuners with inverted AGC sense have a negative slope.
    //If the gain slope is not known, implement a function that
    //  returns 1 if the AGC sense is non-inverted,
    //  and returns -1 if the AGC sense is inverted.
    AVL_uint32 (*fpGetAGCSlope)(struct AVL_Tuner *, AVL_int32 *);

    //Voltage at which gain reaches minimum value.  Voltage in millivolts.
    //For a tuner with non-inverted sense (positive slope), this will be a small value.
    //For a tuner with inverted sense (negative slope), this will be a large value.
    AVL_uint32 (*fpGetMinGainVoltage)(struct AVL_Tuner *, AVL_uint32 *);

    //Voltage at which gain reaches its maximum value. Voltage in millivolts.
    //For a tuner with non-inverted sense (positive slope), this will be a large value.
    //For a tuner with inverted sense (negative slope), this will be a small value.
    AVL_uint32 (*fpGetMaxGainVoltage)(struct AVL_Tuner *, AVL_uint32 *);

    //RF Frequency step size in Hz
    AVL_uint32 (*fpGetRFFreqStepSize)(struct AVL_Tuner *, AVL_uint32 *);

  } AVL_Tuner;

#ifdef AVL_CPLUSPLUS
}
#endif

#endif
