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

#ifndef _AVL62X1_API_H_
#define _AVL62X1_API_H_

#include "AVL62X1_DVBSx.h"
#include "AVL_Tuner.h"

#ifdef AVL_CPLUSPLUS
extern "C"
{
#endif

  AVL_ErrorCode AVL62X1_GetChipID(AVL_uint16 slave_addr, AVL_puint32 pChipId);
  AVL_ErrorCode AVL62X1_Initialize(struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_LockTP(struct AVL62X1_CarrierInfo *pCarrierInfo, struct AVL62X1_StreamInfo *pStreamInfo, AVL_bool blind_sym_rate, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_GetLockStatus(enum AVL62X1_LockStatus *eLockStat, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_GetLostLockStatus(enum AVL62X1_LostLockStatus *eLostLockStat, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_GetSNR(AVL_pint16 piSNR_x100db, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_GetSignalInfo(struct AVL62X1_CarrierInfo *pCarrierInfo, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_GetSignalStrength(AVL_puint16 puiSignalStrength, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_GetSignalQuality(AVL_puint16 puiSignalQuality, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_ResetPER(struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_GetPER(AVL_puint32 puiPER_x1e9, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_DiscoverStreams(struct AVL62X1_CarrierInfo *pCarrierInfo, AVL_bool blind_sym_rate, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_GetDiscoveryStatus(enum AVL62X1_DiscoveryStatus *eDiscoveryStat, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_GetStreamNumber(AVL_puchar pStreamNum, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_GetStreamList(struct AVL62X1_StreamInfo *pStreams, const AVL_uchar max_num_streams, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_SwitchStream(struct AVL62X1_StreamInfo *pStreamInfo, struct AVL62X1_Chip *pAVL_Chip);

  AVL_ErrorCode AVL62X1_IDiseqc_ReadModulationData(AVL_puchar pucBuff, AVL_puchar pucSize, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_IDiseqc_SendModulationData(const AVL_puchar pucBuff, AVL_uchar ucSize, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_IDiseqc_GetTxStatus(struct AVL62X1_Diseqc_TxStatus *pTxStatus, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_IDiseqc_GetRxStatus(struct AVL62X1_Diseqc_RxStatus *pRxStatus, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_IDiseqc_SendTone(AVL_uchar ucTone, AVL_uchar ucCount, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_IDiseqc_Start22K(struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_IDiseqc_Stop22K(struct AVL62X1_Chip *pAVL_Chip);

  AVL_ErrorCode AVL62X1_OpenTunerI2C(struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_CloseTunerI2C(struct AVL62X1_Chip *pAVL_Chip);

  AVL_ErrorCode AVL62X1_SetGPIODir(enum AVL62X1_GPIO_Pin ePin, enum AVL62X1_GPIO_Pin_Direction eDir, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_SetGPIOVal(enum AVL62X1_GPIO_Pin ePin, enum AVL62X1_GPIO_Pin_Value eVal, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_GetGPIOVal(enum AVL62X1_GPIO_Pin ePin, enum AVL62X1_GPIO_Pin_Value *peVal, struct AVL62X1_Chip *pAVL_Chip);

  AVL_ErrorCode AVL62X1_BlindScan_Start(struct AVL62X1_BlindScanParams *pBSParams, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_BlindScan_Cancel(struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_BlindScan_GetStatus(struct AVL62X1_BlindScanInfo *pBSInfo, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_BlindScan_GetCarrierList(const struct AVL62X1_BlindScanParams *pBSParams, struct AVL62X1_BlindScanInfo *pBSInfo, struct AVL62X1_CarrierInfo *pCarriers, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_BlindScan_ConfirmCarrier(const struct AVL62X1_BlindScanParams *pBSParams, struct AVL62X1_CarrierInfo *pCarrierInfo, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_BlindScan_GetStreamList(struct AVL62X1_CarrierInfo *pCarrier, struct AVL62X1_StreamInfo *pStreams, const AVL_uchar max_num_streams, struct AVL62X1_Chip *pAVL_Chip);

  AVL_ErrorCode AVL62X1_Optimize_Carrier(struct AVL_Tuner *pTuner, struct AVL62X1_CarrierInfo *pCarrierInfo);
  AVL_ErrorCode AVL62X1_Enable_T2MIRawMode(struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_Disable_T2MIRawMode(struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_GetPLSXLFSRValue(AVL_puint32 pXLFSRValue, struct AVL62X1_Chip *pAVL_Chip);

  AVL_ErrorCode AVL62X1_Manual_Set_T2MI_PID(AVL_uint16 T2MI_PID, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_Manual_Set_T2MI_PID_1(AVL_uint16 T2MI_PID, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_AutoDetect_T2MI_PID_Enable(struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_Get_Current_Stream_T2MI_PID(AVL_puint16 puiT2MI_PID, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_Set_Current_Stream_T2MI_PID(AVL_uint16 uiT2MI_PID, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_Set_T2MI_MPLP_id_ScanTime(AVL_uint16 Frame_Num, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_Get_T2MI_PLPid(struct AVL62X1_T2MI_MPLP *PLP_List, struct AVL62X1_Chip *pAVL_Chip);
  AVL_ErrorCode AVL62X1_Get_StreamType(enum AVL62X1_DVBStreamType *StreamType, struct AVL62X1_Chip *pAVL_Chip);
#ifdef AVL_CPLUSPLUS
}
#endif

#endif
