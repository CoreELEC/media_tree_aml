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

#include <media/dvb_frontend.h>
#include "avl6261.h"

#include "AVL62X1_API.h"
#include "AVL_Tuner.h"

//#include "avl6261_fw.h"

#define dbg_avl(fmt, args...)                         \
  do                                                  \
  {                                                   \
    if (debug_avl)                                    \
      printk("AVL: %s: " fmt "\n", __func__, ##args); \
  } while (0);

MODULE_PARM_DESC(debug_avl, "\n\t\t Enable AVL demodulator debug information");
static int debug_avl;
module_param(debug_avl, int, 0644);

AVL_Tuner default_avl_tuner = {
  .ucBlindScanMode = 0,
  .vpMorePara = NULL,
  .fpInitializeFunc = NULL,
  .fpLockFunc = NULL,
  .fpGetLockStatusFunc = NULL,
  .fpGetRFStrength = NULL,
  .fpGetMaxLPF = NULL,
  .fpGetMinLPF = NULL,
  .fpGetLPFStepSize = NULL,
  .fpGetAGCSlope = NULL,
  .fpGetMinGainVoltage = NULL,
  .fpGetMaxGainVoltage = NULL,
  .fpGetRFFreqStepSize = NULL
};

static int InitErrorStat_Demod(struct avl6261_priv *priv)
{
  AVL_ErrorCode r = AVL_EC_OK;
  AVL62X1_ErrorStatConfig stErrorStatConfig;
  struct AVL62X1_BERConfig stBERConfig;

  stErrorStatConfig.eErrorStatMode = AVL62X1_ERROR_STAT_AUTO;
  stErrorStatConfig.eAutoErrorStatType = AVL62X1_ERROR_STAT_TIME;
  stErrorStatConfig.uiTimeThresholdMs = 3000;
  stErrorStatConfig.uiNumberThresholdByte = 0;

  r = IRx_ErrorStatMode_AVL62X1(&stErrorStatConfig, priv->chip);

  stBERConfig.eBERTestPattern = AVL62X1_TEST_LFSR_23;
  stBERConfig.eBERFBInversion = AVL62X1_LFSR_FB_INVERTED;
  stBERConfig.uiLFSRSynced = 0;
  stBERConfig.uiLFSRStartPos = 4;
  r |= IRx_ResetBER_AVL62X1(&stBERConfig, priv->chip);

  return r;
}

static int avl6261_init_dvbs(struct dvb_frontend *fe)
{
  struct avl6261_priv *priv = fe->demodulator_priv;
  struct AVL62X1_Diseqc_Para Diseqc_para;
  AVL_ErrorCode r = AVL_EC_OK;

  Diseqc_para.uiToneFrequencyKHz = 22;
  Diseqc_para.eTXGap = AVL62X1_DTXG_15ms;
  Diseqc_para.eTxWaveForm = AVL62X1_DWM_Normal;
  Diseqc_para.eRxTimeout = AVL62X1_DRT_150ms;
  Diseqc_para.eRxWaveForm = AVL62X1_DWM_Normal;

  r |= IDiseqc_Initialize_AVL62X1(&Diseqc_para, priv->chip);
  if (AVL_EC_OK != r)
  {
    dbg_avl("Diseqc Init failed !\n");
  }
  else
  {
    priv->config->eDiseqcStatus = AVL62X1_DOS_Initialized;
  }

  r = (int)AVL62X1_SetGPIODir(AVL62X1_GPIO_Pin_LNB_PWR_EN, AVL62X1_GPIO_DIR_OUTPUT, priv->chip);
  r |= (int)AVL62X1_SetGPIODir(AVL62X1_GPIO_Pin_LNB_PWR_SEL, AVL62X1_GPIO_DIR_OUTPUT, priv->chip);

  return r;
}

static int avl6261_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
  struct avl6261_priv *priv = fe->demodulator_priv;
  AVL_ErrorCode ret = AVL_EC_OK;

  dbg_avl("%s: %d\n", __func__, enable);

  if (enable)
  {
    ret = AVL62X1_OpenTunerI2C(priv->chip);
  }
  else
  {
    ret = AVL62X1_CloseTunerI2C(priv->chip);
  }
  return ret;
}

static int avl6261_set_dvbs(struct dvb_frontend *fe)
{
  struct avl6261_priv *priv = fe->demodulator_priv;
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  AVL_ErrorCode r = AVL_EC_OK;
  struct AVL62X1_CarrierInfo CarrierInfo;
  struct AVL62X1_StreamInfo StreamInfo;
  dbg_avl("Freq:%d khz,sym:%d hz", c->frequency, c->symbol_rate);

  CarrierInfo.m_carrier_index = 0;
  CarrierInfo.m_rf_freq_kHz = c->frequency;
  CarrierInfo.m_carrier_freq_offset_Hz = 0; //carrier frequency offset (from RF freq) in Hz
  CarrierInfo.m_symbol_rate_Hz = c->symbol_rate;
  CarrierInfo.m_roll_off = 0;
  CarrierInfo.m_signal_type = 0;
  CarrierInfo.m_PL_scram_key = AVL62X1_PL_SCRAM_AUTO;
  CarrierInfo.m_PLS_ACM = 0; //PLS if CCM, 0 if ACM
  CarrierInfo.m_SIS_MIS = 0;
  CarrierInfo.m_num_streams = 0; //number of supported streams (that can be output)
  CarrierInfo.m_SNR_dB_x100 = 0; //
  CarrierInfo.m_modulation = 0;
  CarrierInfo.m_pilot = 0;
  CarrierInfo.m_dvbs2_fec_length = 0;
  CarrierInfo.m_coderate.m_dvbs2_code_rate = 0;
  CarrierInfo.m_dvbs2_ccm_acm = 0; //1:CCM, 0:ACM

  StreamInfo.m_carrier_index = 0; //index of carrier (AVL62X1_CarrierInfo.m_CarrierIndex) that this stream is in
  StreamInfo.m_stream_type = AVL62X1_TRANSPORT;
  StreamInfo.m_ISI = c->stream_id;
  StreamInfo.m_PLP_ID = 0;         // use when lock TP
  StreamInfo.PLP_List.PLP_Num = 0; // save scan out the PLP list for T2MI ISI

  r = AVL62X1_LockTP(&CarrierInfo, &StreamInfo, false, priv->chip);

  return r;
}

static int avl6261_set_dvbmode(struct dvb_frontend *fe,
                               enum fe_delivery_system delsys)
{
  struct avl6261_priv *priv = fe->demodulator_priv;
  AVL_ErrorCode ret = AVL_EC_OK;
  AVL_ErrorCode r = AVL_EC_OK;
  struct AVL62X1_VerInfo ver_info;

  /* already in desired mode */
  if (priv->delivery_system == delsys)
    return 0;

  dbg_avl("initing demod for delsys=%d", delsys);

  switch (priv->delivery_system)
  {
  case SYS_DVBS:
    if (delsys == SYS_DVBS2)
      return 0;
    break;
  case SYS_DVBS2:
    if (delsys == SYS_DVBS)
      return 0;
    break;
  default:
    break;
  }
  priv->delivery_system = delsys;

  //Reset Demod
  r = AVL_IBSP_Reset();
  if (AVL_EC_OK != r)
  {
    dbg_avl("Failed to Resed demod via BSP!\n");
    return 0;
  }

  // boot the firmware here
  r |= AVL62X1_Initialize(priv->chip);
  if (AVL_EC_OK != r)
  {
    dbg_avl("AVL_AVL62X1_Initialize failed !\n");
    return (r);
  }

  r |= IBase_GetVersion_AVL62X1(&ver_info, priv->chip);
  if (AVL_EC_OK != r)
  {
    dbg_avl("IBase_GetVersion_AVL62X1 failed\n");
    return (r);
  }
  dbg_avl("FW version %d.%d.%d\n", ver_info.m_Patch.m_Major, ver_info.m_Patch.m_Minor, ver_info.m_Patch.m_Build);
  dbg_avl("API version %d.%d.%d\n", ver_info.m_API.m_Major, ver_info.m_API.m_Minor, ver_info.m_API.m_Build);

  switch (priv->delivery_system)
  {
  case SYS_DVBS:
  case SYS_DVBS2:
  default:
    ret |= avl6261_init_dvbs(fe);
    break;
  }

  ret |= InitErrorStat_Demod(priv);

  if (ret)
  {
    dev_err(&priv->i2c->dev, "%s: demod init failed",
            KBUILD_MODNAME);
  }

  return ret;
}

AVL_ErrorCode AVL_SX_DiseqcSendCmd(struct avl6261_priv *priv, AVL_puchar pCmd, u8 CmdSize)
{
  AVL_ErrorCode r = AVL_EC_OK;
  struct AVL62X1_Diseqc_TxStatus TxStatus;
  dbg_avl(" %*ph", CmdSize, pCmd);

  r = AVL62X1_IDiseqc_SendModulationData(pCmd, CmdSize, priv->chip);
  if (r != AVL_EC_OK)
  {
    printk("AVL_SX_DiseqcSendCmd failed !\n");
  }
  else
  {
    do
    {
      msleep(5);
      r |= AVL62X1_IDiseqc_GetTxStatus(&TxStatus, priv->chip);
    } while (TxStatus.m_TxDone != 1);
    if (r == AVL_EC_OK)
    {
    }
    else
    {
      printk("AVL_SX_DiseqcSendCmd Err. !\n");
    }
  }
  return (int)(r);
}

static int avl6261_diseqc(struct dvb_frontend *fe,
                          struct dvb_diseqc_master_cmd *cmd)
{
  struct avl6261_priv *priv = fe->demodulator_priv;

  return AVL_SX_DiseqcSendCmd(priv, cmd->msg, cmd->msg_len);
}

static int avl6261_burst(struct dvb_frontend *fe, enum fe_sec_mini_cmd burst)
{
  struct avl6261_priv *priv = fe->demodulator_priv;
  int ret;

  ret = (int)AVL62X1_IDiseqc_SendTone(burst == SEC_MINI_A ? 1 : 0, 1, priv->chip);

  return ret;
}

static int avl6261_set_tone(struct dvb_frontend *fe, enum fe_sec_tone_mode tone)
{
  struct avl6261_priv *priv = fe->demodulator_priv;
  int r = AVL_EC_OK;

  dbg_avl("tone: %d", tone);
  switch (tone)
  {
  case SEC_TONE_ON:
    if (priv->chip->m_Diseqc_OP_Status != AVL62X1_DOS_InContinuous)
    {
      r = (int)AVL62X1_IDiseqc_Stop22K(priv->chip);
    }
    break;
  case SEC_TONE_OFF:
    if (priv->chip->m_Diseqc_OP_Status == AVL62X1_DOS_InContinuous)
    {
      r = (int)AVL62X1_IDiseqc_Start22K(priv->chip);
    }
    break;
  default:
    return -EINVAL;
  }
  return r;
}

static int avl6261_set_voltage(struct dvb_frontend *fe, enum fe_sec_voltage voltage)
{
  struct avl6261_priv *priv = fe->demodulator_priv;
  AVL62X1_GPIO_Pin_Value pwr, vol;
  int ret;

  dbg_avl("volt: %d", voltage);

  switch (voltage)
  {
  case SEC_VOLTAGE_OFF:
    pwr = AVL62X1_GPIO_VALUE_LOGIC_0;
    vol = AVL62X1_GPIO_VALUE_LOGIC_0;
    break;
  case SEC_VOLTAGE_13:
    //power on
    pwr = AVL62X1_GPIO_VALUE_LOGIC_1;
    vol = AVL62X1_GPIO_VALUE_LOGIC_0;
    break;
  case SEC_VOLTAGE_18:
    //power on
    pwr = AVL62X1_GPIO_VALUE_LOGIC_1;
    vol = AVL62X1_GPIO_VALUE_HIGH_Z;
    break;
  default:
    return -EINVAL;
  }
  ret = (int)AVL62X1_SetGPIOVal(AVL62X1_GPIO_Pin_LNB_PWR_EN, pwr, priv->chip);
  ret |= (int)AVL62X1_SetGPIOVal(AVL62X1_GPIO_Pin_LNB_PWR_SEL, vol, priv->chip);
  return ret;
}

static int avl6261_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
  struct avl6261_priv *priv = fe->demodulator_priv;
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  int ret = 0;
  AVL62X1_LockStatus lock = 0;
  AVL_int16 SNR_x100db = 0;
  AVL_int16 SignalStrength = 0;

  switch (priv->delivery_system)
  {
  case SYS_DVBS:
  case SYS_DVBS2:
    ret = AVL62X1_GetLockStatus(&lock, priv->chip);
    //dbg_avl("lock: %d", lock);
    if (lock == AVL62X1_STATUS_LOCK)
    {
      ret |= AVL62X1_GetSNR(&SNR_x100db, priv->chip);
      dbg_avl("SNR_x100db: %d", SNR_x100db);
      if (ret || SNR_x100db > 10000)
        SNR_x100db = 0;
    }
    else
    {
      *status = 0;
      return ret;
    }
    break;
  default:
    *status = 0;
    return 1;
  }

  if (ret)
  {
    *status = 0;
    return ret;
  }
  *status = FE_HAS_SIGNAL;

  //dbg_avl("%s", read_stdout(priv->chip));

  ret = AVL62X1_GetSignalStrength(&SignalStrength, priv->chip);

  c->strength.len = 2;

  c->strength.stat[1].scale = FE_SCALE_RELATIVE;
  c->strength.stat[1].uvalue = (SignalStrength * 65535) / 100;

  c->strength.stat[0].scale = FE_SCALE_DECIBEL;
  c->strength.stat[0].svalue = -80 + SignalStrength / 2;

  if (lock == AVL62X1_STATUS_LOCK)
  {
    *status |= FE_HAS_CARRIER | FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
    c->cnr.len = 2;
    c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
    c->cnr.stat[0].svalue = SNR_x100db * 10;
    c->cnr.stat[1].scale = FE_SCALE_RELATIVE;
    c->cnr.stat[1].uvalue = ((SNR_x100db + 300) / 10) * 250;
    if (c->cnr.stat[1].uvalue > 0xffff)
      c->cnr.stat[1].uvalue = 0xffff;
  }
  else
  {
    c->cnr.len = 1;
    c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
  }
  return ret;
}

static int avl6261_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  int i;

  *strength = 0;
  for (i = 0; i < c->strength.len; i++)
    if (c->strength.stat[i].scale == FE_SCALE_RELATIVE)
      *strength = (u16)c->strength.stat[i].uvalue;

  return 0;
}

static int avl6261_read_snr(struct dvb_frontend *fe, u16 *snr)
{
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  int i;

  *snr = 0;
  for (i = 0; i < c->cnr.len; i++)
    if (c->cnr.stat[i].scale == FE_SCALE_RELATIVE)
      *snr = (u16)c->cnr.stat[i].uvalue;

  return 0;
}

static int avl6261_read_ber(struct dvb_frontend *fe, u32 *ber)
{
  struct avl6261_priv *priv = fe->demodulator_priv;
  int ret;

  *ber = 10e7;
  ret = (int)AVL62X1_GetPER(ber, priv->chip);
  if (!ret)
    *ber /= 100;

  return ret;
}

static int avl6261fe_algo(struct dvb_frontend *fe)
{
  return DVBFE_ALGO_HW;
}

//static  struct dtv_frontend_properties _last_dtv;

static int avl6261_set_frontend(struct dvb_frontend *fe)
{
  int ret;
  //struct avl6261_priv *priv = fe->demodulator_priv;

  /* setup tuner */
  if (fe->ops.tuner_ops.set_params)
  {
    if (fe->ops.i2c_gate_ctrl)
      fe->ops.i2c_gate_ctrl(fe, 1);
    ret = fe->ops.tuner_ops.set_params(fe);
    if (fe->ops.i2c_gate_ctrl)
      fe->ops.i2c_gate_ctrl(fe, 0);
    if (ret)
      return ret;
  }

  dbg_avl("ACQUIRE");
  ret = avl6261_set_dvbs(fe);

  return ret;
}

static int avl6261_tune(struct dvb_frontend *fe, bool re_tune,
                        unsigned int mode_flags, unsigned int *delay, enum fe_status *status)
{
  *delay = HZ / 5;
  if (re_tune)
  {
    int ret = avl6261_set_frontend(fe);
    if (ret)
      return ret;
  }
  return avl6261_read_status(fe, status);
}

static int avl6261_set_property(struct dvb_frontend *fe,
                                u32 cmd, u32 data)
{
  int ret = 0;
  fe->ops.info.frequency_min_hz = 950000000;
  fe->ops.info.frequency_max_hz = 2150000000;
  fe->ops.info.frequency_stepsize_hz = 0;

  return ret;
}

static int avl6261_init(struct dvb_frontend *fe)
{
  return 0;
}

static int avl6261_sleep(struct dvb_frontend *fe)
{
  return 0;
}

static void avl6261_release(struct dvb_frontend *fe)
{
  struct avl6261_priv *priv = fe->demodulator_priv;
  release_firmware(priv->fw);
  kfree(priv->chip);
  kfree(priv);
}

static struct dvb_frontend_ops avl6261_ops = {
    .delsys = {SYS_DVBS, SYS_DVBS2},
    .info = {
        .name = "Availink avl6261",
        .frequency_min_hz = 950000000,
        .frequency_max_hz = 2150000000,
        .frequency_stepsize_hz = 0,
        .frequency_tolerance_hz = 0,
        .symbol_rate_min = 1000000,
        .symbol_rate_max = 55000000,
        .caps =
            FE_CAN_FEC_1_2 |
            FE_CAN_FEC_2_3 |
            FE_CAN_FEC_3_4 |
            FE_CAN_FEC_4_5 |
            FE_CAN_FEC_5_6 |
            FE_CAN_FEC_6_7 |
            FE_CAN_FEC_7_8 |
            FE_CAN_FEC_AUTO |
            FE_CAN_QPSK |
            FE_CAN_QAM_16 |
            FE_CAN_QAM_32 |
            FE_CAN_QAM_64 |
            FE_CAN_QAM_AUTO |
            FE_CAN_TRANSMISSION_MODE_AUTO |
            FE_CAN_MUTE_TS |
            FE_CAN_2G_MODULATION |
            FE_CAN_MULTISTREAM |
            FE_CAN_INVERSION_AUTO |
            FE_CAN_RECOVER},

    .release = avl6261_release,
    .init = avl6261_init,

    .sleep = avl6261_sleep,
    .i2c_gate_ctrl = avl6261_i2c_gate_ctrl,

    .read_status = avl6261_read_status,
    .read_signal_strength = avl6261_read_signal_strength,
    .read_snr = avl6261_read_snr,
    .read_ber = avl6261_read_ber,
    .set_tone = avl6261_set_tone,
    .set_voltage = avl6261_set_voltage,
    .diseqc_send_master_cmd = avl6261_diseqc,
    .diseqc_send_burst = avl6261_burst,
    .get_frontend_algo = avl6261fe_algo,
    .tune = avl6261_tune,

    .set_property = avl6261_set_property,
    .set_frontend = avl6261_set_frontend,
};

struct dvb_frontend *avl6261_attach(struct avl6261_config *config,
                                    struct i2c_adapter *i2c)
{
  struct avl6261_priv *priv;
  AVL_ErrorCode ret;
  u32 id;
  int fw_status;
  unsigned int fw_maj, fw_min, fw_build;

  dbg_avl("start demod attach");

  priv = kzalloc(sizeof(struct avl6261_priv), GFP_KERNEL);
  if (priv == NULL)
    goto err;

  dbg_avl("priv alloc'ed = %llx", (unsigned long long int)priv);

  memcpy(&priv->frontend.ops, &avl6261_ops,
         sizeof(struct dvb_frontend_ops));

  priv->frontend.demodulator_priv = priv;
  priv->config = config;
  priv->i2c = i2c;
  priv->delivery_system = -1;
  priv->chip = kzalloc(sizeof(struct AVL62X1_Chip), GFP_KERNEL);
  if (priv->chip == NULL)
    goto err1;
  dbg_avl("chip alloc'ed = %llx", (unsigned long long int)priv->chip);
  
  //                               I2C slave address (8b)                       Demod ID (3b)
  priv->chip->usI2CAddr = ((AVL_uint16)config->demod_address) | (((AVL_uint16)(config->i2c_id & 0x7)) << 8);
  dbg_avl("usI2CAddr = %x", priv->chip->usI2CAddr);
  priv->chip->e_Xtal = AVL62X1_RefClk_27M;
  //priv->chip->pPatchData = ucPatchData;
  priv->chip->pTuner = &default_avl_tuner;
  priv->chip->e_TunerPol = AVL62X1_Spectrum_Invert;
  priv->chip->e_Mode = AVL62X1_MPM_Parallel;
  priv->chip->e_ClkPol = AVL62X1_MPCP_Rising;
  priv->chip->e_ClkPhase = AVL62X1_MPCP_Phase_0;
  priv->chip->e_ClkAdapt = AVL62X1_MPCA_Adaptive;
  priv->chip->e_Format = AVL62X1_MPF_TS;
  priv->chip->e_SerPin = AVL62X1_MPSP_DATA0;
  priv->chip->m_MPEGFrequency_Hz = 130000000;
  dbg_avl("chip initialized");

  // associate i2c_id/slaveAddr with i2c_adapter
  AVL_IBSP_I2C_Adapter(priv->chip->usI2CAddr, i2c);
  dbg_avl("i2c associated\n");

  /* get chip id */
  ret = AVL62X1_GetChipID(priv->chip->usI2CAddr, &id);
  if (ret)
  {
    dev_err(&priv->i2c->dev, "%s: attach failed reading id",
            KBUILD_MODNAME);
    goto err2;
  }

  dbg_avl("chip_id= %d\n", id);

  if (id != 0x62615ca8)
  {
    dev_err(&priv->i2c->dev, "%s: attach failed, id mismatch",
            KBUILD_MODNAME);
    goto err2;
  }

  dev_info(&priv->i2c->dev, "%s: found AVL6261 id=0x%x",
           KBUILD_MODNAME, id);

  fw_status = request_firmware(&priv->fw, "avl6261.patch", i2c->dev.parent);
  if (fw_status < 0)
  {
    dev_err(&priv->i2c->dev, "%s: firmware file not found",
            KBUILD_MODNAME);
    goto err2;
  } else {
    priv->chip->pPatchData = (unsigned char *)(priv->fw->data);
    fw_maj = priv->chip->pPatchData[24]; //major rev
    fw_min = priv->chip->pPatchData[25]; //SDK-FW API rev
    fw_build = (priv->chip->pPatchData[26]<<8) | priv->chip->pPatchData[27]; //internal rev
    if(fw_min != AVL62X1_API_VER_MINOR) //SDK-FW API rev must match
    {
      dev_err(&priv->i2c->dev,
              "%s: Firmware version %d.%d.%d incompatible with this driver version",
              KBUILD_MODNAME, fw_maj, fw_min, fw_build);
      dev_err(&priv->i2c->dev,
              "%s: Firmware minor version must be %d",
              KBUILD_MODNAME, AVL62X1_API_VER_MINOR);
      release_firmware(priv->fw);
      goto err2;
    }
    else
    {
      dev_info(&priv->i2c->dev,
               "%s: Firmware version %d.%d.%d found",
               KBUILD_MODNAME, fw_maj, fw_min, fw_build);
    }
  }
  

  if (!avl6261_set_dvbmode(&priv->frontend, SYS_DVBS))
  {
    return &priv->frontend;
  }

err2:
  kfree(priv->chip);
err1:
  kfree(priv);
err:
  return NULL;
}
EXPORT_SYMBOL_GPL(avl6261_attach);

MODULE_DESCRIPTION("Availink AVL6261 DVB-S/S2/S2X demodulator driver");
MODULE_AUTHOR("Availink, Inc. (opensource@availink.com)");
MODULE_LICENSE("GPL");
MODULE_VERSION(AVL6261_VERSION);
