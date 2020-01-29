/*
 * Airoha Technology AV201x silicon tuner driver
 *
 * Copyright (C) 2014 Luis Alves <ljalvs@gmail.com>
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

#include "av201x_avl_top.h"
#include "av201x_avl_top_priv.h"

#define dbg_av(fmt, args...) \
	do {\
		if (debug_av)\
			printk("AVL: %s: " fmt "\n", __func__, ##args);\
	} while (0)
MODULE_PARM_DESC(debug_avl, "\n\t\t Enable AVL demodulator debug information");
static int debug_av = 1;



/* write one register */
static int av201x_wr(struct AVL_Tuner * pTuner, u8 addr, u8 data)
{
  AVL_uint32 r = 0;
  r |=  AV201X_I2C_write(pTuner, addr, &data, 1);
	return r;
}

/* read register, apply masks, write back */
static int av201x_regmask(struct AVL_Tuner * pTuner,
	u8 reg, u8 setmask, u8 clrmask)
{
	int ret;
	u8 b = 0;
	if (clrmask != 0xff) {
		ret = AV201X_I2C_read(pTuner, reg, &b);
		if (ret)
			return ret;
		b &= ~clrmask;
	}
	return av201x_wr(pTuner, reg, b | setmask);
}

static void av201x_release(struct dvb_frontend *fe)
{
	struct av201x_avl_priv *priv = fe->tuner_priv;
	dbg_av("%s()\n", __func__);

	kfree(priv->pTuner);
	kfree(priv);
	priv = NULL;
}

static int av201x_init(struct dvb_frontend *fe)
{
	struct av201x_avl_priv *priv = fe->tuner_priv;
	int ret;
	dbg_av("%s()\n", __func__);
  ret = AV201X_Initialize(priv->pTuner);
  
	if (ret != (AVL_TUNER_EC_OK)) {
    dbg_av("%s() failed\n", __func__);
  }
  return ret;
}

static int av201x_sleep(struct dvb_frontend *fe)
{
	struct av201x_avl_priv *priv = fe->tuner_priv;
	int ret;
	dbg_av("%s()\n", __func__);

	ret = av201x_regmask(priv->pTuner, REG_TUNER_CTRL, AV201X_SLEEP, 0);
	if (ret)
		dbg_av("%s() failed\n", __func__);
	return ret;
}


static int av201x_set_params(struct dvb_frontend *fe)
{
	struct av201x_avl_priv *priv = fe->tuner_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  static unsigned int maxLPF = 0;
  static unsigned int minLPF = 0;
	int ret;

	dbg_av("%s() delivery_system=%d frequency=%d " \
			"symbol_rate=%d\n", __func__,
			c->delivery_system, c->frequency, c->symbol_rate);

  priv->pTuner->uiRFFrequencyHz = ((unsigned long int)c->frequency) * 1000;
  priv->pTuner->uiLPFHz = (((unsigned long int)c->symbol_rate) *1350) + 2000000;
  if(maxLPF == 0) 
    priv->pTuner->fpGetMaxLPF(priv->pTuner, &maxLPF);
  if(minLPF == 0) 
    priv->pTuner->fpGetMinLPF(priv->pTuner, &minLPF);
  if(priv->pTuner->uiLPFHz > maxLPF) {
    priv->pTuner->uiLPFHz = maxLPF;
  } else if(priv->pTuner->uiLPFHz < minLPF) {
    priv->pTuner->uiLPFHz = minLPF;
  }
  ret = AV201X_Lock(priv->pTuner);
  
	if (ret != AVL_TUNER_EC_OK)
		dbg_av("%s() failed\n", __func__);
	return ret;
}

static  int   AV201x_agc         [] = {     0,  82,   100,  116,  140,  162,  173,  187,  210,  223,  254,  255};
static  int   AV201x_level_dBm_10[] = {    90, -50,  -263, -361, -463, -563, -661, -761, -861, -891, -904, -910};

static int av201x_get_rf_strength(struct dvb_frontend *fe, u16 *st)
{
	//struct av201x_avl_priv *priv = fe->tuner_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int   if_agc, index, table_length, slope, *x, *y;

	if_agc = *st;
	x = AV201x_agc;
	y = AV201x_level_dBm_10;
	table_length = sizeof(AV201x_agc)/sizeof(int);

	
	/* Finding in which segment the if_agc value is */
	for (index = 0; index < table_length; index ++)
		if (x[index] > if_agc ) break;

	/* Computing segment slope */
	slope =  ((y[index]-y[index-1])*1000)/(x[index]-x[index-1]);
	/* Linear approximation of rssi value in segment (rssi values will be in 0.1dBm unit: '-523' means -52.3 dBm) */
	*st = 1000 + ((y[index-1] + ((if_agc - x[index-1])*slope + 500)/1000))/10;

	c->strength.len = 1;
	c->strength.stat[0].scale = FE_SCALE_DECIBEL;
	c->strength.stat[0].svalue = ((y[index-1] + ((if_agc - x[index-1])*slope + 500)/1000)) * 100;

	return 0;
}


static const struct dvb_tuner_ops av201x_tuner_ops = {
	.info = {
		.name           = "Airoha Technology AV201x AVL",
		.frequency_min_hz = (unsigned int)(850 * 1000) * (unsigned int)1000,
		.frequency_max_hz = (unsigned int)(2300 * 1000) * (unsigned int)1000,
	},

	.release = av201x_release,

	.init = av201x_init,
	.sleep = av201x_sleep,
	.set_params = av201x_set_params,
	.get_rf_strength = av201x_get_rf_strength,
};

struct dvb_frontend *av201x_avl_attach(struct dvb_frontend *fe,
		struct av201x_avl_config *cfg, struct i2c_adapter *i2c)
{
	struct av201x_avl_priv *priv = NULL;

	priv = kzalloc(sizeof(struct av201x_avl_priv), GFP_KERNEL);
	if (priv == NULL) {
		dbg_av("%s() attach failed : alloc1\n", __func__);
		return NULL;
	}

	priv->cfg = cfg;
	priv->i2c = i2c;

  priv->pTuner =  kzalloc(sizeof(struct AVL_Tuner), GFP_KERNEL);
	if (priv->pTuner == NULL) {
    kfree(priv);
		dbg_av("%s() attach failed : alloc2\n", __func__);
		return NULL;
  }
  priv->pTuner->usTunerI2CAddr = cfg->i2c_address;
  priv->pTuner->ucTunerLocked = 0;
  priv->pTuner->uiRFFrequencyHz = 1000*1000*1000;
  priv->pTuner->uiLPFHz = 34*1000*1000;
  priv->pTuner->ucBlindScanMode = 0;
  priv->pTuner->vpMorePara = NULL;
  priv->pTuner->fpInitializeFunc =&AV201X_Initialize;
  priv->pTuner->fpLockFunc = &AV201X_Lock;
  priv->pTuner->fpGetLockStatusFunc = &AV201X_GetLockStatus;
  priv->pTuner->fpGetRFStrength = NULL;
  priv->pTuner->fpGetMaxLPF = &AV201X_GetMaxLPF;
  priv->pTuner->fpGetMinLPF = &AV201X_GetMinLPF;
  priv->pTuner->fpGetLPFStepSize = NULL;
  priv->pTuner->fpGetAGCSlope = NULL;
  priv->pTuner->fpGetMinGainVoltage = NULL;
  priv->pTuner->fpGetMaxGainVoltage = NULL;
  priv->pTuner->fpGetRFFreqStepSize = NULL;

	AVL_IBSP_I2C_Adapter(0, i2c);
  
	dev_info(&priv->i2c->dev,
		"%s: Airoha Technology AV201x successfully attached\n",
		KBUILD_MODNAME);

	memcpy(&fe->ops.tuner_ops, &av201x_tuner_ops,
			sizeof(struct dvb_tuner_ops));

	fe->tuner_priv = priv;
	return fe;
}
EXPORT_SYMBOL_GPL(av201x_avl_attach);
EXPORT_SYMBOL_GPL(AV201X_Initialize);
EXPORT_SYMBOL_GPL(AV201X_GetLockStatus);
EXPORT_SYMBOL_GPL(AV201X_Lock);
EXPORT_SYMBOL_GPL(AV201X_GetMaxLPF);
EXPORT_SYMBOL_GPL(AV201X_GetMinLPF);
EXPORT_SYMBOL_GPL(AV201X_Time_DELAY_MS);
EXPORT_SYMBOL_GPL(AV201X_I2C_write);
EXPORT_SYMBOL_GPL(AV201X_I2C_read);

MODULE_DESCRIPTION("Airoha Technology AV201x silicon tuner driver");
MODULE_AUTHOR("Availink, Inc. <opensource@availink.com>");
MODULE_LICENSE("GPL");
