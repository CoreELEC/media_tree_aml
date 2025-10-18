/*
 * Demod driver wrappers for aml_dvb_extern module
 *
 * Copyright (C) 2025 Marek Czerski <ma.czerski@gmail.com>
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

#include <linux/amlogic/aml_demod_common.h>
#include <linux/amlogic/aml_tuner.h>
#include "avl6862.h"
#include "avl6261.h"
#include "mxl603.h"
#include "m88rs6060.h"
#include "r848.h"
#include "r912.h"
#include "av201x_avl_top.h"
#include "cxd2878.h"

static void aml_dvb_extern_reset(const struct gpio_config *reset)
{
		if (aml_gpio_is_valid(reset->pin)) {
			pr_info("Demod: performing reset using gpio %d, with value %d\n",
					reset->pin,
					reset->value);
			aml_gpio_direction_output(reset->pin, reset->value);
			msleep(100);
			aml_gpio_set_value(reset->pin, !reset->value);
			msleep(100);
		}
}

static struct dvb_frontend *aml_avl6x62_attach(const struct demod_config *cfg, bool dual_tuner)
{
	struct avl6862_config avl68xxcfg = {
		.demod_address = cfg->i2c_addr,
		.dual_tuner = dual_tuner,
		.ts_serial = cfg->ts_out_mode ? 0 : 1, /* ts_out_mode: serial or parallel; 0: serial, 1: parallel. */
		.gpio_lock_led = 0,
	};
	struct dvb_frontend *fe;

	aml_dvb_extern_reset(&cfg->reset);
	fe = avl6862_attach(&avl68xxcfg, cfg->i2c_adap);
	if (IS_ERR_OR_NULL(fe))
		return NULL;

	if (cfg->tuner0.id != AM_TUNER_NONE) {
		const struct tuner_module * tuner = aml_get_tuner_module(cfg->tuner0.id);
		if (tuner->attach(tuner, fe, &cfg->tuner0) == NULL) {
			pr_err("AVL68xx: failed to attach tuner0 %s\n", tuner->name);
		}
	}
	else {
		pr_err("AVL68xx: Missing tuner0 config\n");
	}

	if (cfg->tuner1.id != AM_TUNER_NONE) {
		pr_err("AVL68xx: failed to attach tuner1, dual tuner not supported\n");
	}

	return fe;
}

struct dvb_frontend *aml_avl68xx_attach(const struct demod_config *cfg) {
	return aml_avl6x62_attach(cfg, true);
}

struct dvb_frontend *aml_avl6762_attach(const struct demod_config *cfg) {
	return aml_avl6x62_attach(cfg, false);
}

static struct dvb_frontend *aml_avl6221c_attach(const struct demod_config *cfg)
{
	struct avl6261_config avl6261cfg = {
		.i2c_id = 0,
		.i2c_adapter = cfg->i2c_adap,
		.demod_address = cfg->i2c_addr,
		.tuner_address = (cfg->tuner0.id != AM_TUNER_NONE) ? cfg->tuner0.i2c_addr : 0,
		.eDiseqcStatus = 0,
	};
	struct dvb_frontend *fe;

	aml_dvb_extern_reset(&cfg->reset);
	fe = avl6261_attach(&avl6261cfg, cfg->i2c_adap);
	if (IS_ERR_OR_NULL(fe))
		return NULL;

	if (cfg->tuner0.id != AM_TUNER_NONE) {
		const struct tuner_module * tuner = aml_get_tuner_module(cfg->tuner0.id);
		if (tuner->attach(tuner, fe, &cfg->tuner0) == NULL) {
			pr_err("AVL6221c: failed to attach tuner0 %s\n", tuner->name);
		}
	}
	else {
		pr_err("AVL6221c: Missing tuner0 config\n");
	}

	return fe;
}

struct dvb_frontend *aml_mxl603_attach(struct dvb_frontend *fe,
				       const struct tuner_config *cfg)
{
	struct mxl603_config mxl603cfg = {
		.xtal_freq_hz = cfg->xtal, /* XTAL Frequency, 0: 16MHz; 1: 24MHz */
		.if_freq_hz = cfg->if_hz, /* 0  = 3.65MHz
									 1  = 4MHz
									 2  = 4.1MHz
									 3  = 4.15MHz
									 4  = 4.5MHz
									 5  = 4.57MHz
									 6  = 5MHz
									 7  = 5.38MHz
									 8  = 6MHz
									 9  = 6.28MHz
									 10 = 7.2MHz
									 11 = 8.25MHz
									 12 = 35.25MHz
									 13 = 36MHz
									 14 = 36.15MHz
									 15 = 36.65MHz
									 16 = 44MHz */
		.agc_type = cfg->if_agc, /* AGC mode selection, self (0) or closed loop (1) */
		.xtal_cap = cfg->xtal_cap, /* XTAL capacity, 1 LSB = 1pF, maximum is 31pF */
		.gain_level = cfg->if_amp, /* IF out gain level */
		.if_out_gain_level = 11, /* IF out gain level (only for terrestial) */
		.agc_set_point = 66, /* AGC attack point set value */
		.agc_invert_pol = 0, /* Config AGC Polarity inversion */
		.invert_if = cfg->if_invert, /* IF spectrum is inverted or not */
		.loop_thru_enable = cfg->lt_out, /* Loop-Through enable */
		.clk_out_enable = 1, /* enable or disable clock out */
		.clk_out_div = 0, /* indicate if XTAL frequency is dived by 4 or not */
		.clk_out_ext = 0, /* enable or disable external clock out */
		.xtal_sharing_mode = cfg->xtal_mode, /* XTAL sharing mode. default Master, MXL608_ENABLE to config Slave mode */
		.single_supply_3_3V = cfg->dual_power ? 0 : 1, /* dual_power: 0: 3.3v, 1: 1.8v and 3.3v. */
	};
	aml_dvb_extern_reset(&cfg->reset);
	return mxl603_attach(fe, cfg->i2c_adap, cfg->i2c_addr, &mxl603cfg);
}

struct dvb_frontend *aml_r848_attach(struct dvb_frontend *fe,
				       const struct tuner_config *cfg)
{
	struct r848_config r848cfg = {
		.i2c_address = cfg->i2c_addr,
		.xtal = cfg->xtal, /* XTAL Frequency in Hz, typicaly 16000000 */
		.R848_DetectTfType = 0, /* 0: R848_UL_USING_BEAD, 1: R848_UL_USING_270NH */
		.R848_Xtal_Pwr = 3, /*
				0 = XTAL_SMALL_LOWEST
				1 = XTAL_SMALL_LOW,
				2 = XTAL_SMALL_HIGH,
				3 = XTAL_SMALL_HIGHEST,
				4 = XTAL_LARGE_HIGHEST,
				5 = XTAL_CHECK_SIZE */
		.R848_Xtal_Pwr_tmp = 4, /* same as R848_Xtal_Pwr */
		.R848_SetTfType = 1, /*
				0 = R848_TF_NARROW			270n/68n   (ISDB-T, DVB-T/T2)
				1 = R848_TF_BEAD			Bead/68n   (DTMB)
				2 = R848_TF_NARROW_LIN		270n/68n   (N/A)
				3 = R848_TF_NARROW_ATV_LIN	270n/68n   (ATV)
				4 = R848_TF_BEAD_LIN		Bead/68n   (PAL_DK for China Hybrid TV)
				5 = R848_TF_NARROW_ATSC		270n/68n   (ATSC, DVB-C, J83B)
				6 = R848_TF_BEAD_LIN_ATSC	Bead/68n   (ATSC, DVB-C, J83B)
				7 = R848_TF_82N_BEAD		Bead/82n   (DTMB)
				8 = R848_TF_82N_270N		270n/82n   (OTHER Standard) */
	};
	aml_dvb_extern_reset(&cfg->reset);
	return r848_attach(fe, &r848cfg, cfg->i2c_adap);
}

struct dvb_frontend *aml_r912_attach(struct dvb_frontend *fe,
				       const struct tuner_config *cfg)
{
	struct r912_config r912cfg = {
		.i2c_address = cfg->i2c_addr,
	};
	aml_dvb_extern_reset(&cfg->reset);
	return r912_attach(fe, &r912cfg, cfg->i2c_adap);
}

struct dvb_frontend *aml_m88dm6k_attach(const struct demod_config *cfg)
{
	struct i2c_client *client;
	struct i2c_board_info info;
	struct m88rs6060_cfg m88rs6060_config;
	struct dvb_frontend *fe;

/* settings taken from tbs5930_frontend_m88rs6060_attach */
	memset(&m88rs6060_config,0,sizeof(m88rs6060_config));
	m88rs6060_config.fe = &fe;
	m88rs6060_config.clk = 27000000;
	m88rs6060_config.i2c_wr_max = 33;
	m88rs6060_config.ts_mode = cfg->ts_out_mode ? MtFeTsOutMode_Parallel : MtFeTsOutMode_Serial;
	m88rs6060_config.demod_adr = cfg->i2c_addr;
	m88rs6060_config.tuner_adr = 0x2c;
	m88rs6060_config.repeater_value = 0x11;

	aml_dvb_extern_reset(&cfg->reset);
	memset(&info, 0, sizeof(struct i2c_board_info));
	strlcpy(info.type, "m88rs6060", I2C_NAME_SIZE);
	info.addr = cfg->i2c_addr;
	info.platform_data = &m88rs6060_config;
	request_module(info.type);
	client = i2c_new_client_device(cfg->i2c_adap, &info);
	if(IS_ERR_OR_NULL(client) || IS_ERR_OR_NULL(client->dev.driver)) {
		pr_err("M88RS6060: failed to attach i2c client\n");
		return NULL;
	}
	if (!try_module_get(client->dev.driver->owner)) {
		i2c_unregister_device(client);
		pr_err("M88RS6060: failed to load module?\n");
		return NULL;
	}

	pr_info("M88RS6060: demod attached\n");
	return fe;
}

struct dvb_frontend *aml_av201x_attach(struct dvb_frontend *fe,
				       const struct tuner_config *cfg, av201x_id_t id)
{
	struct av201x_avl_config av201xcfg = {
		.i2c_address = cfg->i2c_addr,
		.id = id,
		.xtal_freq = cfg->xtal, /* XTAL Frequency in kHz */
	};
	aml_dvb_extern_reset(&cfg->reset);
	return av201x_avl_attach(fe, &av201xcfg, cfg->i2c_adap);
}

struct dvb_frontend *aml_av2011_attach(struct dvb_frontend *fe,
				       const struct tuner_config *cfg)
{
	return aml_av201x_attach(fe, cfg, ID_AV2011);
}

struct dvb_frontend *aml_av2012_attach(struct dvb_frontend *fe,
				       const struct tuner_config *cfg)
{
	return aml_av201x_attach(fe, cfg, ID_AV2012);
}

struct dvb_frontend *aml_av2018_attach(struct dvb_frontend *fe,
				       const struct tuner_config *cfg)
{
	return aml_av201x_attach(fe, cfg, ID_AV2018);
}

struct dvb_frontend *aml_cxd2878_attach(const struct demod_config *cfg)
{
	struct cxd2878_config cxd2878cfg = {
		.addr_slvt = cfg->i2c_addr,
		.xtal = cfg->xtal, /* XTAL Frequency, 0: 16MHz; 1: 24MHz, 2: 32MHz */
		.tuner_addr = 0, /* tuner handeled outside the demod driver */
		.tuner_xtal = 0,
		.ts_mode = cfg->ts_out_mode, /* ts_out_mode: serial or parallel; 0: serial, 1: parallel. */
		.ts_ser_data = cfg->ts_data_pin, /* Serial output pin of TS data. 0: Output from TSDATA0, 1: Output from TSDATA7 */
		.ts_clk = cfg->ts_clk, /* Serial TS clock gated on valid TS data or is continuous. 0: Gated, 1: Continuous */
		.ts_clk_mask = 1,
		.ts_valid = 0,
		.atscCoreDisable = 0,
		.lock_flag = 1,
		.write_properties = NULL,
		.read_properties = NULL,
		.write_eeprom = NULL,
		.read_eeprom = NULL,
		.RF_switch = NULL,
		.rf_port = 0,
		.TS_switch = NULL,
		.LED_switch = NULL,
	};

	struct dvb_frontend *fe = cxd2878_attach(&cxd2878cfg, cfg->i2c_adap);
	if (IS_ERR_OR_NULL(fe))
		return NULL;

	if (cfg->tuner0.id != AM_TUNER_NONE) {
		const struct tuner_module * tuner = aml_get_tuner_module(cfg->tuner0.id);
		if (tuner->attach(tuner, fe, &cfg->tuner0) == NULL) {
			pr_err("CXD2878: failed to attach tuner0 %s\n", tuner->name);
		}
	}
	else {
		pr_err("CXD2878: Missing tuner0 config\n");
	}

	return fe;
}

static int __init aml_dvb_extern_wrappers_init(void)
{
	tuner_attach_register_cb(AM_TUNER_MXL603, aml_mxl603_attach);
	tuner_attach_register_cb(AM_TUNER_R848, aml_r848_attach);
	tuner_attach_register_cb(AM_TUNER_R912, aml_r912_attach);
	tuner_attach_register_cb(AM_TUNER_AV2011, aml_av2011_attach);
	tuner_attach_register_cb(AM_TUNER_AV2012, aml_av2012_attach);
	tuner_attach_register_cb(AM_TUNER_AV2018, aml_av2018_attach);
	demod_attach_register_cb(AM_DTV_DEMOD_AVL68xx, aml_avl68xx_attach);
	demod_attach_register_cb(AM_DTV_DEMOD_AVL6762, aml_avl6762_attach);
	demod_attach_register_cb(AM_DTV_DEMOD_M88DM6K, aml_m88dm6k_attach);
	demod_attach_register_cb(AM_DTV_DEMOD_AVL6221C, aml_avl6221c_attach);
	demod_attach_register_cb(AM_DTV_DEMOD_CXD2878, aml_cxd2878_attach);
	return 0;
}

module_init(aml_dvb_extern_wrappers_init);

MODULE_DESCRIPTION("DVB demodulator driver wrappers for aml_dvb_extern module");
MODULE_AUTHOR("Marek Czerski (ma.czerski@gmail.com)");
MODULE_LICENSE("GPL");
