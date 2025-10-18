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
#include "mxl603.h"
#include "m88rs6060.h"

struct dvb_frontend *aml_avl68xx_attach(const struct demod_config *cfg)
{
	struct avl6862_config avl68xxcfg = {
		.demod_address = cfg->i2c_addr,
		.dual_tuner = cfg->tuner1.id != AM_TUNER_NONE ? 1 : 0,
		.ts_serial = cfg->ts_out_mode ? 0 : 1, /* ts_out_mode: serial or parallel; 0: serial, 1: parallel. */
		.gpio_lock_led = 0,
	};

	struct dvb_frontend *fe = avl6862_attach(&avl68xxcfg, cfg->i2c_adap);
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
	return mxl603_attach(fe, cfg->i2c_adap, cfg->i2c_addr, &mxl603cfg);
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
EXPORT_SYMBOL_GPL(aml_m88dm6k_attach);

static int __init aml_dvb_extern_wrappers_init(void)
{
	demod_attach_register_cb(AM_DTV_DEMOD_AVL68xx, aml_avl68xx_attach);
	demod_attach_register_cb(AM_DTV_DEMOD_M88DM6K, aml_m88dm6k_attach);
	tuner_attach_register_cb(AM_TUNER_MXL603, aml_mxl603_attach);
	return 0;
}

module_init(aml_dvb_extern_wrappers_init);

MODULE_DESCRIPTION("DVB demodulator driver wrappers for aml_dvb_extern module");
MODULE_AUTHOR("Marek Czerski (ma.czerski@gmail.com)");
MODULE_LICENSE("GPL");
