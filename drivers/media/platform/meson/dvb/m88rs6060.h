/*
 *    Driver for DVBS/S2/S2X Satellite demod/tuner(all-in-one) M88RS6060.
 *
 *    Copyright (C) 2019 Igor Mokrushin aka McMCC <mcmcc@mail.ru>
 *
 *    - Code based on driver for Montage Technology M88RS6000 by Max Nibble <nibble.max@gmail.com>
 *    - DVB-S2X not supported, it was not possible to test!
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
#ifndef M88RS6060_H
#define M88RS6060_H

#include <linux/dvb/frontend.h>
#include <media/dvb_frontend.h>

struct m88rs6060_config {
	u8 demod_address; /* the demodulator's i2c address */
	u8 pin_ctrl; /* LNB pin control */
	u8 ci_mode; /* 0: no ci, others: ci mode */
	u8 ts_mode; /* 0: Parallel, 1: Serial */
	u8 tuner_readstops;
	u8 name_ext_fw[64]; /* Name external firmware for demodulator */
	/* Set device param to start dma */
	int (*set_ts_params)(struct dvb_frontend *fe, int is_punctured);
	/* Set LNB voltage */
	int (*set_voltage)(struct dvb_frontend* fe, enum fe_sec_voltage voltage);
};

extern struct dvb_frontend *m88rs6060_attach(struct m88rs6060_config *config, struct i2c_adapter *i2c);

#endif /* M88RS6060_H */
