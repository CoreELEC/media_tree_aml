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

#ifndef AV201X_TOP_H
#define AV201X_TOP_H

#include <linux/kconfig.h>
#include <media/dvb_frontend.h>
//#include "dvb_frontend.h"
#include "av201x_avl_top_priv.h"

typedef enum av201x_id {
	ID_AV2011,
	ID_AV2012,
	ID_AV2018,
} av201x_id_t;

struct av201x_avl_config {
	/* tuner i2c address */
	uint8_t i2c_address;
	/* tuner type */
	av201x_id_t id;

	/* crystal freq in kHz */
	uint32_t xtal_freq;
};

extern struct dvb_frontend *av201x_avl_attach(struct dvb_frontend *fe,
		struct av201x_avl_config *cfg, struct i2c_adapter *i2c);

#endif /* AV201X_H */
