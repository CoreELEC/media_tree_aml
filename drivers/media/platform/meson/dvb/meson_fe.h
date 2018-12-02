// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2018-present Team CoreELEC (https://coreelec.org)

 
#ifndef __MESON_FE_H
#define __MESON_FE__H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dvb/version.h>
#include <linux/platform_device.h>
#include "media/dvb_frontend.h"

struct ts_input {
	int                  mode;
	struct pinctrl      *pinctrl;
	int                  control;
};

struct fe_ops {
	struct dvb_frontend 	*fe;
	struct i2c_adapter 	*i2c;	
	struct ts_input	   	ts[3];
	struct device       	*dev;
	struct platform_device  *pdev;
	struct pinctrl      	*card_pinctrl;
	void __iomem 		*demux_base;
	void __iomem 		*afifo_base;
	int			demux_irq;
	int			afifo_irq;
	u32 total_nims;
#ifdef CONFIG_ARM64
	int fec_reset;
	int power_ctrl;
#endif
};

void get_fe_ops(struct fe_ops *p);
int set_external_vol_gpio(int *demod_id, int on);

#endif /* __MESON_FE__H */
