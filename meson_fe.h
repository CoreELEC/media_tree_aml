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

#define TOTAL_AML_INPUTS 	2
#define BASE_IRQ 32
#define AM_IRQ(reg)             (reg + BASE_IRQ)
#define INT_DEMUX               AM_IRQ(23)
#define INT_DEMUX_1             AM_IRQ(5)
#define INT_DEMUX_2             AM_IRQ(53)
#define INT_ASYNC_FIFO_FILL     AM_IRQ(18)
#define INT_ASYNC_FIFO_FLUSH    AM_IRQ(19)
#define INT_ASYNC_FIFO2_FILL    AM_IRQ(24)
#define INT_ASYNC_FIFO2_FLUSH   AM_IRQ(25)

struct ts_input {
	int                  mode;
	struct pinctrl      *pinctrl;
	int                  control;
};

struct fe_ops {
	struct dvb_frontend 	*fe[TOTAL_AML_INPUTS];
	struct i2c_adapter 	*i2c[TOTAL_AML_INPUTS];
	struct ts_input	   	ts[3];
	struct device       	*dev;
	struct platform_device  *pdev;
	struct pinctrl      	*card_pinctrl;
	void __iomem 		*demux_base;
	void __iomem 		*afifo_base;
	int			demux_irq[TOTAL_AML_INPUTS];
	int			afifo_irq[TOTAL_AML_INPUTS];
	u32 			total_nims;
	int 			fec_reset[TOTAL_AML_INPUTS];
	int 			power_ctrl[TOTAL_AML_INPUTS];
};

void get_fe_ops(struct fe_ops *p);
int set_external_vol_gpio(int *demod_id, int on);

#endif /* __MESON_FE__H */
