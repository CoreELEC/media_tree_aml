// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2018-present Team CoreELEC (https://coreelec.org)

#include <linux/reset.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/usb.h>
#include <linux/platform_device.h>
#include "meson_fe.h"

#include "avl6862.h"
#include "r912.h"
#include "ascot3.h"
#include "cxd2837.h"
#include "mxl603.h"
#include "avl6211.h"
#include "mn88436.h"
#include "c_stb_regs_define.h"
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
static struct clk *dvb_demux_clk_ctl;
static struct clk *dvb_afifo_clk_ctl;
static struct clk *dvb_ahbarb0_clk_ctl;
static struct clk *dvb_uparsertop_clk_ctl;
static struct reset_control *dvb_demux_reset_ctl;
static struct reset_control *dvb_async0_reset_ctl;
static struct reset_control *dvb_dmx0_reset_ctl;
#else
static struct reset_control *dvb_demux_reset_ctl;
static struct reset_control *dvb_afifo_reset_ctl;
static struct reset_control *dvb_ahbarb0_reset_ctl;
static struct reset_control *dvb_uparsertop_reset_ctl;
#endif

#define TOTAL_AML_INPUTS 	1


static struct fe_ops meson_dvb;

static struct r912_config r912cfg = {
	.i2c_address = 0x7A,	
};
static struct avl6862_config avl6862cfg = {
	.demod_address = 0x14,
	.tuner_address = 0x7A,
	.ts_serial = 0,
};
static struct avl6862_config avl6762cfg = {
	.demod_address = 0x14,
	.tuner_address = 0,
	.ts_serial = 0,
};
static struct cxd2837_cfg cxd2837cfg = {
		.adr = 0x6C,
		.if_agc_polarity = 1,
		.rfain_monitoring = 0,
		.ts_error_polarity = 0,
		.clock_polarity = 1,
		.ifagc_adc_range = 0,
		.spec_inv = 0,
		.xtal = XTAL_20500KHz,
		.ts_clock = SERIAL_TS_CLK_MID_FULL,
};
struct ascot3_config ascot3cfg = {
		.i2c_address = 0x60,
};
static struct mxl603_config mxl603cfg = {
		.xtal_freq_hz = MXL603_XTAL_24MHz,
		.if_freq_hz = MXL603_IF_5MHz,
		.agc_type = MXL603_AGC_SELF,
		.xtal_cap = 16,
		.gain_level = 11,
		.if_out_gain_level = 11,
		.agc_set_point = 66,
		.agc_invert_pol = 0,
		.invert_if = 1,
		.loop_thru_enable = 0,
		.clk_out_enable = 1,
		.clk_out_div = 0,
		.clk_out_ext = 0,
		.xtal_sharing_mode = 0,
		.single_supply_3_3V = 1,
};
static struct mxl603_config mxl603cfg_atsc = {
		.xtal_freq_hz = MXL603_XTAL_24MHz,
		.if_freq_hz = MXL603_IF_5MHz,
		.agc_type = MXL603_AGC_EXTERNAL,
		.xtal_cap = 31,
		.gain_level = 11,
		.if_out_gain_level = 11,
		.agc_set_point = 66,
		.agc_invert_pol = 0,
		.invert_if = 0,
		.loop_thru_enable = 0,
		.clk_out_enable = 1,
		.clk_out_div = 0,
		.clk_out_ext = 0,
		.xtal_sharing_mode = 0,
		.single_supply_3_3V = 1,
};
static struct mxl603_config mxl608cfg = {
		.xtal_freq_hz = MXL603_XTAL_16MHz,
		.if_freq_hz = MXL603_IF_4_1MHz,
		.agc_type = MXL603_AGC_SELF,
		.xtal_cap = 12,
		.gain_level = 11,
		.if_out_gain_level = 11,
		.agc_set_point = 66,
		.agc_invert_pol = 0,
		.invert_if = 1,
		.loop_thru_enable = 0,
		.clk_out_enable = 1,
		.clk_out_div = 0,
		.clk_out_ext = 0,
		.xtal_sharing_mode = 0,
		.single_supply_3_3V = 1,
};

static struct avl6211_config avl6211cfg[] = {
	{
		.tuner_address = 0xC4,
		.tuner_i2c_clock = 200,	
		.demod_address = 0x0C,
		.mpeg_pol = 1,
		.mpeg_mode = 0,
		.mpeg_format = 0,
		.demod_refclk = 9,
		.mpeg_pin = 0,
		.tuner_rfagc = 1,
		.tuner_spectrum = 0,
		.use_lnb_pin59 = 1,
		.use_lnb_pin60 = 0,
		.set_external_vol_gpio = set_external_vol_gpio,
	},
};


void get_fe_ops(struct fe_ops *p)
{
	memcpy(p, &meson_dvb, sizeof(struct fe_ops));
}
EXPORT_SYMBOL(get_fe_ops);

int set_external_vol_gpio(int *demod_id, int on)
{ 
	int ret = 0;

	if (on)
		on = 1;

	if (*demod_id == 0) 
		ret = gpio_direction_output(meson_dvb.power_ctrl, on);

	return ret;
}


void reset_demod(void)
{
	gpio_direction_output(meson_dvb.fec_reset, 0);
	msleep(600);
	gpio_direction_output(meson_dvb.fec_reset, 1);
	msleep(200);

}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
                           
int aml_read_cbus(unsigned int reg)
{	
	int val;
	val = readl(meson_dvb.demux_base + ((reg - STB_VERSION) << 2));
	return val;
}
void aml_write_cbus(unsigned int reg, unsigned int val)
{
	writel(val, meson_dvb.demux_base + ((reg - STB_VERSION) << 2));
}
int aml_read_vcbus(unsigned int reg)
{	
	int val;
	val = readl(meson_dvb.afifo_base + ((reg - ASYNC_FIFO_REG0) << 2));
	return val;
}
void aml_write_vcbus(unsigned int reg, unsigned int val)
{
	writel(val, meson_dvb.afifo_base + ((reg - ASYNC_FIFO_REG0) << 2));
}
EXPORT_SYMBOL(aml_read_cbus);
EXPORT_SYMBOL(aml_write_cbus);
EXPORT_SYMBOL(aml_read_vcbus);
EXPORT_SYMBOL(aml_write_vcbus);
#endif

static int fe_dvb_probe(struct platform_device *pdev)
{
	int i;
	u32 i2c = 1;
	int ret = 0;
	const char *str;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	struct resource *r;
#else
        struct device_node *np;
#endif
	meson_dvb.pdev = pdev;
	meson_dvb.dev  = &pdev->dev;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "stbtop");
	meson_dvb.demux_base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(meson_dvb.demux_base)) {
		dev_err(&pdev->dev, "Couldn't remap STBTOP memory\n");
		return PTR_ERR(meson_dvb.demux_base);
	}
 
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "afifo");
	meson_dvb.afifo_base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(meson_dvb.afifo_base)) {
		dev_err(&pdev->dev, "Couldn't remap AFIFO memory\n");
		return PTR_ERR(meson_dvb.afifo_base);
	}
	of_property_read_u32(pdev->dev.of_node, "i2c_adap_id", &i2c);
#else
        np = of_find_node_by_name(NULL, "dvbfe");
	if (np)
		of_property_read_u32(np, "dtv_demod0_i2c_adap_id", &i2c);
#endif
	dev_info(&pdev->dev, "i2c_adap_id=%d\n", i2c);	
	ret = of_property_read_string(pdev->dev.of_node, "dev_name", &str);
	if (!ret) 
		dev_info(&pdev->dev, "dev_name=%s\n", str);

	if (pdev->dev.of_node) {
		for (i = 0; i <  TOTAL_AML_INPUTS; i++) {
			char buf[32];
			const char *str;

			snprintf(buf, sizeof(buf), "ts%d", i);
			ret = of_property_read_string(pdev->dev.of_node, buf, &str);
			if (!ret) {
				if (!strcmp(str, "parallel")) {
					dev_info(&pdev->dev, "%s: parallel\n", buf);
					snprintf(buf, sizeof(buf), "p_ts%d", i);
					meson_dvb.ts[i].mode    = 0;
					meson_dvb.ts[i].pinctrl = devm_pinctrl_get_select(&pdev->dev, buf);
				}
				else if (!strcmp(str, "serial")) {
					dev_info(&pdev->dev, "%s: serial\n", buf);
					snprintf(buf, sizeof(buf), "s_ts%d", i);
					meson_dvb.ts[i].mode    = 1;
					meson_dvb.ts[i].pinctrl = devm_pinctrl_get_select(&pdev->dev, buf);
				}
			}
		}
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	dvb_demux_clk_ctl = devm_clk_get(&pdev->dev, "demux");
	dev_info(&pdev->dev, "dmx clk ctl = %p\n", dvb_demux_clk_ctl);
	clk_prepare_enable(dvb_demux_clk_ctl);

	dvb_afifo_clk_ctl = devm_clk_get(&pdev->dev, "asyncfifo");
	dev_info(&pdev->dev, "asyncfifo clk ctl = %p\n", dvb_afifo_clk_ctl);
	clk_prepare_enable(dvb_afifo_clk_ctl);
	
	dvb_ahbarb0_clk_ctl = devm_clk_get(&pdev->dev, "ahbarb0");
	dev_info(&pdev->dev, "ahbarb0 clk ctl = %p\n", dvb_ahbarb0_clk_ctl);
	clk_prepare_enable(dvb_ahbarb0_clk_ctl);
	
	dvb_uparsertop_clk_ctl = devm_clk_get(&pdev->dev, "uparsertop");
	dev_info(&pdev->dev, "uparsertop clk ctl = %p\n", dvb_uparsertop_clk_ctl);
	clk_prepare_enable(dvb_uparsertop_clk_ctl);

	dvb_demux_reset_ctl = devm_reset_control_get(&pdev->dev, "demux");
	dev_info(&pdev->dev, "dmx rst ctl = %p\n", dvb_demux_reset_ctl);
	reset_control_deassert(dvb_demux_reset_ctl);

	dvb_async0_reset_ctl = devm_reset_control_get(&pdev->dev, "async0");
	dev_info(&pdev->dev, "async0 rst ctl = %p\n", dvb_async0_reset_ctl);
	reset_control_deassert(dvb_async0_reset_ctl);
	
	dvb_dmx0_reset_ctl = devm_reset_control_get(&pdev->dev, "demuxreset0");
	dev_info(&pdev->dev, "dmx0 rst ctl = %p\n", dvb_dmx0_reset_ctl);
	reset_control_deassert(dvb_dmx0_reset_ctl);

	meson_dvb.demux_irq = platform_get_irq_byname(pdev, "demux");
	if (meson_dvb.demux_irq < 0) {
		dev_err(&pdev->dev, "can't find IRQ for demux\n");
		return meson_dvb.demux_irq;
	}
	dev_info(&pdev->dev, "demux irq = %d\n", meson_dvb.demux_irq);

	meson_dvb.afifo_irq = platform_get_irq_byname(pdev, "asyncfifo");
	if (meson_dvb.afifo_irq < 0) {
		dev_err(&pdev->dev, "can't find IRQ for asyncfifo\n");
		return meson_dvb.afifo_irq;
	}
	dev_info(&pdev->dev, "asyncfifo irq = %d\n", meson_dvb.afifo_irq);


#else
	dvb_demux_reset_ctl = devm_reset_control_get(&pdev->dev, "demux");
	dev_info(&pdev->dev, "dmx rst ctl = %p\n", dvb_demux_reset_ctl);
	reset_control_deassert(dvb_demux_reset_ctl);

	dvb_afifo_reset_ctl = devm_reset_control_get(&pdev->dev, "asyncfifo");
	dev_info(&pdev->dev, "asyncfifo rst ctl = %p\n", dvb_afifo_reset_ctl);
	reset_control_deassert(dvb_afifo_reset_ctl);
	
	dvb_ahbarb0_reset_ctl = devm_reset_control_get(&pdev->dev, "ahbarb0");
	dev_info(&pdev->dev, "ahbarb0 rst ctl = %p\n", dvb_ahbarb0_reset_ctl);
	reset_control_deassert(dvb_ahbarb0_reset_ctl);
	
	dvb_uparsertop_reset_ctl = devm_reset_control_get(&pdev->dev, "uparsertop");
	dev_info(&pdev->dev, "uparsertop rst ctl = %p\n", dvb_uparsertop_reset_ctl);
	reset_control_deassert(dvb_uparsertop_reset_ctl);
#endif
	meson_dvb.fec_reset = of_get_named_gpio_flags(pdev->dev.of_node, "fec_reset_gpio-gpios", 0, NULL);
	meson_dvb.power_ctrl = of_get_named_gpio_flags(pdev->dev.of_node, "power_ctrl_gpio-gpios", 0, NULL);
 
	/* FEC_RESET  GPIOY 13*/
	gpio_request(meson_dvb.fec_reset, "meson_dvb"); 
	
	/* INPUT1 POWER CTRL GPIOY 15*/
	gpio_request(meson_dvb.power_ctrl, "meson_dvb"); 
	
	
	meson_dvb.i2c = i2c_get_adapter(i2c);
	dev_info(&pdev->dev, "DVB demod detection in progress ...\n");

	/* RESET DEMOD(s) */
	reset_demod();
	if (meson_dvb.i2c != NULL) {
		dev_info(&pdev->dev, "Found i2c-%d adapter: %s\n", i2c, meson_dvb.i2c->name);

		if (strcmp(str,"wetek-dvb")) {
			if (strcmp(str,"avl6762")) {
				dev_info(&pdev->dev, "Checking for Availink AVL6862 DVB-S2/T2/C demod ...\n");
				avl6862cfg.ts_serial = meson_dvb.ts[0].mode;
				meson_dvb.fe = avl6862_attach(&avl6862cfg, meson_dvb.i2c);
				if (meson_dvb.fe == NULL) {
					dev_info(&pdev->dev, "Failed to find AVL6862 demod!\n");
					goto no_demod;
				}								
				if (r912_attach(meson_dvb.fe, &r912cfg, meson_dvb.i2c) == NULL) {
					dev_info(&pdev->dev, "Failed to find Rafael R912 tuner!\n");
					dev_info(&pdev->dev, "Detaching Availink AVL6268 frontend!\n");
					dvb_frontend_detach(meson_dvb.fe);
					goto no_demod;
				}
			}		
			dev_info(&pdev->dev, "Checking for Availink AVL6762 DVB-T2/C demod ...\n");
			meson_dvb.fe = avl6862_attach(&avl6762cfg, meson_dvb.i2c);
			if (meson_dvb.fe == NULL) {
				dev_info(&pdev->dev, "Failed to find AVL6762 demod!\n");
				goto no_demod;
			}								
			if (mxl603_attach(meson_dvb.fe, meson_dvb.i2c, 0x60, &mxl608cfg) == NULL) {
				dev_info(&pdev->dev, "Failed to find MxL 608 tuner!\n");
				dev_info(&pdev->dev, "Detaching Availink AVL6268 frontend!\n");
				dvb_frontend_detach(meson_dvb.fe);
				goto no_demod;
			}
			meson_dvb.total_nims++;
			dev_info(&pdev->dev, "Total DVB modules found: %d\n", meson_dvb.total_nims);
			return 0;
		}
		reset_demod();
		dev_info(&pdev->dev, "Checking for AVL6211 DVB-S/S2 demod ...\n");
		meson_dvb.fe = avl6211_attach( meson_dvb.i2c, &avl6211cfg[0], 0);
		if (meson_dvb.fe == NULL) {
			dev_info(&pdev->dev, "Failed to find AVL6211 demod!\n");
			goto panasonic;
		}
		meson_dvb.total_nims++;
		dev_info(&pdev->dev, "Total DVB modules found: %d\n", meson_dvb.total_nims);
		return 0;
panasonic:
		reset_demod();
		dev_info(&pdev->dev, "Checking for Panasonic MN88436 ATSC demod ...\n");	
			
		meson_dvb.fe =  mn88436_attach(meson_dvb.i2c, 0);

		if (meson_dvb.fe != NULL) {			
												
			if (mxl603_attach(meson_dvb.fe, meson_dvb.i2c, 0x60, &mxl603cfg_atsc) == NULL) {
				dev_info(&pdev->dev, "Failed to find MxL603 tuner!\n");
				dev_info(&pdev->dev, "Detaching Panasonic MN88436 ATSC frontend!\n");
				dvb_frontend_detach(meson_dvb.fe);
				goto sony;
			}
				
			meson_dvb.total_nims++;
			dev_info(&pdev->dev, "Total DVB modules found: %d\n", meson_dvb.total_nims);
			return 0;
		}
sony:
		reset_demod();
		dev_info(&pdev->dev, "Checking for Sony CXD2837 DVB-C/T/T2 demod ...\n");

		meson_dvb.fe =  cxd2837_attach(meson_dvb.i2c, &cxd2837cfg);

		if (meson_dvb.fe != NULL) {
			if (mxl603_attach(meson_dvb.fe, meson_dvb.i2c, 0x60, &mxl603cfg) == NULL) {
				dev_info(&pdev->dev, "Failed to find MxL603 tuner!\n");
				dev_info(&pdev->dev, "Detaching Sony CXD2837 DVB-C/T/T2 frontend!\n");
				dvb_frontend_detach(meson_dvb.fe);
				return 0;
			}

			meson_dvb.total_nims++;
			dev_info(&pdev->dev, "Total DVB modules found: %d\n", meson_dvb.total_nims);
			return 0;
		}
		i2c_put_adapter(meson_dvb.i2c);
	        meson_dvb.i2c = NULL;
	}
no_demod:
	if (meson_dvb.i2c != NULL)
		i2c_put_adapter(meson_dvb.i2c);
	
	return -EINVAL;
}
static int meson_dvb_remove(struct platform_device *pdev)
{
	if (meson_dvb.fe != NULL) 
		dvb_frontend_detach(meson_dvb.fe);

	if (meson_dvb.i2c != NULL)
		i2c_put_adapter(meson_dvb.i2c);

	gpio_free(meson_dvb.fec_reset);
	gpio_free(meson_dvb.power_ctrl);
	devm_pinctrl_put(meson_dvb.ts[0].pinctrl);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	clk_disable_unprepare(dvb_uparsertop_clk_ctl);
	c lk_disable_unprepare(dvb_ahbarb0_clk_ctl);
	clk_disable_unprepare(dvb_afifo_clk_ctl);
	clk_disable_unprepare(dvb_demux_clk_ctl);
#else
	reset_control_assert(dvb_uparsertop_reset_ctl);
	reset_control_assert(dvb_ahbarb0_reset_ctl);
	reset_control_assert(dvb_afifo_reset_ctl);
	reset_control_assert(dvb_demux_reset_ctl);
#endif
	return 0;
}
static const struct of_device_id meson_dvb_dt_match[] = {
	{
		.compatible = "amlogic,dvb",
	},
	{},
};
static struct platform_driver meson_dvb_detection = {
	.probe		= fe_dvb_probe,
	.remove		= meson_dvb_remove,
	.driver		= {
		.name	= "meson-dvb",
		.owner	= THIS_MODULE,
		.of_match_table = meson_dvb_dt_match,
	}
};

int __init meson_dvb_init(void)
{
	int ret;
	
	memset(&meson_dvb, 0, sizeof(struct fe_ops));
	
	ret = platform_driver_register(&meson_dvb_detection);
	return ret;
}
void __exit meson_dvb_exit(void)
{
	platform_driver_unregister(&meson_dvb_detection);
}

module_init(meson_dvb_init);
module_exit(meson_dvb_exit);

MODULE_DESCRIPTION("Meson DVB demods detection");
MODULE_AUTHOR("afl1 <afl2001@gmail.com>");
MODULE_LICENSE("GPL");
