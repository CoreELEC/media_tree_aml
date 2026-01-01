// SPDX-License-Identifier: GPL-2.0

#include <linux/firmware.h>
#include "m88tc6800_priv.h"

#define M88TC6800_FIRMWARE "dvb-tuner-m88tc6800.fw"

int m88tc6800_write_reg(struct m88tc6800_priv *priv, u8 reg, u8 val)
{
	u8 buf[] = { reg, val };
	struct i2c_msg msg = { .addr = priv->config.addr, .flags = 0,
			       .buf = buf, .len = 2 };
	int ret;

	ret = i2c_transfer(priv->i2c, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		dev_warn(&priv->i2c->dev, "i2c wr failed=%d reg=%02x ", ret, reg);
		ret = -EREMOTEIO;
	}
	return ret;
}

static int m88tc6800_write_data(struct m88tc6800_priv *priv, u8 *data, size_t len)
{
	struct i2c_msg msg = { .addr = priv->config.addr, .flags = 0,
			       .buf = data, .len = len };
	int ret;

	ret = i2c_transfer(priv->i2c, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		dev_warn(&priv->i2c->dev, "i2c data failed=%d ", ret);
		ret = -EREMOTEIO;
	}
	return ret;
}

int m88tc6800_read_reg(struct m88tc6800_priv *priv, u8 reg, u8 *val)
{
	
	u8 buf[1] = { reg };
	struct i2c_msg msg[] = {
		{ .addr = priv->config.addr, .flags = 0,
		  .buf = buf, .len = 1 },
		{ .addr = priv->config.addr, .flags = I2C_M_RD,
		  .buf = val, .len = 1 },
	};
	int ret;

	ret = i2c_transfer(priv->i2c, msg, 2);
	if (ret == 2) {
		ret = 0;
	} else {
		dev_warn(&priv->i2c->dev, "i2c rd failed=%d reg=%02x ", ret, reg);
		ret = -EREMOTEIO;
	}
	return ret;
}

static int m88tc6800_write_fw(struct m88tc6800_priv *priv)
{
	u8 expected_fw_ver = 0xc;
	u8 reg_data;
	u8 buf[65] = {0xc0};
	const struct firmware *fw;
	int ret;
	size_t i;

	ret = request_firmware(&fw, M88TC6800_FIRMWARE, &priv->i2c->dev);
	if (ret) {
		dev_err(&priv->i2c->dev, "M88TC6800 firmware file '%s' not found\n",
			M88TC6800_FIRMWARE);
		return ret;
	}

	dev_info(&priv->i2c->dev, "M88TC6800 uploading firmware '%s'\n",
		 M88TC6800_FIRMWARE);

	m88tc6800_set_reg_bits(priv, 0x05, 1, 2, 2);
	m88tc6800_write_reg(priv, 0xc2, 1);

	for (i = 0; i < fw->size; i += (sizeof(buf) - 1)) {
		size_t len = min(sizeof(buf) - 1, fw->size - i);
		memcpy(&buf[1], &fw->data[i], len);
		ret = m88tc6800_write_data(priv, buf, len + 1);
		if (ret) {
			release_firmware(fw);
			return ret;
		}
	}

	release_firmware(fw);

	m88tc6800_write_reg(priv, 0xc2, 0);

	/* check fw upload status */
	m88tc6800_read_reg(priv, 0xc8, &reg_data);
	dev_info(&priv->i2c->dev, "M88TC6800 fw version=%x\n", reg_data);
	m88tc6800_set_reg_bits(priv, 0x05, 0, 2, 2);

	if (reg_data == expected_fw_ver) {
		priv->mcu_status = true;
		m88tc6800_set_reg_bits(priv, 0x3e, 0, 5, 5);
		return 0;
	}

	dev_err(&priv->i2c->dev, "M88TC6800 fw version mismatch, expected %x\n", expected_fw_ver);
	priv->mcu_status = false;
	return -1;
}

static int m88tc6800_init(struct dvb_frontend *fe)
{
	struct m88tc6800_priv *priv = fe->tuner_priv;
	u8 reg_data;
	int ret;

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	ret = m88tc6800_read_reg(priv, 0x3e, &reg_data);
	if (ret)
		goto err;

	if ((reg_data & 0x20) == 0 && priv->mcu_status == true)
		goto done;

	ret = m88tc6800_write_fw(priv);
	if (ret)
		goto err;

done:
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);
	return 0;

err:
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);
	dev_err(&priv->i2c->dev, "%s: failed=%d\n", __func__, ret);
	return ret;
}

static int m88tc6800_set_params(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct m88tc6800_priv *priv = fe->tuner_priv;
	int ret;
	u32 bw;
	u32 freq_khz = c->frequency / 1000;
	
	dev_dbg(&priv->i2c->dev, 
		"%s: delivery_system=%d frequency=%d bandwidth_hz=%d\n", 
		__func__, c->delivery_system, c->frequency, c->bandwidth_hz);

	if (c->bandwidth_hz <= 6000000) {
		bw = 6000;
	}
	else if (c->bandwidth_hz <= 7000000) {
		bw = 7000;
	}
	else {
		bw = 8000;
	}

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	ret = m88tc6800_set_freq(priv, freq_khz, bw);
	if (ret)
		goto err;

	priv->frequency_hz = freq_khz * 1000;
	priv->bandwidth_hz = bw;

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	return 0;

err:
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	dev_err(&priv->i2c->dev, "%s: failed=%d\n", __func__, ret);
	return ret;
}

static int m88tc6800_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct m88tc6800_priv *priv = fe->tuner_priv;
	*frequency = priv->frequency_hz;
	return 0;
}

static int m88tc6800_get_bandwidth(struct dvb_frontend *fe, u32 *bandwidth)
{
	struct m88tc6800_priv *priv = fe->tuner_priv;
	*bandwidth = priv->bandwidth_hz;
	return 0;
}

static int m88tc6800_get_rf_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct m88tc6800_priv *priv = fe->tuner_priv;
	int power;

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	m88tc6800_get_rf_power(priv, &power);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	return 0;
}

static void m88tc6800_release(struct dvb_frontend *fe)
{
	struct m88tc6800_priv *priv = fe->tuner_priv;

	kfree(priv);
	fe->tuner_priv = NULL;
	
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);
}

static struct dvb_tuner_ops m88tc6800_tuner_ops = {
	.info = {
		.name = "Montage M88TC6800",
		.frequency_min_hz = 48 * MHz,
		.frequency_max_hz = 900 * MHz,
		.frequency_step_hz = 1 * kHz,
	},
	.init              = m88tc6800_init,
	.set_params        = m88tc6800_set_params,
	.get_frequency     = m88tc6800_get_frequency,
	.get_bandwidth     = m88tc6800_get_bandwidth,
	.get_rf_strength   = m88tc6800_get_rf_strength,
	.release           = m88tc6800_release,
};

struct dvb_frontend *m88tc6800_attach(struct dvb_frontend *fe,
				     struct i2c_adapter *i2c,
				     struct m88tc6800_config *config)
{
	struct m88tc6800_priv *priv = NULL;
	int ret;

	priv = kzalloc(sizeof(struct m88tc6800_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&i2c->dev, "kzalloc() failed\n");
		return NULL;
	}

	memcpy(&priv->config, config, sizeof(priv->config));
	priv->i2c = i2c;
	priv->mcu_status = false;

	dev_info(&i2c->dev, "Attaching M88TC6800, driver version " M88TC6800_DRIVER_VERSION "\n");
	fe->tuner_priv = priv;

	ret = m88tc6800_init(fe);
	if (ret)
		goto err;

	memcpy(&fe->ops.tuner_ops, &m88tc6800_tuner_ops,
	       sizeof(struct dvb_tuner_ops));

	return fe;
err:
	kfree(priv);
	return NULL;
}
EXPORT_SYMBOL(m88tc6800_attach);

MODULE_DESCRIPTION("Montage M88TC6800 tuner driver");
MODULE_AUTHOR("Marek Czerski <ma.czerski@gmail.com>");
MODULE_VERSION(M88TC6800_DRIVER_VERSION);
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(M88TC6800_FIRMWARE);
