
#	Meson DVB drivers

obj-$(CONFIG_MESON_DVB) += dvb_meson.o

dvb_meson-objs = \
	aml_dvb_extern_wrappers.o \
	avl6211.o \
	mn88436.o \
	cxd2841er_wetek.o \
	ascot3.o \
	mxl603.o \
	mxl608.o \
	avl6862.o \
	r912.o \
	r848.o \
	tuner_ftm4862.o \
	rda5815m.o \
	avl6261_top.o \
	av201x_avl_top.o \
	av201x_avl_drv.o \
	avl_sdk/avl_bsp.o \
	avl_sdk/AVL62X1_API.o \
	avl_sdk/AVL62X1_DVBSx.o \
	avl_sdk/AVL62X1_Internal.o \
	m88rs6060.o \
	cxd2878.o \
	m88tc6800_priv.o \
	m88tc6800.o

EXTRA_CFLAGS += \
	-DDUAL_TUNER \
	-DCONFIG_MEDIA_TUNER_R848 \
	-I$(src)/avl_sdk \
	-Idrivers/media/dvb-core \
	-Idrivers/media/usb/dvb-usb \
	-Idrivers/media/dvb-frontends \
	-Idrivers/media/tuners
