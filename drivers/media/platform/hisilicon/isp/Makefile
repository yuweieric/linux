# Makefile for hisi ISP driver

hisi_isp-objs := hisi_isp_core.o \
		hisi_isp_common.o \
		hisi_isp_csi.o \
		hisi_isp_sr.o \
		hisi_isp_cvdr.o \
		hisi_isp_isr.o \
		hisi_isp_i2c.o

hisi_ov5645-objs := ov5645.o

hisi_ov5640_main-objs := ov5640_main.o

hisi_ov5640_sub-objs := ov5640_sub.o

obj-$(CONFIG_VIDEO_HISI_ISP)	+= hisi_isp.o
obj-$(CONFIG_VEDIO_HISI_ISP)	+= hisi_ov5645.o
obj-$(CONFIG_VIDEO_HISI_ISP)	+= hisi_ov5640_main.o
obj-$(CONFIG_VIDEO_HISI_ISP)	+= hisi_ov5640_sub.o
