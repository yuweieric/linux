TOP      := drivers/../..
VFMW_DIR := drivers/vcodec/vdec_hivna

include $(VFMW_DIR)/vfmw_v4.0/vfmw_make.cfg

EXTRA_CFLAGS += -Idrivers/vcodec/vdec_hivna
EXTRA_CFLAGS += -Idrivers/vcodec/vdec_hivna/omxvdec
EXTRA_CFLAGS += $(VFMW_CFLAGS) -fno-pic

#build in
obj-$(CONFIG_HI_VCODEC_VDEC) += hi_vfmw.o
hi_vfmw-objs := $(VFMW_CFILES)
