#
# Makefile for the amlogic platform camera interface device drivers.
#

gt2005dri-objs := gt2005.o
gc0308dri-objs := gc0308.o
ov5642dri-objs := ov5642.o
amlvm-objs := common/vm.o common/plat_ctrl.o

obj-$(CONFIG_VIDEO_AMLOGIC_CAPTURE_GT2005) +=gt2005dri.o
obj-$(CONFIG_VIDEO_AMLOGIC_CAPTURE_GC0308) += gc0308dri.o
obj-$(CONFIG_VIDEO_AMLOGIC_CAPTURE_OV5642) += ov5642dri.o
obj-$(CONFIG_AMLOGIC_VIDEOIN_MANAGER)  += amlvm.o
