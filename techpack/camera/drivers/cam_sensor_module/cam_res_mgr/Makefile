# SPDX-License-Identifier: GPL-2.0-only

ifeq ($(PICOVR_CAM_RM_PM8009_CONFIG),1)
ccflags-y += -DCAM_RM_PM8009_CONFIG=1
endif

#ccflags-y += -I$(srctree)/techpack/camera/include/uapi
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_core
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_cpas/include
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_req_mgr
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_cci
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_sensor_io
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_sensor_utils
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_utils

obj-$(CONFIG_SPECTRA_CAMERA) += cam_res_mgr.o
