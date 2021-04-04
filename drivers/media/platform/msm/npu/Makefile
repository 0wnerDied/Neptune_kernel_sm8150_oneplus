msm_npu-objs := npu_dbg.o \
                npu_dev.o \
                npu_debugfs.o \
                npu_host_ipc.o \
                npu_hw_access.o \
                npu_mgr.o
ifeq ($(CONFIG_VIRTIO_NPU),y)
obj-$(CONFIG_MSM_NPU)      := virtio_npu.o
else
obj-$(CONFIG_MSM_NPU)      := msm_npu.o
endif
