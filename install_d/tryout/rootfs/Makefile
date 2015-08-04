TOP = $(CURDIR)/..
include $(TOP)/CONFIG

# Ask the kernel build where it put the kernel build
KERNEL_DIR = $(shell \
    make -C $(TOP)/kernel -s --no-print-directory print-build-dir)
KERNEL_UIMAGE = $(KERNEL_DIR)/arch/arm/boot/uImage

TMBF_BASE = $(TOP)/../..
ROOTFS_ROOT = $(BUILD_ROOT)/rootfs

export FPGA_IMAGE
export TMBF_BASE
export KERNEL_NAME

ROOTFS = $(ROOTFS_TOP)/rootfs $1 TARGET=$(CURDIR) ROOTFS_ROOT=$(ROOTFS_ROOT)

ROOTFS_IMAGE = $(shell $(call ROOTFS,imagedir))

IMAGE = $(TMBF_BASE)/tryout-image

image: rootfs
	rm -rf $(IMAGE)
	mkdir -p $(IMAGE)
	cp $(ROOTFS_IMAGE)/* $(IMAGE)
	cp $(KERNEL_UIMAGE) $(IMAGE)/$(KERNEL_NAME)
	cp $(TOP)/README $(IMAGE)

rootfs:
	$(call ROOTFS,make)

clean:
	-chmod -R u+w $(BUILD_ROOT)
	rm -rf $(BUILD_ROOT)

clean-all: clean
	rm -rf $(IMAGE)


.PHONY: rootfs image clean clean-all
