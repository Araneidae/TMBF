TOP = $(CURDIR)/..

include $(TOP)/CONFIG

# Where the build will take place.  Sources and build will be subdirectories
# of this.
KERNEL_ROOT = $(BUILD_ROOT)/kernel

SOURCE_ROOT = $(KERNEL_ROOT)/src
BUILD_DIR = $(KERNEL_ROOT)/build
HEADERS_DIR = $(KERNEL_ROOT)/headers
TAR_DIRS = /dls_sw/prod/targetOS/tar-files /dls_sw/work/targetOS/tar-files


# A sensible default build target, overridden in configs/$(TARGET)/CONFIG.
BUILD_TARGET = all
include CONFIG


default: build


# If no external build directory is specified, extract and patch the sources
# as specified
SOURCE_DIR = $(SOURCE_ROOT)/linux-$(LINUX_VERSION)

# Source extraction is somewhat involved.  We have to search through what may
# be a list of tar directories to find the tar file, which we then checksum
# against the given MD5_SUM before untarring.  Then we apply all the patches,
# again taking care to catch any failures.  Finally the directory is touched
# to try and avoid redundant rebuilds.
$(SOURCE_DIR):
	mkdir -p $(SOURCE_ROOT)
	for dir in $(TAR_DIRS); do \
            tar_file="$$dir/linux-$(LINUX_VERSION).tar.bz2"; \
            [ -e "$$tar_file" ]  && \
            echo "$(MD5_SUM_$(LINUX_VERSION))  $$tar_file" | md5sum -c  && \
            tar xjf "$$tar_file" -C $(SOURCE_ROOT)  && \
            exit 0; \
        done; exit 1
	for file in $(PATCHES); do \
            patch -p1 -d$(SOURCE_DIR) \
                < $$file  || \
            exit 1; \
        done
	touch $(SOURCE_DIR)


# Export the symbols needed by the kernel build.
export CROSS_COMPILE = $(BINUTILS_DIR)/bin/$(COMPILER_PREFIX)-
export KBUILD_OUTPUT = $(BUILD_DIR)
export ARCH

$(BUILD_DIR)/.config: $(SOURCE_DIR) $(CONFIG)
	mkdir -p $(BUILD_DIR)
	cp $(CONFIG) $@
	$(MAKE) -C $(SOURCE_DIR) oldconfig

build: $(BUILD_DIR)/.config
	$(MAKE) -C $(SOURCE_DIR) $(BUILD_TARGET)

source: $(SOURCE_DIR)
oldconfig: $(BUILD_DIR)/.config

save-config:
	cp -a $(BUILD_DIR)/.config $(CONFIG)


clean:
	rm -rf $(BUILD_DIR)
# 	$(MAKE) -C $(SOURCE_DIR) distclean

clean-all: clean
	rm -rf $(SOURCE_ROOT)
	-rmdir $(KERNEL_ROOT)

print-build-dir:
	@echo -n $(BUILD_DIR)


# Target to install complete build framework in target directory $(HEADERS_DIR)
# Alas this doesn't appear to be supported as part of the kernel.  The code
# below may or may not actually work.
HEADER_BUILD_FILES = include scripts Module.symvers
HEADER_SOURCE_FILES = include scripts Makefile
linux-headers:
	mkdir -p $(HEADERS_DIR)
	cp -a $(HEADER_SOURCE_FILES:%=$(SOURCE_DIR)/%) $(HEADERS_DIR)
	cp -a $(HEADER_BUILD_FILES:%=$(BUILD_DIR)/%) $(HEADERS_DIR)
	mkdir -p $(HEADERS_DIR)/arch/$(ARCH)
	cp -a $(SOURCE_DIR)/arch/$(ARCH)/Makefile $(HEADERS_DIR)/arch/$(ARCH)
	cp -a $(SOURCE_DIR)/arch/$(ARCH)/include $(HEADERS_DIR)/arch/$(ARCH)
	cp -a $(BUILD_DIR)/arch/$(ARCH)/include $(HEADERS_DIR)/arch/$(ARCH)
	# Platform specific includes also need to be installed under arch.
	for dir in $(wildcard $(BUILD_DIR)/arch/$(ARCH)/*); do \
            dir=$$(basename $$dir); \
            srcdir=$(SOURCE_DIR)/arch/$(ARCH)/$$dir/include; \
            if [ -d $$srcdir ]; then \
                mkdir $(HEADERS_DIR)/arch/$(ARCH)/$$dir; \
                cp -a $$srcdir $(HEADERS_DIR)/arch/$(ARCH)/$$dir; \
            fi \
        done


# All other targets are passed through to the kernel's build system.
LOCAL_TARGETS = \
    default build source save-config clean clean-all print-build-dir \
    linux-headers
KERNEL_TARGETS = $(filter-out $(LOCAL_TARGETS),$(MAKECMDGOALS))
$(KERNEL_TARGETS): $(BUILD_DIR)/.config
	make -C $(SOURCE_DIR) $(KERNEL_TARGETS)

.PHONY: $(LOCAL_TARGETS)
