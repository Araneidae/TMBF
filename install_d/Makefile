TOP = ..
include $(TOP)/configure/CONFIG

# Default target is to do nothing, install target will do the work.
build:

# The install target in this makefile will be run after successful completion
# of the rest of the project makefile.  We patch up the installation ready
# for deployment to Libera.
install: locations
	for f in $(foreach ARCH, $(BUILD_ARCHS), $(TOP)/bin/$(ARCH)); do \
            cp runioc st.cmd $$f; \
        done

clean:
	rm -f locations

# The list of possible installation locations is derived from CONFIG.
LOC_SED_COMMAND = '/^([[:alnum:]])_IOC_.*$$/{s//\1/;p;}'
locations: CONFIG
	sed -rn $(LOC_SED_COMMAND) $< | sort | uniq >$@
