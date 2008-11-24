GCC_BASE=/home/targetOS/linux-arm/host/x86-linux/gcc_3.3.3/
ARM_GCC=${GCC_BASE}/bin/arm-linux-g++
ARM_STRIP=${GCC_BASE}/bin/arm-linux-strip

default: Monitor

Monitor: Monitor.cpp
	$(ARM_GCC) -o $@ $<
	$(ARM_STRIP) $@

newMonitor: newMonitor.cpp
	$(ARM_GCC) -o $@ $<
	$(ARM_STRIP) $@

install: Monitor newMonitor
	cp Monitor newMonitor /mt/libera/nfs/testing
