TMBF: Transverse Multi-Bunch Feedback processor
===============================================

`Libera TMBF
<http://www.i-tech.si/accelerators-instrumentation/libera-bunchbybunch/>`_ is an
instrument for stabilising the transverse bunch-by-bunch instabilities that can
arise in a storage ring with a relatively high current density.  This instrument
consists of a fast ADC and DAC coupled through a Xilinx Virtex-II Pro FPGA
together with an Xscale PXA-255 ARM processor for control.

The software provided here implements an EPICS interface to a specific FPGA
implementation running on this instrument.  This system has been running at
`Diamond Light Source <http://diamond.ac.uk>`_ for some years now and is
expected to be installed at `ALBA <http://www.cells.es>`_ some time in 2015.

To build this system you need the following:

* A cross-compiler for the ARM
* An install of EPICS cross-built for the ARM
* The `epicsdbbuilder <https://github.com/Araneidae/epicsdbbuilder>`_ module
* The `epics_device <https://github.com/Araneidae/epics_device>`_ module

To run this system you need a Libera TMBF processor and a copy of our FPGA
image.  This can be obtained by contacting the author at Diamond Light Source.
