from common import *

# # Tune measurement.
# 
# 
# tunescale = Waveform('TUNESCALE', 4096, 'FLOAT',
#     DESC = 'Scale for tune measurement')
# startfreq = aOut('SWPSTARTFREQ', 0, 468, VAL  = 0.05, PREC = 4,
#     DESC = 'Sweep start freq', MDEL = -1, FLNK = tunescale)
# stopfreq  = aOut('SWPSTOPFREQ',  0, 468, VAL  = 0.45, PREC = 4,
#     DESC = 'Sweep stop freq', MDEL = -1, FLNK = tunescale)
# freqstep = aOut('SWPFREQSTEP',  VAL  = 1,  PREC = 4,
#     DESC = 'Phase advance step', FLNK = tunescale)
# longOut('DDCSKEW',
#     VAL = 32, DESC = 'Direct update of DDC skew', FLNK = tunescale)
# 
# records.calcout('SWPFREQSTEP_C',
#     CALC = '(B-A)/4096',
#     INPA = CP(startfreq),
#     INPB = CP(stopfreq),
#     OUT  = PP(freqstep),
#     OOPT = 'Every Time')
# 
# 
# #
# # The TUNESCAN record is designed to be configured to automatically scan.  On
# # each scan it processes the dual buffer (to ensure current IQ data is
# # captured) and then converts this to power and tune measurements.  Finally
# # the soft trigger is fired to trigger another round of capture.
# tune_records = [
#     # Read the IQ data from the internal buffer
#     hb_buf_lower,
#     # Prepare the tune measurement data
#     boolOut('PROCESS_TUNE'),
#     # Update the I and Q waveforms
#     Waveform('DDC_I', 4096,
#         DESC = 'DDC response, I component'),
#     Waveform('DDC_Q', 4096,
#         DESC = 'DDC response, Q component'),
#     # Compute the tune power spectrum
#     Waveform('TUNEPOWER', 4096,
#         DESC = 'DDC power spectrum'),
#     # Compute the peak power and return this as the tune
#     aIn('TUNE', 0, 0.5, '', 4, DESC = 'Measured tune'),
#     aIn('TUNEPHASE', -180, 180, 'deg', 2, DESC = 'Phase at tune'),
#     # Compute the cumulative sum of tune power
#     Waveform('RAWCUMSUM_I', 4096,
#         DESC = 'DDC cumulative sum, I part'),
#     Waveform('RAWCUMSUM_Q', 4096,
#         DESC = 'DDC cumulative sum, Q part'),
#     Waveform('CUMSUM_I', 4096,
#         DESC = 'DDC cumulative sum, I part'),
#     Waveform('CUMSUM_Q', 4096,
#         DESC = 'DDC cumulative sum, Q part'),
#     # Now compute the tune and phase from the cumulative sum
#     aIn('CUMSUMTUNE', 0, 0.5, '', 5, DESC = 'Measured tune using cumsum'),
#     aIn('CUMSUMPHASE', -180, 180, 'deg', 2, DESC = 'Phase at cumsum tune'),
# 
#     # Finally trigger capture of the next round of data.
#     softtrig]
# create_fanout('TUNESCAN', SCAN = '1 second', *tune_records)


