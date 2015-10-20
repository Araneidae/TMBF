# Defaults and configuration for TMBF
# 2015.10.19: Changes by Ubaldo
# Settings for 
# tune sweep_range: from 0.05 to 0.025
# harmonic: from 80 to 20 

harmonic = 20 # 80
bunch = 0

# Settings for basic tune measurement.
tune_threshold = 0.3
min_block_sep = 20
min_block_len = 20

# Settings for peak tune measurement
peak_smoothing = '/16'
peak_fit_threshold = 0.3
peak_max_error = 1
peak_min_width = 0
peak_max_width = 1

# Default tune selection
tune_select = 'Peak Fit'

sweep_dwell_time = 100

blanking_interval = 10000
blanking_source = 'SCLK Input'

sweep_range = 0.025 # 0.05
alarm_range = 0.01
tune_direction = 'Forwards'
keep_feedback = 'No feedback'

dac_output = 0                  # By default DAC output is off
bunch_mode = 'All Bunches'      # By default detect on all bunches
detector_input = 'FIR'


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Mode specific settings

# Multibunch tune measurement
TUNE_sweep_gain = '-42dB'

# Accelerator physics mode
AP_sweep_gain = '0dB'
AP_tune = 0.25
AP_sweep_range = 0.245
AP_alarm_range = 0.245
AP_detector_input = 'ADC'
AP_min_block_len = 5

# Feedback on, single bunch tune measurement
FB_dac_output = 1               # Enable FIR output in this mode
FB_keep_feedback = 'Keep feedback'
FB_harmonic = 20 # = 80
FB_sweep_gain = '-48dB'


# Lab setup
T_tune = 0.7885
T_harmonic = 37
T_detector_input = 'ADC'
