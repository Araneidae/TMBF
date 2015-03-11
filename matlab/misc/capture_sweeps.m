% Simple script to configure and capture IQ sweeps over a variety of gains.

function [result, s] = capture_sweeps(tmbf, dwell, det_gain)
    if ~exist('tmbf', 'var'); tmbf = 'TS-DI-TMBF-01'; end
    if ~exist('dwell', 'var'); dwell = 10000; end
    if ~exist('det_gain', 'var'); det_gain = '-12dB'; end

    prepare(tmbf, dwell, det_gain);

    excitations = {'Off'};
    for n = 2:14
        excitations{n} = sprintf('%ddB', 6 * (n - 14));     % Want -72:6:0
    end

    result = zeros(14, 4096);
    for n = 1:14
        result(n, :) = capture_one(tmbf, excitations{n});
    end

    s = linspace(0, 468, 4097);
    s = s(1:end-1);
end

function prepare(tmbf, dwell, det_gain)
    put = @(pv, value) lcaPut([tmbf ':' pv], value);

    % Configure sweep
    put('SEQ:1:START_FREQ_S', 0);
    put('SEQ:1:END_FREQ_S', 468);
    put('SEQ:1:COUNT_S', 4096);
    put('SEQ:1:HOLDOFF_S', 0);
    put('SEQ:1:DWELL_S', dwell);
    put('SEQ:1:BLANK_S', 'Off');
    put('SEQ:1:ENWIN_S', 'Windowed');
    put('SEQ:1:CAPTURE_S', 'Capture');

    % Ensure super-sequencer not active
    put('SEQ:SUPER:RESET_S', 0);
    put('SEQ:SUPER:COUNT_S', 1);

    % Configure detector
    put('DET:MODE_S', 'All Bunches');
    put('DET:AUTOGAIN_S', 'Fixed Gain');
    put('DET:INPUT_S', 'ADC');
    put('DET:GAIN_S', det_gain);

    % Configure triggers
    put('TRG:BUF:RESET_S', 0);      % Just in case running at the moment
    put('TRG:BUF:SEL_S', 'Soft');
    put('TRG:BUF:MODE_S', 'One Shot');
    put('TRG:SEQ:SEL_S', 'BUF trigger');
    put('TRG:SYNC_S', 'Separate');
end

function iq = capture_one(tmbf, excitation)
    put = @(pv, value) lcaPut([tmbf ':' pv], value);
    get = @(pv) lcaGet([tmbf ':' pv]);
    get_iq = @() [1 1j] * lcaGet({[tmbf ':TUNE:I']; [tmbf ':TUNE:Q']});

    put('SEQ:1:GAIN_S', excitation);
    put('TRG:BUF:ARM_S', 0);

    % Wait for capture to complete
    fprintf(1, '% s', excitation);
    while ~strcmp(get('TRG:SEQ:STATUS'), 'Ready')
        fprintf(1, '.');
        pause(1);
    end
    fprintf(1, '\n');

    iq = get_iq();
end
